#!/usr/bin/env python3
from __future__ import annotations
import argparse,asyncio,json,logging,math,threading,time
from collections import deque
from pathlib import Path
import carla, numpy as np
from aiohttp import web
import live_drive_server as base
import protocol as protocol_v1
import protocol_v2
from detection_overlay import DetectionOverlay,FramePoseHistory
from radar_sensor import FrontRadarSensor,RadarState
from reference_path import ReferencePath,ReferencePathVisualizer,RouteRecorder
from scenario_manager import spawn_fixed_scenario

LOG=logging.getLogger("modular-live-drive"); ROOT=Path(__file__).resolve().parent
CAMERA_ORDER=["CAM_FRONT","CAM_FRONT_RIGHT","CAM_BACK_RIGHT","CAM_BACK","CAM_BACK_LEFT","CAM_FRONT_LEFT"]

class LatestPacketCache:
    def __init__(self):self._c=threading.Condition();self._frame=-1;self._packet=None;self._meta={}
    def publish(self,frame,packet,meta=None):
        with self._c:
            if int(frame)<=self._frame:return
            self._frame=int(frame);self._packet=bytes(packet);self._meta=dict(meta or {});self._c.notify_all()
    def wait_newer(self,last,timeout=2):
        end=time.monotonic()+timeout
        with self._c:
            while self._packet is None or self._frame<=int(last):
                left=end-time.monotonic()
                if left<=0:return None
                self._c.wait(left)
            return self._frame,self._packet,dict(self._meta)
    def status(self):
        with self._c:return {"frame_id":self._frame,**self._meta}

class ModularBrowserControl(base.BrowserControl):
    def __init__(self):super().__init__();self._m=threading.Lock();self._auto=False
    def update_json(self,message):
        super().update_json(message)
        try:d=json.loads(message)
        except Exception:return
        if d.get("type")=="control" and "auto_enabled" in d:
            with self._m:self._auto=bool(d["auto_enabled"])
    def snapshot(self):
        s=super().snapshot()
        with self._m:s["auto_enabled"]=self._auto
        return s

def drive_part(r):return r.get("autodrive",r) if isinstance(r,dict) else {}

class ControlArbiter:
    def __init__(self,high_max_age_s=.30,low_max_age_s=2.5):
        self.high_max_age_s=high_max_age_s;self.low_max_age_s=low_max_age_s;self._l=threading.Lock()
        self.low=self.high=None;self.low_at=self.high_at=None;self.low_times=deque(maxlen=20);self.high_times=deque(maxlen=80);self.active="manual";self.applied={}
    def update_low(self,r):
        now=time.monotonic()
        with self._l:self.low=dict(r);self.low_at=now;self.low_times.append(now)
    def update_high(self,r):
        now=time.monotonic()
        with self._l:self.high=dict(r);self.high_at=now;self.high_times.append(now)
    def choose(self):
        now=time.monotonic()
        with self._l:low=None if self.low is None else dict(self.low);high=None if self.high is None else dict(self.high);la=self.low_at;ha=self.high_at
        lf=low is not None and la is not None and now-la<=self.low_max_age_s
        hf=high is not None and ha is not None and now-ha<=self.high_max_age_s
        if hf and drive_part(high).get("aeb_triggered"):return "high_aeb",drive_part(high)
        if lf and drive_part(low).get("aeb_triggered"):return "low_aeb",drive_part(low)
        c=[]
        if lf:c.append((la,"low_perception",drive_part(low)))
        if hf:c.append((ha,"high_control",drive_part(high)))
        return ("safe_stop",None) if not c else max(c,key=lambda x:x[0])[1:]
    def mark(self,src,d):
        with self._l:self.active=src;self.applied={} if d is None else dict(d)
    @staticmethod
    def rate(q):return None if len(q)<2 or q[-1]<=q[0] else (len(q)-1)/(q[-1]-q[0])
    def status(self):
        now=time.monotonic()
        with self._l:
            low=self.low or {};high=self.high or {};la=self.low_at;ha=self.high_at;lr=self.rate(list(self.low_times));hr=self.rate(list(self.high_times));active=self.active;ap=dict(self.applied)
        ld,hd=drive_part(low),drive_part(high)
        return {"active_source":active,"active_state":ap.get("fsm_state"),"aeb_triggered":bool(ap.get("aeb_triggered",False)),
            "low":{"rate_hz":None if lr is None else round(lr,2),"age_ms":None if la is None else round((now-la)*1000,1),"frame_id":low.get("frame_id"),"inference_ms":low.get("inference_ms"),"gateway_roundtrip_ms":low.get("gateway_roundtrip_ms"),"fsm_state":ld.get("fsm_state")},
            "high":{"rate_hz":None if hr is None else round(hr,2),"age_ms":None if ha is None else round((now-ha)*1000,1),"frame_id":high.get("frame_id"),"processing_ms":high.get("processing_ms"),"gateway_roundtrip_ms":high.get("gateway_roundtrip_ms"),"fsm_state":hd.get("fsm_state"),"aeb_triggered":hd.get("aeb_triggered"),"d_safe_m":hd.get("d_safe_m"),"d_radar_m":hd.get("d_radar_m")}}

class TrajectoryVisualizer:
    def draw(self,world,result):
        planner=drive_part(result).get("planner") or {}; cands=planner.get("candidates") or []; best=planner.get("best_trajectory") or []
        def loc(p,z=.15):return carla.Location(float(p["x"]),float(p["y"]),float(p.get("z",0))+z)
        try:
            for c in cands:
                pts=c.get("points",[]) if isinstance(c,dict) else c
                for a,b in zip(pts[:-1],pts[1:]):world.debug.draw_line(loc(a),loc(b),thickness=.035,color=carla.Color(120,120,255),life_time=1.2,persistent_lines=False)
            for a,b in zip(best[:-1],best[1:]):world.debug.draw_line(loc(a,.2),loc(b,.2),thickness=.11,color=carla.Color(20,255,80),life_time=1.2,persistent_lines=False)
        except Exception:LOG.exception("trajectory draw failed")

class ModularSimulation(base.CarlaSimulation):
    def __init__(self,*a,control_cache,radar_state,reference_path,recorder,route_visualizer,arbiter,**kw):
        super().__init__(*a,**kw);self.control_cache=control_cache;self.radar_state=radar_state;self.reference_path=reference_path;self.recorder=recorder;self.route_visualizer=route_visualizer;self.arbiter=arbiter
    def metadata(self,frame_id):
        p=self.pose_history.get(frame_id)
        if p is None:
            t=self.ego.get_transform();x,y,z=t.location.x,t.location.y,t.location.z;yaw=t.rotation.yaw;speed=self.speed_kmh/3.6
        else:
            m=p["actor_to_world"];x,y,z=float(m[0,3]),float(m[1,3]),float(m[2,3]);yaw=math.degrees(math.atan2(float(m[1,0]),float(m[0,0])));speed=float(p["speed_kmh"])/3.6
        return {"ego":{"x":x,"y":y,"z":z,"yaw_deg":yaw,"speed_mps":speed},"radar":self.radar_state.snapshot(),"route":self.reference_path.segment(x,y)}
    def perception_metadata(self,frame_id):d=self.metadata(frame_id);d["schema"]="perception_input_v2";return d
    def _apply_browser_control(self):
        st=self.controls.snapshot()
        if not st.get("auto_enabled",False):self.arbiter.mark("manual",None);return super()._apply_browser_control()
        if st["view_index"]!=self.current_view:self.current_view=st["view_index"];self.camera.set_transform(base.CAMERA_VIEWS[self.current_view]["transform"])
        src,d=self.arbiter.choose();c={} if d is None else d.get("control",{})
        steer=float(np.clip(c.get("steer",0),-1,1));throttle=float(np.clip(c.get("throttle",0),0,1));brake=float(np.clip(c.get("brake",.6 if d is None else 0),0,1))
        if d and d.get("aeb_triggered"):throttle,brake=0,1
        self.ego.apply_control(carla.VehicleControl(throttle=throttle,steer=steer,brake=brake,hand_brake=False,reverse=False,manual_gear_shift=False));self.arbiter.mark(src,d)
    def _record_tick(self,frame_id,tick_elapsed_ms):
        super()._record_tick(frame_id,tick_elapsed_ms);t=self.ego.get_transform();self.recorder.record(t.location.x,t.location.y,t.location.z,t.rotation.yaw);self.route_visualizer.maybe_draw(self.world,time.monotonic())
        md=self.metadata(frame_id);pkt=protocol_v2.pack_control_input(frame_id,time.time_ns(),md);self.control_cache.publish(frame_id,pkt,{"bytes":len(pkt),"radar":md["radar"]})

async def index(req):return web.FileResponse(ROOT/"modular_index.html")
async def status(req):
    s=req.app["simulation"]
    return web.json_response({"ok":s.error is None and s.ready.is_set(),"camera_view":base.CAMERA_VIEWS[s.current_view]["name"],"performance":s.performance_status(),"six_camera":None if s.camera_rig is None else s.camera_rig.status(),"legacy_batch":s.batch_cache.status(),"control_input":s.control_cache.status(),"radar":req.app["radar_sensor"].status(),"reference_path":s.reference_path.status(),"route_recording":s.recorder.status(),"edge":req.app["result_store"].status(),"control":s.arbiter.status(),"scenario":req.app["scenario_status"]})
async def rr_start(req):return web.json_response(req.app["simulation"].recorder.start())
async def rr_stop(req):return web.json_response(req.app["simulation"].recorder.stop())
async def rr_toggle(req):return web.json_response(req.app["simulation"].recorder.toggle())

async def gateway_perception(req):
    ws=web.WebSocketResponse(heartbeat=10,max_msg_size=64*1024*1024,compress=False);await ws.prepare(req);s=req.app["simulation"];store=req.app["result_store"];store.set_gateway(True,req.remote)
    try:
        async for msg in ws:
            if msg.type==web.WSMsgType.TEXT:
                cmd=json.loads(msg.data)
                if cmd.get("type")!="ready":await ws.send_json({"type":"error","message":"expected ready"});continue
                item=await asyncio.get_running_loop().run_in_executor(None,s.batch_cache.wait_newer,int(cmd.get("last_frame_id",-1)),2.0)
                if item is None:await ws.send_json({"type":"no_data"});continue
                frame,v1,_=item;b=protocol_v1.unpack_batch(v1,verify_crc=True);jpg={c["name"]:c["jpeg"] for c in b["cameras"]}
                await ws.send_bytes(protocol_v2.pack_perception_batch(frame,b["timestamp_ns"],CAMERA_ORDER,jpg,s.perception_metadata(frame)))
            elif msg.type==web.WSMsgType.BINARY:
                r=protocol_v2.unpack_perception_result(msg.data);r.pop("_wire",None);store.update(r);s.arbiter.update_low(r);req.app["trajectory_visualizer"].draw(s.world,r)
            else:break
    finally:store.set_gateway(False)
    return ws

async def gateway_control(req):
    ws=web.WebSocketResponse(heartbeat=10,max_msg_size=4*1024*1024,compress=False);await ws.prepare(req);s=req.app["simulation"]
    async for msg in ws:
        if msg.type==web.WSMsgType.TEXT:
            cmd=json.loads(msg.data)
            item=await asyncio.get_running_loop().run_in_executor(None,s.control_cache.wait_newer,int(cmd.get("last_frame_id",-1)),1.0)
            if item is None:await ws.send_json({"type":"no_data"})
            else:await ws.send_bytes(item[1])
        elif msg.type==web.WSMsgType.BINARY:
            r=protocol_v2.unpack_control_result(msg.data);r.pop("_wire",None);s.arbiter.update_high(r)
        else:break
    return ws

async def shutdown(app):app["radar_sensor"].stop();await base.on_shutdown(app)

def reset_to_route(s,r):
    p=r.first()
    if p is None:return False
    s.ego.set_transform(carla.Transform(carla.Location(p.x,p.y,p.z+.15),carla.Rotation(yaw=p.yaw_deg)));s.ego.set_target_velocity(carla.Vector3D());return True

def main():
    p=argparse.ArgumentParser();p.add_argument("--carla-host",default="127.0.0.1");p.add_argument("--carla-port",type=int,default=2000);p.add_argument("--town",default="Town03");p.add_argument("--http-host",default="0.0.0.0");p.add_argument("--http-port",type=int,default=8080);p.add_argument("--max-speed-kmh",type=float,default=30);p.add_argument("--route-csv",default=str(ROOT/"routes"/"route.csv"));p.add_argument("--route-raw-csv",default=str(ROOT/"routes"/"route_raw.csv"));p.add_argument("--scenario-json",default="");p.add_argument("--reset-ego-to-route-start",action="store_true");p.add_argument("--min-score",type=float,default=.185);a=p.parse_args()
    logging.basicConfig(level=logging.INFO,format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    controls=ModularBrowserControl();batch=base.LatestBatchCache();cc=LatestPacketCache();poses=FramePoseHistory();overlay=DetectionOverlay(poses,base.DISPLAY_WIDTH,base.DISPLAY_HEIGHT,base.CAMERA_FOV,max_result_age_seconds=3,min_score=a.min_score);source=base.LatestVideoFrame(overlay);store=base.ResultStore(overlay)
    route=ReferencePath(a.route_csv);rec=RouteRecorder(route,a.route_raw_csv);rv=ReferencePathVisualizer(route);rs=RadarState();arb=ControlArbiter()
    sim=ModularSimulation(a.carla_host,a.carla_port,a.town,source,controls,batch,poses,traffic_vehicles=0,traffic_walkers=0,freeze_traffic=True,max_speed_kmh=a.max_speed_kmh,control_cache=cc,radar_state=rs,reference_path=route,recorder=rec,route_visualizer=rv,arbiter=arb);sim.start()
    if a.reset_ego_to_route_start and not reset_to_route(sim,route):LOG.warning("route missing; cannot reset ego")
    scen=spawn_fixed_scenario(a.scenario_json,sim);rad=FrontRadarSensor(rs);rad.start(sim.world,sim.ego)
    app=web.Application(client_max_size=2*1024*1024);app.update({"simulation":sim,"source":source,"controls":controls,"result_store":store,"radar_sensor":rad,"scenario_status":scen,"trajectory_visualizer":TrajectoryVisualizer()})
    app.router.add_get("/",index);app.router.add_get("/status",status);app.router.add_get("/health",status);app.router.add_get("/gateway/perception",gateway_perception);app.router.add_get("/gateway/control",gateway_control);app.router.add_post("/route/record/start",rr_start);app.router.add_post("/route/record/stop",rr_stop);app.router.add_post("/route/record/toggle",rr_toggle);app.router.add_post("/offer",base.offer);app.on_shutdown.append(shutdown)
    web.run_app(app,host=a.http_host,port=a.http_port,access_log=LOG)
if __name__=="__main__":main()

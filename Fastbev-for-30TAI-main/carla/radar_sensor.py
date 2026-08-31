#!/usr/bin/env python3
import math,threading,time,carla
class RadarState:
    def __init__(self):
        self._lock=threading.Lock(); self._sample={"valid":False,"nearest_distance_m":None,"relative_velocity_mps":None,"detection_count":0,"radar_frame_id":-1,"sample_ts_ns":0}
    def update(self,m):
        ds=list(m); valid=[d for d in ds if float(d.depth)>0.15 and math.isfinite(float(d.depth))]
        n=min(valid,key=lambda d:float(d.depth)) if valid else None
        s={"valid":n is not None,"nearest_distance_m":None if n is None else float(n.depth),"relative_velocity_mps":None if n is None else float(n.velocity),"detection_count":len(ds),"radar_frame_id":int(m.frame),"sample_ts_ns":time.time_ns()}
        with self._lock:self._sample=s
    def snapshot(self):
        with self._lock:s=dict(self._sample)
        s["age_ms"]=None if not s["sample_ts_ns"] else (time.time_ns()-s["sample_ts_ns"])/1e6
        return s
class FrontRadarSensor:
    def __init__(self,state,horizontal_fov_deg=12,vertical_fov_deg=5,range_m=35,sensor_tick_s=0.05,points_per_second=2000):
        self.state=state; self.horizontal_fov_deg=float(horizontal_fov_deg); self.vertical_fov_deg=float(vertical_fov_deg); self.range_m=float(range_m); self.sensor_tick_s=float(sensor_tick_s); self.points_per_second=int(points_per_second); self.actor=None
    def start(self,world,ego):
        bp=world.get_blueprint_library().find("sensor.other.radar")
        for k,v in [("horizontal_fov",self.horizontal_fov_deg),("vertical_fov",self.vertical_fov_deg),("range",self.range_m),("sensor_tick",self.sensor_tick_s),("points_per_second",self.points_per_second)]:bp.set_attribute(k,str(v))
        self.actor=world.spawn_actor(bp,carla.Transform(carla.Location(x=2.4,z=0.85)),attach_to=ego); self.actor.listen(self.state.update)
    def stop(self):
        if self.actor:
            try:self.actor.stop();self.actor.destroy()
            except Exception:pass
            self.actor=None
    def status(self):return {**self.state.snapshot(),"horizontal_fov_deg":self.horizontal_fov_deg,"range_m":self.range_m,"sensor_tick_s":self.sensor_tick_s}

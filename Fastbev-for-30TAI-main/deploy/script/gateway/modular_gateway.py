#!/usr/bin/env python3
from __future__ import annotations
import argparse,asyncio,base64,json,socket,time
import aiohttp
import protocol_v2 as protocol

EXPECTED=["CAM_FRONT","CAM_FRONT_RIGHT","CAM_BACK_RIGHT","CAM_BACK","CAM_BACK_LEFT","CAM_FRONT_LEFT"]

class EdgeConnection:
    def __init__(self,host,port,timeout,label):self.host=host;self.port=port;self.timeout=timeout;self.label=label;self.sock=None
    def close(self):
        if self.sock:
            try:self.sock.close()
            except OSError:pass
        self.sock=None
    def connect(self):
        self.close();self.sock=socket.create_connection((self.host,self.port),timeout=self.timeout);self.sock.settimeout(self.timeout);self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);print(f"[{self.label}] connected {self.host}:{self.port}",flush=True)
    def exchange(self,pkt):
        err=None
        for n in range(2):
            try:
                if self.sock is None:self.connect()
                protocol.send_packet(self.sock,pkt);return protocol.recv_packet(self.sock)
            except Exception as e:
                err=e;self.close();print(f"[{self.label}] exchange failed {n+1}/2: {e}",flush=True)
                if n==0:time.sleep(.1)
        raise ConnectionError(err)

async def perception_once(a,edge):
    timeout=aiohttp.ClientTimeout(total=None,sock_connect=10,sock_read=None)
    async with aiohttp.ClientSession(timeout=timeout) as session:
        async with session.ws_connect(a.server_perception,heartbeat=10,max_msg_size=64*1024*1024,compress=0) as ws:
            last=-1;print("[LOW] server connected",flush=True)
            while True:
                await ws.send_json({"type":"ready","last_frame_id":last});m=await ws.receive()
                if m.type==aiohttp.WSMsgType.BINARY:
                    t=time.perf_counter();b=protocol.unpack_perception_batch(m.data);names=[c["name"] for c in b["cameras"]]
                    if names!=EXPECTED:raise protocol.ProtocolError(f"camera order mismatch {names}")
                    rpacket=await asyncio.get_running_loop().run_in_executor(None,edge.exchange,m.data);r=protocol.unpack_perception_result(rpacket);r.pop("_wire",None);rtt=(time.perf_counter()-t)*1000;r["gateway_roundtrip_ms"]=round(rtt,2);r["gateway_received_wall_ns"]=time.time_ns();await ws.send_bytes(protocol.pack_perception_result(r))
                    if a.viewer_push_url:
                        payload={"frame_id":str(b["frame_id"]),"capture_ts_ns":int(b["timestamp_ns"]),"inference_ms":float(r.get("inference_ms",0)),"gateway_roundtrip_ms":round(rtt,2),"source":r.get("source","edge"),"objects":r.get("objects",[]),"images":{c["name"]:base64.b64encode(c["jpeg"]).decode("ascii") for c in b["cameras"]}}
                        try:
                            async with session.post(a.viewer_push_url,json=payload,timeout=aiohttp.ClientTimeout(total=a.viewer_timeout)) as resp:
                                if resp.status!=200:raise RuntimeError(f"viewer HTTP {resp.status}")
                        except Exception as e:print(f"[VIEWER] skipped: {e}",flush=True)
                    d=r.get("autodrive",{});print(f"[LOW] frame={b['frame_id']} infer={r.get('inference_ms',0):.1f}ms rtt={rtt:.1f}ms state={d.get('fsm_state')}",flush=True);last=int(b["frame_id"])
                elif m.type==aiohttp.WSMsgType.TEXT:
                    d=json.loads(m.data)
                    if d.get("type")=="no_data":await asyncio.sleep(.01)
                    elif d.get("type")=="error":raise RuntimeError(d.get("message"))
                else:raise ConnectionError(f"low websocket closed: {m.type}")

async def control_once(a,edge):
    timeout=aiohttp.ClientTimeout(total=None,sock_connect=10,sock_read=None)
    async with aiohttp.ClientSession(timeout=timeout) as session:
        async with session.ws_connect(a.server_control,heartbeat=10,max_msg_size=4*1024*1024,compress=0) as ws:
            last=-1;print("[HIGH] server connected",flush=True)
            while True:
                await ws.send_json({"type":"ready","last_frame_id":last});m=await ws.receive()
                if m.type==aiohttp.WSMsgType.BINARY:
                    t=time.perf_counter();inp=protocol.unpack_control_input(m.data);frame=int(inp["frame_id"]);rpkt=await asyncio.get_running_loop().run_in_executor(None,edge.exchange,m.data);r=protocol.unpack_control_result(rpkt);r.pop("_wire",None);rtt=(time.perf_counter()-t)*1000;r["gateway_roundtrip_ms"]=round(rtt,3);r["gateway_received_wall_ns"]=time.time_ns();await ws.send_bytes(protocol.pack_control_result(r));d=r.get("autodrive",{});print(f"[HIGH] frame={frame} board={r.get('processing_ms',0):.3f}ms rtt={rtt:.3f}ms AEB={d.get('aeb_triggered')}",flush=True);last=frame
                elif m.type==aiohttp.WSMsgType.TEXT:
                    d=json.loads(m.data)
                    if d.get("type")=="no_data":await asyncio.sleep(.002)
                    elif d.get("type")=="error":raise RuntimeError(d.get("message"))
                else:raise ConnectionError(f"high websocket closed: {m.type}")

async def forever(label,coro,edge,delay):
    while True:
        try:await coro()
        except asyncio.CancelledError:raise
        except Exception as e:edge.close();print(f"[{label}] reconnect: {e}",flush=True);await asyncio.sleep(delay)

async def main_async(a):
    low=EdgeConnection(a.edge_host,a.edge_port,a.edge_timeout,"EDGE-LOW");high=EdgeConnection(a.edge_host,a.control_port,a.control_timeout,"EDGE-HIGH")
    try:await asyncio.gather(forever("LOW",lambda:perception_once(a,low),low,a.reconnect_delay),forever("HIGH",lambda:control_once(a,high),high,a.reconnect_delay))
    finally:low.close();high.close()

def main():
    p=argparse.ArgumentParser();p.add_argument("--server-perception",default="ws://10.134.143.120:8080/gateway/perception");p.add_argument("--server-control",default="ws://10.134.143.120:8080/gateway/control");p.add_argument("--edge-host",default="127.0.0.1");p.add_argument("--edge-port",type=int,default=5200);p.add_argument("--control-port",type=int,default=5201);p.add_argument("--edge-timeout",type=float,default=15);p.add_argument("--control-timeout",type=float,default=2);p.add_argument("--reconnect-delay",type=float,default=1);p.add_argument("--viewer-push-url",default="");p.add_argument("--viewer-timeout",type=float,default=5);a=p.parse_args()
    try:asyncio.run(main_async(a))
    except KeyboardInterrupt:print("\nStopped.")
if __name__=="__main__":main()

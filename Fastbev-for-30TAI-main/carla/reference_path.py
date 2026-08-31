#!/usr/bin/env python3
from __future__ import annotations
import csv, hashlib, math, threading
from dataclasses import dataclass
from pathlib import Path
import numpy as np

@dataclass(frozen=True)
class RoutePoint:
    x:float; y:float; z:float; yaw_deg:float; s_m:float
    def as_dict(self): return {"x":self.x,"y":self.y,"z":self.z,"yaw_deg":self.yaw_deg,"s_m":self.s_m}

def cumulative_s(xyz):
    if len(xyz)==0:return np.zeros(0)
    return np.concatenate([[0.0],np.cumsum(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1))])

def smooth_resample(raw_xyz,spacing_m=0.75):
    if len(raw_xyz)<2:return raw_xyz.copy()
    kept=[raw_xyz[0]]
    for p in raw_xyz[1:]:
        if np.linalg.norm(p[:2]-kept[-1][:2])>=0.15: kept.append(p)
    xyz=np.asarray(kept,dtype=float)
    if len(xyz)<2:return xyz.copy()
    s=cumulative_s(xyz); total=float(s[-1])
    if total<1e-6:return xyz[:1]
    tangent=np.zeros_like(xyz)
    tangent[0]=(xyz[1]-xyz[0])/max(1e-6,s[1]-s[0])
    tangent[-1]=(xyz[-1]-xyz[-2])/max(1e-6,s[-1]-s[-2])
    for i in range(1,len(xyz)-1):
        tangent[i]=(xyz[i+1]-xyz[i-1])/max(1e-6,s[i+1]-s[i-1])
    qs=np.arange(0,total,max(0.2,float(spacing_m)))
    if not len(qs) or qs[-1]<total: qs=np.append(qs,total)
    out=[]; seg=0
    for q in qs:
        while seg+1<len(s)-1 and q>s[seg+1]: seg+=1
        h=max(1e-6,s[seg+1]-s[seg]); t=(q-s[seg])/h; t2=t*t; t3=t2*t
        out.append((2*t3-3*t2+1)*xyz[seg]+(t3-2*t2+t)*h*tangent[seg]+(-2*t3+3*t2)*xyz[seg+1]+(t3-t2)*h*tangent[seg+1])
    return np.asarray(out)

def xyz_to_points(xyz):
    if len(xyz)==0:return []
    s=cumulative_s(xyz); out=[]
    for i,p in enumerate(xyz):
        if len(xyz)==1:yaw=0.0
        else:
            d=(xyz[i+1]-p) if i+1<len(xyz) else (p-xyz[i-1])
            yaw=math.degrees(math.atan2(float(d[1]),float(d[0])))
        out.append(RoutePoint(float(p[0]),float(p[1]),float(p[2]),yaw,float(s[i])))
    return out

def save_route_csv(path,points):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=["x","y","z","yaw_deg","s_m"]); w.writeheader()
        for p in points:w.writerow(p.as_dict())

def load_route_csv(path):
    path=Path(path)
    with path.open("r",newline="",encoding="utf-8") as f:
        rows=list(csv.DictReader(f))
    if not rows:return []
    xyz=np.asarray([[float(r["x"]),float(r["y"]),float(r["z"])] for r in rows],dtype=float)
    derived=xyz_to_points(xyz); out=[]
    for i,r in enumerate(rows):
        out.append(RoutePoint(float(r["x"]),float(r["y"]),float(r["z"]),
            float(r["yaw_deg"]) if r.get("yaw_deg") not in (None,"") else derived[i].yaw_deg,
            float(r["s_m"]) if r.get("s_m") not in (None,"") else derived[i].s_m))
    return out

class ReferencePath:
    def __init__(self,path):
        self.path=Path(path); self._lock=threading.Lock(); self._points=[]; self._route_id="missing"; self.reload_if_exists()
    def reload_if_exists(self):
        with self._lock:
            if not self.path.is_file(): self._points=[]; self._route_id="missing"; return False
            self._points=load_route_csv(self.path); self._route_id=hashlib.sha1(self.path.read_bytes()).hexdigest()[:12]; return True
    def replace(self,points): save_route_csv(self.path,points); self.reload_if_exists()
    def snapshot(self):
        with self._lock:return list(self._points),self._route_id
    def first(self):
        pts,_=self.snapshot(); return pts[0] if pts else None
    def status(self):
        pts,rid=self.snapshot(); return {"loaded":bool(pts),"path":str(self.path),"route_id":rid,"points":len(pts),"length_m":0 if not pts else round(pts[-1].s_m,2)}
    def segment(self,x,y,behind_points=0,ahead_points=64):
        pts,rid=self.snapshot()
        if not pts:return {"route_id":rid,"nearest_index":-1,"points":[]}
        idx=min(range(len(pts)),key=lambda i:(pts[i].x-x)**2+(pts[i].y-y)**2)
        s=max(0,idx-behind_points); e=min(len(pts),idx+ahead_points)
        return {"route_id":rid,"nearest_index":idx,"start_index":s,"points":[p.as_dict() for p in pts[s:e]]}

class RouteRecorder:
    def __init__(self,reference_path,raw_path,sample_distance_m=0.75,spline_spacing_m=0.75):
        self.reference_path=reference_path; self.raw_path=Path(raw_path); self.sample_distance_m=float(sample_distance_m); self.spline_spacing_m=float(spline_spacing_m)
        self._lock=threading.Lock(); self._recording=False; self._raw=[]; self._message=""
    def start(self):
        with self._lock:self._recording=True; self._raw=[]; self._message="recording"
        return self.status()
    def record(self,x,y,z,yaw_deg):
        with self._lock:
            if not self._recording:return
            if self._raw and math.hypot(x-self._raw[-1][0],y-self._raw[-1][1])<self.sample_distance_m:return
            self._raw.append((float(x),float(y),float(z),float(yaw_deg)))
    def stop(self):
        with self._lock:
            if not self._recording:return self.status()
            self._recording=False; raw=list(self._raw)
        if len(raw)<2:
            with self._lock:self._message="too_few_points"
            return self.status()
        self.raw_path.parent.mkdir(parents=True,exist_ok=True)
        with self.raw_path.open("w",newline="",encoding="utf-8") as f:
            w=csv.writer(f); w.writerow(["x","y","z","yaw_deg"]); w.writerows(raw)
        xyz=np.asarray([[r[0],r[1],r[2]] for r in raw],dtype=float)
        points=xyz_to_points(smooth_resample(xyz,self.spline_spacing_m)); self.reference_path.replace(points)
        with self._lock:self._message=f"saved_{len(points)}_points"
        return self.status()
    def toggle(self):
        with self._lock:active=self._recording
        return self.stop() if active else self.start()
    def status(self):
        with self._lock:return {"recording":self._recording,"raw_points":len(self._raw),"raw_path":str(self.raw_path),"output_route":str(self.reference_path.path),"message":self._message}

class ReferencePathVisualizer:
    def __init__(self,reference_path,refresh_seconds=0.8):self.reference_path=reference_path; self.refresh_seconds=float(refresh_seconds); self._last=0.0
    def maybe_draw(self,world,now):
        if now-self._last<self.refresh_seconds:return
        self._last=now; pts,_=self.reference_path.snapshot()
        if len(pts)<2:return
        import carla
        for a,b in zip(pts[:-1],pts[1:]):
            world.debug.draw_line(carla.Location(a.x,a.y,a.z+0.12),carla.Location(b.x,b.y,b.z+0.12),thickness=0.08,color=carla.Color(0,220,255),life_time=max(1.2,self.refresh_seconds*2),persistent_lines=False)

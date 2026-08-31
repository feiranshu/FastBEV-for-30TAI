#!/usr/bin/env python3
import json
from pathlib import Path
import carla
def _transform(s):
    l=s.get("location",s); r=s.get("rotation",s)
    return carla.Transform(carla.Location(x=float(l["x"]),y=float(l["y"]),z=float(l.get("z",0.3))),carla.Rotation(pitch=float(r.get("pitch",0)),yaw=float(r.get("yaw",0)),roll=float(r.get("roll",0))))
def spawn_fixed_scenario(path,simulation):
    if not path:return {"loaded":False,"actors":0,"reason":"disabled"}
    doc=json.loads(Path(path).read_text(encoding="utf-8")); actual=simulation.world.get_map().name.rsplit("/",1)[-1]; expected=doc.get("town")
    if expected and expected.rsplit("/",1)[-1]!=actual:raise RuntimeError(f"scenario town mismatch: {expected} != {actual}")
    lib=simulation.world.get_blueprint_library(); count=0
    for i,s in enumerate(doc.get("actors",[])):
        actor=simulation.world.try_spawn_actor(lib.find(s["blueprint"]),_transform(s["transform"]))
        if actor is None:raise RuntimeError(f"failed to spawn fixed actor #{i}")
        actor.set_target_velocity(carla.Vector3D()); actor.set_target_angular_velocity(carla.Vector3D())
        if s.get("freeze",True):actor.set_simulate_physics(False)
        simulation.actor_ids.append(actor.id); count+=1
    return {"loaded":True,"actors":count,"path":str(path)}

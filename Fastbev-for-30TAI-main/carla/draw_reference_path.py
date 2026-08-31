#!/usr/bin/env python3
import argparse,os,time
from pathlib import Path
import carla
from reference_path import load_route_csv
def main():
 p=argparse.ArgumentParser();p.add_argument("--carla-host",default="127.0.0.1");p.add_argument("--carla-port",type=int,default=2000);p.add_argument("--route",default=str(Path(__file__).resolve().parent/"routes"/"route.csv"));p.add_argument("--pid-file",default=str(Path(__file__).resolve().parent.parent/"run"/"reference_path_draw.pid"));p.add_argument("--refresh",type=float,default=.5);p.add_argument("--life-time",type=float,default=1.2);a=p.parse_args()
 pts=load_route_csv(a.route);pid=Path(a.pid_file);pid.parent.mkdir(parents=True,exist_ok=True);pid.write_text(str(os.getpid()));client=carla.Client(a.carla_host,a.carla_port);client.set_timeout(10);world=client.get_world()
 try:
  while True:
   for x,y in zip(pts[:-1],pts[1:]):world.debug.draw_line(carla.Location(x.x,x.y,x.z+.12),carla.Location(y.x,y.y,y.z+.12),thickness=.08,color=carla.Color(0,220,255),life_time=a.life_time,persistent_lines=False)
   time.sleep(a.refresh)
 finally:
  try:pid.unlink()
  except FileNotFoundError:pass
if __name__=="__main__":main()

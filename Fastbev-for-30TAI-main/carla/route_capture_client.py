#!/usr/bin/env python3
import argparse,json,urllib.request
p=argparse.ArgumentParser();p.add_argument("action",choices=["start","stop","toggle"]);p.add_argument("--server",default="http://127.0.0.1:8080");a=p.parse_args()
req=urllib.request.Request(a.server.rstrip("/")+"/route/record/"+a.action,method="POST")
with urllib.request.urlopen(req,timeout=5) as r:print(json.dumps(json.loads(r.read()),ensure_ascii=False,indent=2))

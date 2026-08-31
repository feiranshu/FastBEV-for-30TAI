#!/usr/bin/env python3
import argparse,os,signal,time
from pathlib import Path
def main():
 p=argparse.ArgumentParser();p.add_argument("--pid-file",default=str(Path(__file__).resolve().parent.parent/"run"/"reference_path_draw.pid"));p.add_argument("--wait",type=float,default=1.4);a=p.parse_args();f=Path(a.pid_file)
 if not f.exists():print("no draw process");return
 pid=int(f.read_text().strip())
 try:os.kill(pid,signal.SIGTERM)
 except ProcessLookupError:pass
 try:f.unlink()
 except FileNotFoundError:pass
 time.sleep(a.wait);print("reference-path lines expired")
if __name__=="__main__":main()

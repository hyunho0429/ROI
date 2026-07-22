#!/usr/bin/env python3
import math
import time
import socket
import ctypes
import csv
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.append(str(ROOT))

from lib.define.EgoVehicleStatus import EgoVehicleStatus

UBUNTU_IP="192.168.0.200"
EGO_STATUS_PORT=1911

PATH_FILE=ROOT/"processed_global_path.txt"
LOG_FILE=ROOT/f"cte_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"

SEARCH_BACKWARD=8
SEARCH_FORWARD=80
MAX_INDEX_ADVANCE_PER_CYCLE=15
LOG_PERIOD=0.1

def clamp(v,a,b): return max(a,min(b,v))
def dist(x1,y1,x2,y2): return math.hypot(x2-x1,y2-y1)

def norm(a):
    while a>math.pi:a-=2*math.pi
    while a<-math.pi:a+=2*math.pi
    return a

def load_path():
    p=[]
    with open(PATH_FILE) as f:
        for line in f:
            s=line.split()
            if len(s)>=2:
                z=float(s[2]) if len(s)>=3 else 0.0
                p.append((float(s[0]),float(s[1]),z))
    return p

def heading(path,i,step=3):
    i0=max(0,min(len(path)-1,i))
    i1=max(0,min(len(path)-1,i+step))
    if i0==i1:i0=max(0,i0-1)
    x0,y0,_=path[i0]
    x1,y1,_=path[i1]
    return math.atan2(y1-y0,x1-x0)

def nearest(path,x,y,yaw_deg,prev):
    s=max(0,prev-SEARCH_BACKWARD)
    e=min(len(path),prev+SEARCH_FORWARD+1)
    yaw=math.radians(yaw_deg)
    bi=prev;bs=1e9
    for i in range(s,e):
        px,py,_=path[i]
        d=dist(x,y,px,py)
        he=abs(norm(heading(path,i)-yaw))
        sc=d+8*he+0.15*max(0,prev-i)
        if sc<bs:
            bs=sc;bi=i
    bi=min(bi,prev+MAX_INDEX_ADVANCE_PER_CYCLE)
    return bi

def signed_cte(path,idx,x,y):
    c=[]
    if idx>0:c.append((idx-1,idx))
    if idx<len(path)-1:c.append((idx,idx+1))
    best=None
    for a,b in c:
        x1,y1,_=path[a]
        x2,y2,_=path[b]
        vx=x2-x1; vy=y2-y1
        l2=vx*vx+vy*vy
        if l2<1e-9: continue
        t=((x-x1)*vx+(y-y1)*vy)/l2
        t=max(0,min(1,t))
        px=x1+t*vx; py=y1+t*vy
        d=math.hypot(x-px,y-py)
        cross=vx*(y-py)-vy*(x-px)
        sd=d if cross>=0 else -d
        if best is None or abs(sd)<abs(best):
            best=sd
    return best if best is not None else 0.0

def main():
    path=load_path()
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind((UBUNTU_IP,EGO_STATUS_PORT))
    size=ctypes.sizeof(EgoVehicleStatus)
    prev=0
    last=0
    with open(LOG_FILE,"w",newline="") as f:
        w=csv.writer(f)
        w.writerow(["time","x","y","yaw","speed","signed_cte"])
        print("logging ->",LOG_FILE)
        while True:
            data,_=sock.recvfrom(4096)
            if len(data)<size: continue
            st=EgoVehicleStatus.from_buffer_copy(data[:size])
            now=time.time()
            if now-last<LOG_PERIOD:
                continue
            last=now
            x=float(st.pos_x)
            y=float(st.pos_y)
            yaw=float(st.yaw)
            speed=float(abs(st.signed_vel))
            prev=nearest(path,x,y,yaw,prev)
            cte=signed_cte(path,prev,x,y)
            w.writerow([now,x,y,yaw,speed,cte])
            f.flush()

if __name__=="__main__":
    main()

# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import pickle, sys
import numpy as np
c = pickle.load(open(sys.argv[1], "rb"))
names, V, F = c["names"], c["verts"], c["faces"]
idx = {n: i for i, n in enumerate(names)}
rng = np.random.default_rng(1)
def samples(i, dens=300.0):
    v, f = V[i], F[i]
    a, b, cc = v[f[:,0]], v[f[:,1]], v[f[:,2]]
    A = 0.5*np.linalg.norm(np.cross(b-a, cc-a), axis=1)
    n = rng.poisson(np.minimum(A*dens, 4000))
    k = np.repeat(np.arange(len(f)), n)
    u, w = rng.random(len(k)), rng.random(len(k))
    fl = u+w>1; u[fl], w[fl] = 1-u[fl], 1-w[fl]
    return a[k]+(b[k]-a[k])*u[:,None]+(cc[k]-a[k])*w[:,None]
M = [("Zvezda_SM",0,(0,0),2.25,.3,"ru"),("Zarya_FGB",0,(0,0),2.15,.3,"ru"),("Node1",0,(0,0),2.3,.25,"us"),("USLab",0,(0,0),2.3,.25,"us"),
("Node2",0,(0,0),2.3,.25,"us"),("PMA1",0,(0,0),1.5,.12,"pma"),("Node3",1,(31.3,0),2.3,.25,"us"),("Cupola",2,(31.21,-6.69),1.5,.1,"us"),
("Columbus",1,(46.72,0),2.3,.25,"us"),("JEM_PM",1,(46.72,0),2.3,.25,"us"),("JEM_PS",2,(46.86,-9.95),2.5,.2,"us"),
("Airlock",1,(31.1,0),1.8,.15,"us"),("Bishop_Airlock",1,(31.29,0),1.4,.1,"us"),("BEAM",1,(27.33,0),1.8,.12,"beam"),
("MLM",2,(15.2,0),2.2,.3,"ru"),("MRM1",2,(24.6,0),1.2,.2,"ru"),("MRM2",2,(12.05,0),1.4,.12,"ru"),("Russian_RSNode_DockingModule",2,(12.05,0),1.7,.1,"ru"),("PMM",1,(36.55,0),2.3,.2,"us")]
for nm,ax,c0,cap,step,paint in M:
    p = samples(idx[nm])
    ns=[i for i in range(3) if i!=ax]
    centre=np.array(c0,float)
    near=None
    for _ in range(3):
        rr=np.hypot(*(p[:,ns]-centre).T); near=p[rr<cap]
        centre=np.array([0.5*(np.percentile(near[:,ns[0]],1)+np.percentile(near[:,ns[0]],99)),0.5*(np.percentile(near[:,ns[1]],1)+np.percentile(near[:,ns[1]],99))])
    rr=np.hypot(*(near[:,ns]-centre).T); t=near[:,ax]
    rows=[]
    for a in np.arange(t.min(),t.max(),step):
        m=(t>=a)&(t<a+step)
        if m.sum()<4: continue
        rows.append((a+step/2,float(np.percentile(rr[m],90))))
    r=np.array(rows)
    if len(r)>4:
        r[1:-1,1]=np.median(np.c_[r[:-2,1],r[1:-1,1],r[2:,1]],axis=1)
    keep=[0,len(r)-1]
    def rec(a,b):
        if b<=a+1:return
        tt=(r[a+1:b,0]-r[a,0])/(r[b,0]-r[a,0]); e=np.abs(r[a+1:b,1]-(r[a,1]+tt*(r[b,1]-r[a,1])))
        k=int(e.argmax())
        if e[k]>0.08: keep.append(a+1+k);rec(a,a+1+k);rec(a+1+k,b)
    rec(0,len(r)-1); keep=sorted(set(keep))
    print(f'{nm} {"xyz"[ax]} c=[{centre[0]:.2f},{centre[1]:.2f}] span {t.min():.2f}..{t.max():.2f} {paint}')
    print("  "+", ".join(f"[{r[k,0]:.2f}, {r[k,1]:.2f}]" for k in keep))

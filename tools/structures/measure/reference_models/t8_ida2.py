# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import sys
import pickle, numpy as np
c = pickle.load(open(sys.argv[1], "rb")); names,V,F,ch=c["names"],c["verts"],c["faces"],c["children"]; idx={n:i for i,n in enumerate(names)}
p=np.array([51.87,0.02,0.72]); x=np.array([1.,0,0]); up=np.array([0,-0.8660254,-0.5]); y=np.cross(up,x)
print("frame y",y)
def tolocal(v):
    w=v-p; return np.c_[w@x,w@y,w@up]
# body profile
v=tolocal(V[idx["IDA2"]]); f=F[idx["IDA2"]]
a,b,cc=v[f[:,0]],v[f[:,1]],v[f[:,2]]
A=0.5*np.linalg.norm(np.cross(b-a,cc-a),axis=1); rng=np.random.default_rng(0)
n=rng.poisson(np.minimum(A*4000,3000)); k=np.repeat(np.arange(len(f)),n); u,w=rng.random(len(k)),rng.random(len(k)); fl=u+w>1;u[fl],w[fl]=1-u[fl],1-w[fl]
s=a[k]+(b[k]-a[k])*u[:,None]+(cc[k]-a[k])*w[:,None]; r=np.hypot(s[:,1],s[:,2])
print("IDA body slices ax: r p3 p50 p97")
for t in np.arange(-0.66,0.02,0.04):
    m=(s[:,0]>=t)&(s[:,0]<t+0.04)
    if m.sum()>10: print(f"  {t:+.2f} {np.percentile(r[m],3):.3f} {np.median(r[m]):.3f} {np.percentile(r[m],97):.3f}")
# components
def comps(fa,nv):
    p_=list(range(nv))
    def fd(a):
        while p_[a]!=a:
            p_[a]=p_[p_[a]];a=p_[a]
        return a
    for t in fa:
        ra,rb,rc=fd(t[0]),fd(t[1]),fd(t[2]);p_[rb]=ra;p_[rc]=ra
    return np.array([fd(i) for i in range(nv)])
for nm in ["IDA2_Details_DockingTargets","IDA2_Details_Handrails","IDA2_Details_Misc","IDA2_Details_Connectors"]:
    i=idx[nm];v=tolocal(V[i]);lab=comps(F[i],len(v))
    print("==",nm,len(set(lab)))
    rows=[]
    for l in set(lab):
        q=v[lab==l]
        if len(q)<8:continue
        lo,hi=q.min(0),q.max(0);e=hi-lo
        if e.max()<0.08: continue
        cen=(lo+hi)/2;ph=np.degrees(np.arctan2(cen[2],cen[1]))%360;rr=np.hypot(q[:,1],q[:,2])
        rows.append((round(ph),round(float(cen[0]),2),round(float(rr.min()),2),round(float(rr.max()),2),e.round(2).tolist()))
    rows.sort()
    for r_ in rows[:40]:print("  ",r_)

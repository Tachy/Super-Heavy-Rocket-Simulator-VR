# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import sys
import pickle, numpy as np
c = pickle.load(open(sys.argv[1], "rb")); names,V,F=c["names"],c["verts"],c["faces"]; idx={n:i for i,n in enumerate(names)}
def corners(p):
    m=p.mean(0); u,s,vt=np.linalg.svd(p-m,full_matrices=False)
    a=(p-m)@vt.T; lo,hi=a.min(0),a.max(0)
    cs=[m+vt[0]*x+vt[1]*y for x,y in ((lo[0],lo[1]),(hi[0],lo[1]),(hi[0],hi[1]),(lo[0],hi[1]))]
    return np.array(cs).round(2).tolist(), (hi-lo).round(2).tolist()
for n in ["IROSA_Deployed_P44A","IROSA_Deployed_P62B","IROSA_Deployed_P64B","IROSA_Deployed_S41A","IROSA_Deployed_S43A","IROSA_Deployed_S61B","P1_Radiator","S1_Radiator"]:
    print(n,*corners(V[idx[n]]))
for n,th,ax in [("Zvezda_SM",2.4,1),("Zarya_FGB",2.4,1),("MLM",2.4,1)]:
    v=V[idx[n]]
    for s in (-1,1):
        m=(s*v[:,ax]>th); 
        print(n,s,len(v[m]),*corners(v[m]))
# P4 array parts: blanket z split
v=V[idx["P4_Array_2A"]]; 
print("blanket y core", np.percentile(v[np.abs(v[:,2]+5.51)>2,1],[1,50,99]).round(2), "mast rows y", np.percentile(v[np.abs(v[:,2]+5.51)<1.5,1],[1,50,99]).round(2))
print("blanket z edges", np.round(np.percentile(v[np.abs(v[:,2]+5.51)>1.5,2],[0,1,49,51,99,100]),2))
xs=v[(np.abs(v[:,2]+5.51)>1.5),0]; print("blanket x hist", np.histogram(xs,bins=10)[0], xs.min().round(2), xs.max().round(2))

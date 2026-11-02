# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import sys
import pickle, numpy as np
c = pickle.load(open(sys.argv[1], "rb")); names,V,F=c["names"],c["verts"],c["faces"]; idx={n:i for i,n in enumerate(names)}
def show(n,ax,bins=30):
    v=V[idx[n]]; h,e=np.histogram(v[:,ax],bins=bins)
    print(n,"xyz"[ax],[ (round(float(e[i]),2),int(h[i])) for i in range(bins) if h[i]>0])
for n in ["P4_Array_2A","IROSA_Deployed_P44A","P1_Radiator"]:
    for a in range(3): show(n,a,24)
# zvezda wing, zarya wing, MLM excess, truss sections
v=V[idx["Zvezda_SM"]]; m=np.abs(v[:,1])>2.3; print("zv wing", v[m].min(0).round(2), v[m].max(0).round(2)); s=v[m]
print(" y hist",np.histogram(s[:,1],bins=12)[0], np.histogram(s[:,0],bins=8)); print(" z",np.histogram(s[:,2],bins=8))
v=V[idx["Zarya_FGB"]]; m=np.abs(v[:,1])>2.3; print("za wing", v[m].min(0).round(2), v[m].max(0).round(2))
v=V[idx["MLM"]]; m=np.abs(v[:,1])>2.4; print("mlm far", v[m].min(0).round(2), v[m].max(0).round(2), len(v[m]))
m=(np.abs(v[:,1])<2.4)&((v[:,0]<12.4)|(v[:,0]>17)); print("mlm x-out", v[m].min(0).round(2), v[m].max(0).round(2), len(v[m]))
for n in ["Truss_S0","Truss_P1","Truss_S1","Truss_P3","Truss_P4","Truss_P5","Truss_P6","Z1"]:
    v=V[idx[n]]
    print(n,"x",np.percentile(v[:,0],[0,5,50,95,100]).round(2),"z",np.percentile(v[:,2],[0,5,50,95,100]).round(2))
for j in ["PORT_ALPHA_ROT","STBD_ALPHA_ROT","PORT_TRRJ_GAMMA_ROT"]:
    print(j,c["origin"][idx[j]].round(2))
for n in ["SHR","MBS","MT","JEM_EF","AMS","ELC_1","ELC_2","ELC_3","ELC_4","ESP1","ESP2","ESP3","Columbus_BARTOLOMEO"]:
    v=V[idx[n]]; print(n,v.min(0).round(2),v.max(0).round(2))

"""Redraws docs/images/results.png from data/nexarm_arrivals.csv (ISO 9283 accuracy AP and repeatability RP).
Run from anywhere:  python tools/make_results_figure.py   (needs matplotlib)"""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import csv,collections,math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":9})
r=list(csv.DictReader(open(os.path.join(ROOT, 'data', 'nexarm_arrivals.csv'))))
fig,axs=plt.subplots(1,2,figsize=(7.2,3.6))
C={"A":"#1f6fb2","B":"#c0392b"}
for ax,st in zip(axs,"AB"):
    p=[(float(x['ex']),float(x['ey'])) for x in r if x['station']==st]
    n=len(p); mx=sum(a for a,b in p)/n; my=sum(b for a,b in p)/n
    l=[math.hypot(a-mx,b-my) for a,b in p]; mxl=max(l)
    lb=sum(l)/n; s=(sum((x-lb)**2 for x in l)/(n-1))**.5; RP=lb+3*s; AP=math.hypot(mx,my)
    cnt=collections.Counter(p)
    ax.add_patch(Circle((mx,my),mxl,fc=C[st],alpha=0.07,ec=C[st],ls="--",lw=1))
    for (a,b),c in cnt.items():
        ax.scatter(a,b,s=40+28*c,color=C[st],alpha=0.85,edgecolor="white",lw=1.2,zorder=3)
        ax.text(a,b+0.42,f"×{c}",ha="center",fontsize=8,color="#33414e")
    ax.scatter(0,0,marker="+",s=220,color="black",lw=2,zorder=4)
    ax.text(0,-0.4,"taught point",fontsize=8,ha="center",va="top")
    ax.scatter(mx,my,marker="x",s=70,color="black",lw=1.6,zorder=5)
    ax.annotate("",xy=(mx,my),xytext=(0,0),arrowprops=dict(arrowstyle="-|>",color="#555",lw=1.1,ls=":"),zorder=2)
    ax.text(mx,my-0.55-mxl*0.0,"mean",fontsize=8,ha="center")
    ax.set_title(f"Station {st}  (x = {201 if st=='A' else 301} mm)",fontsize=10,weight="bold")
    ax.set_xlim(-5.5,1.5); ax.set_ylim(-2.5,2.5); ax.set_aspect("equal")
    ax.grid(color="#e3e6ea",lw=0.7); ax.set_axisbelow(True)
    for sp in ["top","right"]: ax.spines[sp].set_visible(False)
    ax.set_xlabel("X error (mm)  ← towards base"); 
    if st=="A": ax.set_ylabel("Y error (mm)")
    ax.text(-5.35,-2.35,f"n = {n}\nAP = {AP:.2f} mm\nRP = {RP:.2f} mm\nmax spread = {mxl:.2f} mm",fontsize=8,va="bottom",
            bbox=dict(fc="white",ec="#c8ced5",boxstyle="round,pad=0.35"))
fig.tight_layout()
fig.savefig(os.path.join(ROOT, 'docs', 'images', 'results.png'),dpi=220,facecolor="white")

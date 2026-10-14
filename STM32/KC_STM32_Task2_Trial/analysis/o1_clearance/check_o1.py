"""O1-only sampled clearance study. No hardware access or firmware changes.
Run: python3 check_o1.py. All lengths cm, heading left positive.
Endpoint measurements do not bound actual intermediate turn trajectories.
"""
import math,json
from pathlib import Path
R=Path(__file__).resolve().parent
O1=[(-5,0),(5,0),(5,10),(-5,10)]
ROUTES={'LEFT':[('L',45),('F',10),('R',45),('F',9),('R',45),('F',10),('L',45)],'RIGHT':[('R',45),('F',7),('L',45),('F',20),('L',45),('F',26),('B',45)]}
def rotate(s,f,h):
 a=math.radians(h);return s*math.cos(a)-f*math.sin(a),f*math.cos(a)+s*math.sin(a)
def box(x,y,h,margin=0):
 return [(x+dx,y+dy) for dx,dy in (rotate(s,f,h) for s,f in [(-9.4-margin,-11.5-margin),(9.4+margin,-11.5-margin),(9.4+margin,11.5+margin),(-9.4-margin,11.5+margin)])]
def edges(p):return list(zip(p,p[1:]+p[:1]))
def pd(p,a,b):
 dx,dy=b[0]-a[0],b[1]-a[1];u=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
 return math.hypot(p[0]-a[0]-u*dx,p[1]-a[1]-u*dy)
def gap(p,q):
 overlaps=[]
 for a,b in edges(p)+edges(q):
  nx,ny=-(b[1]-a[1]),b[0]-a[0];norm=math.hypot(nx,ny);nx/=norm;ny/=norm
  pp=[x*nx+y*ny for x,y in p];qq=[x*nx+y*ny for x,y in q]
  overlaps.append(min(max(pp),max(qq))-max(min(pp),min(qq)))
 if min(overlaps)>=0:return -min(overlaps)
 return min([pd(v,a,b) for v in p for a,b in edges(q)]+[pd(v,a,b) for v in q for a,b in edges(p)])
def path(side,g=32,x=0,heading=0,model='hermite',overshoot=0):
 y=-g-11.5
 yield 0,x,y,heading
 for k,(cmd,n) in enumerate(ROUTES[side]):
  if k==3:n=math.floor(n+g-32+.5)
  if cmd=='F':dx,dy=rotate(0,n+overshoot,heading);dh=0
  else:
   s,f={'L':(-13,13.25),'R':(15,18),'B':(-1.5,-12.5)}[cmd]
   dx,dy=rotate(s,f,heading);dh=45 if cmd=='L' else -45
  steps=max(1,math.ceil(math.hypot(dx,dy)/.2),math.ceil(abs(dh)/.5))
  for j in range(1,steps+1):
   u=j/steps
   if cmd=='F' or model=='linear':px,py=u*dx,u*dy
   else:
    # Cubic endpoint fit, tangents aligned with signed vehicle heading.
    # This is one plausible trajectory, NOT a fitted steering/wheelbase model.
    length=math.hypot(dx,dy)*math.radians(45)/(2*math.sin(math.radians(22.5)))
    sign=-1 if cmd=='B' else 1
    ax,ay=rotate(0,sign*length,heading);bx,by=rotate(0,sign*length,heading+dh)
    a=u**3-2*u*u+u;b=-2*u**3+3*u*u;c=u**3-u*u
    px,py=a*ax+b*dx+c*bx,a*ay+b*dy+c*by
   yield k+1,x+px,y+py,heading+u*dh
  x+=dx;y+=dy;heading+=dh

def study(side,g=32,x=0,heading=0,model='hermite',overshoot=0,margin=0):
 best1=(1e9,None);best2=(1e9,None);end=None
 for k,px,py,h in path(side,g,x,heading,model,overshoot):
  p=box(px,py,h,margin);d=gap(p,O1);d2=70-max(v[1] for v in p)
  pose={'segment':k,'x':round(px,3),'y':round(py,3),'heading':round(h,3)}
  if d<best1[0]:best1=(d,pose)
  if d2<best2[0]:best2=(d2,pose)
  end=pose
 return {'side':side,'accepted_gap':g,'start_x':x,'start_heading':heading,'model':model,'straight_extra':overshoot,'body_margin':margin,'o1_min':round(best1[0],3),'o1_at':best1[1],'o2_plane_min':round(best2[0],3),'o2_at':best2[1],'end':end}
nominal=[study(s,model=m) for s in ROUTES for m in ['linear','hermite']]
# Scenario tests, not probability estimates or certified uncertainty bounds.
scenarios=[study(s,g,x,h,m,o) for s in ROUTES for g in [30,30.5,32,33.5,34] for x in [-2,0,2] for h in [-2,0,2] for m in ['linear','hermite'] for o in [0,1]]
summary={s:{'o1_worst':min((v for v in scenarios if v['side']==s),key=lambda v:v['o1_min']),'o2_worst':min((v for v in scenarios if v['side']==s),key=lambda v:v['o2_plane_min'])} for s in ROUTES}
(R/'results.json').write_text(json.dumps({'nominal':nominal,'scenario_count':len(scenarios),'worst_scenarios':summary},indent=2)+'\n')
print(json.dumps({'nominal':nominal,'worst_scenarios':summary},indent=2))

/* Offline-calibrated parking entry trial. Sampled geometry is NOT a physical
 * clearance proof: turn endpoints alone do not determine the true sweep.
 * Park60x50; start fully inside, centreline approximately X0. Rear gap0..27
 * yields conservative wall envelope frontY=-11.5, mouthY<=38.5; intersect
 * mouth with O1-front minus the rules minimum60cm plus3cm US uncertainty.
 * Two illustrative turn fits + inflated body5cm + diagonal overrun0..2cm.
 * Static scratch avoids exceeding the motor task's2KB stack. Single owner.
 */
#ifndef KC_ENTRY_GEOMETRY_H
#define KC_ENTRY_GEOMETRY_H
#include <math.h>
#include <stdint.h>
#define KCE_MAX_POSES 320  /* 90 deg templates: 4 x (1+30+16+30) = 308 */
#define KCE_MARGIN 5.0f
/* Longest entry/trim diagonal (cm). Was 70: run 700394561 (O2 LEFT, wide O2,
 * 60 cm gaps) came home 67.6 out and needed 74. Footprint checks still apply. */
#define KCE_MAX_DIAGONAL 85.0f
#define KCE_PI 0.017453292519943295f
typedef struct {float x,y,c,s;} KcePose;
static KcePose kce_poses[KCE_MAX_POSES];
static unsigned kce_count;
static unsigned kce_ends[4];
static float kce_target;
static void kce_add(float x,float y,float h){
 if(kce_count<KCE_MAX_POSES){KcePose *p=&kce_poses[kce_count++];p->x=x;p->y=y;p->c=cosf(h*KCE_PI);p->s=sinf(h*KCE_PI);}
}
static int kce_overlap(const KcePose *p,float y,float xmin,float ymin,float xmax,float ymax){
 float dx=(xmin+xmax)*.5f-p->x,dy=(ymin+ymax)*.5f-(p->y+y);
 float rx=(xmax-xmin)*.5f,ry=(ymax-ymin)*.5f;
 float w=9.4f+KCE_MARGIN,l=11.5f+KCE_MARGIN,c=fabsf(p->c),s=fabsf(p->s);
 if(fabsf(dx)>rx+w*c+l*s || fabsf(dy)>ry+w*s+l*c)return 0;
 if(fabsf(dx*p->c+dy*p->s)>w+rx*c+ry*s)return 0;
 if(fabsf(-dx*p->s+dy*p->c)>l+rx*s+ry*c)return 0;
 return 1;
}
/* turn 45: calibrated 45 deg turns. turn 90: the legacy 90 deg turn fits
 * (LEFT 15.0 fwd / 31.0 side, RIGHT 20.5 / 38.0) for lanes too wide for a
 * 45 deg diagonal; leg 1 is then a straight across, not a diagonal. */
static int kce_template_turn(float x,float target,float diagonal,float turn){
 kce_count=0;kce_target=target;
 if(!isfinite(x)||!isfinite(diagonal)||diagonal<0||diagonal>KCE_MAX_DIAGONAL)return 0;
 int wide=turn>60;
 for(unsigned model=0;model<2;model++)for(unsigned coast=0;coast<2;coast++){
  float px=x,py=0,h=180;
  kce_add(px,py,h);
  for(unsigned leg=0;leg<3;leg++){
   int left=(x>=0)?leg==2:leg==0;
   float f=leg==1?diagonal+2*coast:wide?(left?15.0f:20.5f):(left?13.25f:18.0f);
   float side=leg==1?0:wide?(left?-31.0f:38.0f):(left?-13.0f:15.0f);
   float dh=leg==1?0:(left?1.0f:-1.0f)*(wide?90.0f:45.0f);
   float r=h*KCE_PI,dx=side*cosf(r)-f*sinf(r),dy=f*cosf(r)+side*sinf(r);
   unsigned steps=leg==1?16:wide?30:20;
   float length=hypotf(dx,dy)*(wide?1.11072073f:1.02617215f); /* arc/chord */
   for(unsigned j=1;j<=steps;j++){
    float u=(float)j/steps,ox=u*dx,oy=u*dy;
    if(model && leg!=1){
     float a=u*u*u-2*u*u+u,b=-2*u*u*u+3*u*u,c=u*u*u-u*u;
     float ax=-length*sinf(r),ay=length*cosf(r);
     float bx=-length*sinf((h+dh)*KCE_PI),by=length*cosf((h+dh)*KCE_PI);
     ox=a*ax+b*dx+c*bx;oy=a*ay+b*dy+c*by;
    }
    kce_add(px+ox,py+oy,h+u*dh);
   }
   px+=dx;py+=dy;h+=dh;
  }
  kce_ends[model*2+coast]=kce_count-1;
 }
 return kce_count<KCE_MAX_POSES;
}
static int kce_template_to(float x,float target,float diagonal){return kce_template_turn(x,target,diagonal,45);}
static inline int kce_template(float x,float diagonal){return kce_template_to(x,0,diagonal);} /* tests */
static float kce_mouth(float o1_front){
 /* Rules: opening->O1 front >=60cm. Initial fresh-US anchor is allowed3cm
  * measurement uncertainty. Intersect with fully-inside50cm park bound. */
 return fminf(38.5f,o1_front-60.0f+3.0f);
}
static int kce_pose_clear(const KcePose *p,float start_y,float o1_front,float o2_front){
 float mouth=kce_mouth(o1_front);if(mouth<11.5f)return 0;
 float reach=(9.4f+KCE_MARGIN)*fabsf(p->s)+(11.5f+KCE_MARGIN)*fabsf(p->c);
 if(p->y+start_y-reach<=-11.5f || p->y+start_y+reach>=o2_front)return 0;
 /* Any part crossing the opening must already fit inside its width.
  * No lateral entry through the side wall, even if the endpoint is centred. */
 float sideways=(9.4f+KCE_MARGIN)*fabsf(p->c)+(11.5f+KCE_MARGIN)*fabsf(p->s);
 if(p->y+start_y-reach<=mouth && fabsf(p->x)+sideways>=30)return 0;
 return !kce_overlap(p,start_y,-5,o1_front,5,o1_front+10) &&
        !kce_overlap(p,start_y,30,-38.5f,40,mouth) &&
        !kce_overlap(p,start_y,-40,-38.5f,-30,mouth);
}
static int kce_clear(float start_y,float o1_front,float o2_front){
 if(!isfinite(start_y)||!isfinite(o1_front)||!isfinite(o2_front))return 0;
 for(unsigned k=0;k<4;k++){
  const KcePose *end=&kce_poses[kce_ends[k]];
  if(fabsf(end->x-kce_target)>2.5f)return 0;
  /* Aligned above O1 would still drive through O1 on straight parking entry. */
  if(kce_target==0 && end->y+start_y+16.5f>=o1_front)return 0;
 }
 for(unsigned j=0;j<kce_count;j++)if(!kce_pose_clear(&kce_poses[j],start_y,o1_front,o2_front))return 0;
 return 1;
}
static int kce_reposition_clear(float x,float from,float to,float o1,float o2){
 KcePose p={x,0,-1,0};
 unsigned n=(unsigned)ceilf(fabsf(to-from)/2)+1;
 for(unsigned k=0;k<=n;k++)if(!kce_pose_clear(&p,from+(to-from)*(float)k/n,o1,o2))return 0;
 return 1;
}
/* Pick the middle of the widest safe interval, not its barely-clear boundary.
 * Candidate range stays below the return-lane start already traversed. */
static int kce_choose_turn(float x,float current_y,float known_y,float o1,float o2,float *diagonal,float *chosen,float turn){
 float d;
 if(turn>60){
  /* Two 90 deg turns already move 53 (x>=0: RIGHT 38 + LEFT 15 fwd) or
   * 51.5 (x<0: LEFT 31 + RIGHT 20.5) toward the centre; drive the rest. */
  d=roundf(fabsf(x)-(x>=0?53.0f:51.5f));
  if(d<0 && d>-2.5f)d=0;
  if(d<0)return 0;
 }else{
  float shift=x>=0?15+(13.25f-13)/1.414213562f:13+(18.0f-15)/1.414213562f;
  d=roundf((fabsf(x)-shift)*1.414213562f);
  if(d==1)d=2;
 }
 if(!kce_template_turn(x,0,d,turn))return 0;
 /* Entry location is constrained by O1/park geometry, not proximity to
  * the current pose. A distant O2 leaves a long safe straight before O1.
  * Check that whole connecting lane using kce_reposition_clear below. */
 float low=fmaxf(0,o1-100),high=fminf(o1+100,fminf(known_y-2,current_y+60));
 unsigned best=0,run=0;float begin=0,best_begin=0;
 for(float y=ceilf(low);y<=floorf(high);y+=1){
  if(kce_clear(y,o1,o2) && kce_reposition_clear(x,current_y,y,o1,o2)){
   if(!run)begin=y;
   run++;
   if(run>best){best=run;best_begin=begin;}
  }else run=0;
 }
 if(best<5)return 0; /* Insufficient interval for encoder/reposition residual. */
 *diagonal=d;*chosen=best_begin+(float)(best-1)*.5f;return kce_template_turn(x,0,d,turn);
}
static int kce_choose(float x,float current_y,float known_y,float o1,float o2,float *diagonal,float *chosen){
 return kce_choose_turn(x,current_y,known_y,o1,o2,diagonal,chosen,45);
}
/* Bring a wide lane inward while still between the obstacles. Different
 * targets account for measured left/right turn translations. */
static float kce_trim_target(float x){return x>=0?30.0f:-26.0f;}
static int kce_trim_choose_turn(float x,float target,float current_y,float o1,float o2,float *diagonal,float *chosen,float turn){
 float d;
 if(turn>60){
  d=roundf(fabsf(x)-fabsf(target)-(x>=0?53.0f:51.5f));
  if(d<0 && d>-2.5f)d=0;
  if(d<0 || d>KCE_MAX_DIAGONAL)return 0;
 }else{
  float shift=x>=0?15+(13.25f-13)/1.414213562f:13+(18.0f-15)/1.414213562f;
  d=roundf((fabsf(x)-fabsf(target)-shift)*1.414213562f);
  if(d<2 || d>KCE_MAX_DIAGONAL)return 0;
 }
 unsigned best=0,run=0;float begin=0,best_begin=0;
 for(float y=ceilf(current_y-30);y<=floorf(current_y);y+=1){
  if(!kce_template_turn(x,target,d,turn))return 0;
  float end_y=y+kce_poses[kce_ends[0]].y;
  float end_x=kce_poses[kce_ends[0]].x;
  int clear=kce_clear(y,o1,o2) && kce_reposition_clear(x,current_y,y,o1,o2);
  float final_d,final_y;
  if(clear)clear=kce_choose(end_x,end_y,end_y,o1,o2,&final_d,&final_y);
  if(clear){if(!run)begin=y;run++;if(run>best){best=run;best_begin=begin;}}
  else run=0;
 }
 if(best<5)return 0;
 *diagonal=d;*chosen=best_begin+(best-1)*.5f;
 return kce_template_turn(x,target,d,turn);
}
static int kce_trim_choose_to(float x,float target,float current_y,float o1,float o2,float *diagonal,float *chosen){
 return kce_trim_choose_turn(x,target,current_y,o1,o2,diagonal,chosen,45);
}
static int kce_trim_choose(float x,float current_y,float o1,float o2,float *diagonal,float *chosen){
 return kce_trim_choose_to(x,kce_trim_target(x),current_y,o1,o2,diagonal,chosen);
}
/* Smaller inward correction for a short O1-O2 gap: only far enough in for the
 * side IR (accepts <=45 cm) to see O1 on the way home. */
static float kce_partial_target(float x){return x>=0?40.0f:-40.0f;}
#endif

/* Offline-calibrated parking entry trial. Sampled geometry is NOT a physical
 * clearance proof: turn endpoints alone do not determine the true sweep.
 * Park60x50; start fully inside, centreline approximately X0. Rear gap0..27
 * yields conservative wall envelope frontY=-11.5, mouthY<=38.5.
 * Two illustrative turn fits + inflated body5cm + diagonal overrun0..2cm.
 * Static scratch avoids exceeding the motor task's2KB stack. Single owner.
 */
#ifndef KC_ENTRY_GEOMETRY_H
#define KC_ENTRY_GEOMETRY_H
#include <math.h>
#include <stdint.h>
#define KCE_MAX_POSES 256
#define KCE_MARGIN 5.0f
#define KCE_PI 0.017453292519943295f
typedef struct {float x,y,c,s;} KcePose;
static KcePose kce_poses[KCE_MAX_POSES];
static unsigned kce_count;
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
static int kce_template(float x,float diagonal){
 kce_count=0;
 if(!isfinite(x)||!isfinite(diagonal)||diagonal<0||diagonal>70)return 0;
 for(unsigned model=0;model<2;model++)for(unsigned coast=0;coast<2;coast++){
  float px=x,py=0,h=180;
  kce_add(px,py,h);
  for(unsigned leg=0;leg<3;leg++){
   int left=(x>=0)?leg==2:leg==0;
   float f=leg==1?diagonal+2*coast:(left?13.25f:18.0f);
   float side=leg==1?0:(left?-13.0f:15.0f),dh=leg==1?0:(left?45.0f:-45.0f);
   float r=h*KCE_PI,dx=side*cosf(r)-f*sinf(r),dy=f*cosf(r)+side*sinf(r);
   unsigned steps=leg==1?16:20;
   float length=hypotf(dx,dy)*1.02617215f;
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
 }
 return kce_count<KCE_MAX_POSES;
}
static int kce_pose_clear(const KcePose *p,float start_y,float o1_front,float o2_front){
 float reach=(9.4f+KCE_MARGIN)*fabsf(p->s)+(11.5f+KCE_MARGIN)*fabsf(p->c);
 if(p->y+start_y-reach<=-11.5f || p->y+start_y+reach>=o2_front)return 0;
 return !kce_overlap(p,start_y,-5,o1_front,5,o1_front+10) &&
        !kce_overlap(p,start_y,30,-38.5f,40,38.5f) &&
        !kce_overlap(p,start_y,-40,-38.5f,-30,38.5f);
}
static int kce_clear(float start_y,float o1_front,float o2_front){
 if(!isfinite(start_y)||!isfinite(o1_front)||!isfinite(o2_front))return 0;
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
static int kce_choose(float x,float current_y,float known_y,float o1,float o2,float *diagonal,float *chosen){
 float shift=x>=0?15+(13.25f-13)/1.414213562f:13+(18.0f-15)/1.414213562f;
 float d=roundf((fabsf(x)-shift)*1.414213562f);
 if(d==1)d=2;
 if(!kce_template(x,d))return 0;
 float low=fmaxf(0,current_y-40),high=fminf(known_y-2,current_y+60);
 unsigned best=0,run=0;float begin=0,best_begin=0;
 for(float y=ceilf(low);y<=floorf(high);y+=1){
  if(kce_clear(y,o1,o2) && kce_reposition_clear(x,current_y,y,o1,o2)){
   if(!run)begin=y;
   run++;
   if(run>best){best=run;best_begin=begin;}
  }else run=0;
 }
 if(best<5)return 0; /* Insufficient interval for encoder/reposition residual. */
 *diagonal=d;*chosen=best_begin+(float)(best-1)*.5f;return 1;
}
#endif

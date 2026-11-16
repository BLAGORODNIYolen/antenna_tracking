#pragma once
#include <math.h>
inline bool isNew(uint32_t seq, uint32_t old) { return (int32_t)(seq-old)>0; }
inline float wrap360(float a) { a=fmodf(a,360); return a<0?a+360:a; }
inline bool chooseTarget(float az,float current,float low,float high,float offset,float &result) {
  const float base=az-offset;
  int first=(int)ceilf((low-base)/360), last=(int)floorf((high-base)/360);
  if(first>last) return false;
  int k=(int)roundf((current-base)/360);
  k=max(first,min(last,k)); result=base+360*k; return true;
}

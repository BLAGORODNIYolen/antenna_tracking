#pragma once
#include <ctype.h>
#include <string.h>
// Wire messages are flat ASCII objects. Validate the complete input before ArduinoJson,
// which otherwise tolerates a suffix after the first value. Reject duplicate keys.
inline bool strictFlatJson(const char* s) {
  const char* p=s;
  auto ws=[&](){ while(*p==' ' || *p=='\r' || *p=='\t') ++p; };
  auto str=[&]()->bool {
    if(*p++!='"') return false;
    while(*p && *p!='"') {
      unsigned char c=*p++;
      if(c<32 || c>126 || c=='\\') return false; // wire profile emits unescaped ASCII
    }
    if(*p!='"') return false;
    ++p; return true;
  };
  const char* names[32]; uint8_t lengths[32],count=0;
  ws(); if(*p++!='{') return false; ws();
  if(*p=='}') { ++p; ws(); return !*p; }
  for(;;) {
    const char* name=p;
    if(!str() || count==32 || p-name>255) return false;
    uint8_t len=p-name;
    for(uint8_t i=0;i<count;i++) if(lengths[i]==len && !strncmp(names[i],name,len)) return false;
    names[count]=name; lengths[count++]=len;
    ws(); if(*p++!=':') return false; ws();
    if(*p=='"') { if(!str()) return false; }
    else if(!strncmp(p,"true",4)) p+=4;
    else if(!strncmp(p,"false",5)) p+=5;
    else if(!strncmp(p,"null",4)) p+=4;
    else {
      if(*p=='-') ++p;
      if(*p=='0') ++p;
      else { if(*p<'1'||*p>'9') return false; while(isdigit(*p)) ++p; }
      if(*p=='.') { ++p; if(!isdigit(*p)) return false; while(isdigit(*p)) ++p; }
      if(*p=='e'||*p=='E') { ++p; if(*p=='+'||*p=='-') ++p; if(!isdigit(*p)) return false; while(isdigit(*p)) ++p; }
    }
    ws(); if(*p=='}') { ++p; ws(); return !*p; }
    if(*p++!=',') return false;
    ws();
  }
}

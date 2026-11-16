#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <ArduinoJson.h>
#include "config.h"
#include "types.h"
#include "protocol_uart.h"
#include <math.h>
#include "strict_json.h"
static char input[MAX_PACKET+1],output[MAX_PACKET+1];
static StaticJsonDocument<JSON_OBJECT_SIZE(32)> rxDoc;
static StaticJsonDocument<JSON_OBJECT_SIZE(16)> txDoc;
static uint16_t used=0,outLen=0,outPos=0;
static bool dropping=false;
static uint32_t outSeq=0;
static bool numeric(JsonVariant v) { return !v.is<bool>() && v.is<float>() && isfinite(v.as<float>()); }
static bool validUint32(JsonVariant v) { return !v.is<bool>() && v.is<uint32_t>(); }
static bool angle(const char* key,const char* valid,float lo,float hi) {
  if(!rxDoc.containsKey(key) || !rxDoc[valid].is<bool>()) return false;
  if(!rxDoc[valid].as<bool>()) return rxDoc[key].isNull();
  JsonVariant v=rxDoc[key];
  return numeric(v) && v.as<float>()>=lo && v.as<float>()<=hi && (hi!=360 || v.as<float>()<360);
}
static bool parse(Request &r) {
  if(!strictFlatJson(input)) return false;
  if(deserializeJson(rxDoc,input,DeserializationOption::NestingLimit(4)) || !rxDoc.is<JsonObject>()) return false;
  if(!validUint32(rxDoc["seq"]) || !rxDoc["version"].is<int>() || rxDoc["version"].is<bool>() || !rxDoc["protocol"].is<const char*>()) return false;
  const char* name=rxDoc["protocol"];
  r={}; r.seq=rxDoc["seq"];
  if(!strcmp(name,"ANTENNA_CONTROL")) {
    if(rxDoc["version"]!=1 || !rxDoc["action"].is<const char*>()) return false;
    const char* a=rxDoc["action"];
    if(!strcmp(a,"DISARM")) { r.kind=2; return true; }
    if(strcmp(a,"CALIBRATE") || !numeric(rxDoc["az_mech_deg"])) return false;
    r.kind=1; r.az=rxDoc["az_mech_deg"]; return true;
  }
  //if(strcmp(name,"DRONE_SIM") || rxDoc["version"]!=2) return false;
  if(!validUint32(rxDoc["sim_ms"]) || !validUint32(rxDoc["host_ms"]) || !rxDoc["mode"].is<const char*>()) return false;
  const char* mode=rxDoc["mode"];
  if(strcmp(mode,"RUN") && strcmp(mode,"PAUSE") && strcmp(mode,"STOP")) return false;
  const char* keys[]={"x","y","z","vx","vy","vz","speed"};
  for(uint8_t i=0;i<7;i++) if(!numeric(rxDoc[keys[i]])) return false;
  if(rxDoc["speed"].as<float>()<0 || !angle("az_deg","az_valid",0,360) || !angle("el_deg","el_valid",-90,90)) return false;
  r.azValid=rxDoc["az_valid"]; r.elValid=rxDoc["el_valid"];
  if(r.azValid) r.az=rxDoc["az_deg"];
  if(r.elValid) r.el=rxDoc["el_deg"];
  return true;
}
static const char* reason(uint8_t r) {
  switch(r) { case 1:return "CONFIG_REQUIRED"; case 2:return "UNCALIBRATED_OR_ARMED"; case 3:return "AZ_UNREACHABLE"; case 4:return "QUEUE_FULL"; case 5:return "AZ_LIMIT_STOP"; default:return "OK"; }
}
static float rounded(float v) { return roundf(v*1000)/1000; }
static void stage() {
  size_t n=measureJson(txDoc);
  if(n+1>MAX_PACKET) return;
  outLen=serializeJson(txDoc,output,sizeof(output)); output[outLen++]='\n'; outPos=0;
}
void uartTask(void*) {
  uint32_t nextState=millis();
  for(;;) {
    // Limit read work to keep control task responsive during continuous malformed input.
    for(uint8_t budget=0;budget<32 && Serial.available();budget++) {
      char c=Serial.read();
      if(dropping) { if(c=='\n') dropping=false; continue; }
      if((uint8_t)c<32 && c!='\n' && c!='\r' && c!='\t') { dropping=true; used=0; rxInvalid++; continue; }
      if(c=='\n') {
        input[used]=0;
        Request request;
        if(parse(request)) {
          if(xQueueSend(requests,&request,0)==pdPASS) rxValid++; else rxInvalid++;
        } else rxInvalid++;
        used=0;
      } else if(used>=MAX_PACKET-1) { used=0; dropping=true; rxOverflow++; rxInvalid++; }
      else input[used++]=c;
    }
    if(outPos<outLen) {
      int room=Serial.availableForWrite();
      uint16_t n=min((uint16_t)room,(uint16_t)(outLen-outPos));
      if(n) outPos+=Serial.write((const uint8_t*)output+outPos,n);
    } else {
      Ack a;
      if(xQueueReceive(acknowledgements,&a,0)==pdPASS) {
        txDoc.clear(); txDoc["protocol"]="ANTENNA_ACK"; txDoc["version"]=1; txDoc["seq"]=a.seq; txDoc["ok"]=a.ok; txDoc["reason"]=reason(a.reason); stage();
      } else if((int32_t)(millis()-nextState)>=0) {
        nextState=millis()+40;
        State s; taskENTER_CRITICAL(); s=sharedState; taskEXIT_CRITICAL();
        txDoc.clear(); txDoc["protocol"]="ANTENNA_STATE"; txDoc["version"]=2; txDoc["seq"]=outSeq++;
        txDoc["source_seq"]=nullptr; if(s.hasSource) txDoc["source_seq"]=s.seq;
        txDoc["mcu_ms"]=millis(); txDoc["cmd_az_deg"]=nullptr; txDoc["cmd_el_deg"]=nullptr;
        if(s.azKnown) { float a=rounded(s.cmdAz); txDoc["cmd_az_deg"]=a>=360?0:a; }
        if(s.elKnown) txDoc["cmd_el_deg"]=rounded(s.cmdEl);
        txDoc["act_az_deg"]=nullptr; txDoc["act_el_deg"]=nullptr;
        txDoc["act_az_valid"]=false; txDoc["act_el_valid"]=false;
        txDoc["status"]=s.stale?"STALE":(s.fault?"FAULT":"NO_FEEDBACK");
        txDoc["reason"]=reason(s.fault);
        // Optional counters only when they fit; mandatory fields never removed.
        txDoc["rx_bad"]=rxInvalid; txDoc["rx_ovf"]=rxOverflow;
        if(measureJson(txDoc)+1>MAX_PACKET) { txDoc.remove("rx_bad"); txDoc.remove("rx_ovf"); }
        stage();
      }
    }
    taskYIELD();
  }
}

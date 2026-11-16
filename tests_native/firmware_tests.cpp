#include <cassert>
#include <iostream>
#include <functional>
#include "../firmware/AntennaTracker/antenna_controller.cpp"
#include "../firmware/AntennaTracker/protocol_uart.cpp"
uint32_t clockUs=0;
SerialMock Serial;
Queue q={sizeof(Request),{}},aq={sizeof(Ack),{}};
QueueHandle_t requests=&q,acknowledgements=&aq;
State sharedState={};
uint32_t rxValid=0,rxInvalid=0,rxOverflow=0;
struct Done{};
static std::function<void()> onYield;
void yieldTest(){ clockUs+=1000; onYield(); }
static void send(uint8_t kind,uint32_t seq,float az,float el,bool av=true,bool ev=true){
 Request r={kind,seq,az,el,av,ev}; xQueueSend(requests,&r,0);
}
int main(){
 // Exercise production firmware parser, not a Python reimplementation.
 const char* good="{\"protocol\":\"DRONE_SIM\",\"version\":2,\"seq\":1,\"sim_ms\":0,\"host_ms\":0,\"mode\":\"RUN\",\"x\":0,\"y\":1,\"z\":0,\"vx\":0,\"vy\":0,\"vz\":0,\"speed\":0,\"az_deg\":0,\"el_deg\":0,\"az_valid\":true,\"el_valid\":true}";
 strcpy(input,good); Request parsed; assert(parse(parsed));
 for(const char* bad:{"{}", "{\"protocol\":\"ANTENNA_CONTROL\",\"version\":1,\"seq\":true,\"action\":\"DISARM\"}","{\"protocol\":\"ANTENNA_CONTROL\",\"version\":1,\"seq\":1,\"action\":\"CALIBRATE\",\"az_mech_deg\":null}"}) {
  strcpy(input,bad); assert(!parse(parsed));
 }
 for(auto pair: {std::make_pair("\"version\":2","\"version\":2.0"),std::make_pair("\"seq\":1","\"seq\":-1"),std::make_pair("\"az_deg\":0","\"az_deg\":360"),std::make_pair("\"el_valid\":true","\"el_valid\":false")}) {
  std::string bad=good; auto at=bad.find(pair.first); bad.replace(at,strlen(pair.first),pair.second);
  strcpy(input,bad.c_str()); assert(!parse(parsed));
 }
 // A single deterministic execution of production motor task + bundled GyverStepper.
 long held=0; bool captured=false; uint32_t seq=0;
 onYield=[&](){
  auto ms=millis();
  assert(azMotor.getCurrent()*stepAngle()>=AZ_MIN_MECH);
  assert(azMotor.getCurrent()*stepAngle()<=AZ_MAX_MECH);
  if(ms==1)send(0,seq++,90,45);
  if(ms==50){assert(!sharedState.armed);assert(azMotor.getCurrent()==0);send(1,0,0,0);}
  if(ms==51){assert(sharedState.armed);assert(!aq.items.empty());}
  if(ms>=60 && ms<3500 && ms%40==0)send(0,seq++,90,45);
  // Incoming near-boundary reversal can extend Gyver's braking target.
  if(ms>=3500 && ms<5000 && ms%40==0)send(0,seq++,270,30);
  if(ms==5300){assert(sharedState.stale);held=azMotor.getCurrent();captured=true;}
  if(ms>5300 && ms<5500 && captured)assert(azMotor.getCurrent()==held);
  if(ms==5500)send(0,seq++,180,20);
  if(ms==5501){assert(sharedState.fault==3);assert(azMotor.getCurrent()==held);}
  if(ms==5550)send(0,seq++,0,20,false,true);
  if(ms==5551)assert(azMotor.getCurrent()==held);
  if(ms==5600)send(2,1,0,0);
  if(ms==5601)assert(!sharedState.armed);
  if(ms==5650)send(0,seq++,90,45);
  if(ms==5800){assert(!sharedState.armed);assert(azMotor.getCurrent()==held);throw Done();}
 };
 try{controlTask(nullptr);}catch(Done&){}
 std::cout<<"Production parser + motor task: calibration, boundary, reversal, stale, invalid axis, unreachable, disarm passed\n";
}

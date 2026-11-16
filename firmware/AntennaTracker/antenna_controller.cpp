#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Servo.h>
#include <GyverStepper2.h>
#include "config.h"
#include "types.h"
#include "safety.h"
#include "antenna_controller.h"
// Gyver phase order: IN1, IN3, IN2, IN4 (see bundled library examples).
static GStepper2<STEPPER4WIRE_HALF> azMotor(4096,IN1,IN3,IN2,IN4);
static Servo elMotor;
static float stepAngle() { return 360.0f/(STEPS_PER_REV*EXTERNAL_RATIO); }
static int pulse(float e) { return (int)lroundf(EL_PULSE_AT_MIN+(e-EL_MIN)*(EL_PULSE_AT_MAX-EL_PULSE_AT_MIN)/(EL_MAX-EL_MIN)); }
void controlTask(void*) {
  State s={}; s.stale=true; s.fault=MECHANICS_CONFIGURED?2:1;
  azMotor.setMaxSpeed(AZ_MAX_SPEED); azMotor.setAcceleration(AZ_ACCELERATION);
  azMotor.reverse(AZ_REVERSE); azMotor.autoPower(false); azMotor.disable();
  float elTarget=EL_INITIAL,elOutput=EL_INITIAL;
  uint32_t servoAt=millis(),controlSeq=0; bool hasControl=false;
  Ack lastAck={};
  for(;;) {
    Request r;
    // One FIFO keeps control actions in receive order; bounded work per iteration.
    if(xQueueReceive(requests,&r,0)==pdPASS) {
      if(r.kind) {
        if(hasControl && r.seq==controlSeq) xQueueSend(acknowledgements,&lastAck,0);
        else if(!hasControl || isNew(r.seq,controlSeq)) {
          Ack a={r.seq,false,0};
          if(r.kind==2) {
            azMotor.brake(); s.armed=false; s.fault=2;
            // Keep the holding torque and last PWM, but require recalibration to re-arm.
            if(s.azKnown) s.cmdAz=wrap360(azMotor.getCurrent()*stepAngle()+AZ_NORTH_OFFSET);
            if(s.elKnown) s.cmdEl=elOutput;
            a.ok=true;
          } else if(!MECHANICS_CONFIGURED) a.reason=1;
          else if(s.armed) a.reason=2; // disarm before changing coordinate reference
          else if(r.az<AZ_MIN_MECH || r.az>AZ_MAX_MECH) a.reason=3;
          else {
            long pos=lroundf(r.az/stepAngle());
            if(pos*stepAngle()<AZ_MIN_MECH || pos*stepAngle()>AZ_MAX_MECH) a.reason=3;
            else {
              azMotor.brake(); azMotor.setCurrent(pos); azMotor.enable();
              elOutput=elTarget=EL_INITIAL;
              // Set stored pulse BEFORE attach to avoid an initial default 1500us pulse.
              elMotor.writeMicroseconds(pulse(elOutput));
              if(!elMotor.attached()) elMotor.attach(SERVO_PIN, min(EL_PULSE_AT_MIN,EL_PULSE_AT_MAX),max(EL_PULSE_AT_MIN,EL_PULSE_AT_MAX));
              elMotor.writeMicroseconds(pulse(elOutput));
              s.armed=true; s.azKnown=s.elKnown=true; s.cmdAz=wrap360(pos*stepAngle()+AZ_NORTH_OFFSET); s.cmdEl=elTarget; s.fault=0;
              a.ok=true;
            }
          }
          controlSeq=r.seq; hasControl=true; lastAck=a; xQueueSend(acknowledgements,&a,0);
        }
      } else if(!s.hasSource || isNew(r.seq,s.seq)) {
        s.hasSource=true; s.seq=r.seq; s.lastRx=millis(); s.stale=false;
        if(s.armed) {
          s.fault=0;
          if(r.azValid) {
            float chosen;
            if(chooseTarget(r.az,azMotor.getCurrent()*stepAngle(),AZ_MIN_MECH,AZ_MAX_MECH,AZ_NORTH_OFFSET,chosen)) {
              long pos=lroundf(chosen/stepAngle());
              // Quantization must not put even one half-step beyond the configured boundary.
              pos=max((long)ceilf(AZ_MIN_MECH/stepAngle()),min((long)floorf(AZ_MAX_MECH/stepAngle()),pos));
              if(pos!=azMotor.getTarget() || !azMotor.getStatus()) azMotor.setTarget(pos);
              s.cmdAz=wrap360(pos*stepAngle()+AZ_NORTH_OFFSET);
            } else {
              azMotor.brake(); s.cmdAz=wrap360(azMotor.getCurrent()*stepAngle()+AZ_NORTH_OFFSET); s.fault=3;
            }
          } else {
            azMotor.brake(); s.cmdAz=wrap360(azMotor.getCurrent()*stepAngle()+AZ_NORTH_OFFSET);
          }
          if(r.elValid) { elTarget=constrain(r.el,EL_MIN,EL_MAX); s.cmdEl=elTarget; }
          else { elTarget=elOutput; s.cmdEl=elOutput; }
        }
      }
    }
    uint32_t now=millis();
    bool stale=!s.hasSource || (uint32_t)(now-s.lastRx)>STALE_MS;
    if(stale && !s.stale) {
      azMotor.brake(); elTarget=elOutput;
      if(s.azKnown) s.cmdAz=wrap360(azMotor.getCurrent()*stepAngle()+AZ_NORTH_OFFSET);
      if(s.elKnown) s.cmdEl=elOutput;
    }
    s.stale=stale;
    if(s.armed && !stale) {
      // Gyver acceleration can temporarily extend a target while braking/reversing.
      // Enforce a hard bound on the NEXT physical step, not just requested targets.
      long next=azMotor.getCurrent()+azMotor.dir;
      if(azMotor.getStatus() && (next*stepAngle()<AZ_MIN_MECH || next*stepAngle()>AZ_MAX_MECH)) {
        azMotor.brake(); s.fault=5;
        s.cmdAz=wrap360(azMotor.getCurrent()*stepAngle()+AZ_NORTH_OFFSET);
      } else azMotor.tick();
      if((uint32_t)(now-servoAt)>=20) {
        // Clamp slew interval to prevent a jump after scheduling delay.
        float delta=EL_RATE_DEG_S*min((uint32_t)(now-servoAt),(uint32_t)40)/1000.0f;
        elOutput+=constrain(elTarget-elOutput,-delta,delta);
        elMotor.writeMicroseconds(pulse(elOutput));
        servoAt=now;
      }
    } else servoAt=now;
    taskENTER_CRITICAL(); sharedState=s; taskEXIT_CRITICAL();
    // Equal-priority tasks explicitly yield: WDT tick (15ms) never sets step timing.
    taskYIELD();
  }
}

#pragma once
class Servo {
 bool on=false;
 public:
 void writeMicroseconds(int){}
 bool attached(){return on;}
 void attach(int,int,int){on=true;}
};

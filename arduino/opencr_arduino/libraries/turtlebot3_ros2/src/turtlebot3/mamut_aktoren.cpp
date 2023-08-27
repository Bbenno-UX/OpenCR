#include "../../include/turtlebot3/mamut_aktoren.h"
// struct stepper{
// int steppin;
// int dirpin;
// int goal_pos;
// int akt_pos;
// unsigned int speed;
// unsigned long dschritt;
// bool stepstate;
// };
unsigned long cntt=0;
void Mamut_aktoren::init(Dynamixel2Arduino& dxl_mamut){
  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  sync_write_param.id_count = 2;
  sync_write_param.xel[0].id = DYNXL_ID_1;
  sync_write_param.xel[1].id = DYNXL_ID_2;
  diestepper[0].steppin=STEP1;
  diestepper[0].dirpin=DIR1;
  diestepper[0].speed=STEPSPEED;
  diestepper[0].dschritt=micros();
  diestepper[1].steppin=STEP2;
  diestepper[1].dirpin=DIR2;
  diestepper[1].speed=STEPSPEED;
  diestepper[1].dschritt=micros();
  pinMode(DIR1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(STEP1,OUTPUT);
  pinMode(STEP2,OUTPUT);
  pinMode(11,OUTPUT);
  digitalWrite(11,HIGH);
  digitalWrite(DIR1,HIGH);
  dynxl_active=true;
  if(!dxl_mamut.ping(DYNXL_ID_1) || !dxl_mamut.ping(DYNXL_ID_2)){
    Serial1.println("failed to ping dxl");
    dynxl_active=false;
  }

}
void Mamut_aktoren::update(unsigned long tims,int16_t mamut_werte[],Dynamixel2Arduino& dxl_mamut){
cntt++;
if(millis()-coun>tims){
coun=millis();

Serial1.print(cntt);
Serial1.print("zyklen1\t");
cntt=0;
Serial1.print(mamut_werte[STEPPER_1]);
Serial1.println("\tStepper");
diestepper[0].goal_pos=mamut_werte[aktoren::STEPPER_1];
diestepper[1].goal_pos=mamut_werte[aktoren::STEPPER_2];
sync_write_param.addr = 104;
sync_write_param.length = 4;
int32_t mamut_dynxl_1_speed=mamut_werte[2];
int32_t mamut_dynxl_2_speed=mamut_werte[3];
//Serial1.println("ssjjs");
//sync_write_param.is_info_changed=true;
memcpy(sync_write_param.xel[0].data, &mamut_dynxl_1_speed, sync_write_param.length);
memcpy(sync_write_param.xel[1].data, &mamut_dynxl_2_speed, sync_write_param.length);
  
  if(dynxl_active && !dxl_mamut.syncWrite(sync_write_param)){
    Serial1.println("failed to send to dxl");
    
  }
}
stepper_motor_update(&diestepper[0]);
stepper_motor_update(&diestepper[1]);
}
void Mamut_aktoren::settorque(bool onoff,Dynamixel2Arduino& dxl_mamut){

  bool ret = false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  sync_write_param.xel[0].data[0] = onoff;
  sync_write_param.xel[1].data[0] = onoff;
  //sync_write_param.is_info_changed=true;
  if(dynxl_active &&!dxl_mamut.syncWrite(sync_write_param) == true){
    ret = true;
    Serial1.println("failed to set_torque");
  }


}
void stepper_motor_update(struct stepper* stepp){
    
    if(micros()-stepp->dschritt>0.5*1000000/stepp->speed){
    stepp->dschritt=micros();
    if(stepp->goal_pos==stepp->akt_pos){
        return;
    }
    digitalWrite(stepp->steppin,stepp->stepstate);
    stepp->stepstate=stepp->stepstate==0;
    if(stepp->goal_pos<stepp->akt_pos){
       
           digitalWrite(stepp->dirpin,1);
           stepp->akt_pos--;
       }
    
     else if(stepp->goal_pos>stepp->akt_pos){
            digitalWrite(stepp->dirpin,0);
            stepp->akt_pos++;
        }
  
    }
}

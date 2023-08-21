#include "../../include/turtlebot3/mamut_aktoren.h"

void Mamut_aktoren::init(Dynamixel2Arduino& dxl_mamut){
  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  sync_write_param.id_count = 2;
  sync_write_param.xel[0].id = DYNXL_ID_1;
  sync_write_param.xel[1].id = DYNXL_ID_2;
  pinMode(DIR1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(STEP1,OUTPUT);
  pinMode(STEP2,OUTPUT);
  if(!dxl_mamut.ping(DYNXL_ID_1) || !dxl_mamut.ping(DYNXL_ID_2)){
    Serial1.println("failed to ping dxl");
  }

}
void Mamut_aktoren::update(unsigned long tims,int32_t mamut_werte[],Dynamixel2Arduino& dxl_mamut){
if(millis()-coun>tims){
coun=millis();
diestepper[0].goal_pos=mamut_werte[STEPPER_1];
diestepper[1].goal_pos=mamut_werte[STEPPER_2];
sync_write_param.addr = 104;
sync_write_param.length = 4;
//sync_write_param.is_info_changed=true;
memcpy(sync_write_param.xel[0].data, &mamut_werte[2], sync_write_param.length);
memcpy(sync_write_param.xel[1].data, &mamut_werte[3], sync_write_param.length);
  if(!dxl_mamut.syncWrite(sync_write_param)){
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
  if(!dxl_mamut.syncWrite(sync_write_param) == true){
    ret = true;
    Serial1.println("failed to set_torque");
  }


}
void stepper_motor_update(struct stepper* stepp){
    
    if(millis()-stepp->dschritt>0.5*1000000/stepp->speed){
    stepp->dschritt=millis();
    if(stepp->goal_pos==stepp->akt_pos){
        return;
    }
    digitalWrite(stepp->steppin,stepp->stepstate);
    stepp->stepstate=stepp->stepstate==0;
    if(stepp->goal_pos<stepp->akt_pos){
       
           digitalWrite(stepp->dirpin,0);
           stepp->akt_pos--;
       }
    
    else if(stepp->goal_pos>stepp->akt_pos){
           digitalWrite(stepp->dirpin,1);
           stepp->akt_pos++;
       }
  
    }
}

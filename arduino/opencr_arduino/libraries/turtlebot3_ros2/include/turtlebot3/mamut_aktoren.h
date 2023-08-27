
#include "controltabl.h"
#include "Dynamixel2Arduino.h"
#define DIR1 8
#define DIR2 12
#define STEP1 9
#define STEP2 10
#define DYNXL_ID_1 11
#define DYNXL_ID_2 12
#define STEPSPEED 200
enum aktoren{STEPPER_1,STEPPER_2,DYNXL_1,DYNXL_2};
struct stepper{
int steppin;
int dirpin;
int goal_pos;
int akt_pos;
unsigned int speed;
unsigned long dschritt;
bool stepstate;
};
void stepper_motor_update(struct stepper* stepp);
class Mamut_aktoren{
bool dynxl_active;
ParamForSyncWriteInst_t sync_write_param;
unsigned long coun;
public:

void init(Dynamixel2Arduino& dxl_mamut);
void settorque(bool onoff,Dynamixel2Arduino& dxl_mamut);
void update(unsigned long tims,int16_t mamut_werte[],Dynamixel2Arduino& dxl_mamut);
struct stepper diestepper[2];
};

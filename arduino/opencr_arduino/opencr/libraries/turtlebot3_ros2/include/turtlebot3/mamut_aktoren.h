
#include "controltabl.h"
#include "Dynamixel2Arduino.h"
#define DIR1 12
#define DIR2 13
#define STEP1 14
#define STEP2 15
#define DYNXL_ID_1 11
#define DYNXL_ID_2 12
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

ParamForSyncWriteInst_t sync_write_param;
unsigned long coun;
public:
void init();
void update(unsigned long tims,int32_t mamut_werte[],Dynamixel2Arduino& dxl_mamut);
struct stepper diestepper[2];
};

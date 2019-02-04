#include "motion_ctl.hpp"
#include "ev3api.h"
#include "Clock.h"
#include "math.h"

using ev3api::Clock;

#define liting_radius 10; // liting spot radius [mm]

//#define DEBUG_AT_MEC
//#define DEBUG_AT_JITAKU
#define   DEBUG_AT_HOTEL

#define SEESAW_SINGLE
//#define SEESAW_DOUBLE

Clock*       gClock;

Motion_Ctl::Motion_Ctl(){

}

void Motion_Ctl::init( ){
  gClock            = new Clock();
  MOTION_MODE       = LINE_TRACE;


  LUG_MODE          = LUG_START;

  SEESAW_MODE       = SEESAW_START;

  ZONE              = FIRST_STRAIGHT_ZONE;

  mYaw_angle_offset = 0.0;
  left_line_edge    = true;
  tail_stand_mode   = false;
  tail_lug_mode     = false;
  gyro_offset_en    = false;
  gyro_offset_value = 0;
  pwm_ctl_en        = false;
  pwm_ctl_value     = 0;


  //  falling_seesaw    = false;
  //  hilling_seesaw    = false;
  bat_mv            = ev3_battery_voltage_mV();
  seesaw_log        = 0;
  seesaw_debug      = false;

  gForward->init_pid(0.05, 0.01,0.001,dT_4ms);
  gAve_500_dat->init();
  gAve_125_dat->init(0.0);


}

void Motion_Ctl::SetCurrentData(int     linevalue,
				bool    green_flag,
				float   xvalue,
				float   yvalue,
				float   pre_50mm_x, //20180512 kota
				float   pre_50mm_y,

				//20181015 -----
				float   ave_x,
				float   ave_y,

				float   ave_vel_x,
				float   ave_vel_y,
				// ---- 20181015

				float   odo,
				float   velocity,

				//20181015 ----
				float ave_velo,
				float ave_accel,
				float pre_velo_0p5sec,
				float ave_wheel_load,
				// ---- 20181015

				float   yawrate,
				float   abs_angle,
				float   ave_angle,
				int     robo_tail_angle,
				bool    robo_stop,
				bool    robo_forward,
				bool    robo_back,
				bool    robo_turn_left,
				bool    robo_turn_right,
				bool    dansa,
				int16_t sonar_dis,
				bool    robo_balance_mode,
				bool    robo_lug_mode,
				int     max_forward,
				float   ref_yawrate,
				float   max_yawrate,
				float   min_yawrate
				 ) {

  mLinevalue         = linevalue;
  mGreen_flag        = green_flag;
  mXvalue            = xvalue;
  mYvalue            = yvalue;
  mPre_50mm_x        = pre_50mm_x;//50mm saki 20180512 kota
  mPre_50mm_y        = pre_50mm_y;//50mm saki 20180512 kota

  //20181015 -----
  mAve_x             = ave_x;
  mAve_y             = ave_y;
  mAve_vel_x         = ave_vel_x;
  mAve_vel_y         = ave_vel_y;
  // ---- 20181015

  mOdo               = odo;
  mVelocity          = velocity;

  //20181015 ----
  mAve_velo          = ave_velo;
  mAve_accel         = ave_accel;
  mPre_velo_0p5sec   = pre_velo_0p5sec;
  mAve_wheel_load    = ave_wheel_load;
  // ---- 20181015

  mYawrate           = yawrate;
  mYawangle          = abs_angle + mYaw_angle_offset;
  mAve_yaw_angle     = ave_angle;

  mTail_angle        = robo_tail_angle;
  mRobo_stop         = robo_stop;
  mRobo_forward      = robo_forward;
  mRobo_back         = robo_back;
  mRobo_turn_left    = robo_turn_left;
  mRobo_turn_right   = robo_turn_right;
  mDansa             = dansa;
  mSonar_dis         = sonar_dis;
  mRobo_balance_mode = robo_balance_mode;
  mRobo_lug_mode     = robo_lug_mode;

  mMax_Forward       = max_forward;
  mRef_Yawrate       = ref_yawrate;
  mMax_Yawrate       = max_yawrate;
  mMin_Yawrate       = min_yawrate;
}



void Motion_Ctl::set_mode_LT() {
  MOTION_MODE = LINE_TRACE;
}

void Motion_Ctl::set_mode_map_trace() {
  MOTION_MODE = MAP_TRACE;
}

void Motion_Ctl::set_mode_LUG() {
  MOTION_MODE = LUG;
}

void Motion_Ctl::set_mode_seesaw() {
  MOTION_MODE = SEESAW;
}



void Motion_Ctl::set_mode_tail_std_debug() {
  MOTION_MODE = TAIL_STAND_DEBUG;
}

void Motion_Ctl::set_mode_debug() {
  MOTION_MODE = DEBUG;
}

void Motion_Ctl::set_zone_start(){           ZONE = START_ZONE;}
void Motion_Ctl::set_zone_1st_straight(){    ZONE = FIRST_STRAIGHT_ZONE;}
void Motion_Ctl::set_zone_enter_1st_corner(){ZONE = ENTER_1ST_CORNER_ZONE;}
void Motion_Ctl::set_zone_1st_corner(){      ZONE = FIRST_CORNER_ZONE;}
void Motion_Ctl::set_zone_2nd_straight(){    ZONE = SECOND_STRAIGHT_ZONE;}
void Motion_Ctl::set_zone_enter_2nd_corner(){ZONE = ENTER_2ND_CORNER_ZONE;}
void Motion_Ctl::set_zone_2nd_corner(){      ZONE = SECOND_CORNER_ZONE;}
void Motion_Ctl::set_zone_3rd_straight(){    ZONE = THIRD_STRAIGHT_ZONE;}
void Motion_Ctl::set_zone_3rd_corner(){      ZONE = THIRD_CORNER_ZONE;}
void Motion_Ctl::set_zone_4th_straight(){    ZONE = FOURTH_STRAIGHT_ZONE;}
void Motion_Ctl::set_zone_4th_corner(){      ZONE = FOURTH_CORNER_ZONE;}
void Motion_Ctl::set_zone_5th_corner(){      ZONE = FIFTH_CORNER_ZONE;}
void Motion_Ctl::set_zone_1st_gray(){        ZONE = FIRST_GRAY_ZONE;}
void Motion_Ctl::set_zone_lug(){             ZONE = LUG_ZONE;}
void Motion_Ctl::set_zone_back_lug(){        ZONE = BACK_LUG_ZONE;}
void Motion_Ctl::set_zone_2nd_gray(){        ZONE = SECOND_GRAY_ZONE;}
void Motion_Ctl::set_zone_seesaw(){          ZONE = SEESAW_ZONE;}
void Motion_Ctl::set_zone_garage(){          ZONE = GARAGE_ZONE;}


void Motion_Ctl::set_zone_lost(){            ZONE = LOST;}


void Motion_Ctl::run(float xvalue, float yvalue, float yawangle) {

  switch(MOTION_MODE){

  case LINE_TRACE:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    ref_tail_angle = TAIL_ANGLE_RUN;
    tail_stand_mode = false;
    break;

  case MAP_TRACE:
    forward =  mMax_Forward;

    if( mVelocity > 0){
      MapTracer(mXvalue, mYvalue, mYawangle);
    }else{
      yawratecmd      = 0.0;
    }

    ref_tail_angle = TAIL_ANGLE_RUN;
    tail_stand_mode = false;
    break;

  case LUG:
    forward =  mMax_Forward;
    break;

  case SEESAW:
    seesaw_run( );
    break;



  case  TAIL_STAND_DEBUG:
    forward         = 40;
    yawratecmd      = 0.0;
    seesaw_run( );
    break;


  case DEBUG:
    seesaw_debug      = true;
    seesaw_run( );

    break;

  case  DEBUG_1:
    forward         = 0;
    yawratecmd      = 0.0;

    if(mRobo_balance_mode == false){
      tail_stand_mode = false;
      }

    break;

  default:
    forward = 0;
    break;
  }

}

/*void Motion_Ctl::LineTracerYawrate(int line_value) {

  y_t = (((float)line_value-50.0)/50.0) * (float)liting_radius;//20180530 kota

  if(y_t > 10.0) y_t = 10.0;
  if(y_t < -10.0) y_t = -10.0;
   y_t = y_t + 7.0*(y_t/8.0)*(y_t/8.0)*(y_t/8.0);

  //    yawratecmd = y_t/4.0;
  yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));

  if(yawratecmd > mMax_Yawrate){
    yawratecmd =  mMax_Yawrate;

  }else if (yawratecmd < mMin_Yawrate){
    yawratecmd = mMin_Yawrate;
  }else{
    yawratecmd = yawratecmd;
    }

    y_t_prev = y_t;

}
*/

void Motion_Ctl::LineTracerYawrate(int line_value) {

  float pos_yaw_step;
  float neg_yaw_step;

  pos_yaw_step = mMax_Yawrate - mRef_Yawrate;
  pos_yaw_step = pos_yaw_step/50.0;

  neg_yaw_step = mMin_Yawrate - mRef_Yawrate;
  neg_yaw_step = neg_yaw_step/50.0;

  /*
  y_t = (float)line_value-50.0;

  y_t = y_t * YAW_STEP;

  if(y_t > YAW_LIMIT)        y_t = YAW_LIMIT;
  if(y_t < -1.0 * YAW_LIMIT) y_t = -1.0 * YAW_LIMIT;
  */
  y_t = (float)line_value-50.0;

  if(y_t >= 0){
    yawratecmd = mRef_Yawrate + (y_t * pos_yaw_step);
  }else{
    yawratecmd = mRef_Yawrate - (y_t * neg_yaw_step);
  }
}


void Motion_Ctl::MapTracer(float mXvalue, float mYvalue, float mYawangle) {

  //	float extend_gain = 1.0;
  float Virtual_point_dist = 50.0;

  float x0,x1,x2,y0,y1,y2,a,a2,b,b2,r2,x10,y10,x12,y12;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  switch(ZONE){
  case FIRST_STRAIGHT_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = STRAIGT_01[0];
    y1  = STRAIGT_01[1];
    x2  = STRAIGT_01[2];
    y2  = STRAIGT_01[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case FIRST_CORNER_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = CIRCLE_01[0];
    y1  = CIRCLE_01[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_01[2];
    //	  y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case SECOND_STRAIGHT_ZONE:

    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1 = STRAIGT_02[0];
    y1 = STRAIGT_02[1];
    x2 = STRAIGT_02[2];
    y2 = STRAIGT_02[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case SECOND_CORNER_ZONE:
    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1 = CIRCLE_02[0];
    y1 = CIRCLE_02[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_02[2];
    //	  y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case THIRD_CORNER_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = CIRCLE_03[0];
    y1  = CIRCLE_03[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_03[2];
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case THIRD_STRAIGHT_ZONE:

    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1 = STRAIGT_03[0];
    y1 = STRAIGT_03[1];
    x2 = STRAIGT_03[2];
    y2 = STRAIGT_03[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case FOURTH_CORNER_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = CIRCLE_04[0];
    y1  = CIRCLE_04[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_04[2];
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case FOURTH_STRAIGHT_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = STRAIGT_04[0];
    y1  = STRAIGT_04[1];
    x2  = STRAIGT_04[2];
    y2  = STRAIGT_04[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case FIFTH_CORNER_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = CIRCLE_05[0];
    y1  = CIRCLE_05[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_05[2];
    //    y_t = -1.0 * y_t; //20180530


    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  case GARAGE_ZONE:

    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1 = STRAIGT_05[0];
    y1 = STRAIGT_05[1];
    x2 = STRAIGT_05[2];
    y2 = STRAIGT_05[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;

    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /* 2018 Right Right Right Right Right Right Right Right Right Right Right Right Right Right Right 2018 */
  default:
    yawratecmd = 0.0;
    break;
  }
}


//** RIGHT Seesaw RUN ***********************************************************Right **//
//** RIGHT Seesaw RUN ***********************************************************Right **//

void Motion_Ctl::seesaw_run( ){

  //  static bool get_ref_odo;
  static unsigned int ref_clock;

  switch(SEESAW_MODE){
    //** RIGHT Seesaw RUN ***********************************************************Right **//
  case SEESAW_START:
    seesaw_log  = 0;

    ref_odo          = mOdo + 9000;

    //    max_load         = 0;
    //    min_load         = 999;
    
    det_slip         = false;
    det_seesaw_edge  = false; 
    det_get_on       = false; 
    fail_get_on      = false; 

    //    det_fall_seesaw  = false;
    det_down_seesaw  = false;
    fail_fall_seesaw = false;

    //det_rise_seesaw  = false;
    det_up_seesaw    = false;
    fail_rise_seesaw = false;

    drop_down        = false;

    //    get_ref_odo      = false;

    motor_break_mode      = false;

    ref_tail_angle   = TAIL_ANGLE_RUN; //0817 tada



    if(seesaw_debug){//----------------------- DEBUG SEESAW -----------------------

      forward         = 20;

      if(mLinevalue > DET_GRAY_LINE){
	LineTracerYawrate(100);
      }else{
	LineTracerYawrate(mLinevalue);
      }

      if(mVelocity <=5){
	yawratecmd = 0;
      }



      //-------------------------------------------------------
#ifdef DEBUG_AT_JITAKU
      seesaw_log  = 11;
      if((mYvalue <= (490-500)) && (mLinevalue > DET_GRAY_LINE)){
	ref_angle = gAve_500_dat->average_500(mYawangle);
      }

      if(mYvalue < (490-650)){
	ref_angle   = ref_angle;
	gAve_500_dat->init();
	ref_odo     = mOdo + 150;
	SEESAW_MODE = CORRECT_ANGLE;

      }
#endif

      //-------------------------------------------------------
#ifdef DEBUG_AT_MEC
      seesaw_log  = 22;
      if((mXvalue >= (360+300)) && (mLinevalue > DET_GRAY_LINE)){
	ref_angle = gAve_500_dat->average_500(mYawangle);
      }
      if(mXvalue > (360+500)){
	ref_angle   = ref_angle;
	gAve_500_dat->init();
	gAve_125_dat->init(0.0);
	ref_odo     = mOdo + 150;
	SEESAW_MODE = CORRECT_ANGLE;
      }
#endif
      //-------------------------------------------------------

      //-------------------------------------------------------
#ifdef DEBUG_AT_HOTEL
      seesaw_log  = 33;
      yawratecmd  =  0;

      if(mXvalue > (360+150)){
	ref_angle   = 0.0;
	gAve_500_dat->init();
	gAve_125_dat->init(0.0);
	ref_odo     = mOdo + 150;
	SEESAW_MODE = CORRECT_ANGLE;
      }

#endif
      //-------------------------------------------------------

  }else{//----------------------- NORMAL SEESAW -----------------------

      forward         = 20;

      if(mYvalue < 1700){
	LineTracerYawrate(mLinevalue);
      }else{
	if(mLinevalue > DET_GRAY_LINE){
	  LineTracerYawrate(100);
	}else{
	  LineTracerYawrate(mLinevalue);
	}
      }

      /*
      if((mYvalue >= FIRST_GRAY_AREA[1]) && (mLinevalue > DET_GRAY_LINE)){
	ref_angle = gAve_500_dat->average_500(mYawangle);
      }
      */

      //      if(mYvalue >= (FIRST_GRAY_AREA[3] - 50)){
      if(mYvalue >=  (FIRST_GRAY_AREA[1] - 20)  ){
	//	ref_angle   = ref_angle;
	ref_angle   = RAD_88p5_DEG;
	gAve_500_dat->init();
	//	ref_odo     = mOdo + 230;
	ref_odo     = mOdo + 150;
	SEESAW_MODE = CORRECT_ANGLE;
      }

      
    }//----------------------- NORMAL SEESAW -----------------------
    
    break;

  case CORRECT_ANGLE:
    ref_tail_angle   = 30;

    seesaw_log  = 400;
    forward    = 20;
    yawratecmd = 10*LUG_YAW_GAIN*(ref_angle - mYawangle);

    if(mVelocity > 10){
      load_01  = gAve_500_dat->average_500(mAve_wheel_load);
      ref_load = load_01 * LOAD_GAIN;
    }
    
    
    if(mOdo >= ref_odo){
      if(mAve_wheel_load > ref_load){
	edge_odo       = mOdo;
	dash_pos       = edge_odo - SEESAW_DASH_POS;
	get_on_pos     = edge_odo + SEESAW_GET_ON_POS;
	stop_pos_1st   = edge_odo + SEESAW_1ST_POS;
        stop_pos_2nd   = edge_odo + SEESAW_2ND_POS;
        stop_pos_3rd   = edge_odo + SEESAW_3RD_POS;

	axis_pos       = edge_odo + 350;
	seesaw_end_pos = edge_odo + 600;
	garage_pos     = edge_odo + SEESAW_STOP_POS;

	ref_odo        = mOdo - SEESAW_SET_POS;
	SEESAW_MODE    = SET_POS; 
      }
    }

    if(mOdo >= (ref_odo + 80)){
	edge_odo       = mOdo;
	dash_pos       = edge_odo - SEESAW_DASH_POS;
	get_on_pos     = edge_odo + SEESAW_GET_ON_POS;
	stop_pos_1st   = edge_odo + SEESAW_1ST_POS;
        stop_pos_2nd   = edge_odo + SEESAW_2ND_POS;
        stop_pos_3rd   = edge_odo + SEESAW_3RD_POS;

	axis_pos       = edge_odo + 350;
	seesaw_end_pos = edge_odo + 600;
	garage_pos     = edge_odo + SEESAW_STOP_POS;

	ref_odo        = mOdo - SEESAW_SET_POS;
	SEESAW_MODE    = SET_POS; 
      }
    
    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//
      case SET_POS:
	seesaw_log  = 500;
	forward    = -40;
	yawratecmd = 10*LUG_YAW_GAIN*(ref_angle - mYawangle);

      //1101_1818-----

      if(mOdo < ref_odo){
	forward    = 0;
	yawratecmd = 0;
	gAve_500_dat->init();
	SEESAW_MODE = APPROACH_TO_SEESAW;
      }

      break;


//** RIGHT Seesaw RUN ***********************************************************Right **//
  case  APPROACH_TO_SEESAW:
    seesaw_log  = 600;

    if(mVelocity > 100){
      load_01    = gAve_500_dat->average_500(mAve_wheel_load);
    }

    forward    = SEESAW_DASH_FORWARD;
    yawratecmd = 10*LUG_YAW_GAIN*(ref_angle - mYawangle);

    ref_tail_angle = TAIL_ANGLE_RUN;

    if(mOdo > dash_pos){  //	dash_pos       = edge_odo - SEESAW_DASH_POS;

      if(load_01 <= 0){
	load_01 = 0.1;
      }

      ref_load    = load_01 * 1.6;
      SEESAW_MODE = GET_ON_SEESAW;
      gAve_500_dat->init();
    }
    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//
  case  GET_ON_SEESAW:
    seesaw_log  = 700;

    //pitch_ctl
    gyro_offset_en    = true;
    gyro_offset_value = SEESAW_DASH_GYRO_OFFSET;

    forward           = SEESAW_DASH_FORWARD;
    yawratecmd        = 0.0;
    
    //SLIP CTL---------------------------------------------------------------------------
    /*
    if(mOdo > (edge_odo + 100)){
      seesaw_log  = 710;
      if(mAve_velo > 270 ){
	det_slip   = true;   
      }else if (mAve_velo < 20){
	det_slip   = false;
      }else{
	det_slip = det_slip;
      }

      if(det_slip){
	seesaw_log  = 720;
	forward = 0;
      }else{
	forward = SEESAW_1ST_FORWARD;
      }
    }

    gAve_500_dat->average_500(mAve_velo);
    */
    //---------------------------------------------------------------------------SLIP CTL

    if(mOdo >= get_on_pos){ //	get_on_pos     = edge_odo + SEESAW_GET_ON_POS;
      seesaw_log  = 24;
      det_get_on  = true;
      det_slip    = false;
      //      ref_odo     = edge_odo + SEESAW_1ST_POS;
      SEESAW_MODE = GO_UP_1ST_SEESAW;
    }

    if(fail_get_on){
      seesaw_log  = 25;
      SEESAW_MODE = APPROACH_TO_SEESAW;
    }
    break;


//** RIGHT Seesaw RUN ***********************************************************Right **//
  case  GO_UP_1ST_SEESAW:
    seesaw_log  = 800;

    forward           = SEESAW_1ST_FORWARD;
    yawratecmd        = 0.0;

    gyro_offset_en    = true;
    gyro_offset_value = SEESAW_1ST_GYRO_OFFSET;


#ifdef SEESAW_SINGLE
    if(mOdo >= stop_pos_1st){
      motor_break_mode = false;
      SEESAW_MODE      = STOP_1ST_SEESAW;
      ref_clock        = gClock->now() + SEESAW_1ST_STOP_TIME;
    }
#endif

#ifdef SEESAW_DOUBLE
    if(mOdo >= stop_pos_1st){                    //stop_pos_1st   = edge_odo + SEESAW_1ST_POS;
      motor_break_mode = false;
      SEESAW_MODE      = STOP_1ST_SEESAW;
      ref_clock        = gClock->now() + SEESAW_1ST_STOP_TIME;
    }
#endif


    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//

//NOT USED FOR NOW 181027
  case  STOP_1ST_SEESAW:
    seesaw_log        = 900;
    gyro_offset_en    = true;
    gyro_offset_value = 10;

#ifdef SEESAW_DOUBLE
    forward           = 0;
#endif


#ifdef SEESAW_SINGLE
    if(gClock->now() > ref_clock){
      seesaw_log  = 930;
      gyro_offset_en    = true;
      //      gyro_offset_value = SEESAW_1ST_STOP_GYRO_OFFSET;
      //      forward           = SEESAW_2ND_FORWARD;
      gyro_offset_value = SEESAW_1ST_STOP_GYRO_OFFSET;
      forward           = SEESAW_1ST_STOP_FORWARD;

      //      if(mVelocity < 0){
      if(mPre_velo_0p5sec < 0){
	SEESAW_MODE = GET_OFF_SEESAW;
      }
    }
#endif


#ifdef SEESAW_DOUBLE
    if(gClock->now() > ref_clock){
      seesaw_log  = 930;
      gyro_offset_en    = true;
      gyro_offset_value = SEESAW_1ST_STOP_GYRO_OFFSET;
      forward           = SEESAW_1ST_STOP_FORWARD;

      if(mVelocity < 0){
	SEESAW_MODE = GO_UP_2ND_SEESAW;
      }
    }
#endif

    if(mOdo > seesaw_end_pos){
      seesaw_log  = 950;
      gyro_offset_en    = true;
      gyro_offset_value = 0;
      forward           = 0;
      SEESAW_MODE       = GET_OFF_SEESAW;
    }
    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//
  case  GO_UP_2ND_SEESAW:

    seesaw_log  = 1000;

    gyro_offset_en    = true;
    gyro_offset_value = SEESAW_2ND_GYRO_OFFSET;
    forward           = SEESAW_2ND_FORWARD;

    if(mOdo <= stop_pos_2nd){                            //stop_pos_2nd   = edge_odo + SEESAW_2ND_POS;
      motor_break_mode = false;
      SEESAW_MODE      = STOP_2ND_SEESAW;
      ref_clock        = gClock->now() + SEESAW_2ND_STOP_TIME;
    }

    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//
//NOT USED FOR NOW 181027
  case  STOP_2ND_SEESAW:
    seesaw_log  = 1100;

    gyro_offset_en    = true;
    gyro_offset_value = -5;
    forward           =  0;

    if(gClock->now() > ref_clock){
      seesaw_log        = 1130;
      gyro_offset_en    = true;
      gyro_offset_value = SEESAW_2ND_STOP_GYRO_OFFSET;
      forward           = SEESAW_2ND_STOP_FORWARD;

      if(mVelocity > 0){
	SEESAW_MODE = GO_UP_3RD_SEESAW;
      }
    }

    if(mOdo > seesaw_end_pos){
      seesaw_log        = 1150;
      gyro_offset_en    = true;
      gyro_offset_value = 0;
      forward           = 0;
      SEESAW_MODE       = GET_OFF_SEESAW;
    }

    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//
  case  GO_UP_3RD_SEESAW:
    seesaw_log  = 1200;
    
    gyro_offset_en    = true;
    gyro_offset_value = SEESAW_3RD_GYRO_OFFSET;
    forward           = SEESAW_3RD_FORWARD;
    yawratecmd        = 0.0;


    if(mOdo >= stop_pos_3rd){
      seesaw_log      = 1270;
      //det_fall_seesaw = true;
      SEESAW_MODE     = STOP_GARAGE;
    }

    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//
//NOT USED FOR NOW 181027
  case  STOP_3RD_SEESAW:
    seesaw_log  = 1300;

    if(det_down_seesaw){
      seesaw_log  = 81;
      SEESAW_MODE = GET_OFF_SEESAW;
    }else if(fail_fall_seesaw){
      seesaw_log       = 82;
      //det_fall_seesaw  = false;
      fail_fall_seesaw = false;
      SEESAW_MODE      = GO_UP_3RD_SEESAW;
    }else if(drop_down){
      seesaw_log       = 83;
      SEESAW_MODE      = STOP_GARAGE;
    }
    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//
  case  GET_OFF_SEESAW:
    seesaw_log  = 1400;
    
    gyro_offset_en    = true;
    gyro_offset_value = 0;
    forward           = 50;

    if(mOdo > stop_pos_3rd){
      seesaw_log  = 1410;
      SEESAW_MODE = STOP_GARAGE;
    }

    break;

//** RIGHT Seesaw RUN ***********************************************************Right **//
  case  STOP_GARAGE:
    seesaw_log  = 1500;

    if(tail_stand_mode == false){
      forward    = gForward->calc_pid(garage_pos, mOdo);
      forward    = 0.4 * forward;
      yawratecmd = 0.0;
    }

    //    if( (gAve_500_dat->average_500(mAve_velo) < 10) && (gAve_500_dat->average_500(mAve_velo) > -10) ){
    if(mRobo_stop){
      seesaw_log  = 1510;
      tail_stand_mode   = true;
    }

    if(mVelocity < 10){
      seesaw_log     = 1520;
      ref_tail_angle = TAIL_ANGLE_LUG;
      ref_clock      = gClock->now() + 5000;
      SEESAW_MODE    = STOP_GARAGE_TAIL; 
      ref_odo        = mOdo + 70;
    }

    break;


//** RIGHT Seesaw RUN ***********************************************************Right **//
  case  STOP_GARAGE_TAIL:
    seesaw_log  = 1600;
    pwm_ctl_en        = true;
    pwm_ctl_value     = 0;

    if(gClock->now() > ref_clock){
      if(mOdo > ref_odo){
	pwm_ctl_value     = 0;
      }else{
	pwm_ctl_value     = 5;
      }
    }

    break;
//** RIGHT Seesaw RUN ***********************************************************Right **//

  default:
    seesaw_log  = 999;
    forward      = 0;
    yawratecmd    = 0;
    ref_tail_angle = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;
    break;
  }
}

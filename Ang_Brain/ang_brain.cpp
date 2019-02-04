 /******************************************************************************
  *  ang_brain.cpp (for LEGO Mindstorms EV3)
  *  Created on: 2018/04/22
  *  Implementation of the Class Ang_Robo
  *  Author: Koaru Ota
  *  https://github.com/KaoruOta/anago2018_beta
  *  https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo

 --- basic structure ---

 void Ang_Brain::run()

 void Ang_Brain::det_navigation() {

   if(DRIVE_MODE == LINE_TRACE){

     switch(ZONE){
     case START_ZONE:
     case FIRST_STRAIGHT_ZONE:
     case ENTER_1ST_CORNER_ZONE:
     case FIRST_CORNER_ZONE:
     case SECOND_STRAIGHT_ZONE:
     case ENTER_2ND_CORNER_ZONE:
     case SECOND_CORNER_ZONE:
     case THIRD_STRAIGHT_ZONE:
     case THIRD_CORNER_ZONE:
     case FOURTH_STRAIGHT_ZONE:
     case FOURTH_CORNER_ZONE:
     case FIRST_GRAY_ZONE:
     case LUG_ZONE:
     case SECOND_GRAY_ZONE:
     case GARAGE_ZONE:
     case LOST:

 #ifdef ET_2018_RIGHT
     case START_ZONE:
     ....same as LEFT
 }
 else if(DRIVE_MODE == TRACK){

 #ifdef LEFT_2018
     switch(ZONE){
     case START_ZONE:
     case FIRST_STRAIGHT_ZONE:
     case ENTER_1ST_CORNER_ZONE:
     case FIRST_CORNER_ZONE:
     case SECOND_STRAIGHT_ZONE:
     case ENTER_2ND_CORNER_ZONE:
     case SECOND_CORNER_ZONE:
     case THIRD_STRAIGHT_ZONE:
     case THIRD_CORNER_ZONE:
     case FOURTH_STRAIGHT_ZONE:
     case FOURTH_CORNER_ZONE:
     case FIRST_GRAY_ZONE:
     case LUG_ZONE:
     case SECOND_GRAY_ZONE:
     case GARAGE_ZONE:
     case LOST:

 #ifdef ET_2018_RIGHT
     switch(ZONE){
     case START_ZONE:
     ....same as LEFT
 }

 else if(DRIVE_MODE == DEBUG){}

 *****************************************************************************/

#include <stdlib.h>
#include "ev3api.h"
#include "ang_brain.hpp"
#define liting_radius 10; // liting spot radius [mm]

 Ang_Brain::Ang_Brain() {

 }

 void Ang_Brain::init() {
   ZONE               = START_ZONE;
   DRIVE_MODE         = LINE_TRACE;
   line_trace_mode    = true;
   //   forward_curve_mode = true;
   forward_curve_mode = false;
   gMotion_Ctl->init();
   mMax_Forward = 10;
   re_start     = false;
 }


 void Ang_Brain::set_drive_mode_LT(){
   DRIVE_MODE = LINE_TRACE;
   line_trace_mode = true;
 }

 void Ang_Brain::set_drive_mode_TK(){
   DRIVE_MODE = TRACK;
   line_trace_mode = false;
 }
 void Ang_Brain::set_drive_mode_DB(){
   DRIVE_MODE = DEBUG;
   line_trace_mode = false;
 }




 void Ang_Brain::run() {

   det_navigation();

   gMotion_Ctl->SetCurrentData(mLinevalue,
			       mGreen_flag,
			       mXvalue,
			       mYvalue,
			       mPre_50mm_x,
			       mPre_50mm_y,

			       //20181015 -----
			       mAve_x,
			       mAve_y,
			       
			       mAve_vel_x,
			       mAve_vel_y,
			       // ---- 20181015
			       mOdo,
			       mVelocity,

			       //20181015 ----
			       mAve_velo,
			       mAve_accel,
			       mPre_velo_0p5sec,
			       mAve_wheel_load,
			       // ---- 20181015

			       mYawrate,
			       mYawangle,
			       mAve_yaw_angle,
			       mTail_angle,
			       mRobo_stop,
			       mRobo_forward,
			       mRobo_back,
			       mRobo_turn_left,
			       mRobo_turn_right,
			       mDansa,
			       mSonar_dis,
			       mRobo_balance_mode,
			       mRobo_lug_mode,
			       mMax_Forward,
			       mRef_Yawrate,
			       mMax_Yawrate,
			       mMin_Yawrate
			       );


   gMotion_Ctl->run(mXvalue,mYvalue,mYawangle);

   GetCalcResult(gMotion_Ctl->forward,
		 gMotion_Ctl->yawratecmd,
		 gMotion_Ctl->ref_tail_angle,
		 gMotion_Ctl->tail_stand_mode,
		 gMotion_Ctl->tail_lug_mode,
		 //		 gMotion_Ctl->gyro_offset_plus,
		 gMotion_Ctl->gyro_offset_en,
		 //		 gMotion_Ctl->falling_seesaw,
		 //		 gMotion_Ctl->hilling_seesaw);
		 gMotion_Ctl->gyro_offset_value,
		 gMotion_Ctl->pwm_ctl_en,
		 gMotion_Ctl->pwm_ctl_value);

   //180905-----
   log_dat = gMotion_Ctl->seesaw_log;
   //----180905

 }

/****************************************************************************************/
//2018 04 21 Kaoru Ota
//    //https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
/****************************************************************************************/
void Ang_Brain::det_navigation() {

  float        yaw_time;
  //  float        ref_velocity;
  static float ref_odo;
  static float dif_odo;

  static int   ref_forward;
  static float acl_forward;

  // static float ref_y;

  if(DRIVE_MODE == LINE_TRACE){
    //     forward_curve_mode = true;
    forward_curve_mode = false;

    switch(ZONE){
/* Right Right Right *************************************************************** Line Trace */      
    case START_ZONE:
      gMotion_Ctl->set_mode_LT();

      //MAX FORWARD GEN-------------------------------------------------------
      //      ref_velocity = mOdo;

      ref_odo = mOdo;

      if(ref_odo < 0){
	ref_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward = 10 + (ref_odo * ACCEL_GAIN); //181110 Kokeru Gein
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > START_FORWARD_VAL){
	mMax_Forward = START_FORWARD_VAL;
      }

      //181101
      if(re_start){
	mMax_Forward = 70;
      }

      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
      if(mVelocity > 10){//koko
	mMin_Yawrate = MINUS_RAD_22P5_DEG; //koko
	 mRef_Yawrate = 0;
	 mMax_Yawrate = RAD_22P5_DEG;
       }else{
	 mMin_Yawrate = 0;
	 mRef_Yawrate = 0;
	 mMax_Yawrate = 0;
       }
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
      if (mPre_50mm_x > FIRST_STRAIGHT_AREA[0]){ //koko
	 ZONE = FIRST_STRAIGHT_ZONE;
	 gMotion_Ctl->set_mode_LT();
	 gMotion_Ctl->set_zone_1st_straight();

	 //FOR NEXT MAX FORWARD GEN--------------------------------------------
	 ref_forward = mMax_Forward;
	 ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

	 //DET START FAIL------------------------------------------------------
       }else if(mXvalue < START_AREA[0]){
	 ZONE = START_BACK;
       }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE
	 DRIVE_MODE = TRACK;
	 ZONE = FIRST_STRAIGHT_ZONE;
	 gMotion_Ctl->set_mode_map_trace();
	 gMotion_Ctl->set_zone_1st_straight();
	 mMax_Forward = 50;
	 mMin_Yawrate = -1.0 * RAD_6_DEG;
	 mMax_Yawrate = RAD_6_DEG;
       }
       break;
       

 /* Right Right Right *************************************************************** Line Trace */
    case START_BACK:
      gMotion_Ctl->set_mode_map_trace();
      gMotion_Ctl->set_zone_1st_straight();
      //      mMax_Forward = 30;
      mMax_Forward = 70; //181110
      if(mXvalue > 360){
	re_start = true;
	ZONE     = START_ZONE;
      }
      break;


 /* Right Right Right *************************************************************** Line Trace */
    case FIRST_STRAIGHT_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }
      
      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FIRST_STRAIGHT_FORWARD_VAL){
	mMax_Forward =  FIRST_STRAIGHT_FORWARD_VAL;
      }
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------ //koko
       mMin_Yawrate = MINUS_RAD_22P5_DEG; 
       mRef_Yawrate = 0;
       mMax_Yawrate = RAD_22P5_DEG;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_x > ENTER_1ST_CORNER_AREA[0]){
	 ZONE = ENTER_1ST_CORNER_ZONE;

	 //FOR NEXT MAX FORWARD GEN--------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

       }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE
	 /*
	 DRIVE_MODE = TRACK;
	 ZONE = FIRST_STRAIGHT_ZONE;
	 gMotion_Ctl->set_mode_map_trace();
	 gMotion_Ctl->set_zone_1st_straight();
	 mMax_Forward = 50;
	 mMax_Yawrate = RAD_6_DEG;
	 mMin_Yawrate = -1.0 * RAD_6_DEG;
	 */

       }
       break;


 /* Right Right Right *************************************************************** Line Trace */
     case ENTER_1ST_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
      dif_odo = mOdo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > ENTER_1ST_CORNER_FORWARD_VAL){
	mMax_Forward =  ENTER_1ST_CORNER_FORWARD_VAL;
      }
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       mMin_Yawrate = MINUS_RAD_22P5_DEG;
       mRef_Yawrate = 0.0;
       yaw_time     = CIRCLE_01_LENGTH/mVelocity;
       mMax_Yawrate = (CIRCLE_01_ANGLE/yaw_time) + RAD_22P5_DEG;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_x >FIRST_CORNER_AREA[0]){

	 ZONE = FIRST_CORNER_ZONE;
	 gMotion_Ctl->set_zone_1st_corner();

	 //FOR NEXT MAX FORWARD GEN--------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

	//DET LOST THE LINE-----------------------------------------------------
       }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_45_DEG)){ //LOST THE LINE

	  DRIVE_MODE = TRACK;
	  ZONE       = FIRST_CORNER_ZONE;
	  gMotion_Ctl->set_mode_map_trace();
	  gMotion_Ctl->set_zone_1st_corner();

	}
	break;

  /* Right Right Right *************************************************************** Line Trace */
      case FIRST_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FIRST_CORNER_FORWARD_VAL){
	mMax_Forward =  FIRST_CORNER_FORWARD_VAL;
      }
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
      yaw_time     = CIRCLE_01_LENGTH/mVelocity;
      mRef_Yawrate = CIRCLE_01_ANGLE/yaw_time;
      mMax_Yawrate = mRef_Yawrate + RAD_22P5_DEG;

      mMin_Yawrate = MINUS_RAD_22P5_DEG;
      mRef_Yawrate = 0.0;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
      if ((mPre_50mm_y > 1060 )&& (mPre_50mm_x < SECOND_STRAIGHT_AREA[0])){
	ZONE = SECOND_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_2nd_straight();

	//FOR NEXT MAX FORWARD GEN--------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_225_DEG) ){ //LOST THE LINE -> MAP TRACE MODE
	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();
	ZONE       = FIRST_CORNER_ZONE;
	gMotion_Ctl->set_zone_1st_corner();
      }
      break;


 /* Right Right Right *************************************************************** Line Trace */
     case SECOND_STRAIGHT_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
       dif_odo = mOdo - ref_odo;
       if(dif_odo < 0){
	 dif_odo = 0;
       }
       
       if(mPre_velo_0p5sec > MAX_VELOCITY){
	 mMax_Forward = 	mMax_Forward;
       }else{
	 acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	 mMax_Forward = (int)acl_forward;
       }
       
       if(mMax_Forward > SECOND_STRAIGHT_FORWARD_VAL){
	 mMax_Forward = SECOND_STRAIGHT_FORWARD_VAL;
       }
      //-------------------------------------------------------MAX FORWARD GEN
       

      //REF YAW RATE GEN------------------------------------------------------
       mMin_Yawrate = MINUS_RAD_22P5_DEG;
       mRef_Yawrate = 0;
       mMax_Yawrate = RAD_22P5_DEG;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_x < ENTER_2ND_CORNER_AREA[0]){
	 ZONE = S_CORNER_ZONE;
	 gMotion_Ctl->set_zone_enter_2nd_corner();

	 //FOR NEXT MAX FORWARD GEN----------------------------------
	 ref_forward = mMax_Forward;
	 ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

       }else if ((mAve_yaw_angle < RAD_135_DEG) || (mAve_yaw_angle > RAD_225_DEG)){ //LOST THE LINE -> MAP TRACE MODE

	 DRIVE_MODE = TRACK;
	 gMotion_Ctl->set_mode_map_trace();

	 ZONE = SECOND_STRAIGHT_ZONE;
	 gMotion_Ctl->set_zone_2nd_straight();
       }
       break;


/* Right Right Right *************************************************************** Line Trace */
/*  NOT USED NOW 20181014 ***********************************************************************/
     case ENTER_2ND_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
       dif_odo = mOdo - ref_odo;
       if(dif_odo < 0){
	 dif_odo = 0;
       }

       if(mPre_velo_0p5sec > MAX_VELOCITY){
	 mMax_Forward = 	mMax_Forward;
       }else{
	 acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	 mMax_Forward = (int)acl_forward;
       }

       if(mMax_Forward > ENTER_2ND_CORNER_FORWARD_VAL){
	 mMax_Forward = ENTER_2ND_CORNER_FORWARD_VAL;
       }
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       yaw_time     = CIRCLE_02_LENGTH/mVelocity;
       mMax_Yawrate = (CIRCLE_02_ANGLE/yaw_time) + RAD_22P5_DEG;
       mRef_Yawrate = 0.0;
       mMin_Yawrate = MINUS_RAD_22P5_DEG;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_x < SECOND_CORNER_AREA[0]){

	 ZONE = SECOND_CORNER_ZONE;
	 gMotion_Ctl->set_zone_2nd_corner();

	//FOR NEXT MAX FORWARD GEN--------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

       }else if ((mAve_yaw_angle < RAD_135_DEG) || (mAve_yaw_angle > RAD_225_DEG)){ //LOST THE LINE -> MAP TRACE MODE

	 DRIVE_MODE = TRACK;
	 gMotion_Ctl->set_mode_map_trace();

	 ZONE = SECOND_CORNER_ZONE;
	 gMotion_Ctl->set_zone_2nd_corner();

       }
       break;

/* Right Right Right *************************************************************** Line Trace */
/* NOT USED NOW 20181014 ************************************************************************/
     case SECOND_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
       mMax_Forward = SECOND_CORNER_FORWARD_VAL;
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       yaw_time     = CIRCLE_02_LENGTH/mVelocity;
       mRef_Yawrate = CIRCLE_02_ANGLE/yaw_time;

       mMax_Yawrate = mRef_Yawrate + RAD_22P5_DEG;
       mRef_Yawrate = 0;
       mMin_Yawrate = MINUS_RAD_30_DEG;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_y < (THIRD_CORNER_AREA[3] + 150)){

	 ZONE = THIRD_STRAIGHT_ZONE;
	 gMotion_Ctl->set_zone_3rd_straight();

       }else if ((mAve_yaw_angle < RAD_135_DEG) || (mAve_yaw_angle > RAD_315_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	 DRIVE_MODE = TRACK;
	 gMotion_Ctl->set_mode_map_trace();

	 ZONE = SECOND_CORNER_ZONE;
	 gMotion_Ctl->set_zone_2nd_corner();
       }
       break;

/* Right Right Right *************************************************************** Line Trace */
/*  NOT USED NOW 20181014 ***********************************************************************/
     case THIRD_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
       mMax_Forward = THIRD_CORNER_FORWARD_VAL;
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       yaw_time     = CIRCLE_03_LENGTH/mVelocity;
       mRef_Yawrate = CIRCLE_03_ANGLE/yaw_time;

       mMin_Yawrate = mRef_Yawrate - RAD_22P5_DEG;
       mRef_Yawrate = 0;
       mMax_Yawrate = RAD_30_DEG;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_x < THIRD_STRAIGHT_AREA[2]){
	 ZONE = THIRD_STRAIGHT_ZONE;
	 gMotion_Ctl->set_zone_3rd_straight();

       }else if ((mAve_yaw_angle < RAD_135_DEG) || (mAve_yaw_angle > RAD_315_DEG)){ //LOST THE LINE -> MAP TRACE MODE

	 DRIVE_MODE = TRACK;
	 gMotion_Ctl->set_mode_map_trace();

	 ZONE = THIRD_CORNER_ZONE;
	 gMotion_Ctl->set_zone_3rd_corner();

       }
       break;


 /* Right Right Right *************************************************************** Line Trace */
     case S_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
       mMax_Forward = S_CORNER_FORWARD_VAL;
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       if(mVelocity < 300){
	 yaw_time     = CIRCLE_02_LENGTH/300;
       }else{
	 yaw_time     = CIRCLE_02_LENGTH/mVelocity;
       }

       mRef_Yawrate = CIRCLE_02_ANGLE/yaw_time;
       mMax_Yawrate = mRef_Yawrate + RAD_30_DEG;
       mMin_Yawrate = -1.0 * mMax_Yawrate;
       mRef_Yawrate = 0;
       //------------------------------------------------------REF YAW RATE GEN



       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_x < THIRD_STRAIGHT_AREA[2]){
	 ZONE = THIRD_STRAIGHT_ZONE;
	 gMotion_Ctl->set_zone_3rd_straight();

	//FOR NEXT MAX FORWARD GEN--------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN
       }

       break;

 /* Right Right Right *************************************************************** Line Trace */
    case THIRD_STRAIGHT_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
      dif_odo = mOdo - ref_odo;
      
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > THIRD_STRAIGHT_FORWARD_VAL){
	mMax_Forward = THIRD_STRAIGHT_FORWARD_VAL;
      }
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
      mMin_Yawrate = MINUS_RAD_30_DEG;
      mRef_Yawrate = 0;
      mMax_Yawrate = RAD_45_DEG;
      //------------------------------------------------------REF YAW RATE GEN


      //DET RUNNING AREA------------------------------------------------------
      if (mPre_50mm_x < FOURTH_CORNER_AREA[2]){
	ZONE = FOURTH_CORNER_ZONE;
	gMotion_Ctl->set_zone_4th_corner();

	//FOR NEXT MAX FORWARD GEN--------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

       }else if ((mAve_yaw_angle < RAD_135_DEG) || (mAve_yaw_angle > RAD_225_DEG)){ //LOST THE LINE -> MAP TRACE MODE

	 DRIVE_MODE = TRACK;
	 gMotion_Ctl->set_mode_map_trace();

	 ZONE = THIRD_STRAIGHT_ZONE;
	 gMotion_Ctl->set_zone_3rd_straight();
       }
       break;


 /* Right Right Right *************************************************************** Line Trace */
    case FOURTH_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FOURTH_CORNER_FORWARD_VAL){
	mMax_Forward = FOURTH_CORNER_FORWARD_VAL;
      }
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       yaw_time     = CIRCLE_04_LENGTH/mVelocity;
       mRef_Yawrate = CIRCLE_04_ANGLE/yaw_time;
       mMin_Yawrate = mRef_Yawrate - RAD_22P5_DEG;

       mRef_Yawrate = 0;
       mMax_Yawrate = mRef_Yawrate + RAD_22P5_DEG;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
       //https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
       if ((mPre_50mm_y > 1250 )&& (mPre_50mm_x > FOURTH_STRAIGHT_AREA[0])){
	 ZONE = FOURTH_STRAIGHT_ZONE;
	 gMotion_Ctl->set_zone_4th_straight();

	//FOR NEXT MAX FORWARD GEN--------------------------------------------
	 ref_forward = mMax_Forward;
	 ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

       }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG ) || (mAve_yaw_angle > RAD_225_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	 DRIVE_MODE = TRACK;
	 gMotion_Ctl->set_mode_map_trace();

	 ZONE = FOURTH_CORNER_ZONE;
	 gMotion_Ctl->set_zone_4th_corner();
       }
       break;


 /* Right Right Right *************************************************************** Line Trace */
    case FOURTH_STRAIGHT_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FOURTH_STRAIGHT_FORWARD_VAL){
	mMax_Forward = FOURTH_STRAIGHT_FORWARD_VAL;
      }
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       mMin_Yawrate = MINUS_RAD_22P5_DEG;
       mRef_Yawrate = 0;
       mMax_Yawrate = RAD_22P5_DEG;
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_x > ENTER_5TH_CORNER_AREA[0]){
	 ZONE = ENTER_5TH_CORNER_ZONE;

	//FOR NEXT MAX FORWARD GEN--------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN

       }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE
	 DRIVE_MODE = TRACK;
	 gMotion_Ctl->set_mode_map_trace();

	 ZONE = FOURTH_STRAIGHT_ZONE;
	 gMotion_Ctl->set_zone_4th_straight();
       }
       break;


 /* Right Right Right *************************************************************** Line Trace */
     case ENTER_5TH_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
       mMax_Forward = ENTER_5TH_CORNER_FORWARD_VAL;      
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       mMin_Yawrate = MINUS_RAD_22P5_DEG;
       mRef_Yawrate = 0.0;

       if(mVelocity > 300){
	 yaw_time     = CIRCLE_05_LENGTH/mVelocity;
       }else{
	 yaw_time     = CIRCLE_05_LENGTH/300;
       }

       mMax_Yawrate = (CIRCLE_05_ANGLE/yaw_time);
       //------------------------------------------------------REF YAW RATE GEN



       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_x >FIFTH_CORNER_AREA[0]){
	 ZONE = FIFTH_CORNER_ZONE;
	 gMotion_Ctl->set_zone_5th_corner();

	 //LAUNCH SEESAW RUN-------------------------------------------------
	 gMotion_Ctl->set_mode_seesaw();
	 //-------------------------------------------------LAUNCH SEESAW RUN

	//FOR NEXT MAX FORWARD GEN--------------------------------------------
	 ref_forward = mMax_Forward;
	 ref_odo     = mOdo;
	 //--------------------------------------------FOR NEXT MAX FORWARD GEN
	 
       }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_45_DEG)){ //LOST THE LINE
	 DRIVE_MODE = TRACK;
	 ZONE       = FIFTH_CORNER_ZONE;
	 gMotion_Ctl->set_mode_map_trace();
	 gMotion_Ctl->set_zone_5th_corner();
       }
       break;


 /* Right Right Right *************************************************************** Line Trace */
     case FIFTH_CORNER_ZONE:

      //MAX FORWARD GEN-------------------------------------------------------
       mMax_Forward = FIFTH_CORNER_FORWARD_VAL;
      //-------------------------------------------------------MAX FORWARD GEN


      //REF YAW RATE GEN------------------------------------------------------
       mMin_Yawrate = MINUS_RAD_90_DEG;
       mRef_Yawrate = 0;
       mMax_Yawrate = RAD_90_DEG; //koko
       //------------------------------------------------------REF YAW RATE GEN


       //DET RUNNING AREA------------------------------------------------------       
       if (mPre_50mm_y > FIRST_GRAY_AREA[1]){
	 ZONE = FIRST_GRAY_ZONE;
	 gMotion_Ctl->set_zone_1st_gray();

       }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_135_DEG)){ //LOST THE LINE

	 DRIVE_MODE = TRACK;
	 ZONE       = FIFTH_CORNER_ZONE;
	 gMotion_Ctl->set_mode_map_trace();
	 gMotion_Ctl->set_zone_5th_corner();
       }
       break;


 /* Right Right Right *************************************************************** Line Trace */
     case FIRST_GRAY_ZONE:

       mMax_Forward = FIRST_GRAY_FORWARD_VAL;

       mMin_Yawrate = MINUS_RAD_45_DEG;
       mRef_Yawrate = 0;
       mMax_Yawrate = RAD_45_DEG; //koko

       //DET RUNNING AREA------------------------------------------------------
       if (mPre_50mm_y > SEESAW_AREA[1]){
	 ZONE = SEESAW_ZONE;
	 gMotion_Ctl->set_zone_seesaw();
       }else if ((mAve_yaw_angle < MINUS_RAD_45_DEG)||(mAve_yaw_angle > RAD_135_DEG)){ //LOST THE LINE

	 DRIVE_MODE = TRACK;
	 ZONE       = GARAGE_ZONE;
	 gMotion_Ctl->set_mode_map_trace();
	 gMotion_Ctl->set_zone_garage();
       }
       break;

 /* Right Right Right *************************************************************** Line Trace */
     case SEESAW_ZONE:

       mMax_Forward = 60;

       mMin_Yawrate = MINUS_RAD_5_DEG;

       mRef_Yawrate = 0;
       mMax_Yawrate = RAD_5_DEG;

      break;


/* Right Right Right *************************************************************** Line Trace */
    case LUG_ZONE:


      break;


/* Right Right Right *************************************************************** Line Trace */
    case SECOND_GRAY_ZONE:

      break;



/* Right Right Right *************************************************************** Line Trace */
    case GARAGE_ZONE:

      break;

/* Right Right Right *************************************************************** Line Trace */
    case LOST:
	mMax_Forward = 0;
	mRef_Yawrate = 0;

	gMotion_Ctl->set_zone_lost();

      break;


    default:
      break;
    }


/**************************************************************************************************************/
/**  TRACK MODE                                                                                              **/
/**  TRACK MODE                                                                                              **/
/**  TRACK MODE                                                                                              **/
/**  TRACK MODE                                                                                              **/
/**************************************************************************************************************/
  }else if(DRIVE_MODE == TRACK){
    forward_curve_mode = true;
    line_trace_mode = false;

/********************************************/
/* 2018 Right course*/
/********************************************/

    switch(ZONE){

/* Right Right *************************** Right ********************* TRACK */ 
    case START_ZONE:
      
      gMotion_Ctl->set_mode_LT();

      mMax_Forward = 100;
      mMax_Yawrate = RAD_6_DEG;
      mMin_Yawrate = -1.0 * RAD_6_DEG;

      if (det_area(FIRST_STRAIGHT_AREA[0],FIRST_STRAIGHT_AREA[1],FIRST_STRAIGHT_AREA[2],FIRST_STRAIGHT_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FIRST_STRAIGHT_ZONE;
	gMotion_Ctl->set_mode_map_trace();
	gMotion_Ctl->set_zone_1st_straight();
	mMax_Forward = 100;
	mMax_Yawrate =  RAD_6_DEG;
	mMin_Yawrate = -1.0 * RAD_6_DEG;
      }else if (det_area(START_AREA[0],START_AREA[1],START_AREA[2],START_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
    case FIRST_STRAIGHT_ZONE:

      if (det_area(ENTER_1ST_CORNER_AREA[0],ENTER_1ST_CORNER_AREA[1],ENTER_1ST_CORNER_AREA[2],ENTER_1ST_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = ENTER_1ST_CORNER_ZONE;
	//	gMotion_Ctl->set_zone_enter_1st_corner(); //may not be used 180624 kota
      }else if (det_area(FIRST_STRAIGHT_AREA[0],FIRST_STRAIGHT_AREA[1],FIRST_STRAIGHT_AREA[2],FIRST_STRAIGHT_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
      case ENTER_1ST_CORNER_ZONE:

	mMax_Forward = 100;
	mMax_Yawrate = RAD_45_DEG;
	mMin_Yawrate = -1.0 * RAD_45_DEG;

      if (det_area(FIRST_CORNER_AREA[0],FIRST_CORNER_AREA[1],FIRST_CORNER_AREA[2],FIRST_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FIRST_CORNER_ZONE;
	gMotion_Ctl->set_zone_1st_corner();
      }else if (det_area(ENTER_1ST_CORNER_AREA[0],ENTER_1ST_CORNER_AREA[1],ENTER_1ST_CORNER_AREA[2],ENTER_1ST_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
      case FIRST_CORNER_ZONE:

	mMax_Forward = 100;
	mMax_Yawrate = RAD_45_DEG;
	mMin_Yawrate = -1.0 * RAD_45_DEG;

      if (det_area(SECOND_STRAIGHT_AREA[0],SECOND_STRAIGHT_AREA[1],SECOND_STRAIGHT_AREA[2],SECOND_STRAIGHT_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = SECOND_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_2nd_straight();
      }else if (det_area(FIRST_CORNER_AREA[0],FIRST_CORNER_AREA[1],FIRST_CORNER_AREA[2],FIRST_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
      case SECOND_STRAIGHT_ZONE:

	mMax_Forward = 100;
	mMax_Yawrate =  RAD_6_DEG;
	mMin_Yawrate = -1.0 * RAD_6_DEG;

      if (det_area(ENTER_2ND_CORNER_AREA[0],ENTER_2ND_CORNER_AREA[1],ENTER_2ND_CORNER_AREA[2],ENTER_2ND_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = ENTER_2ND_CORNER_ZONE;
	gMotion_Ctl->set_zone_enter_2nd_corner();
      }else if (det_area(SECOND_STRAIGHT_AREA[0],SECOND_STRAIGHT_AREA[1],SECOND_STRAIGHT_AREA[2],SECOND_STRAIGHT_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
      case ENTER_2ND_CORNER_ZONE:

	mMax_Forward = 100;
	mMax_Yawrate =  RAD_90_DEG;
	mMin_Yawrate = -1.0 * RAD_90_DEG;

      if (det_area(SECOND_CORNER_AREA[0],SECOND_CORNER_AREA[1],SECOND_CORNER_AREA[2],SECOND_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = SECOND_CORNER_ZONE;
	gMotion_Ctl->set_zone_2nd_corner();
      }else if (det_area(ENTER_2ND_CORNER_AREA[0],ENTER_2ND_CORNER_AREA[1],ENTER_2ND_CORNER_AREA[2],ENTER_2ND_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
      case SECOND_CORNER_ZONE:

	mMax_Forward = 100;
	mMax_Yawrate =  RAD_90_DEG;
	mMin_Yawrate = -1.0 * RAD_90_DEG;

      if (det_area(THIRD_CORNER_AREA[0],THIRD_CORNER_AREA[1],THIRD_CORNER_AREA[2],THIRD_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = THIRD_CORNER_ZONE;
	gMotion_Ctl->set_zone_3rd_corner();
      }else if (det_area(SECOND_CORNER_AREA[0],SECOND_CORNER_AREA[1],SECOND_CORNER_AREA[2],SECOND_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
      case THIRD_CORNER_ZONE:

	mMax_Forward = 100;
	mMax_Yawrate =  RAD_90_DEG;
	mMin_Yawrate = -1.0 * RAD_90_DEG;

      if (det_area(THIRD_STRAIGHT_AREA[0],THIRD_STRAIGHT_AREA[1],THIRD_STRAIGHT_AREA[2],THIRD_STRAIGHT_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = THIRD_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_3rd_straight();
      }else if (det_area(THIRD_CORNER_AREA[0],THIRD_CORNER_AREA[1],THIRD_CORNER_AREA[2],THIRD_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
      case THIRD_STRAIGHT_ZONE:

	mMax_Forward = 100;
	mMax_Yawrate =  RAD_6_DEG;
	mMin_Yawrate = -1.0 * RAD_6_DEG;

      if (det_area(FOURTH_CORNER_AREA[0],FOURTH_CORNER_AREA[1],FOURTH_CORNER_AREA[2],FOURTH_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FOURTH_CORNER_ZONE;
	gMotion_Ctl->set_zone_4th_corner();
      }else if (det_area(THIRD_STRAIGHT_AREA[0],THIRD_STRAIGHT_AREA[1],THIRD_STRAIGHT_AREA[2],THIRD_STRAIGHT_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
      case FOURTH_CORNER_ZONE:

	mMax_Forward = 50;
	mMax_Yawrate =  RAD_90_DEG;
	mMin_Yawrate = -1.0 * RAD_90_DEG;

      if (det_area(FOURTH_STRAIGHT_AREA[0],FOURTH_STRAIGHT_AREA[1],FOURTH_STRAIGHT_AREA[2],FOURTH_STRAIGHT_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FOURTH_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_4th_straight();
      }else if (det_area(FOURTH_CORNER_AREA[0],FOURTH_CORNER_AREA[1],FOURTH_CORNER_AREA[2],FOURTH_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;




/* Right Right *************************** Right ********************* TRACK */ 
      case FOURTH_STRAIGHT_ZONE:

	mMax_Forward = 100;
	mMax_Yawrate =  RAD_6_DEG;
	mMin_Yawrate = -1.0 * RAD_6_DEG;

      if (det_area(FIFTH_CORNER_AREA[0],FIFTH_CORNER_AREA[1],FIFTH_CORNER_AREA[2],FIFTH_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FIFTH_CORNER_ZONE;
	gMotion_Ctl->set_zone_5th_corner();
      }else if (det_area(FOURTH_STRAIGHT_AREA[0],FOURTH_STRAIGHT_AREA[1],FOURTH_STRAIGHT_AREA[2],FOURTH_STRAIGHT_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;


/* Right Right *************************** Right ********************* TRACK */ 
      case FIFTH_CORNER_ZONE:

	mMax_Forward = 50;
	mMax_Forward = 0;
	mMax_Yawrate =  RAD_90_DEG;
	mMin_Yawrate = -1.0 * RAD_90_DEG;

      if (det_area(FIRST_GRAY_AREA[0],FIRST_GRAY_AREA[1],FIRST_GRAY_AREA[2],FIRST_GRAY_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FIRST_GRAY_ZONE;
	//	gMotion_Ctl->set_zone_1st_gray();
	gMotion_Ctl->set_zone_garage();

      }else if (det_area(FIFTH_CORNER_AREA[0],FIFTH_CORNER_AREA[1],FIFTH_CORNER_AREA[2],FIFTH_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }


      break;


/* Right Right *************************** Right ********************* TRACK */ 
      case FIRST_GRAY_ZONE:

	mMax_Forward = 30;
	mMax_Yawrate =  RAD_6_DEG;
	mMin_Yawrate = -1.0 * RAD_6_DEG;

	//      if (det_area(LUG_AREA[0],LUG_AREA[1],LUG_AREA[2],LUG_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	if(mPre_50mm_y > SEESAW_AREA[1]){
	ZONE = SEESAW_ZONE;
	gMotion_Ctl->set_mode_seesaw();

      }else if (det_area(FIRST_GRAY_AREA[0],FIRST_GRAY_AREA[1],FIRST_GRAY_AREA[2],FIRST_GRAY_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;


/* Right Right *************************** Right ********************* TRACK */ 
      case LUG_ZONE:


      break;


/* Right Right *************************** Right ********************* TRACK */ 
      case SECOND_GRAY_ZONE:

      break;



/* Right Right *************************** Right ********************* TRACK */ 
      case GARAGE_ZONE:
      if (det_area(GARAGE_AREA[0],GARAGE_AREA[1],GARAGE_AREA[2],GARAGE_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/* Right Right *************************** Right ********************* TRACK */ 
    case LOST:
	mMax_Forward =    0;
	mMax_Yawrate =  0.0;
	mMin_Yawrate = -0.0;
	gMotion_Ctl->set_zone_lost();

      break;


    default:
      break;
    }
/**2018 Right course ***************************/

  }else if(DRIVE_MODE == DEBUG){
     //    forward_curve_mode = true;
    forward_curve_mode = false;
    //    line_trace_mode    = true;
    line_trace_mode    = false;
    
    gMotion_Ctl->set_mode_debug();
    mMax_Forward =  40;
    /*
    mMin_Yawrate = MINUS_RAD_90_DEG;
    mRef_Yawrate = 0;
    mMax_Yawrate = RAD_90_DEG; //koko
    */
    mMin_Yawrate = MINUS_RAD_45_DEG;
    mRef_Yawrate = 0;
    mMax_Yawrate = RAD_45_DEG; //koko

  }else{
    mMax_Forward = 200;
    mMax_Yawrate = 0;
    mMin_Yawrate = 0;
  }

}

/****************************************************************************************/
//2018 04 21 Kaoru Ota
//
/****************************************************************************************/

bool Ang_Brain::det_area(float x_left, float y_under, float x_right, float y_top, float x_value, float y_value){
  if(x_left < x_value && x_value <= x_right && y_under < y_value && y_value <= y_top){
    return true;
  }else{
    return false;
  }
}

void Ang_Brain::setEyeCommand(int     linevalue,
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
			      float ave_wheel_load,
			      // ---- 20181015

                              float   pre_velo_0p5sec,
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
                              int16_t sonar_dis){

  mLinevalue       = linevalue;
  mGreen_flag      = green_flag;
  mXvalue          = xvalue;
  mYvalue          = yvalue;
  mPre_50mm_x      = pre_50mm_x;//50mm saki 20180512 kota
  mPre_50mm_y      = pre_50mm_y;//50mm saki 20180512 kota

  //20181015 -----
  mAve_x           = ave_x;
  mAve_y           = ave_y;
  mAve_vel_x       = ave_vel_x;
  mAve_vel_y       = ave_vel_y;
  // ---- 20181015

  mOdo             = odo; 
  mVelocity        = velocity;

  //20181015 ----
  mAve_velo        = ave_velo;
  mAve_accel       = ave_accel;
  mAve_wheel_load  = ave_wheel_load;
  // ---- 20181015

  mPre_velo_0p5sec = pre_velo_0p5sec;
  mYawrate         = yawrate;
  mYawangle        = abs_angle;
  mAve_yaw_angle   = ave_angle;

  mTail_angle      = robo_tail_angle;
  mRobo_stop       = robo_stop;
  mRobo_forward    = robo_forward;
  mRobo_back       = robo_back;
  mRobo_turn_left  = robo_turn_left;
  mRobo_turn_right = robo_turn_right;
  mDansa           = dansa;
  mSonar_dis       = sonar_dis;
}


void Ang_Brain::setRoboCommand(bool robo_balance_mode, bool robo_lug_mode){
  mRobo_balance_mode = robo_balance_mode;
  mRobo_lug_mode     = robo_lug_mode;
}

void Ang_Brain::GetCalcResult(int   forward_calc,
			      float yawratecmd_calc,
			      float ref_tail_angle_calc,
			      bool  tail_stand_mode_calc,
			      bool  tail_lug_mode_calc,
			      //			      bool  gyro_offset_plus_calc,
			      bool  gyro_offset_en_calc,
			      //			      bool  falling_seesaw_calc,
			      //			      bool  hilling_seesaw_calc){
			      int  gyro_offset_value_calc,
			      bool pwm_ctl_en_calc,
			      int  pwm_ctl_value_calc){
  
  forward          = forward_calc;
  yawratecmd       = yawratecmd_calc;
  ref_tail_angle   = ref_tail_angle_calc;
  tail_stand_mode  = tail_stand_mode_calc;
  tail_lug_mode    = tail_lug_mode_calc;
  //  gyro_offset_plus = gyro_offset_plus_calc;
  gyro_offset_en = gyro_offset_en_calc;
  gyro_offset_value = gyro_offset_value_calc;
  //  falling_seesaw   = falling_seesaw_calc;
  //  hilling_seesaw   = hilling_seesaw_calc;
  pwm_ctl_en        = pwm_ctl_en_calc;
  pwm_ctl_value     = pwm_ctl_value_calc;

}


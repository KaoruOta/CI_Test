/******************************************************************************
 *  ang_brain.h (for LEGO Mindstorms EV3)
 *  Created on: 2018/04/21
 *  Implementation of the Class ang_brain
 *  Author: Kaoru Ota
 *****************************************************************************/

#ifndef EV3_APP_ANAGOBRAIN_H_
#define EV3_APP_ANAGOBRAIN_H_
#include "motion_ctl.hpp"

using namespace std;

class Ang_Brain {
public:
  explicit Ang_Brain();//コンストラクタ
  void init();

  void set_drive_mode_LT();
  void set_drive_mode_TK();
  void set_drive_mode_DB();

  void run();
  void setEyeCommand(int     linevalue,
		     bool    green_flag,
		     float   xvalue,
		     float   yvalue,
		     float   pre_50mm_x,
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
		     //			   float   yawangle,
		     float   abs_angle,
		     float   ave_angle,
		     int     robo_tail_angle,
		     bool    robo_stop,
		     bool    robo_forward,
		     bool    robo_back,
		     bool    robo_turn_left,
		     bool    robo_turn_right,
		     bool    dansa,
		     int16_t sonar_dis);

  void setRoboCommand(bool robo_balance_mode, bool robo_lug_mode);

  Motion_Ctl *gMotion_Ctl   = new Motion_Ctl();

  bool  line_trace_mode;
  //あなご手足に渡す変数
  int   forward;         //前進目標値
  float yawratecmd;      //目標ヨーレート
  float ref_tail_angle;    //尻尾角度
  bool  forward_curve_mode;
  bool  tail_stand_mode; //倒立走行フラグ
  bool  tail_lug_mode;
  //  bool  gyro_offset_plus;

  bool  gyro_offset_en;
  int   gyro_offset_value;

  bool  pwm_ctl_en;
  int   pwm_ctl_value;

  //  bool  falling_seesaw;
  //  bool  hilling_seesaw;
  int   log_dat;

  bool re_start; //20181112

private:
  void det_navigation();

  bool det_area(float x_left, float y_under, float x_right, float y_top, float x_value, float y_value);


  void GetCalcResult(int forward_calc,
		     float yawratecmd_calc,
		     float ref_tail_angle_calc,
		     bool  tail_stand_mode_calc,
		     bool  tail_stand_lug_calc,
		     bool  gyro_offset_en_calc,
		     //		     bool  falling_seesaw_calc,
		     //		     bool  hilling_seesaw_calc);
		     int  gyro_offset_value_calc,
		     bool pwm_ctl_en_calc,
		     int  pwm_ctl_value_clc);

  //    StrategyDet *gStrategyDet = new StrategyDet();


  int   Mmode;
  int   mLinevalue;  //ライン検出値
  bool  mGreen_flag;  //ライン検出値

  float mXvalue;     //x座標
  float mYvalue;     //y座標

  float mPre_50mm_x;//50mm saki 20180512 kota
  float mPre_50mm_y;//50mm saki 20180512 kota

  //20181015 -----
  float mAve_x;
  float mAve_y;
  float mAve_vel_x;
  float mAve_vel_y;
  // ---- 20181015

  float mOdo;        //Total Distance from Start point
  float mVelocity;   //速度

  //20181015 ----
  float mAve_velo;
  float mAve_accel;
  float mAve_wheel_load;
  // ---- 20181015

  float mPre_velo_0p5sec;
  float mYawrate;    //ヨーレート
  float mYawangle;   //ヨー角
  float mAve_yaw_angle;

  int   mTail_angle;
  //signals for robo movement
  bool  mRobo_stop       = false;
  bool  mRobo_forward    = false;
  bool  mRobo_back       = false;
  bool  mRobo_turn_left  = false;
  bool  mRobo_turn_right = false;
  bool  mDansa;      //段差検出値

  int16_t mSonar_dis;
  bool  mRobo_balance_mode;
  bool  mRobo_lug_mode;

  int   mMax_Forward;

  float mRef_Yawrate;
  float mMax_Yawrate;
  float mMin_Yawrate;

  enum Zone{
    START_ZONE,
    START_BACK,
    FIRST_STRAIGHT_ZONE,
    ENTER_1ST_CORNER_ZONE,
    FIRST_CORNER_ZONE,
    SECOND_STRAIGHT_ZONE,
    ENTER_2ND_CORNER_ZONE,
    SECOND_CORNER_ZONE,
    THIRD_STRAIGHT_ZONE,
    THIRD_CORNER_ZONE,
    S_CORNER_ZONE,
    FOURTH_STRAIGHT_ZONE,
    FOURTH_CORNER_ZONE,
    ENTER_5TH_CORNER_ZONE,
    FIFTH_CORNER_ZONE,
    FIRST_GRAY_ZONE,
    LUG_ZONE,
    BACK_LUG_ZONE,
    SECOND_GRAY_ZONE,
    SEESAW_ZONE,
    GARAGE_ZONE,
    LOST
  };

  enum Drive_Mode{
    LINE_TRACE,
    TRACK,
    DEBUG
  };


  Zone       ZONE;
  Drive_Mode DRIVE_MODE;


};

#endif  // EV3_APP_ANAGOBRAIN_H_

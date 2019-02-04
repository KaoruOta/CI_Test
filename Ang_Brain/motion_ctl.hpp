#include "parameter.h"
#include "util.hpp"

class Motion_Ctl {
public:
  explicit Motion_Ctl();//コンストラクタ

  void init ();
  void SetCurrentData(int     linevalue,
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
		      );

  void set_mode_LT();
  void set_mode_map_trace();
  void set_mode_LUG();
  void set_mode_seesaw();
  
  void set_mode_tail_std_debug();
  void set_mode_debug();

  void set_zone_start();
  void set_zone_1st_straight();
  void set_zone_enter_1st_corner();
  void set_zone_1st_corner();
  void set_zone_2nd_straight();
  void set_zone_enter_2nd_corner();
  void set_zone_2nd_corner();
  void set_zone_3rd_straight();
  void set_zone_3rd_corner();
  void set_zone_4th_straight();
  void set_zone_4th_corner();
  void set_zone_5th_corner();
  void set_zone_1st_gray();
  void set_zone_lug();
  void set_zone_back_lug();
  void set_zone_2nd_gray();
  void set_zone_seesaw();
  void set_zone_garage();
  void set_zone_lost();

  void run(float xvalue, float yvalue, float yawangle);//走行戦略を計算

  
  bool  left_line_edge = true; //non used now, 
  int   forward;
  float yawratecmd;
  float ref_tail_angle;
  bool  tail_stand_mode;
  bool  tail_lug_mode;
  bool  gyro_offset_en;
  int   gyro_offset_value;
  bool  pwm_ctl_en;
  int   pwm_ctl_value;

  //  bool  falling_seesaw;
  //  bool  hilling_seesaw;


  //SEESAW VARIABLE
  int   seesaw_log;
  //  static float ref_x;
  float ref_y;
  float ref_odo;
  float edge_odo;
  float dash_pos;
  float get_on_pos;
  float stop_pos_1st;
  float stop_pos_2nd;
  float stop_pos_3rd;

  float axis_pos;
  float seesaw_end_pos;
  float garage_pos;

  float ref_angle;
  float max_speed;

  //  float load_00;
  float load_01;
  float ref_load;
  //  float max_load;
  //  float min_load;
  bool  det_slip;
  bool  det_seesaw_edge; 
  bool  det_get_on; 
  bool  fail_get_on; 

  //  bool  det_fall_seesaw;
  bool  det_down_seesaw;
  bool  fail_fall_seesaw;

  //  bool  det_rise_seesaw;
  bool  det_up_seesaw;
  bool  fail_rise_seesaw;

  bool  drop_down;
  bool motor_break_mode;

private:
  void LineTracerYawrate(int line_value);              //ライントレース（ヨーレート）
  void MapTracer(float mXvalue, float mYvalue, float mYawangle);  //仮想ゲート走行 0827 tada


  void seesaw_run();



  PID *gForward = new PID();
  Average_500_Data *gAve_500_dat  = new Average_500_Data();
  Average_125_Data *gAve_125_dat  = new Average_125_Data();


    enum Motion_Mode{
      MAP_TRACE,
      LINE_TRACE,
      LUG,
      SEESAW,
      TAIL_STAND_DEBUG,
      DEBUG,
      DEBUG_1
    };
    
    enum Zone{
      START_ZONE,
      FIRST_STRAIGHT_ZONE,
      ENTER_1ST_CORNER_ZONE,
      FIRST_CORNER_ZONE,
      SECOND_STRAIGHT_ZONE,
      ENTER_2ND_CORNER_ZONE,
      SECOND_CORNER_ZONE,
      THIRD_STRAIGHT_ZONE,
      THIRD_CORNER_ZONE,
      FOURTH_STRAIGHT_ZONE,
      FOURTH_CORNER_ZONE,
      FIFTH_CORNER_ZONE,
      FIRST_GRAY_ZONE,
      LUG_ZONE,
      BACK_LUG_ZONE,
      SECOND_GRAY_ZONE,
      SEESAW_ZONE,
      GARAGE_ZONE,
      LOST
    };

    enum Lug_Mode{
      LUG_START,
      APPROACH_TO_LUG,
      TAIL_ON_1ST,
      POS_ADJ_1ST,
      LUG_MODE_1ST,
      LUG_1ST,
      PRE_1ST_TURN,
      TURN_1ST,

      APPROACH_TO_2ND_LUG,
      LUG_MODE_2ND,
      LUG_2ND,
      PRE_2ND_TURN,
      TURN_2ND,

      APPROACH_TO_3RD_LUG,
      LUG_MODE_3RD,
      LUG_3RD,

      TAIL_STAND_UP,
      TURN_90,
      FIND_GREEN,
      TURN_M90,
      FIND_LEFT_EDGE,
      Y_POS_ADJ,
      TURN_ZERO,
      GRAY_GARAGE,

      LUG_DEBUG_00
    };

  enum Seesaw_Mode{
    SEESAW_START,
    CORRECT_ANGLE,
    SET_POS,
    APPROACH_TO_SEESAW,
    GET_ON_SEESAW,
    GO_UP_1ST_SEESAW,
    STOP_1ST_SEESAW,
    GO_UP_2ND_SEESAW,
    STOP_2ND_SEESAW,
    GO_UP_3RD_SEESAW,
    STOP_3RD_SEESAW,
    GET_OFF_SEESAW,
    STOP_GARAGE,
    STOP_GARAGE_TAIL,
    TEST_01,
    DONE


  };



  Motion_Mode MOTION_MODE;
  Zone        ZONE;
  Lug_Mode    LUG_MODE;
  Seesaw_Mode SEESAW_MODE;

  int   Mmode;
  int   mLinevalue; //ライン検出値
  bool  mGreen_flag;

  float mXvalue;    //x座標
  float mYvalue;    //y座標

  float mPre_50mm_x;//50mm saki 20180512 kota
  float mPre_50mm_y;//50mm saki 20180512 kota

  //20181015 -----
  float mAve_x;
  float mAve_y;
  float mAve_vel_x;
  float mAve_vel_y;
  // ---- 20181015

  float mOdo;       //Total distance [mm] from start point
  float mVelocity;     //速度

  //20181015 ----
  float mAve_velo;
  float mAve_accel;
  float mPre_velo_0p5sec;
  float mAve_wheel_load;
  // ---- 20181015

  float mYawrate;   //ヨーレート
  float mYawangle;  //ヨー角
  float mAve_yaw_angle;
  int   mTail_angle;
  float mYaw_angle_offset;

  float ref_x;


  //signals for robo movement
  bool  mRobo_stop       = false;
  bool  mRobo_forward    = false;
  bool  mRobo_back       = false;
  bool  mRobo_turn_left  = false;
  bool  mRobo_turn_right = false;

  bool    mDansa;      //段差検出値
  int16_t mSonar_dis;

  bool  mRobo_balance_mode;
  bool  mRobo_lug_mode;

  int   mMax_Forward;
  float mRef_Yawrate;
  float mMax_Yawrate;
  float mMin_Yawrate;
  
  float y_t;
  float y_t_prev; //0818 tada. passed y_t
  float pg = 0.35;//暫定0.9 //0818 tada
  float df = 0.024;//暫定-0.1 //0818 tada
  int   bat_mv;

  bool  seesaw_debug;
};

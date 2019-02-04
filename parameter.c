#define SEESAW_SINGLE
//#define SEESAW_DOUBLE


/**** SEE           ****/
/**** ADJ_PARAMETER ****/

//Parameter of Robo

int TAIL_ANGLE_STAND_UP       = 97; /* 完全停止時の角度[度]     */
//int TAIL_ANGLE_BALANCE_START  = 100;
int TAIL_ANGLE_BALANCE_START  = 99;
int TAIL_ANGLE_LAUNCH         = 105;

int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ON_ANGLE       = 85; /* 完全停止時の角度[度]     */
//int TAIL_ANGLE_LUG      = 75; /* 3点移動時の角度[度]      */
//int TAIL_ANGLE_LUG      = 70; /* 3点移動時の角度[度]      */
//int TAIL_ANGLE_LUG      = 65; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_LUG      = 68; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_GARAGE   = 94; /* 完全停止時の角度[度]     */

float WHEEL_R       = 49.75; //Wheel radius 2018
int   RoboTread     = 184; //トレッド長さ[mm]

float MAX_VELOCITY      = 400; /**** ADJ_PARAMETER ****/
//float MAX_VELOCITY      = 450; /**** ADJ_PARAMETER ****/

//Parameter of time length unit
float dT_4ms   = 0.004;

float PAI         =  3.1472;

float RAD_1_DEG    = 0.0175; //deg@1rad 
float RAD_5_DEG    = 0.0873; //
float RAD_6_DEG    = 0.1047; //
float RAD_15_DEG   = 0.2618; //
float RAD_22P5_DEG = 0.3927; //
float RAD_30_DEG   = 0.5236; //
float RAD_45_DEG   = 0.7854; //
float RAD_89_DEG   = 1.5533; //
float RAD_88p5_DEG = 1.5446; //
float RAD_87_DEG   = 1.5184; //
float RAD_90_DEG   = 1.5708; //
float RAD_120_DEG  = 2.0944; //
float RAD_135_DEG  = 2.3562; //
float RAD_150_DEG  = 2.6180; //
float RAD_180_DEG  = 3.1472; //
float RAD_225_DEG  = 3.9270; //
float RAD_270_DEG  = 4.7124;
float RAD_315_DEG  = 5.4978; //
float RAD_345_DEG  = 6.0214; //
float RAD_360_DEG  = 6.2832; //
float RAD_450_DEG  = 7.8540;

float MINUS_RAD_5_DEG    = -0.0873; //
float MINUS_RAD_15_DEG   = -0.2618; //
float MINUS_RAD_22P5_DEG = -0.3927; //
float MINUS_RAD_30_DEG   = -0.5236; //
float MINUS_RAD_45_DEG   = -0.7854; //
float MINUS_RAD_60_DEG   = -1.0472; //
float MINUS_RAD_90_DEG   = -1.5708; //
float MINUS_RAD_135_DEG  = -2.3562; //
float MINUS_RAD_145_DEG  = -2.5307; //
float MINUS_RAD_180_DEG  = -3.1472; //
float MINUS_RAD_225_DEG  = -3.9270; //
float MINUS_RAD_270_DEG  = -4.7124;

float YAW_LIMIT    = 0.393;   //PI/8 see anago synario
float YAW_STEP     = 0.00786; //YAW_LIMIT/50 see anago synario

//LUG
int   MAX_LUG_FORWARD        = 30;

//map data
//https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
float LUG_1st_STOP_X         = 4590; /**** ADJ_PARAMETER ****/
float LUG_2nd_STOP_X         = 4190; /**** ADJ_PARAMETER ****/
float LUG_3rd_STOP_X         = 4590; /**** ADJ_PARAMETER ****/

float LUG_YAW_GAIN           = 2.0;
//int   TAIL_STD_LINE_DET      = 49; /**** ADJ_PARAMETER ****/

//int   TAIL_STD_LINE_DET      = 70; /**** ADJ_PARAMETER ****/
//int   TAIL_STD_LINE_DET      = 50; /**** ADJ_PARAMETER ****/
int   TAIL_STD_LINE_DET      = 72; /**** ADJ_PARAMETER ****/

int   CALIB_LINE_100_MAX_THRS = 20;
int   CALIB_LINE_50_MAX_THRS  = 100;
int   CALIB_LINE_50_MIN_THRS  = 40;
int   CALIB_LINE_0_MIN_THRS   = 80;

//HONBAN
int   COLOR_SENSOR_OFFSET    = 17; /**** ADJ_PARAMETER ****/
float COLOR_SENSOR_GAIN      = 2.9; /**** ADJ_PARAMETER ****/

int   DET_GRAY_LINE          = 10; /**** ADJ_PARAMETER ****/
//int   DET_GRAY_LINE          = 30; /**** ADJ_PARAMETER ****/

//YOBI
//int   COLOR_SENSOR_OFFSET    = 10; /**** ADJ_PARAMETER ****/
//float COLOR_SENSOR_GAIN      = 1.5; /**** ADJ_PARAMETER ****/

//Parameter of Seesaw
float LOAD_GAIN          = 1.5; //koko

int SEESAW_SET_POS       = 500;
int SEESAW_DASH_POS      = 250; //1113 ok
int SEESAW_GET_ON_POS    = 100;

int SEESAW_1ST_POS       = 420;
int SEESAW_2ND_POS       = 420;
int SEESAW_3RD_POS       = 650;
int SEESAW_STOP_POS      = 800;

int SEESAW_DASH_GYRO_OFFSET = -15; //1113 ok
int SEESAW_DASH_FORWARD     =  80;

int SEESAW_1ST_GYRO_OFFSET      = -10;
int SEESAW_1ST_FORWARD          =  40;
int SEESAW_1ST_STOP_TIME        = 500;

#ifdef SEESAW_SINGLE
int SEESAW_1ST_STOP_GYRO_OFFSET =  0;
int SEESAW_1ST_STOP_FORWARD     = 50;
#endif

#ifdef SEESAW_DOUBLE
int SEESAW_1ST_STOP_GYRO_OFFSET =  15;
int SEESAW_1ST_STOP_FORWARD     = -40;
#endif

int SEESAW_2ND_GYRO_OFFSET      =  10;
int SEESAW_2ND_FORWARD          = -40;
int SEESAW_2ND_STOP_TIME        = 500;
int SEESAW_2ND_STOP_GYRO_OFFSET = -15;
int SEESAW_2ND_STOP_FORWARD     =  80;

int SEESAW_3RD_GYRO_OFFSET  = -15;
int SEESAW_3RD_FORWARD      =  40;

int SEESAW_FALL_DET_ACCEL   = 200;
//int SEESAW_RISE_DET_ACCEL   = -200;


//Parameter of Garage
//float GARAGE_X_POS          = 4980;
float GARAGE_X_POS          = 4990; /**** ADJ_PARAMETER ****/


//map data
//https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
//(x0,y0) - (x1,y1)

//int START_FORWARD_VAL            = 180; //koko
int START_FORWARD_VAL            = 30; //koko

int FIRST_STRAIGHT_FORWARD_VAL   = 180;
int ENTER_1ST_CORNER_FORWARD_VAL = 120;
int FIRST_CORNER_FORWARD_VAL     = 120;
int SECOND_STRAIGHT_FORWARD_VAL  =  70;
int S_CORNER_FORWARD_VAL         =  70;
int THIRD_STRAIGHT_FORWARD_VAL   = 120;
int FOURTH_CORNER_FORWARD_VAL    = 100;
int FOURTH_STRAIGHT_FORWARD_VAL  = 180;
int ENTER_5TH_CORNER_FORWARD_VAL =  60;
int FIFTH_CORNER_FORWARD_VAL     =  40;

int ENTER_2ND_CORNER_FORWARD_VAL =  70;
int SECOND_CORNER_FORWARD_VAL    =  70;
int THIRD_CORNER_FORWARD_VAL     = 70;
int GOAL_VAL                     = 100;
int FIRST_GRAY_FORWARD_VAL       =  60;
int LUG_FORWARD_VAL              = 100;
int BACK_LUG_FORWARD_VAL         = 100;
int SECOND_GRAY_FORWARD_VAL      = 100;
int SEESAW_FORWARD_VAL           = 100; //
int GARAGE_FORWARD_VAL           = 100;

float ACCEL_GAIN               = 1.0;
float DECEL_GAIN               = 1.0;

float START_AREA[4]            = { 300,    0,  500,  490};
float FIRST_STRAIGHT_AREA[4]   = { 500,    0, 2400,  600};
float ENTER_1ST_CORNER_AREA[4] = {2100,    0, 2400,  600};
float FIRST_CORNER_AREA[4]     = {2400,    0, 3500, 2000};
float SECOND_STRAIGHT_AREA[4]  = {1960, 1250, 2400, 2000};
float ENTER_2ND_CORNER_AREA[4] = {1960, 1250, 2100, 2000};

float SECOND_CORNER_AREA[4]    = {1200, 1250, 1960, 2000};
float THIRD_CORNER_AREA[4]     = {1200,  600, 1960, 1250};

float S_CORNER_AREA[4]         = {1200,  600, 1960, 2000};

float THIRD_STRAIGHT_AREA[4]   = { 520,  600, 1200, 1250};
float FOURTH_CORNER_AREA[4]    = {   0,  600,  520, 2000};
float FOURTH_STRAIGHT_AREA[4]  = { 520, 1250, 3090, 2000};
//float ENTER_5TH_CORNER_AREA[4] = {2790, 1250, 3090, 2000};
float ENTER_5TH_CORNER_AREA[4] = {2590, 1250, 3090, 2000};

float FIFTH_CORNER_AREA[4]     = {3090, 1250, 4000, 1970};

float FIRST_GRAY_AREA[4]       = {3090, 1970, 4000, 2120};
float LUG_AREA[4]              = {0,0,0,0};
float BACK_LUG_AREA[4]         = {0,0,0,0};
float SECOND_GRAY_AREA[4]      = {0,0,0,0};
float SEESAW_AREA[4]           = {3090, 2280, 4000, 2880};
float GARAGE_AREA[4]           = {3090, 2880, 4000, 3250};

float SEESAW_EDGE[2]           = {3430, 2370};


//straigt (x0,y0) - (x1,y1)
//circle (x0,y0,r)
float STRAIGT_01[4] = {   0,  490, 2400, 490};
float CIRCLE_01[3]  = {2400, 1060,  570};
float STRAIGT_02[4] = {2400, 1630, 1960, 1630};
float CIRCLE_02[3]  = {1960, 1250,  380};
float CIRCLE_03[3]  = {1200, 1250,  380};
float STRAIGT_03[4] = {1200,  870,  520,  870};
float CIRCLE_04[3]  = { 520, 1250,  380};
float STRAIGT_04[4] = { 520, 1630, 3090, 1630};
float CIRCLE_05[3]  = {3090, 1970,  340};
float STRAIGT_05[4] = {3430, 1970, 3430, 3250};

float CIRCLE_01_LENGTH = 1791;
float CIRCLE_01_ANGLE  = 3.141593;

float CIRCLE_02_LENGTH = 597;
float CIRCLE_02_ANGLE  = 1.570796;

float CIRCLE_03_LENGTH = 597;
float CIRCLE_03_ANGLE  = -1.570796;

float CIRCLE_04_LENGTH = 1194;
float CIRCLE_04_ANGLE  = -3.14159;

float CIRCLE_05_LENGTH = 534;
float CIRCLE_05_ANGLE  = 1.570796;


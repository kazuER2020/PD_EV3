#include <stdlib.h>  // abs関数を使用

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// 共通定数:
#define COUNT 10000
#define INTEGER_MAX 1000000000                                         						
#define VOLUME 100
#define TEMPO 200

// 各閾値:
#define DETECT_RANGE  10  // 超音波の反応する距離
#define HOLD_DISTANCE 160 // ペットボトルを持ったまま置き場所へ移動する距離(角度指定)
#define DEG90         165 // 90度まわるときの角度値
#define HAND_POWER    100  // アーム上下の出力値 
#define EDGE_CHANGE   10000 // エッジ切り替えを行うエンコーダ値

// 流す音楽:
#define MUSIC_1 music12
#define MUSIC_2 music5
#define MUSIC_3 music6
#define MUSIC_4 music4
#define MUSIC_5 music1


// センサ・モータ定義:
/* センサ */
#define COLOR_PORT EV3_PORT_1
#define ULTRASONIC_PORT EV3_PORT_2
/* モータ */
#define MOTOR_HAND EV3_PORT_A
#define MOTOR_R    EV3_PORT_B
#define MOTOR_L    EV3_PORT_C

#define KP 0.26
#define KI 0.29
#define KD 1
#define SPEED 100

// #ifで使う:
#define BEEP 1   // ブザー音

/* 共通の変数: */
int mode = 0;  /* ロボットの動作状態 */

long lEncoder[3] = {
/* スタート場所からつかむまでのエンコーダ値 */
  0, 
/* つかんでから置くまでのエンコーダ値     */
  0,
/* 予備                          */ 
  0
};
/* ペットボトルの色 */
colorid_t object_color = COLOR_NONE; // 初期状態は"色なし"と判断
colorid_t c;
/* タイマ用[1ms] */
unsigned long cnt0 = 0;
unsigned long cnt1 = 0;
unsigned long cnt2 = 0;
unsigned long cnt3 = 0;
unsigned int iTimer10 = 0;

/* uc流す用 */
int ucFlag = 0;
memfile_t memfile;

/* ライントレース用変数: */
rgb_raw_t rgb_val;            /* ラインの各要素(r,g,b)の値 */
int iServoPwm = 0;            /* ステアリング比 */
int iSensorBefore;            /* 前回のセンサ値保存           */
int iSensor0 = 0;             /* センサの初期値 */
int iSensor0_raw[3];
int iGray = 0;
long enc_avg  = 0;  // 各エンコーダの平均値

unsigned long cnt_sensorRate = 0; /* センサ取得間隔(デフォルトで1[ms]) */
float lasterror = 0, integral = 0, error = 0, steer = 0;

/* 液晶に出す文字列 */
char statusInfo[100] = {" "};  // 現在の状態
char lcdPattern[100] = {" "};  // 現在のモード番号

void main_task(intptr_t unused) {
  init_IO();
  init_music_info();
  ev3_sta_cyc(TEST_EV3_CYC1);
  act_tsk(SUB_TASK);
  
  cnt_sensorRate = 0;
  object_color = COLOR_NONE;
  speed_dir(0,0);
  ev3_led_set_color(LED_OFF);

  resetEncoder(MOTOR_L);
  resetEncoder(MOTOR_R);
  resetEncoder(MOTOR_HAND);

  // TODO: Calibrate using maximum mode => 100
  RingTone(NOTE_C5, 100, VOLUME);
  ev3_lcd_draw_string("WHITE::set     \n", 0, 10);  // LCDに現在の状態を定義
  while(!ev3_button_is_pressed(UP_BUTTON)){
    sprintf(lcdPattern,"ultra=%d ",ultrasonic_Read());
    sprintf(statusInfo, "enc= %ld              \n", enc_avg);
    ev3_lcd_draw_string(lcdPattern, 0, 60);  // LCDに現在の状態を定義
  }
  while(ev3_button_is_pressed(UP_BUTTON));
  ev3_color_sensor_get_rgb_raw(COLOR_PORT, &rgb_val); /* アナログセンサ情報取得       */
  int white = rgb_val.r;
  RingTone(NOTE_C5, 100, VOLUME);

  ev3_lcd_draw_string("BLACK::set     \n", 0, 10);  // LCDに現在の状態を定義
  while(!ev3_button_is_pressed(DOWN_BUTTON));
  while(ev3_button_is_pressed(DOWN_BUTTON));
  ev3_color_sensor_get_rgb_raw(COLOR_PORT, &rgb_val); /* アナログセンサ情報取得       */
  int black = rgb_val.r;
  RingTone(NOTE_C5, 100, VOLUME);

  ev3_color_sensor_get_rgb_raw(COLOR_PORT, &rgb_val); /* アナログセンサ情報取得       */
  iSensor0 = (white - black) / 2 + black;  // 初期値を保存
  
  // 初期の各成分を入れとく:
  iSensor0_raw[0] = rgb_val.r + 30;
  iSensor0_raw[1] = rgb_val.g + 30;
  iSensor0_raw[2] = rgb_val.b + 30;

  mode = 0;
  enc_avg = 0;
  
  ev3_lcd_draw_string("Ready...          \n", 0, 10);  // LCDに現在の状態を定義
  while(!ev3_button_is_pressed(ENTER_BUTTON));
  while(ev3_button_is_pressed(ENTER_BUTTON));
  
  while(1){
    /* 毎回行う処理****************************************************/
    //sprintf(lcdPattern, "r=%d ,g=%d ,b=%d              \n",rgb_val.r, rgb_val.g, rgb_val.b);
    ev3_lcd_draw_string(statusInfo, 0, 10);  // LCDに現在の状態を定義
    ev3_lcd_draw_string(lcdPattern, 0, 30);  // 動作パターンも表示
    ev3_color_sensor_get_rgb_raw(COLOR_PORT, &rgb_val); /* アナログセンサ情報取得       */
    DebugColor(object_color);
    /*****************************************************************/

    /* 各モードによって振り分け */
    switch(mode){
      case 0:
        // スタート直後初期設定:
        resetEncoder(MOTOR_L);
        resetEncoder(MOTOR_R);
        resetEncoder(MOTOR_HAND);
        
        mode = 1;
        cnt1 = 0;
        enc_avg = 0;
        break;

      case 1:
        // ライントレース開始: エッジ切り替えまで走行
        // ループ後(10000?)にエッジ切り替えを行う.
        if (enc_avg < EDGE_CHANGE)  //　ループしてないときは通常走行 
        {
          steer_go(SPEED, makePD());
        }
        else{  // ループが終わっていたらエッジ切り替え
          steer_go(SPEED, -1*makePD());
        }

        if (enc_avg > 16500 && iGray == 0)  // 灰色通過? (18000は通過してしまう)
        {
          iGray = 1;
          steer_go(0,0);
          RingTone(NOTE_C5, 100, VOLUME);
          mode = 2; 
        }
        break;
    
      case 2:
        // 灰色トレース直後の処理
        steer_go(20, -1*makePD());  // 逆エッジを走行
        if (ultrasonic_Read() <= DETECT_RANGE) // 緑を検出したら停止
        {
          steer_go(0,0);
          cnt1 = 0;
          mode = 3;
        }
        break;

      case 3:
        // ペットボトルの分別処理: センサを上げる
        /**/
        handMove(100, 120);
        /**/
        cnt1 = 0;
        mode = 4;
        break;

      case 4:
        // ペットボトルの分別処理: 色判断
        c = getColor();
        if ((c == COLOR_RED) ||
             (c == COLOR_YELLOW) ||
             (c == COLOR_BLUE) || 
             (c == COLOR_GREEN))
        {
          steer_go(0, 0);   // ペットボトルを見つけたら停止する  
          object_color = c;
          handMove(100, -120);
          cnt1 = 0;
          mode = 5;
        }
        else{
          steer_go(8, 0);  // ゆっくり前進  
        }
        break;

      case 5:
        // ペットボトルの分別処理: 方向判断
        speed_setDeg(30, 480);
        /**/
        RingTone(NOTE_C5, 1500, VOLUME);
        /**/
        switch(object_color){
          case COLOR_BLUE:
          case COLOR_GREEN:
            steer_go(0,0);
            mode = 10;
            cnt1 = 0;
            break;
          
          case COLOR_RED:
            steer_go(0,0);
            mode = 20;
            cnt1 = 0;
            break;
          
          case COLOR_YELLOW:
            steer_go(0,0);
            mode = 30;
            RingTone(NOTE_C6, 100, VOLUME);
            speed_setDeg(30, 180);
            cnt1 = 0;
            break;
          
          default:
            /* NOT REACHED */
            break;
        }

        case 30:
          steer_go(0,0);
          if (cnt1 < 1000)
          {
            steer_go(SPEED, makePD());
          }
          else{
            mode = 100;
            cnt1 = 0;
          }
          break;

        case 100:
          steer_go(0,0);
          mode = 101;
          cnt1 = 0;
          break;

        case 101:
          steer_go(0, 0);  // 動作終了
          break;

        default:
          /* NOT REACHED */
          break;
      }
   }
  ext_tsk();
}
// 1msごとに割込み
void interrupt_1ms(intptr_t idx){
  
  cnt_sensorRate++;
  cnt0++;
  cnt1++;
  cnt2++;
  cnt3++;
  iTimer10++;
  if (mode <= 2 || mode == 11 || mode == 21 || mode == 30)
  {
    ev3_color_sensor_get_rgb_raw(COLOR_PORT, &rgb_val); /* アナログセンサ情報取得       */  
  }
  enc_avg = (ev3_motor_get_counts(MOTOR_R) + ev3_motor_get_counts(MOTOR_L)) / 2;
  enc_avg = -1*enc_avg;  

  if (iGray != 1 && enc_avg > EDGE_CHANGE)
  {
    iGray = gray_detect(); 
  }
}

void sub_task(intptr_t unused){
  /*
  while(mode != 101){
    Play_music(0);  
  }
  RingTone(0, 0, 0);
  ext_tsk();
  */
}

// I/O初期化
void init_IO( void ){

  // センサの初期化:
  ev3_sensor_config(COLOR_PORT, COLOR_SENSOR);
  ev3_sensor_config(ULTRASONIC_PORT, ULTRASONIC_SENSOR);
  // モータの初期化:
  ev3_motor_config(MOTOR_HAND, MEDIUM_MOTOR );  /* PORT BにモーターMを設定 */
  ev3_motor_reset_counts(EV3_PORT_A);           /* PORT Bのモータの角位置をゼロにリセット */
  ev3_motor_config(MOTOR_R, LARGE_MOTOR); /* PORT BにモーターLを設定 */
  ev3_motor_reset_counts(EV3_PORT_B);     /* PORT Bのモータの角位置をゼロにリセット */
  ev3_motor_config(MOTOR_L, LARGE_MOTOR); /* PORT CにモーターLを設定 */
  ev3_motor_reset_counts(EV3_PORT_C);     /* PORT Cのモータの角位置をゼロにリセット */

  ev3_lcd_set_font(EV3_FONT_MEDIUM);      /*文字の大きさを大きく*/
}

void RingTone(int freq, int time, int vol){
  	ev3_speaker_set_volume(vol);
  	ev3_speaker_play_tone(freq, time-50);
  	tslp_tsk(time);   
}

/************************************************************************/
/* ライントレース関数(アナログのみ)                                     */
/* 引数 なし                                                      */
/* 戻り値 操作量                                                         */
/************************************************************************/
int makePD( void ) {
    error = getAnalogSensor();
    integral = error + integral * 0.5;
    steer = KP * error + KI * integral + KD * (error - lasterror);
    lasterror = error;

    return -1*steer;
}
/************************************************************************/
/* ライントレース関数(アナログのみ)                                     */
/* 引数 なし                                                      */
/* 戻り値 操作量                                                         */
/************************************************************************/
int makePD_Gray( void ) {
    
    error = getAnalogSensor_Gray();
    integral = error + integral * 0.5;
    steer = KP * error + KI * integral + KD * (error - lasterror);
    lasterror = error;

    return -1*steer;
}

/************************************************************************/
/* ライントレース関数(アナログのみ)                                     */
/* 引数 P値、D値                                                        */
/* 戻り値 なし                                                            */
/************************************************************************/
void makePD2( float kp , float kd , int power ) {
    int i, iP, iD, iRet;
    
    i = getAnalogSensor();
    iP = kp * i;                    /* 比例 */
    iD = kd * (iSensorBefore - i);  /* 微分 */

    iRet = iP - iD;
    iRet /= 2;

    /* PWM上限の設定 */
    if (iRet > 100)  iRet = 100;
    if (iRet < -100) iRet = -100;

    iServoPwm = iRet;
    iSensorBefore = i;
}


/************************************************************************/
/* カラーセンサ(アナログ値)取得                                                    */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor( void )
{
  int ret;
  ev3_color_sensor_get_rgb_raw(COLOR_PORT, &rgb_val); /* アナログセンサ情報取得       */
  ret = rgb_val.r - iSensor0; // 赤色のみ取得
  return ret;
}

/************************************************************************/
/* カラーセンサ(アナログ値)取得                                                    */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor_Gray( void )
{
  int ret;
  ev3_color_sensor_get_rgb_raw(COLOR_PORT, &rgb_val); /* アナログセンサ情報取得       */
  ret = rgb_val.r - iSensor0_raw[0]; // 赤色のみ取得
  return ret;
}

// 床の色取得
colorid_t getColor( void ){
  return ev3_color_sensor_get_color(COLOR_PORT);
}

// ライントレース用関数:
void steer_go(int speed, int raito){
  if( speed== 0 ){
    ev3_motor_stop(MOTOR_R, true);
    ev3_motor_stop(MOTOR_L, true);
  }
  else{
    ev3_motor_steer(MOTOR_L, MOTOR_R, -speed, raito);
  }
}


// モータ速度制御(ステアリングをかける用)
void speed_dir( int accele, int raito ) {
  if(accele == 0){
    ev3_motor_stop(MOTOR_L, true);
    ev3_motor_stop(MOTOR_R, true);
  }
  if (accele != 0){
    ev3_motor_steer(MOTOR_L, MOTOR_R, accele, raito);
  }

}

// モータ速度制御(角度指定)
void speed_setDeg( int accele, int degree) {
  /* モータ制御 */
  resetEncoder(MOTOR_L);
  resetEncoder(MOTOR_R);
  while(abs(ev3_motor_get_counts(MOTOR_R)) > degree){
  	ev3_motor_steer(MOTOR_L, MOTOR_R, -accele, 0);
  }
  if(accele == 0){
    ev3_motor_stop(MOTOR_R, true);
    ev3_motor_stop(MOTOR_L, true);
  }
}

// その場で旋回する処理(角度指定)
void speed_setTank( int accele_l, int accele_r, int degree) {
  	int raito;
  	resetEncoder(MOTOR_L);
  	resetEncoder(MOTOR_R);
  /* モータ制御 */
	//resetEncoder(MOTOR_L);   // 足回り(左)の回転角をリセット 
    //resetEncoder(MOTOR_R);   // 足回り(右)の回転角をリセット
    if(accele_r < 0){
    	raito = 100;
    }
    else{
    	raito = -100;
    }
   	while(abs(ev3_motor_get_counts(MOTOR_R)) < DEG90){
   		ev3_motor_steer(MOTOR_L,MOTOR_R,abs(accele_r), raito);
   		//ev3_motor_set_power(MOTOR_R,accele_r);
   		//ev3_motor_set_power(MOTOR_L,accele_l);
   	}
}

// 直進(角度指定)
void speed_strait( int accele, int degree) {
  	resetEncoder(MOTOR_L);
  	resetEncoder(MOTOR_R);
    
   	while(abs(ev3_motor_get_counts(MOTOR_R)) < degree){
   		ev3_motor_steer(MOTOR_L,MOTOR_R,accele, 0);
   		//ev3_motor_set_power(MOTOR_R,accele_r);
   		//ev3_motor_set_power(MOTOR_L,accele_l);
   	}
}

// モータの角度をリセット
void resetEncoder(motor_port_t port){
  ev3_motor_reset_counts (port);
}

// アームを動かす
void handMove( int power, int degree ){
	degree = -1*degree;
	ev3_motor_rotate(MOTOR_HAND, degree, power, true);
    if(power == 0){
		ev3_motor_stop(MOTOR_HAND, true);  // 回転完了を待たずに次の処理へ
	}
}


// アームを動かす2
void handMove2( int power, int degree, bool_t state){
  ev3_motor_rotate(MOTOR_HAND, degree, power, state);
  if(power == 0){
    ev3_motor_stop(MOTOR_HAND, true);  // 回転完了を待たずに次の処理へ
  }
}

// 超音波を読む
// 戻り値:障害物との距離
int ultrasonic_Read( void ){
	return ev3_ultrasonic_sensor_get_distance(ULTRASONIC_PORT);
}

// 色に応じてLEDを切り替える処理
void DebugColor(colorid_t c){
	switch(c){
		case COLOR_YELLOW:
			// 黄色の時
			ev3_led_set_color(LED_ORANGE);
			break;

		case COLOR_RED:
			// 赤色の時
			ev3_led_set_color(LED_RED);
			break;

		case COLOR_BLUE:
    case COLOR_GREEN:
		 	ev3_led_set_color(LED_GREEN);
		 	break;

		default:
		 	ev3_led_set_color(LED_OFF);
		 	break;
	}
}

// 灰色検出:
// 1 あり, 0 なし
int gray_detect( void ){
  static int g = 0;
  int ret = 0;
  ev3_color_sensor_get_rgb_raw(COLOR_PORT, &rgb_val); /* アナログセンサ情報取得       */
  
  if (rgb_val.r > 55 && rgb_val.g > 70 && rgb_val.b > 69 )
  {
    ret = 1;
  }

  return ret;
}




//音符定義
#define _C2  0
#define _C2S 1
#define _D2  2
#define _D2S 3
#define _E2  4
#define _F2  5
#define _F2S 6
#define _G2  7
#define _G2S 8
#define _A2  9
#define _A2S 10
#define _B2  11
#define _C3  12
#define _C3S 13
#define _D3  14
#define _D3S 15
#define _E3  16
#define _F3  17
#define _F3S 18
#define _G3  19
#define _G3S 20
#define _A3  21
#define _A3S 22
#define _B3  23
#define _C4  24
#define _C4S 25
#define _D4  26
#define _D4S 27
#define _E4  28
#define _F4  29
#define _F4S 30
#define _G4  31
#define _G4S 32
#define _A4  33
#define _A4S 34
#define _B4  35
#define _C5  36
#define _C5S 37
#define _D5  38
#define _D5S 39
#define _E5  40
#define _F5  41
#define _F5S 42
#define _G5  43
#define _G5S 44
#define _A5  45
#define _A5S 46
#define _B5  47 
#define _C6  48
#define _C6S 49
#define _D6  50
#define _D6S 51
#define _E6  52
#define _F6  53
#define _F6S 54
#define _G6  55
#define _G6S 56
#define _A6  57
#define _A6S 58
#define _B6  59
#define _C7  60
#define _C7S 61
#define _D7  62
#define _D7S 63
#define _E7  64
#define _F7  65
#define _F7S 66
#define _G7  67
#define _G7S 68
#define _A7  69
#define _A7S 70
#define _B7  71
#define _R   -1
#define _BF5 73
#define _BF4 73

//音符周波数の定義
//C,CS,D,DS,E,F,FS,G,GS,A,AS,B
const int tones[75] = {65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123,
                       131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247,
                       262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
                       523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
                       1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
                       2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,
                       0, 0,0
                      };


//******************************************
//
//各楽譜を定義
//楽譜を追加する場合は連番で作成してください
//
//******************************************
#define MUSIC_NUM 5

int musicNum = 0;
int musicInfo[MUSIC_NUM][2]; //0:曲長, 1:最高周波数

//ジングルベル
unsigned char music1[54][2] = {
  {_E5, 4}, {_E5, 4}, {_E5, 2}, {_E5, 4}, {_E5, 4}, {_E5, 2}, {_E5, 4}, {_G5, 4}, {_C5, 4}, {_R, 8},
  {_D5, 8}, {_E5, 1}, {_F5, 4}, {_F5, 4}, {_F5, 4}, {_R, 8}, {_F5, 8}, {_F5, 4}, {_E5, 4}, {_E5, 4},
  {_E5, 4}, {_E5, 4}, {_D5, 4}, {_D5, 4}, {_C5, 4}, {_D5, 2}, {_G5, 2}, {_E5, 4}, {_E5, 4}, {_E5, 2},
  {_E5, 4}, {_E5, 4}, {_E5, 2}, {_E5, 4}, {_G5, 4}, {_C5, 4}, {_R, 8}, {_D5, 8}, {_E5, 1}, {_F5, 4},
  {_F5, 4}, {_F5, 4}, {_R, 8}, {_F5, 8}, {_F5, 4}, {_E5, 4}, {_E5, 4}, {_E5, 4}, {_G5, 4}, {_G5, 4},
  {_F5, 4}, {_D5, 4}, {_C5, 3}, {_R, 4}
};

//箱根八里
unsigned char music2[164][2] = {
  {_C3, 8}, {_C3, 8}, {_E3, 8}, {_E3, 8}, {_G3, 8}, {_G3, 8}, {_G3, 4}, {_C4, 4}, {_A3, 8}, {_A3, 8},
  {_G3, 4}, {_R, 4}, {_E3, 4}, {_E3, 8}, {_E3, 8}, {_G3, 4}, {_E3, 4}, {_D3, 8}, {_D3, 8}, {_E3, 8},
  {_D3, 8}, {_C3, 4}, {_R, 4}, {_C4, 8}, {_C4, 4}, {_C4, 8}, {_A3, 8}, {_A3, 8}, {_R, 4}, {_C4, 8},
  {_C4, 4}, {_C4, 8}, {_G3, 8}, {_G3, 8}, {_R, 4}, {_A3, 4}, {_G3, 8}, {_G3, 8}, {_E3, 8}, {_E3, 8},
  {_E3, 4}, {_G3, 8}, {_G3, 8}, {_E3, 8}, {_E3, 8}, {_D3, 8}, {_D3, 8}, {_D3, 4}, {_C3, 8}, {_C3, 8},
  {_E3, 8}, {_E3, 8}, {_G3, 8}, {_G3, 8}, {_G3, 4}, {_E3, 8}, {_E3, 8}, {_G3, 8}, {_G3, 8}, {_C4, 8},
  {_C4, 8}, {_C4, 4}, {_D4, 4}, {_D4, 4}, {_E4, 4}, {_C4, 4}, {_C4, 4}, {_A3, 4}, {_A3, 4}, {_G3, 4},
  {_R, 4}, {_E3, 4}, {_G3, 4}, {_A3, 4}, {_A3, 4}, {_G3, 4}, {_A3, 4}, {_C4, 4}, {_R, 4}, {_D4, 4},
  {_D4, 4}, {_E4, 4}, {_C4, 4}, {_A3, 4}, {_A3, 4}, {_G3, 4}, {_R, 4}, {_E3, 2}, {_G3, 2}, {_D3, 4},
  {_E3, 8}, {_D3, 8}, {_C3, 4}, {_R, 4}, {_E3, 4}, {_D3, 4}, {_C3, 4}, {_C3, 4}, {_G3, 4}, {_G3, 4},
  {_G3, 4}, {_E3, 4}, {_A3, 2}, {_G3, 4}, {_E3, 4}, {_D3, 8}, {_D3, 8}, {_D3, 8}, {_E3, 8}, {_C3, 4},
  {_R, 4}, {_G3, 4}, {_G3, 8}, {_G3, 8}, {_A3, 8}, {_A3, 8}, {_A3, 8}, {_A3, 8}, {_C4, 4}, {_C4, 8},
  {_C4, 8}, {_D4, 8}, {_D4, 8}, {_D4, 8}, {_D4, 8}, {_E4, 4}, {_D4, 8}, {_E4, 8}, {_C4, 8}, {_C4, 8},
  {_A3, 4}, {_C4, 4}, {_A3, 8}, {_A3, 8}, {_G3, 4}, {_G3, 4}, {_A3, 8}, {_A3, 8}, {_G3, 8}, {_G3, 8},
  {_E3, 8}, {_E3, 8}, {_E3, 4}, {_G3, 4}, {_E3, 8}, {_D3, 8}, {_C3, 4}, {_C3, 4}, {_C4, 4}, {_C4, 4},
  {_C4, 4}, {_C4, 4}, {_D4, 4}, {_D4, 4}, {_C4, 4}, {_A3, 4}, {_G3, 2}, {_E3, 4}, {_C3, 4}, {_D3, 4},
  {_E3, 8}, {_D3, 8}, {_C3, 4}, {_R, 4}
};


//ハウルのメインテーマ
unsigned char music3[69][2] = {
  {_A4S, 8}, {_C5, 8}, {_A4S, 8}, {_C5, 8}, {_A4S, 8}, {_C5, 8}, {_A4, 8}, {_A4S, 8}, {_A4, 8}, {_A4S, 8},
  {_A4, 8}, {_A4S, 8}, {_G4, 8}, {_A4, 8}, {_G4, 8}, {_A4, 8}, {_G4, 8}, {_A4, 8}, {_F4S, 8}, {_E4, 8},
  {_D4S, 2}, {_D3, 8}, {_A3, 8}, {_D4, 8}, {_A4, 2}, {_D4, 4}, {_G4, 4}, {_A4S, 4}, {_D5, 2}, {_D5, 4},
  {_C5, 4}, {_A4S, 4}, {_A4, 4}, {_A4S, 1}, {_G4, 4}, {_A4S, 4}, {_D5, 4}, {_G5, 2}, {_G5, 4}, {_G5, 4},
  {_A5, 4}, {_F5, 8}, {_D5S, 8}, {_F5, 1}, {_A4, 4}, {_D5, 4}, {_F5, 4}, {_A5, 2}, {_G5, 3}, {_G5, 8},
  {_F5, 8}, {_E5, 8}, {_D5S, 8}, {_E5, 8}, {_G5, 2}, {_F5, 4}, {_E5, 8}, {_D5, 8}, {_C5S, 8}, {_D5, 8},
  {_D5, 4}, {_C5, 8}, {_D5, 8}, {_C5, 8}, {_A4S, 8}, {_A4, 4}, {_B4, 8}, {_C5S, 4}, {_D5, 1}
};


//Let it go
unsigned char music4[61][2] = {
  {_R, 8}, {_F5, 8}, {_G5, 8}, {_G5S, 2}, {_D5S, 8}, {_D5S, 8}, {_A5S, 2}, {_G5S, 8}, {_G5, 8}, {_F5, 8},
  {_F5, 8}, {_F5, 4}, {_F5, 8}, {_G5, 4}, {_G5S, 2}, {_F5, 8}, {_G5, 8}, {_G5S, 2}, {_D5S, 8}, {_C6, 8},
  {_A5S, 2}, {_G5S, 8}, {_A5S, 8}, {_C6, 4}, {_C6, 8}, {_C6, 8}, {_C6S, 4}, {_C6, 4}, {_A5S, 8}, {_G5S, 2},
  {_D6S, 4}, {_R, 8}, {_C6, 4}, {_R, 8}, {_A5S, 2}, {_G5S, 4}, {_D6S, 4}, {_R, 8}, {_C6, 4}, {_R, 8},
  {_G5S, 2}, {_R, 16}, {_G5S, 8}, {_G5S, 8}, {_G5, 4}, {_R, 8}, {_D5S, 4}, {_R, 8}, {_D5S, 2}, {_R, 16},
  {_B3, 8}, {_C4S, 4}, {_C4S, 8}, {_C4, 8}, {_C4S, 8}, {_C4, 8}, {_C4S, 8}, {_C4S, 8}, {_C4, 8}, {_G3S, 2},
  {_R, 8}
};


//前々前世
unsigned char music5[107][2] = {
  {_B5, 8}, {_B5, 8}, {_B5, 8}, {_F6S, 4}, {_F6S, 4}, {_F6S, 4}, {_D6S, 8}, {_C6S, 4}, {_C6S, 4}, {_B5, 4},
  {_B5, 8}, {_B5, 8}, {_B5, 8}, {_F6S, 8}, {_F6S, 8}, {_F6S, 8}, {_F6S, 8}, {_F6S, 8}, {_D6S, 8}, {_D6S, 8},
  {_C6S, 4}, {_R, 8}, {_B5, 8}, {_B5, 8}, {_C6S, 8}, {_D6S, 8}, {_C6S, 4}, {_B5, 8}, {_B5, 8}, {_B5, 8},
  {_B5, 8}, {_C6S, 8}, {_D6S, 8}, {_C6S, 4}, {_B5, 8}, {_B5, 8}, {_G5S, 8}, {_B5, 8}, {_G5S, 8}, {_B5, 8},
  {_R, 8}, {_F6S, 4}, {_D6S, 8}, {_F6S, 4}, {_D6S, 4}, {_C6S, 2}, {_R, 4}, {_B5, 8}, {_B5, 8}, {_B5, 8},
  {_F6S, 4}, {_F6S, 4}, {_F6S, 4}, {_D6S, 8}, {_C6S, 4}, {_C6S, 4}, {_B5, 4}, {_B5, 4}, {_B5, 8}, {_F6S, 4},
  {_F6S, 8}, {_F6S, 4}, {_D6S, 4}, {_C6S, 4}, {_R, 4}, {_B5, 4}, {_C6S, 8}, {_D6S, 8}, {_C6S, 4}, {_B5, 8},
  {_B5, 8}, {_B5, 8}, {_B5, 8}, {_C6S, 8}, {_D6S, 8}, {_C6S, 4}, {_B5, 8}, {_B5, 8}, {_G5S, 8}, {_B5, 4},
  {_D6S, 8}, {_C6S, 4}, {_B5, 8}, {_B5, 2}, {_R, 4}, {_R, 8}, {_B5, 8}, {_G5S, 8}, {_B5, 4}, {_D6S, 8},
  {_C6S, 4}, {_B5, 8}, {_B5, 8}, {_G5S, 8}, {_B5, 4}, {_D6S, 8}, {_C6S, 4}, {_B5, 8}, {_B5, 8}, {_G5S, 8},
  {_B5, 8}, {_C6S, 8}, {_D6S, 8}, {_C6S, 4}, {_B5, 8}, {_B5, 2}, {_R, 4}
};


//RPG(sekai no owari)
unsigned char music6[45][2] = {
  {_F5, 8}, {_A5S, 8}, {_C6, 8}, {_D6, 4}, {_R, 8}, {_D6, 8}, {_D6, 4}, {_C6, 8}, {_A5S, 8}, {_C6, 8},
  {_A5, 4}, {_F5, 4}, {_F5, 8}, {_A5S, 8}, {_C6, 8}, {_D6, 4}, {_R, 8}, {_D6, 8}, {_D6, 8}, {_E6, 8},
  {_F6, 8}, {_D6, 4}, {_C6, 8}, {_C6, 2}, {_R, 8}, {_C6, 8}, {_D6, 8}, {_E6, 8}, {_F6, 4}, {_R, 8},
  {_F6, 8}, {_E6, 8}, {_F6, 8}, {_G6, 8}, {_A6, 2}, {_C6, 8}, {_A6S, 8}, {_A6, 8}, {_G6, 8}, {_F6, 2},
  {_A6, 8}, {_A6S, 8}, {_A6, 8}, {_G6, 8}, {_F6, 1}
};

//いつも何度でも(千と千尋の神隠し)
unsigned char music7[][2] = {
  {_F4, 8}, {_G4, 8}, {_A4, 8}, {_F4, 8}, {_C5, 2}, {_A4, 8}, {_G4, 4}, {_C5, 4}, {_G4, 4}, {_F4, 8}, {_D4, 8}, {_A4, 2}, {_F4, 8}, {_E4, 2},
  {_E4, 4}, {_D4, 4}, {_E4, 4}, {_F4, 8}, {_G4, 8}, {_C4, 4}, {_F4, 4}, {_G4, 8}, {_A4, 8}, {_A4S, 4}, {_A4S, 8}, {_A4, 8}, {_G4, 8}, {_F4, 8}, {_G4, 2}, {_R, 8},
  {_F4, 8}, {_G4, 8}, {_A4, 8}, {_F4, 8}, {_C5, 2}, {_A4, 8}, {_G4, 4}, {_C5, 4}, {_G4, 4}, {_F4, 8}, {_D4, 8}, {_D4, 4}, {_E4, 8}, {_F4, 8}, {_C4, 2},
  {_C4, 8}, {_D4, 4}, {_E4, 4}, {_F4, 8}, {_G4, 8}, {_C4, 4}, {_F4, 4}, {_G4, 8}, {_A4, 8}, {_A4S, 4}, {_A4S, 8}, {_A4, 8}, {_G4, 8}, {_F4, 8}, {_F4, 1},
  {_A4, 8}, {_A4S, 8}, {_C5, 4}, {_C5, 4}, {_C5, 4}, {_C5, 4}, {_C5, 8}, {_D5, 8}, {_C5, 8}, {_A4S, 8}, {_A4, 4}, {_A4, 4}, {_A4, 4}, {_A4, 4}, {_A4, 8}, {_A4S, 8}, {_A4, 8}, {_G4, 8}, {_F4, 4}, {_F4, 4}, {_F4, 8}, {_E4, 8}, {_D4, 4}, {_E4, 4}, {_E4, 8}, {_F4, 8}, {_G4, 4}, {_G4, 8}, {_A4, 8}, {_G4, 8}, {_A4, 8}, {_G4, 2}, {_R, 16},
  {_A4, 8}, {_A4S, 8}, {_C5, 4}, {_C5, 4}, {_C5, 4}, {_C5, 4}, {_C5, 8}, {_D5, 8}, {_C5, 8}, {_A4S, 8}, {_A4, 4}, {_A4, 4}, {_A4, 4}, {_A4, 8}, {_A4S, 8}, {_A4, 8}, {_G4, 8}, {_F4, 8}, {_E4, 8}, {_D4, 4}, {_D4, 8}, {_E4, 8}, {_F4, 8}, {_G4, 8}, {_C4, 4}, {_F4, 4}, {_G4, 8}, {_A4, 8}, {_G4, 2}, {_G4, 8}, {_G4, 8}, {_F4, 8}, {_F4, 1}, {_R, 4}

};

//トトロ
unsigned char music8[][2] = {
  {_C5, 4}, {_A4, 8}, {_F4, 4}, {_C5, 4}, {_A4S, 4}, {_G4, 1},
  {_A4S, 4}, {_G4, 8}, {_E4, 4}, {_A4S, 4}, {_A4, 4}, {_F4, 1}, {_R, 8},
  {_A4, 8}, {_C5, 8}, {_D5, 16}, {_C5, 16}, {_E5, 16}, {_F5, 16}, {_G5, 16}, {_F5, 16}, {_A5, 16}, {_C6, 16}, {_D6, 2}, {_D6, 16}, {_E6, 16}, {_C6, 16}, {_A5S, 16}, {_C6, 1}, {_A5S, 1},
  {_A5S, 16}, {_A5, 16}, {_G5, 16}, {_A5, 16}, {_G5, 16}, {_F5, 16}, {_E5, 16}, {_F5, 16}, {_E5, 16}, {_D5, 16}, {_C5, 16}, {_D5, 16}, {_C5, 16}, {_A4S, 16}, {_A4, 16}, {_G4, 16}, {_R, 4},
  {_F4, 4}, {_E4, 4}, {_F4, 8}, {_C4, 1}, {_F4, 4}, {_E4, 4}, {_F4, 8}, {_A4, 1}, {_A4S, 4}, {_A4, 4}, {_G4, 4}, {_F4, 8}, {_A4S, 3}, {_A4, 4}, {_G4, 4}, {_F4, 4}, {_F4, 3}, {_G4, 8}, {_G4, 2}, {_R, 4},
  {_F4, 4}, {_E4, 4}, {_F4, 8}, {_C4, 1}, {_F4, 4}, {_E4, 4}, {_F4, 8}, {_C5, 1},
  {_A4S, 8}, {_A4S, 8}, {_A4S, 8}, {_A4S, 8}, {_A4S, 8}, {_A4, 8}, {_G4, 8}, {_A4S, 1}, {_G4, 8}, {_A4, 8}, {_A4S, 8}, {_A4, 4}, {_A4, 4}, {_A4, 8}, {_G4, 8}, {_F4, 8}, {_A4, 1},
  {_D4, 4}, {_E4, 4}, {_F4, 4}, {_G4, 8}, {_D4, 4}, {_D4, 8}, {_E4, 4}, {_F4, 8}, {_G4, 8}, {_F4, 8}, {_C5, 1},
  {_F4, 8}, {_G4, 8}, {_A4, 8}, {_A4S, 8}, {_C5, 4}, {_A4, 8}, {_F4, 4}, {_C5, 4}, {_A4S, 4}, {_G4, 1}, {_A4S, 4}, {_G4, 8}, {_E4, 4}, {_A4S, 4}, {_A4, 4}, {_F4, 1}, {_R, 8},
  {_C4S, 4}, {_F4, 4}, {_A4S, 4}, {_A4, 4}, {_C5, 8}, {_F4, 2}, {_A4, 8}, {_A4S, 8}, {_A4, 8}, {_A4S, 8}, {_A4, 8}, {_A4S, 8}, {_A4, 8}, {_F4, 8}, {_G4, 2},
  {_F4, 8}, {_G4, 8}, {_A4, 8}, {_A4S, 8}, {_C5, 4}, {_A4, 8}, {_F4, 4}, {_C5, 4}, {_A4S, 4}, {_G4, 1}, {_A4S, 4}, {_G4, 8}, {_E4, 4}, {_A4S, 4}, {_A4, 4}, {_F4, 1}, {_R, 8},
  {_D4, 4}, {_D5, 4}, {_C5, 8}, {_A4S, 8}, {_A4, 8}, {_A4S, 8}, {_C5, 3}, {_F4, 8}, {_F4, 3}, {_A4, 8}, {_A4S, 8}, {_A4, 8}, {_F4, 8}, {_A4S, 8}, {_A4, 8}, {_F4, 8}, {_D5, 8}, {_C5, 1}, {_R, 4},
  {_C4, 8}, {_C4, 8}, {_A4S, 8}, {_A4, 8}, {_G4, 8}, {_A4, 8}, {_F4, 1}
};

//千本桜
unsigned char music9[][2] = {
  {_D5, 8}, {_F5, 8},
  {_G5, 8}, {_G5, 16}, {_G5, 8}, {_R, 16}, {_A5, 8}, {_A5, 4}, {_R, 8}, {_A5, 8}, {_C6, 8}, {_D6, 8}, {_G5, 8}, {_F5, 8}, {_A5, 4}, {_D5, 8}, {_F5, 8},
  {_G5, 8}, {_G5, 16}, {_G5, 8}, {_R, 16}, {_A5, 8}, {_A5, 4}, {_R, 8}, {_A5, 8}, {_A5S, 8}, {_A5, 8}, {_G5, 8}, {_F5, 8}, {_F5, 4}, {_D5, 8}, {_F5, 8},
  {_G5, 8}, {_G5, 16}, {_G5, 8}, {_R, 16}, {_A5, 8}, {_A5, 4}, {_R, 8}, {_A5, 8}, {_C6, 8}, {_D6, 8}, {_G5, 8}, {_F5, 8}, {_A5, 4}, {_D5, 8}, {_F5, 8},
  {_A5S, 4}, {_A5, 4}, {_G5, 4}, {_F5, 4}, {_G5, 8}, {_A5, 8}, {_E5, 8}, {_C5, 8}, {_D5, 4}, {_D5, 8}, {_F5, 8},
  {_G5, 8}, {_G5, 16}, {_G5, 8}, {_R, 16}, {_A5, 8}, {_A5, 4}, {_R, 8}, {_A5, 8}, {_C6, 8}, {_D6, 8}, {_G5, 8}, {_F5, 8}, {_A5, 4}, {_D5, 8}, {_F5, 8},
  {_G5, 8}, {_G5, 16}, {_G5, 8}, {_R, 16}, {_A5, 8}, {_A5, 4}, {_R, 8}, {_A5, 8}, {_A5S, 8}, {_A5, 8}, {_G5, 8}, {_F5, 8}, {_F5, 4}, {_D5, 8}, {_F5, 8},
  {_G5, 8}, {_G5, 16}, {_G5, 8}, {_R, 16}, {_A5, 8}, {_A5, 4}, {_R, 8}, {_A5, 8}, {_C6, 8}, {_D6, 8}, {_G5, 8}, {_F5, 8}, {_A5, 4}, {_D5, 8}, {_F5, 8},
  {_A5S, 4}, {_A5, 4}, {_G5, 4}, {_F5, 4}, {_G5, 8}, {_F5, 8}, {_A5, 8}, {_C6, 8}, {_D6, 1}
};

//おぼろ月夜
unsigned char music10[][2] = {
  {_E5, 8}, {_E5, 8}, {_C5, 4}, {_D5, 8}, {_E5, 8}, {_G5, 8},
  {_G5, 8}, {_A5, 8}, {_G5, 4}, {_D5, 4}, {_E5, 4}, {_C5, 8},
  {_D5, 8}, {_G5, 8}, {_E5, 2}, {_G5, 8}, {_G5, 8}, {_E5, 4},
  {_F5, 8}, {_G5, 8}, {_C6, 8}, {_C6, 8}, {_D6, 8}, {_C6, 4},
  {_G5, 4}, {_A5, 4}, {_E5, 8}, {_D5, 8}, {_D5, 8}, {_C5, 2},
  {_G5, 8}, {_G5, 8}, {_C6, 4}, {_C6, 8}, {_C6, 8}, {_D6, 8},
  {_C6, 8}, {_A5, 8}, {_G5, 4}, {_G5, 8}, {_E5, 8}, {_G5, 4},
  {_A5, 8}, {_E5, 8}, {_E5, 8}, {_D5, 2}, {_C5, 8}, {_D5, 8},
  {_E5, 4}, {_C5, 8}, {_E5, 8}, {_F5, 8}, {_G5, 8}, {_C6, 8},
  {_A5, 4}, {_G5, 4}, {_A5, 4}, {_E5, 8}, {_D5, 8}, {_D5, 8},
  {_C5, 1}
};

//たきび
unsigned char music11[][2] = {
  {_G4, 8}, {_A4, 8}, {_G4, 8}, {_E4, 8}, {_G4, 8}, {_A4, 8}, {_G4, 8}, {_E4, 8}, {_C4, 8}, {_D4, 8}, {_E4, 8}, {_E4, 8}, {_D4, 2},
  {_E4, 8}, {_G4, 8}, {_G4, 8}, {_G4, 8}, {_A4, 8}, {_C5, 8}, {_C5, 8}, {_C5, 8}, {_G4, 8}, {_A4, 8}, {_E4, 8}, {_D4, 8}, {_C4, 2},
  {_D4, 3}, {_E4, 8}, {_F4, 8}, {_E4, 8}, {_D4, 4}, {_E4, 8}, {_G4, 8}, {_G4, 8}, {_E4, 8}, {_G4, 2},
  {_C5, 8}, {_C5, 8}, {_C5, 8}, {_A4, 8}, {_G4, 8}, {_G4, 8}, {_C5, 8}, {_C5, 8}, {_E4, 8}, {_E4, 8}, {_D4, 8}, {_D4, 8}, {_C4, 2}
};

//恋(星野源)
unsigned char music12[][2] = {
  {_C6S, 8}, {_E6, 8}, {_C6S, 8}, {_E6, 8}, {_F6S, 4}, {_E6, 8}, {_C6S, 8}, {_A5, 4}, {_B5, 4}, {_C6S, 4}, {_R, 4},
  {_C6S, 8}, {_E6, 8}, {_C6S, 8}, {_A6, 8}, {_G6S, 4}, {_F6S, 8}, {_E6, 8}, {_F6S, 4}, {_E6, 4}, {_A5, 4}, {_B5, 4},
  {_C6S, 8}, {_E6, 8}, {_C6S, 8}, {_E6, 8}, {_F6S, 4}, {_E6, 8}, {_C6S, 8}, {_A5, 4}, {_B5, 4}, {_C6S, 4}, {_R, 8},
  {_A5, 4}, {_A5, 8}, {_B5, 4}, {_C6S, 4}, {_A5, 8}, {_F6S, 4}, {_E6, 8}, {_E6, 8}, {_C6S, 8}, {_E6, 4}, {_R, 4},
  {_C6S, 8}, {_E6, 8}, {_C6S, 8}, {_E6, 8}, {_F6S, 4}, {_E6, 8}, {_C6S, 8}, {_A5, 4}, {_B5, 4}, {_C6S, 4}, {_R, 4},
  {_C6S, 8}, {_E6, 8}, {_C6S, 8}, {_A6, 8}, {_G6S, 4}, {_F6S, 8}, {_E6, 8}, {_F6S, 4}, {_E6, 4}, {_A5, 4}, {_B5, 4},
  {_C6S, 8}, {_E6, 8}, {_C6S, 8}, {_E6, 8}, {_F6S, 4}, {_E6, 8}, {_C6S, 8}, {_A5, 4}, {_G6S, 4}, {_A6, 4}, {_R, 8},
  {_F6S, 4}, {_E6, 8}, {_E6, 8}, {_C6S, 8}, {_E6, 8}, {_F6S, 8}, {_R, 8},
  {_F6S, 4}, {_E6, 8}, {_E6, 8}, {_C6S, 8}, {_E6, 8}, {_F6S, 8}, {_R, 8},
  {_F6S, 4}, {_E6, 8}, {_E6, 8}, {_F6S, 8}, {_C6S, 8}, {_B5, 8}, {_A5, 8}, {_F5S, 8}, {_A5, 2}, {_R, 4}
};

//KRKRKKTM
unsigned char music13[84][2] = {
  {_C5, 8}, {_D5, 8}, {_E5, 8}, {_F5, 8}, {_G5, 8}, {_A5, 8}, {_B5, 8}, {_C6, 8}, {_R , 8}, {_C6, 8},
  {_B5, 8}, {_A5, 8}, {_G5, 8}, {_F5, 8}, {_E5, 8}, {_D5, 8}, {_C5, 8}, {_R , 8}, {_C6, 4}, {_C6, 4},
  {_R , 8}, {_C6, 4}, {_A5, 8}, {_C6, 8}, {_A5, 8}, {_C6, 4}, {_D6, 4}, {_C5, 8},
  {_D5, 8}, {_E5, 8}, {_F5, 8}, {_G5, 8}, {_A5, 8}, {_B5, 8}, {_C6, 8}, {_R , 8}, {_C6, 8}, {_B5, 8},
  {_A5, 8}, {_G5, 8}, {_G5, 8}, {_A5, 8}, {_B5, 8}, {_C6, 8}, {_R , 8}, {_C6, 4}, {_C6, 4}, {_R , 8},
  {_A5, 4}, {_B5, 8}, {_C6, 8}, {_A5, 8}, {_E6, 4}, {_D6, 4}, {_E5, 4}, {_E5, 4},
  {_F5S, 4}, {_G5S, 4}, {_A5, 4}, {_B5, 4}, {_C6, 4}, {_R , 4}, {_A5, 8}, {_B5, 8}, {_C6, 4},
  {_A5, 8}, {_B5, 8}, {_C6, 4}, {_R , 8}, {_C6, 4}, {_A5, 8}, {_C6, 8}, {_C6, 16}, {_R, 16}, {_C6, 8},
  {_R , 8}, {_C6, 1}, {_R , 8}, {_A5S, 8}, {_R , 8}, {_A5S, 8}, {_C6, 8}, {_R , 8}, {_R , 4}
};

// FLASH(Perfume)
const unsigned char music14[][2] = {
  /* ひ　　　　　ば　　　なの　　よ　　　　う　　　に */
  {_D5S, 16}, {_D5S, 16}, {_F5, 8}, {_G5S, 8}, {_F5, 8}, {_D5S, 8},
  /* FLASH   ()    ひか　　　る　　　()      さ       い　 　　　*/
  {_C5S, 4}, {_R, 8}, {_C6S, 8}, {_C6, 8}, {_R, 8}, {_C6, 8}, {_G5S, 8},
  /* こう　  の　　Light       ing  　 Game    ()      な　　　　*/
  {_BF5, 8}, {_C6, 8}, {_BF5, 8}, {_G5S, 8}, {_F5, 4}, {_R, 8}, {_D5S, 8},
  /* ら　　　し　　　　た　　　お　　　と　　　　　も　　　　　　　*/
  {_C5S, 4}, {_C5S, 8}, {_C6S, 8}, {_C6, 4}, {_C6, 8}, {_G5S, 8},
  /*　お　　　き　　　ざ　　　　り       に 　　　し　　　て 　　*/
  {_BF5, 8}, {_C6, 8}, {_BF5, 8}, {_G5S, 8}, {_F5, 8}, {_G5S, 8}, {_F5, 4},

  /* FLASH   こ　　　　え　　　　る　　　()    さ　　　　い　*/
  {_C5S, 4}, {_C5S, 8}, {_C6S, 8}, {_C6, 8}, {_R, 8}, {_C6, 8}, {_G5S, 8},
  /* こう　　　の　　 Light      ing    ga me     ()      はや*/
  {_BF5, 8}, {_C6, 8}, {_BF5, 8}, {_G5S, 8}, {_F5, 4}, {_R, 8}, {_D5S, 8},
  /* い　　　とき　　の　 なかで  　　() 　　　　ち　　　　は　*/
  {_C5S, 4}, {_C5S, 8}, {_C6S, 8}, { _C6, 8}, {_R, 8}, { _C6, 8}, {_G5S, 8},
  /* 　や      ふ     る       う      FLASH   ひ        か　*/
  {_BF5, 8}, {_C6, 8}, {_BF5, 8}, {_G5S, 8}, {_F5, 8}, {_G5S, 8}, {_F5, 4},


  /* さ　　　い　　　　こう　　　の */
  {_C5S, 4}, {_C5S, 8}, {_C6S, 8}, {_C6, 8}, {_R, 8}, {_C6, 8}, {_G5S, 8},
  {_BF5, 8}, {_C6, 8}, {_BF5, 8}, {_G5S, 8}, {_F5, 4}, {_R, 8}, {_D5S, 8},
  {_C5S, 4}, {_C5S, 8}, {_C6S, 8}, {_C6, 8}, {_R, 8}, {_C6, 8}, {_G5S, 8},
  {_BF5, 4}, {_B5, 8}, {_G5S, 8}, {_F5, 8}, {_G5S, 8}, {_F5, 4},

  {_C5S, 4}, {_C5S, 8}, {_C6S, 8}, {_C6, 8}, {_R, 8}, {_C6, 8}, {_G5S, 8},
  {_BF5, 8}, {_C6, 8}, {_BF5, 8}, {_G5S, 8}, {_F5, 4}, {_R, 8}, {_D5S, 8},
  {_C5S, 4}, {_C5, 8}, {_C6S, 8}, {_C6, 8}, {_R, 8}, {_C6, 8}, {_G5S, 8},
  {_BF5, 4}, {_R, 8}, {_G5S, 8}, {_B5, 8}, {_G5S, 8}, {_F5, 8}, {_C5S, 8},
  {_B4, 4}, {_R, 4}, {_R, 1}
};

//ド・レ・ミ・ファ・ソ・ラ・シ・ド
// C・ D・ E・ F  ・G ・ A・ B・ C
const unsigned char music15[] [2] = {
  {_D5, 4 }, {_E5, 4 }, {_F5, 2 }, {_F5, 4,}, {_A5, 4}, {_G5, 5}, {_F5, 5}, {_E5, 2},
  {_G5, 2 }, {_G5, 5 }, {_A5, 5 }, {_A5S,5},  {_A5, 5}, {_G5, 5}, {_F5, 2}
};
void init_music_info(void)
{
  int maxTone = 0;
  musicInfo[0][0] = (sizeof(MUSIC_1) / sizeof(unsigned char) / 2);
  for (int i = 0; i < musicInfo[0][0]; i++) {
    if (maxTone < tones[MUSIC_1[i][0]]) {
      maxTone = tones[MUSIC_1[i][0]];
    }
  }
  musicInfo[0][1] = maxTone;

  maxTone = 0;
  musicInfo[1][0] = (sizeof(MUSIC_2) / sizeof(unsigned char) / 2);
  for (int i = 0; i < musicInfo[1][0]; i++) {
    if (maxTone < tones[MUSIC_2[i][0]]) {
      maxTone = tones[MUSIC_2[i][0]];
    }
  }
  musicInfo[1][1] = maxTone;

  maxTone = 0;
  musicInfo[2][0] = (sizeof(MUSIC_3) / sizeof(unsigned char) / 2);
  for (int i = 0; i < musicInfo[2][0]; i++) {
    if (maxTone < tones[MUSIC_3[i][0]]) {
      maxTone = tones[MUSIC_3[i][0]];
    }
  }
  musicInfo[2][1] = maxTone;

  maxTone = 0;
  musicInfo[3][0] = (sizeof(MUSIC_4) / sizeof(unsigned char) / 2);
  for (int i = 0; i < musicInfo[3][0]; i++) {
    if (maxTone < tones[MUSIC_4[i][0]]) {
      maxTone = tones[MUSIC_4[i][0]];
    }
  }
  musicInfo[3][1] = maxTone;

  maxTone = 0;
  musicInfo[4][0] = (sizeof(MUSIC_5) / sizeof(unsigned char) / 2);
  for (int i = 0; i < musicInfo[4][0]; i++) {
    if (maxTone < tones[MUSIC_5[i][0]]) {
      maxTone = tones[MUSIC_5[i][0]];
    }
  }
  musicInfo[4][1] = maxTone;
}

int thisPattern = 0; //発色パターンの初期値
int duration = 0;            //音符の拍数
int thisTone = 0;            //音符の周波数
int thisDuration = 0;        //発音の長さ

int now_sw, old_sw;

void Play_music( int musicNum ) {
  for (int i = 0; i < musicInfo[musicNum][0]; i++) {

    //＝＝＝音＝＝＝
    switch (musicNum) {
      case 0: thisTone = MUSIC_1[i][0];
        thisDuration = MUSIC_1[i][1];
        break;
      case 1: thisTone = MUSIC_2[i][0];
        thisDuration = MUSIC_2[i][1];
        break;
      case 2: thisTone = MUSIC_3[i][0];
        thisDuration = MUSIC_3[i][1];
        break;
      case 3: thisTone = MUSIC_4[i][0];
        thisDuration  = MUSIC_4[i][1];
        break;
      case 4: thisTone = MUSIC_5[i][0];
        thisDuration  = MUSIC_5[i][1];
        break;
      default:
        break;
    }

    duration = 650 / thisDuration;

    RingTone(tones[thisTone], duration*1.1, VOLUME); 
    tslp_tsk(duration*1.1);
    
  }
}

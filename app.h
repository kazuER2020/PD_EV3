
#include "target_test.h"

#define MAIN_PRIORITY	5		

#define HIGH_PRIORITY	9		
#define MID_PRIORITY	10
#define LOW_PRIORITY	11


#ifndef TASK_PORTID
#define	TASK_PORTID		1			
#endif /* TASK_PORTID */

#ifndef STACK_SIZE
#define	STACK_SIZE		4096		
#endif /* STACK_SIZE */

#ifndef LOOP_REF
#define LOOP_REF		ULONG_C(1000000)	
#endif /* LOOP_REF */

#ifndef TOPPERS_MACRO_ONLY

extern void	task(intptr_t exinf);
extern void	main_task(intptr_t exinf);
extern void interrupt_1ms(intptr_t idx);
extern void	gpio_irq_dispatcher(intptr_t exinf);
extern void sub_task(intptr_t unused);

void init_IO( void );
void Play_music( int musicNum );
void init_music_info(void);
void RingTone(int freq, int time, int vol);
int makePD( void );
int makePD_Gray( void );
void makePD2( float kp , float kd , int power );
void speed_dir( int accele, int raito );
void speed_setDeg( int accele, int degree);
void handMove( int power, int degree );
void handMove2( int power, int degree, bool_t state);
void speed_setTank( int accele_l, int accele_r, int degree);
void speed_strait( int accele, int degree);
void resetEncoder(motor_port_t port);
void DebugColor(colorid_t c);
void steer_go(int speed,  int raito);
int ultrasonic_Read( void );
int getAnalogSensor( void );
int getAnalogSensor_Gray( void );
int gray_detect( void );
colorid_t getColor( void );
#endif /* TOPPERS_MACRO_ONLY */

// r g b
// 59 74 73 gray

// 20 55 30 green
#include <Arduino.h>
#include <string.h>




/**
*  communication data format : Identifier(1)  data(N)
*  please refer to motor_protocol.xlsx
**/
#define   SET_DC_FREQ_PRO_NUM         5
#define   SET_DC_DIREC_PRO_NUM        3
#define   SET_DC_SPEED_PRO_NUM        4
#define   SET_DC_START_FLAG_PRO_NUM   6
#define   SET_ANGEL                   7
#define   SET_STEP_ENDOF_DEG          8
#define   SET_STEP_ENDOF_STEP         9
#define   SET_STEP_ENDOF_TIME         10
#define   STOP_STEP_MOTOR             12
#define   SET_DC_PARAMS               13
        
#define   BRAKE       2
#define   START       1
#define   STOP        0

#define   MOTOR_NUM_1    0
#define   MOTOR_NUM_2    1


#define   FREQ_7_813KHZ         1        //7.813KHZ
#define   FREQ_0_977KHZ         2        //0.977KHZ
#define   FREQ_0_244KHZ         3        //0.244KHZ
#define   FREQ_0_061KHZ         4        //0.061KHZ


#define   DIRECTION_FOWARD      0  
#define   DIRECTION_REVERSE     2

#define   DIRECTION_CLOCK_WISE  0
#define   DIRECTION_COUNTER_CLOCKWISE  1

#define   EXCITATION_FULLSTEP    0
#define   EXCITATION_HALFSTEP    1

#define   CHOOSE_DC_RETURN       "0101"
#define   CHOOSE_STEP_RETURN     "0102"

/** Set DC motor frequency.
* P1:Frequency of motor.Does not distinguish between specific motor numbers,01-7.813k，02-0.977khz  03-0.244khz  04-0.061khz
*/
void set_dc_freq(char freq)
{
	
	Serial.write(SET_DC_FREQ_PRO_NUM);
	Serial.write(freq);
}

/** Set DC motor direction
 *  P1 :This module have four OUT PINS, support two motor interfaces，choose the number of motor.
 *  P2 :The direction of motor rotation.
 * */
void set_dc_direc(char motor_num,char direc)
{

	Serial.write(SET_DC_DIREC_PRO_NUM);
	Serial.write(motor_num);
	Serial.write(direc);
}

/** Set DC motor speed
 *  P1 :This module have four OUT PINS, support two motor interfaces，choose the number of motor.
 *  P2 :The speed of motor ,range :0-100.
 * */
void set_dc_speed(char motor_num,char speed)
{
	Serial.write(SET_DC_SPEED_PRO_NUM);
	Serial.write(motor_num);
	Serial.write(speed);
}

/** Set DC motor running stat.
 *  P1 :This module have four OUT PINS, support two motor interfaces，choose the number of motor.
 *  P2 :The running stat of motor . 0 - standby stopped,1- running ,2- brake stopped.
 * */
void dc_stat_control(char motor_num,char stat)
{
	Serial.write(SET_DC_START_FLAG_PRO_NUM);
	Serial.write(motor_num);
	Serial.write(stat);
	delay(200);
}



/** Running the motor ,end of target time.
 *  P1: frequency ,determines the speed of step motor.
 *  P2: target time,unit is second,Stop the motor when the target time is reached,if set 0,Indicate that there is no stop condition. 
 *  P3: direction of motor rotation
 *  P4: excitation of motor rotation.Divided into FULLSTEP/HALFSTEP
 * */
void driver_step_endof_time(unsigned short freq,unsigned short times,char direc,char exc)
{
	Serial.write(SET_STEP_ENDOF_TIME);
	Serial.write(freq>>8);
	Serial.write(freq);
	Serial.write(times>>8);
	Serial.write(times);
	Serial.write(direc);
	Serial.write(exc);
}

/** Running the motor ,end of target steps.
 *  P1: frequency ,determines the speed of step motor.
 *  P2: target steps,Stop the motor when the target steps is reached,if set 0,Indicate that there is no stop condition. 
 *  P3: direction of motor rotation
 *  P4: excitation of motor rotation.Divided into FULLSTEP/HALFSTEP
 * */
void driver_step_endof_steps(unsigned short freq,unsigned short steps,char direc,char exc)
{
	Serial.write(SET_STEP_ENDOF_STEP);
	Serial.write(freq>>8);
	Serial.write(freq);
	Serial.write(steps>>8);
	Serial.write(steps);
	Serial.write(direc);
	Serial.write(exc);
}

/**Set the param angle/step,defualt value is 7.5.This value is fixed to the same motor.
*Different motors have different values.If you use a motor that is at a different angle,
* you need set it.
*angle: The angle at which the motor runs in one step
* The value type is float,The result of (float)angel1/angel2 as set value. 
*/
void driver_step_set_angle(unsigned char angel1,unsigned char angel2)
{
	Serial.write(SET_ANGEL);
	Serial.write(angel1);
	Serial.write(angel2);
}

/** Running the motor ,end of target degrees.
 *  P1: frequency ,determines the speed of step motor.
 *  P2: target deg,Stop the motor when the target degree is reached,if set 0,Indicate that there is no stop condition. 
 *  P3: direction of motor rotation
 *  P4: excitation of motor rotation.Divided into FULLSTEP/HALFSTEP
 * */
void driver_step_endof_deg(unsigned short freq,unsigned short deg,char direc,char exc)
{
	Serial.write(SET_STEP_ENDOF_DEG);
	Serial.write(freq>>8);
	Serial.write(freq);
	Serial.write(deg>>8);
	Serial.write(deg);
	Serial.write(direc);
	Serial.write(exc);
}

void step_stop(void)
{
	Serial.write(STOP_STEP_MOTOR);
}



void choose_dc(void)
{
	int recv_len=0;
	unsigned char buf[10]={0};
	while(1)
	{
		Serial.write(1);
		Serial.write(1);
		delay(500);
		/*
		if(Serial.available())
		{
			delay(30);
			recv_len=Serial.available();
			for(int i=0;i<recv_len;i++)
			{
				buf[i]=Serial.read();
			}
		}
		if(0==memcmp(buf,CHOOSE_DC_RETURN,4))
		{
			break;
		}
		delay(1000);
		memset(buf,0,sizeof(buf));
		recv_len=0;
		*/
		break;
	}
}

void choose_step(void)
{
	int recv_len=0;
	unsigned char buf[10]={0};
	while(1)
	{
		Serial.write(1);
		Serial.write(2);
		delay(500);
		/*
		if(Serial.available())
		{
			delay(30);
			recv_len=Serial.available();
			for(int i=0;i<recv_len;i++)
			{
				buf[i]=Serial.read();
			}
		}
		if(0==memcmp(buf,CHOOSE_STEP_RETURN,4))
		{
			break;
		}
		delay(1000);
		memset(buf,0,sizeof(buf));
		recv_len=0;
		*/
		break;
	}
}


void set_dc_motor_param(char motor_num,char freq,char direc,char speed)
{
	set_dc_freq(freq);
	delay(100);
	set_dc_direc(motor_num,direc);
	delay(100);
	set_dc_speed(motor_num,speed);
	delay(100);
}

void set_dc_motor_param_total(char motor_num,char freq,char direc,char speed,char start_flag)
{
	Serial.write(SET_DC_PARAMS);
	Serial.write(motor_num);
	Serial.write(freq);
	Serial.write(direc);
	Serial.write(speed);
	Serial.write(start_flag);
}


//#define DC_MOTOR
#define STEP_MOTOR



void setup()
{
  Serial.begin(115200);
  delay(200);
#ifdef DC_MOTOR
  choose_dc();
#endif

#ifdef STEP_MOTOR
  choose_step();  //chose the type of motor connected
#endif 
  delay(300);
}


void dc_test(void)
{
	/*
	set_dc_motor_param(MOTOR_NUM_1,FREQ_7_813KHZ,DIRECTION_FOWARD,0x20);
	dc_stat_control(MOTOR_NUM_1,START);
	delay(5000);
	dc_stat_control(MOTOR_NUM_1,STOP);
	
	set_dc_motor_param(MOTOR_NUM_1,FREQ_0_977KHZ,DIRECTION_REVERSE,0x40);
	dc_stat_control(MOTOR_NUM_1,START);
	delay(5000);
	dc_stat_control(MOTOR_NUM_1,BRAKE);
	*/
	
	set_dc_motor_param_total(MOTOR_NUM_1,FREQ_7_813KHZ,DIRECTION_FOWARD,0x20,START);
	delay(5000);
	dc_stat_control(MOTOR_NUM_1,BRAKE);不区分具体电机号决定了电机的转速
	delay(1000);
	set_dc_motor_param_total(MOTOR_NUM_1,FREQ_0_977KHZ,DIRECTION_REVERSE,0x40,START);
	delay(5000);
	dc_stat_control(MOTOR_NUM_1,BRAKE);
	delay(1000);
}


void step_test(void)
{
	driver_step_set_angle(15,2);
	delay(100);
	driver_step_endof_time(300,10,DIRECTION_CLOCK_WISE,EXCITATION_FULLSTEP);
	delay(15000);
	driver_step_endof_steps(1000,1000,DIRECTION_COUNTER_CLOCKWISE,EXCITATION_HALFSTEP);
	delay(5000);
	driver_step_endof_deg(300,360,DIRECTION_COUNTER_CLOCKWISE,EXCITATION_HALFSTEP);
	delay(5000);
	step_stop();
}


/** notice!!!!!!!!!!!!!!!
*	The minimum interval between each two data is 80ms
*
**/

void loop()
{
#ifdef DC_MOTOR
  dc_test();
#endif

#ifdef STEP_MOTOR
  step_test();
#endif
}
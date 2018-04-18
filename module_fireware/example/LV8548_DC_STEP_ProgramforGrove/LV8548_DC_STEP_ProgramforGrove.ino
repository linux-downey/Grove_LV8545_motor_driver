#include "Seeed_LV8548_motor_driver.h"
#include "TimerOne.h"


#define TIMER 65

void interrupt(){
  motor.timerFire(TIMER);
}



void setup()
{
    u8 motor_type=0;
	//wire_init();      
    Serial_init();        //Serial1 init

/**This LIB support two kinds of motor(DC/STEP),And use the same hardware interface，So you can only use one of them
 * It is necessary to send choose motor type protocol 0x01 0x01(DC)/0x02(STEP) to choose the motor you want to use.
 * otherwise the motor won't respond to your instructions.
 * */
    motor_type=get_user_choise();  

    motor.set_motor_type(motor_type);                   //set motor type
/**
 * If Using STEP motor ,need to add timing function.
 * */
    if(STEP_MOTOR==motor_type)
    {
        Timer1.initialize(TIMER);
        Timer1.attachInterrupt(interrupt);
    }
/**
 * According to the motor type you choose，init corresponding motor platform.
 * */
    motor.motor_init();
} 

void loop()
{
     uart_recv_data();    /**< recv instructions*/
     commucation_parse();  /**< parsing instructions*/
}





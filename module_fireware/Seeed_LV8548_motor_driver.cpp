#include "Seeed_LV8548_motor_driver.h"

static recv_struct_t recv_struct;
LV8548_Motor motor;



void clean_recv_struct(recv_struct_t* recv_struct)
{
    memset(recv_struct,0,sizeof(recv_struct_t));
}


// void read_callback(int howMany)
// {
//   // u8 index=0;
//   // clean_recv_struct(&recv_struct);
//   // while( Wire.available()>1) 
//   // {
//   //   recv_struct.recv_buf[recv_struct.recv_buf_len++]= Wire.read(); 
//   // }
//   // recv_struct.recv_over_flag=1;

//     // motor.setPWMFrequency(1);
//     // motor.setRotation(0, 1);
//     // motor.setCtlVoltage(0, 20);
//     // motor.setStartFlag(0, 1);
//     // delay(1000);
//     // motor.setStartFlag(0, 0);
// }

// //发送ID
// void request_callback()
// {
//     Wire.write(0x33);
// }



/**
 * Check if the recv_over_flag is set,if yes ,parse the data.
 * */
void commucation_parse()
{
  if((1==recv_struct.recv_over_flag))
      {
          motor.recv_buf_parse(recv_struct.recv_buf,recv_struct.recv_buf_len);
          clean_recv_struct(&recv_struct);
      }
}




// void wire_init()
// {
//     Wire.begin(4);
//     Wire.onReceive(read_callback); 
//     Wire.onRequest(request_callback);
// }


// u8 get_user_choise()
// {
//   while(1)
//   {
//     if(Serial1.available())
//     {
//       delay(50);
//       recv_struct.recv_buf_len=Serial1.available();
//       for(int i=0;i<recv_struct.recv_buf_len;i++)
//       {
//         recv_struct.recv_buf[i]=Serial1.read();
//       }
//      if((recv_struct.recv_buf[0]==0x01)&&(2==recv_struct.recv_buf_len))
//       {
//         if(0x01==recv_struct.recv_buf[1])
//         {
//           Serial1.print("0101");
//           clean_recv_struct(&recv_struct);
//           return DC_MOTOR;
//         }
//         else
//         {
//           Serial1.print("0102");
//           clean_recv_struct(&recv_struct);
//           return STEP_MOTOR;
//         }
//       }
//     }
//   }
// }


void Serial_init()
{
    Serial1.begin(115200);
}


/**
 * UART recv function.
 * If revieve over,The flag recv_struct.recv_over_flag will be SET.
 * */
void uart_recv_data()
{
  if(Serial1.available())
  {
    delay(50);
    recv_struct.recv_buf_len=Serial1.available();
    for(int i;i<recv_struct.recv_buf_len;i++)
    {
      recv_struct.recv_buf[i]=Serial1.read();
    }
    recv_struct.recv_over_flag=1;
  }
}


u8 get_motor_type_from_input()                   /*<<01,01--DC MOTOR,01,02--STEP MOTOR  */
{
    while(1)
    {
        if((1==recv_struct.recv_over_flag)&&(2==recv_struct.recv_buf_len))
        {
            if(SELECT_MOTOR_TYPE==recv_struct.recv_buf[0])
            {
                if(DC_MOTOR==recv_struct.recv_buf[1])
                {
                    clean_recv_struct(&recv_struct);
                    return DC_MOTOR;
                }
                if(STEP_MOTOR==recv_struct.recv_buf[1])
                {
                    clean_recv_struct(&recv_struct);
                    return STEP_MOTOR;
                }
            }
            clean_recv_struct(&recv_struct);
        }
    }
}











/**************************************************************************************************************/
/*******************************************motor part******************************************************/



/**********************************************************************************************************/
/****************************************DC PART ****************************************************/



int DC_motor::dc_init()
{
  /*
   * Set IO9,IO10Pin Fast PWM Mode & PWMF 62.500[kHz]
   */
  TCCR1B &= B11100000;
  TCCR1B |= B00001001;
  TCCR1A &= B11111100;
  TCCR1A |= B00000001;
   /*
   * Set D5Pin Fast PWM Mode & PWMF 62.500[kHz]
   */
  TCCR3B &= B11100000;
  TCCR3B |= B00001001;
  TCCR3A &= B11111100;
  TCCR3A |= B00000001;
  /*
   * Set D6Pin Fast PWM Mode & PWMF 62.500[kHz]
   */
	
  TCCR4B &= B11110000;
  TCCR4B |= B00000001;
  TCCR4D = 0x00;

  inPin[0] = D6;
  inPin[1] = D5;
  inPin[2] = IO9;
  inPin[3] = IO10;
  //set PinMode
  for(int i=0;i<4;i++){
    setPinMode(inPin[i]);
  }
  
  // set init status.
  uint8_t result = SUCCESS;
  result |= setRotation(MOTOR_NO1,ROTATION_SB_SB);
  result |= setCtlVoltage(MOTOR_NO1,DUTY_MIN);
  result |= setRotation(MOTOR_NO2,ROTATION_SB_SB);
  result |= setCtlVoltage(MOTOR_NO2,DUTY_MIN);
  result |= setStartFlag(MOTOR_NO1,FLAG_OFF);
  result |= setStartFlag(MOTOR_NO2,FLAG_OFF);
  result |= setPWMFrequency(1);

  if(SUCCESS == result)
  {
    return SUCCESS;
  }
  else
  {
    return FAILURE;
  }
}


int DC_motor::setTCCR0()
{
  /* 
   *  Set D3,IO11Pin PWMF 7.813[kHz]
   */
  TCCR0B &= B11111000;
  TCCR0B |= B00000010;
}

/*!
 *  \fn     Lib_LV8548DC::setTCCR1()
 *  \brief  Local method.
 */
int DC_motor::setTCCR1()
{
  /*
   * Set IO10Pin,IO9Pin Fast PWM Mode & PWMF 7.813[kHz]
   */
  TCCR1B &= B11100000;
  TCCR1B |= B00001010;
  TCCR1A &= B11111100;
  TCCR1A |= B00000001;
}

/*!
 *  \fn     Lib_LV8548DC::setTCCR3()
 *  \brief  Local method.
 */
int DC_motor::setTCCR3()
{
   /*
   * Set D5Pin Fast PWM Mode & PWMF 7.813[kHz]
   */
  TCCR3B &= B11100000;
  TCCR3B |= B00001010;
  TCCR3A &= B11111100;
  TCCR3A |= B00000001;
}

/*!
 *  \fn     Lib_LV8548DC::setTCCR4()
 *  \brief  Local method.
 */
int DC_motor::setTCCR4()
{
   /*
   * Set D6Pin,IO13Pin Fast PWM Mode & PWMF 7.813[kHz]
   */
  TCCR4A = 0b10100011;
  TCCR4B &= B11110000;
  TCCR4B |= B00000100;
  TCCR4D = 0x00;
}

int DC_motor::initLibPortSet(byte in1,byte in2,byte in3,byte in4)
{
    byte in[4] = {in1,in2,in3,in4};
    for(int j = 0 ; j < 4 ; j++){
        switch(in[j]){
            case 0: return 1;//pin Error
            case 1: return 1;//pin Error
            case 2: return 1;//not PWM Port
            case 3: 
              setTCCR0();
              inPin[j] = D3;
              break;
            case 4: return 1;//not PWM Port
            case 5:
              setTCCR3();
              inPin[j] = D5;
              break;
            case 6:
              setTCCR4();
              inPin[j] = D6;
              break;
            case 7:return 1;//not PWM Port
            case 8:return 1;//not PWM Port
            case 9:
              setTCCR1();
              inPin[j] = IO9;
              break;
            case 10:
              setTCCR1();
              inPin[j] = IO10;
              break;
            case 11:
              setTCCR0();
              inPin[j] = IO11;
              break;
            case 12:return 1;//not PWM Port
            case 13:
              setTCCR4();
              inPin[j] = IO13;
              break;
            default: return 1;
        }
    }
    //set PinMode
    for(int i=0;i<4;i++){
        setPinMode(inPin[i]);
    }

    // set init status.
    uint8_t result = SUCCESS;
    result |= setRotation(MOTOR_NO1, ROTATION_SB_SB);
    result |= setCtlVoltage(MOTOR_NO1, DUTY_MIN);
    result |= setRotation(MOTOR_NO2, ROTATION_SB_SB);
    result |= setCtlVoltage(MOTOR_NO2, DUTY_MIN);
    result |= setStartFlag(MOTOR_NO1, FLAG_OFF);
    result |= setStartFlag(MOTOR_NO2, FLAG_OFF);
    result |= setPWMFrequency(1);

    if (SUCCESS == result)
    {
        return SUCCESS;
    }
    else
    {
        return FAILURE;
    }
}

/** Set DC motor speed
 *  P1 :This module have four OUT PINS, support two motor interfaces，choose the number of motor.
 *  P2 :The speed of motor ,range :0-100.
 *  return : FAILURE or SUCCESS
 * */
int DC_motor::setCtlVoltage(byte motorNo,byte duty)
{
  // motor range check
  if(( motorNo < MOTOR_NO1) || ( motorNo > MOTOR_NO2))
  {
    return FAILURE;
  }
  // duty range check
  if ((duty < DUTY_MIN) || (duty > DUTY_MAX))
  {
    // out of range
    return FAILURE;
  }
  
  // convert duty rate to duty value
  byte dutyVal = dutyRateToValue(duty);
  if(motorNo == MOTOR_NO1){
    motor1Duty = duty;
    motorRotation(MOTOR_NO1);
  }
  else{
    motor2Duty = duty;
    motorRotation(MOTOR_NO2);
  }
#ifdef GUI_DEBUG
  char buf[256];
  sprintf(buf,"Change Duty MotorNo = %d duty=%d \n",motorNo,duty);
  Serial.write(buf);
#endif
  return SUCCESS;
}

/** Set DC motor direction
 *  P1 :This module have four OUT PINS, support two motor interfaces，choose the number of motor.
 *  P2 :The direction of motor rotation.
 *  return : FAILURE or SUCCESS
 * */
int DC_motor::setRotation(byte motorNo,byte select)
{
  // motor range check
  if(( motorNo < MOTOR_NO1) || ( motorNo > MOTOR_NO2))
  {
    return FAILURE;
  }
  // If select is out of the range, return FAILURE
  if (select < ROTATION_FW_SB || select > ROTATION_BK_BK)
  {
    return FAILURE;
  }
#ifdef GUI_DEBUG
  char buf[256];
  sprintf(buf,"Change PWM Mode  MotorNo = %d mode=%d \n",motorNo,select);
  Serial.print(buf);
#endif
  if(motorNo == MOTOR_NO1){
    motor1RotateMode = select;
    motorRotation(MOTOR_NO1);
  }
  else{
    motor2RotateMode = select;
    motorRotation(MOTOR_NO2);
  }
  return SUCCESS;
}

/** Set DC motor running stat.
 *  P1 :This module have four OUT PINS, support two motor interfaces，choose the number of motor.
 *  P2 :The running stat of motor . 0 - standby stopped,1- running ,2- brake stopped.
 *  return : FAILURE or SUCCESS
 * */
int DC_motor::setStartFlag(byte motorNo,byte select)
{
   // motor range check
  if(( motorNo < MOTOR_NO1) || ( motorNo > MOTOR_NO2))
  {
    return FAILURE;
  }
  // If select is out of the range, return FAILURE
  if (select < FLAG_OFF || select > FLAG_BRAKE)
  {
    return FAILURE;
  }
#ifdef GUI_DEBUG
  char buf[256];
  sprintf(buf,"Change StartFlag  MotorNo = %d flag=%d \n",motorNo,select);
  Serial.print(buf);
#endif
  if(motorNo == MOTOR_NO1){
    motor1StartFlag = select;
    motorRotation(MOTOR_NO1);
  }
  else{
    motor2StartFlag = select;
    motorRotation(MOTOR_NO2);
  }
  return SUCCESS;
}


/**
 *  Set motor params and running stat.
 * P1: This module have four OUT PINS, support two motor interfaces，choose the number of motor.
 * p2: Frequency of motor.Does not distinguish between specific motor numbers,01-7.813k，02-0.977khz  03-0.244khz  04-0.061khz
 * P3: The direction of motor rotation.
 * p4: Speed of motor rotation.range: 0-100
 * p5: The running stat of motor . 0 - standby stopped,1- running ,2- brake stopped
 * */

void DC_motor::set_dc_motor_params(char motor_num,char freq,char direc,char speed,char start_flag)
{
    setPWMFrequency(freq);
    setRotation(motor_num, direc);
    setCtlVoltage(motor_num, speed);
    setStartFlag(motor_num, start_flag);
}

byte DC_motor::dutyRateToValue(int rate)
{
  return (byte)round(((double )(rate * 255) / 100));
}

/*!
 *  \fn     Lib_LV8548DCV::dutyRateToReverseValue(byte rate)
 *  \brief  Local method.
 */
byte DC_motor::dutyRateToReverseValue(byte rate)
{
  return (100 - rate);
}

/*!
 *  \fn     Lib_LV8548DCV::motorRotation(byte motorNo)
 *  \brief  Convert duty rate (%) to duty value.

 *  \param  motorNo     Specify motor number.
 *                      0x00(0) motor 1.
 *                      0x01(1) motor 2.
 *
 *  \return The result of executing the API.
 *          0 is success.
 *          1 is failure.
 */
int DC_motor::motorRotation(byte motorNo)
{  
  // convert duty rate to duty value
  if(motorNo == MOTOR_NO1){
    if(motor1StartFlag == FLAG_OFF){
        digitalWrite(inPin[1],LOW);
        digitalWrite(inPin[0],LOW);
        return SUCCESS;
    }
      if(motor1StartFlag == FLAG_BRAKE){
        digitalWrite(inPin[1],HIGH);
        digitalWrite(inPin[0],HIGH);
        return SUCCESS;
    }
    byte dutyVal = dutyRateToValue(motor1Duty);
    switch(motor1RotateMode){
        case ROTATION_FW_BK:
            analogWrite(inPin[0],dutyRateToReverseValue(dutyVal));
            digitalWrite(inPin[1],HIGH);
            break;
        case ROTATION_FW_SB:
            analogWrite(inPin[1],dutyVal);
            digitalWrite(inPin[0],LOW);
            break;
        case ROTATION_RV_BK:
            analogWrite(inPin[1],dutyRateToReverseValue(dutyVal));
            digitalWrite(inPin[0],HIGH);
            break;
        case ROTATION_RV_SB:
            analogWrite(inPin[0],dutyVal);
            digitalWrite(inPin[1],LOW);
            break;
        case ROTATION_SB_SB:
            digitalWrite(inPin[1],LOW);
            digitalWrite(inPin[0],LOW);
            break;
        case ROTATION_BK_BK:
            digitalWrite(inPin[1],HIGH);
            digitalWrite(inPin[0],HIGH);
            break;
    }
  }
  else{
    if(motor2StartFlag == FLAG_OFF){
        digitalWrite(inPin[2],LOW);
        digitalWrite(inPin[3],LOW);
        return SUCCESS;
    }
      if(motor2StartFlag == FLAG_BRAKE){
        digitalWrite(inPin[2],HIGH);
        digitalWrite(inPin[3],HIGH);
        return SUCCESS;
    }
    byte dutyVal = dutyRateToValue(motor2Duty);
    switch(motor2RotateMode){
        case ROTATION_FW_BK:
            analogWrite(inPin[3],dutyRateToReverseValue(dutyVal));
            digitalWrite(inPin[2],HIGH);
            break;
        case ROTATION_FW_SB:
            analogWrite(inPin[2],dutyVal);
            digitalWrite(inPin[3],LOW);
            break;
        case ROTATION_RV_BK:
            analogWrite(inPin[2],dutyRateToReverseValue(dutyVal));
            digitalWrite(inPin[3],HIGH);
            break;
        case ROTATION_RV_SB:
            analogWrite(inPin[3],dutyVal);
            digitalWrite(inPin[2],LOW);
            break;
        case ROTATION_SB_SB:
            digitalWrite(inPin[2],LOW);
            digitalWrite(inPin[3],LOW);
            break;
        case ROTATION_BK_BK:
            digitalWrite(inPin[2],HIGH);
            digitalWrite(inPin[3],HIGH);
            break;
    }
  }
}


/** Set DC motor frequency.
 *  p1: Frequency of motor.Does not distinguish between specific motor numbers,01-7.813k，02-0.977khz  03-0.244khz  04-0.061khz
 *  return : FAILURE or SUCCESS
 * */
int DC_motor::setPWMFrequency(byte selectNo)
{
  byte cs = B00000000;
  pwmFreq = selectNo;
  if((selectNo < PWM_FREQ_DividingRatio_8)||(selectNo > PWM_FREQ_DividingRatio_1024)){
      return FAILURE;
  }
  for(int i = 0 ; i < 4 ;i++){
    switch(selectNo){
        case PWM_FREQ_DividingRatio_8: cs = B00000010; break;
        case PWM_FREQ_DividingRatio_64: cs = B00000011; break;
        case PWM_FREQ_DividingRatio_256: cs = B00000100; break;
        case PWM_FREQ_DividingRatio_1024: cs = B00000101; break;
        default: return FAILURE;
  }
    switch(inPin[i]){
        case IO11:
        case D3:
      {
        TCCR0B &= B11111000;
        TCCR0B |= cs;
        break;
      }
        case IO10:
        case IO9:
      {
        TCCR1B &= B11111000;
        TCCR1B |= cs;
        break;
      }
        case D5:
      {
        TCCR3B &= B11111000;
        TCCR3B |= cs;
        break;
      }
        case D6:
        case IO13:
      {
        switch(selectNo){
          case PWM_FREQ_DividingRatio_8: cs = B00000100; break;
          case PWM_FREQ_DividingRatio_64: cs = B00000111; break;
          case PWM_FREQ_DividingRatio_256: cs = B00001001; break;
          case PWM_FREQ_DividingRatio_1024: cs = B00001011; break;
        }
        TCCR4B &= B11110000;
        TCCR4B |= cs;
        break;
      }
    }
  }
#ifdef GUI_DEBUG
  char buf[256];
  sprintf(buf,"Change Frequency = %d \n",selectNo);
  Serial.write(buf);
#endif

  return SUCCESS;
}

void DC_motor::displayAllforTest(){
    char buf[256];
    sprintf(buf,"*******************************************\n");
    Serial.write(buf);
    sprintf(buf,"MotorPWMMode1 : %d\n",motor1RotateMode);
    Serial.write(buf);
    sprintf(buf,"MotorPWMMode2 : %d\n",motor2RotateMode);
    Serial.write(buf);
    sprintf(buf,"motor1Duty    : %d\n",(int)motor1Duty);
    Serial.write(buf);
    sprintf(buf,"motor2Duty    : %d\n",(int)motor2Duty);
    Serial.write(buf);
    sprintf(buf,"motor1StartFlg: %d\n",motor1StartFlag);
    Serial.write(buf);
    sprintf(buf,"motor2StartFlg: %d\n",motor2StartFlag);
    Serial.write(buf);
    sprintf(buf,"In Pin        : %d,%d,%d,%d\n",inPin[0],inPin[1],inPin[2],inPin[3]);
    Serial.write(buf);
    sprintf(buf,"TimerRegister0: %x,%x\n",TCCR0A,TCCR0B);
    Serial.write(buf);
    sprintf(buf,"TimerRegister1: %x,%x\n",TCCR1A,TCCR1B);
    Serial.write(buf);
    sprintf(buf,"TimerRegister3: %x,%x\n",TCCR3A,TCCR3B);
    Serial.write(buf);
    sprintf(buf,"TimerRegister4: %x,%x\n",TCCR4A,TCCR4B);
    Serial.write(buf);
    sprintf(buf,"*******************************************\n");
    Serial.write(buf);
}



/**********************************************************************************************************/
/****************************************STEP  PART ****************************************************/
void STEP_motor::step_init()
{
    inPin[0] = D5;
	inPin[1] = D6;
	inPin[2] = IO9;
	inPin[3] = IO10;

	for (int i = 0; i < 4; i++) {
		setPinMode(inPin[i]);
	}
	return SUCCESS;
}


/** Running the motor ,end of target degrees.
 *  P1: frequency ,determines the speed of step motor.
 *  P2: target deg,Stop the motor when the target degree is reached,if set 0,Indicate that there is no stop condition. 
 *  P3: direction of motor rotation
 *  P4: excitation of motor rotation.Divided into FULLSTEP/HALFSTEP
 *  return : FAILURE or SUCCESS.
 * */
int STEP_motor::motorRotationDeg(float freq, float deg, byte cwccw, byte exc)
{
	byte tmp;
	if ((freq < FREQ_MIN) || (freq > FREQ_MAX))
	{
		return FAILURE;
	}
	if ((deg < DEG_MIN) || (deg > DEG_MAX))
	{
		return FAILURE;
	}
	if ((cwccw < ROTATION_CW) || (cwccw > ROTATION_CCW))
	{
		return FAILURE;
	}
	if ((exc < FULLSTEP) || (exc > HALFSTEP))
	{
		return FAILURE;
	}
	if (Excitation != exc) PhaseCounter = 0;    
	if (CwCcw != cwccw)                         
	{
		if(cwccw == ROTATION_CW){                
			PhaseCounter++;                      
			if(PhaseCounter >= (exc == FULLSTEP ? 4 : 8)){   
				PhaseCounter = 0;
			}
			
			PhaseCounter++; 
			if(PhaseCounter >= (exc == FULLSTEP ? 4 : 8)){
				PhaseCounter = 0;
			}
		}
		else{                                    
			PhaseCounter--; 
			if(PhaseCounter < 0){
				PhaseCounter = (exc == FULLSTEP ? 3 : 7);
			}
			PhaseCounter--;
			if(PhaseCounter < 0){
				PhaseCounter = (exc == FULLSTEP ? 3 : 7);
			}
		}
	}
	StepFrequency = freqChange(freq, exc);     
	Target_Step = (uint32_t)(deg / StepDeg * (exc == FULLSTEP ? 1.0F : 2.0F));  
	CwCcw = cwccw;    
	Excitation = exc;  
	Now_Step = 0;       
	//TimerCounter = (1000000.0F / StepFrequency);
	//TimerCounter = 0;
	isRotation = true;   
	return SUCCESS;
}


/** Running the motor ,end of target time.
 *  P1: frequency ,determines the speed of step motor.
 *  P2: target time,unit is second,Stop the motor when the target time is reached,if set 0,Indicate that there is no stop condition. 
 *  P3: direction of motor rotation
 *  P4: excitation of motor rotation.Divided into FULLSTEP/HALFSTEP
 *  return : FAILURE or SUCCESS.
 * */
int STEP_motor::motorRotationTime(float freq, uint16_t time, byte cwccw, byte exc)
{
	if ((freq < FREQ_MIN) || (freq > FREQ_MAX))
	{
		return FAILURE;
	}
	if ((time < TIME_MIN) || (time > TIME_MAX))
	{
		return FAILURE;
	}
	if ((cwccw < ROTATION_CW) || (cwccw > ROTATION_CCW))
	{
		return FAILURE;
	}
	if ((exc < FULLSTEP) || (exc > HALFSTEP))
	{
		return FAILURE;
	}
	if (Excitation != exc) PhaseCounter = 0;     
	motorRotationFree();                         
	
	uint32_t temp1 = time;
	uint32_t temp2 = freq;
	StepFrequency = (freqChange(freq, exc));    
	
	uint32_t j = temp1 * temp2;

	Target_Step = j;                            
	CwCcw = cwccw;                              
	Excitation = exc;                           
	Now_Step = 0;                               
	

	
	
	isRotation = true;                          
	return SUCCESS;
}

/** Running the motor ,end of target steps.
 *  P1: frequency ,determines the speed of step motor.
 *  P2: target steps,Stop the motor when the target steps is reached,if set 0,Indicate that there is no stop condition. 
 *  P3: direction of motor rotation
 *  P4: excitation of motor rotation.Divided into FULLSTEP/HALFSTEP
 *  return : FAILURE or SUCCESS.
 * */
int STEP_motor::motorRotationStep(float freq, uint32_t step, byte cwccw, byte exc)
{
	byte tmp;
	if ((freq < FREQ_MIN) || (freq > FREQ_MAX))
	{
		return FAILURE;
	}
	if ((step < STEP_MIN) || (step > STEP_MAX))
	{
		return FAILURE;
	}
	if ((cwccw < ROTATION_CW) || (cwccw > ROTATION_CCW))
	{
		return FAILURE;
	}
	if ((exc < FULLSTEP) || (exc > HALFSTEP))
	{
		return FAILURE;
	}
	if (Excitation != exc) PhaseCounter = 0;
	
	if (CwCcw != cwccw)            
	{
		if(cwccw == ROTATION_CW){
			PhaseCounter++;                                    
			if(PhaseCounter >= (exc == FULLSTEP ? 4 : 8)){
				PhaseCounter = 0;
			}
			
			PhaseCounter++; 
			if(PhaseCounter >= (exc == FULLSTEP ? 4 : 8)){
				PhaseCounter = 0;
			}
		}
		else{
			PhaseCounter--; 
			if(PhaseCounter < 0){
				PhaseCounter = (exc == FULLSTEP ? 3 : 7);
			}
			PhaseCounter--;
			if(PhaseCounter < 0){
				PhaseCounter = (exc == FULLSTEP ? 3 : 7);
			}
		}
	}
	StepFrequency = freqChange(freq, exc);                  
	Target_Step = step;
	CwCcw = cwccw;
	Excitation = exc;
	Now_Step = 0;
	isRotation = true;
	return SUCCESS;
}

/**Stop the motor.
 * 
 * */
void STEP_motor::motorRotationFree()
{
	isRotation = false;
	TimerCounter = 0.0F;
	PhaseCounter = 0;
	Now_Step = 0;
	for (int i = 0; i < 4; i++)
	{
		digitalWrite(inPin[i], LOW);
		
	}
}


/**Set the param angle/step,defualt value is 7.5.This value is fixed to the same motor.Different motors have different values
 * 
 * 
 * */
int STEP_motor::setStepAngle(float deg)
{
	if ((deg < ANGLE_MIN) || (deg > ANGLE_MAX))
	{
		return FAILURE;
	}
	StepDeg = deg;
	return SUCCESS;
}

float STEP_motor::freqChange(float freq, byte exp)
{
	return freq;

	if (exp == FULLSTEP) {
		return freq * 4.0F;
	}
	else {
		return freq * 8.0F;
	}

}


/**
 *  called in TIMER callback.
 *  Check if there is a task running.
 * */
void STEP_motor::timerFire(long timer)  
{
	byte tmp;
	unsigned long time_millisStart, time_millisEnd;
	TimerCounter += (float)timer;
	if (isRotation) {                                       

		if (TimerCounter >= (1000000.0F / (StepFrequency))) {    
		time_millisStart = micros();
			
			if (Target_Step != 0) {
				if (Now_Step >= (Target_Step))                
				{
					motorRotationFree();
					return;
				}
			}
			for (int i = 0; i < 4; i++) {
				if (Excitation == FULLSTEP) {
					digitalWrite(inPin[i], FullstepForward[PhaseCounter][i]);
				}
				else {
					digitalWrite(inPin[i], HalfstepForward[PhaseCounter][i]);
				}
			}
			if(CwCcw == ROTATION_CW){
				PhaseCounter++;
				tmp = (Excitation == FULLSTEP ? 4 : 8);
				if (PhaseCounter >= tmp) {
					PhaseCounter = 0;
				}
			}
			else{
				PhaseCounter--;
				tmp = (Excitation == FULLSTEP ? 4 : 8);
				if (PhaseCounter <= -1) {
					PhaseCounter = tmp-1;
				}
			}
			Now_Step++;
			time_millisEnd = micros();
			TimerCounter = 0.0F;
			//TimerCounter = (float)(time_millisEnd - time_millisStart);
		}
	}
}

void STEP_motor::displayAllforTest() {
	char buf[256];
	sprintf(buf, "*******************************************\n");
	Serial.write(buf);
	dtostrf(StepDeg, 5, 2, buf);
	Serial.write("StepDeg:");
	Serial.write(buf);
	Serial.write("\n");
	dtostrf(StepFrequency, 7, 2, buf);
	Serial.write("StepFrequency:");
	Serial.write(buf);
	Serial.write("\n");
	sprintf(buf, "Target_Step: %u\n", Target_Step);
	Serial.write(buf);
	sprintf(buf, "CwCcw: %d\n", CwCcw);
	Serial.write(buf);
	sprintf(buf, "Excitation: %d\n", Excitation);
	Serial.write(buf);
	sprintf(buf, "isRotation: %d\n", isRotation);
	Serial.write(buf);
	sprintf(buf, "PhaseCounter: %d\n", PhaseCounter);
	Serial.write(buf);
	dtostrf(Now_Step, 5, 2, buf);
	Serial.write("Now_Step:");
	Serial.write(buf);
	Serial.write("\n");
	sprintf(buf, "*******************************************\n");
	Serial.write(buf);
}

/**********************************************************************************************/
/**************************************MOTOR PART **************************************************/

void LV8548_Motor::motor_init()
{
    if(DC_MOTOR==motor_type)
    {
        dc_init();
    }
    else
    {
        step_init();
    }
}


u8 LV8548_Motor::get_motor_type()
{
    return motor_type;
}

void LV8548_Motor::set_motor_type(u8 type)
{
    motor_type=type;
}

/** Parse the data recieved.
 *  P1 : recieve buf
 *  P2 : recieve buf length
 * */
void LV8548_Motor::recv_buf_parse(unsigned char* buf,unsigned int len)
{
	switch(buf[0])
	{
		case GET_ID://send_id();    /**< retain*/
		break;
		case SET_DC_ROTATION_DIRECTION:   
			 setRotation(buf[1],buf[2]);
       //Serial1.println("set rotation");
		break;
		case SET_DC_MOTOR_VOLTAGE:
			setCtlVoltage(buf[1],buf[2]);
      //Serial1.println("set voatage");
		break;
		case SET_DC_PWM_FREQ:
			setPWMFrequency(buf[1]);  
      //Serial1.println("setPWMFrequency");
		break;
		case SET_DC_MOTOR_START_FLAG:
			setStartFlag(buf[1],buf[2]);    
      //Serial1.println("setStartFlag");
		break;

		case SET_STEP_ANGLE:
			setStepAngle((float)buf[1]/(float)buf[2]);
      //Serial1.println("SET_STEP_ANGLE");
		break;
		case SET_STEP_RUN_WITH_ENDOF_DEGREE:
			motorRotationDeg((float)((buf[1]<<8)|buf[2]),(buf[3]<<8)|buf[4],buf[5],buf[6]);
      //Serial1.println("motorRotationDeg");
		break;
		case SET_STEP_RUN_WITH_ENDOF_STEPS:
      //Serial1.println("motorRotationStep");
			motorRotationStep((float)((buf[1]<<8)|buf[2]),(buf[3]<<8)|buf[4],buf[5],buf[6]);
			
		break;
		case SET_STEP_RUN_WITH_ENDOF_TIME:
      //Serial1.println("motorRotationTime");
			 motorRotationTime((float)((buf[1]<<8)|buf[2]),(buf[3]<<8)|buf[4],buf[5],buf[6]);
			 
		break;
		case SET_STEP_STOP:
			motorRotationFree();  
		break;
    case SET_DC_MOTOR_PARAMS:
      set_dc_motor_params(buf[1],buf[2],buf[3],buf[4],buf[5]);
      
      break;
		default:break;
	}
}


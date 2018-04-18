#ifndef _SEEED_LV8548_MOTOR_DRIVER_H
#define _SEEED_LV8548_MOTOR_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
//#include <stdout.h>

#define MAX_RECV_BUF_LEN  128
#define DC_MOTOR    1
#define STEP_MOTOR  2


typedef struct
{
    u8 recv_buf[MAX_RECV_BUF_LEN];
    u32 recv_buf_len;
    u32 recv_over_flag;
}recv_struct_t;



typedef enum
{
	SELECT_MOTOR_TYPE=0X1,
	GET_ID=2,  
	SET_DC_ROTATION_DIRECTION=3,
	SET_DC_MOTOR_VOLTAGE=4,
	SET_DC_PWM_FREQ=5,
	SET_DC_MOTOR_START_FLAG=6,

	SET_STEP_ANGLE=7,
	SET_STEP_RUN_WITH_ENDOF_DEGREE=8,
	SET_STEP_RUN_WITH_ENDOF_STEPS=9,
	SET_STEP_RUN_WITH_ENDOF_TIME=10,
	RETAIN=11,
	SET_STEP_STOP=12,
}protocol_enum_t;





//stypedef unsigned int    u32;
typedef unsigned short  u16;
typedef unsigned char   u8;
typedef int             s32;
typedef short           s16;
typedef char            s8;



class motorControlLibraryBase
{
    public:
        static const int SUCCESS  = 0;   //!< success value define
        static const int FAILURE  = 1;   //!< failure value define
        static const int RESTRICT = 2;   //!< prohibition operate define
        static const uint8_t RX   = 0;   //!< Arduino micro RX pin number define
        static const uint8_t TX   = 1;   //!< Arduino micro TX pin number define
        static const uint8_t D2   = 2;   //!< Arduino micro Degital pin number define
        static const uint8_t D3   = 3;   //!< Arduino micro Degital pin number define
        static const uint8_t D4   = 4;   //!< Arduino micro Degital pin number define
        static const uint8_t D5   = 5;   //!< Arduino micro Degital pin number define
        static const uint8_t D6   = 6;   //!< Arduino micro Degital pin number define
        static const uint8_t D7   = 7;   //!< Arduino micro Degital pin number define
        static const uint8_t IO8  = 8;   //!< Arduino micro I/O pin number define
        static const uint8_t IO9  = 9;   //!< Arduino micro I/O pin number define
        static const uint8_t IO10 = 10;  //!< Arduino micro I/O pin number define
        static const uint8_t IO11 = 11;  //!< Arduino micro I/O pin number define
        static const uint8_t IO12 = 12;  //!< Arduino micro I/O pin number define
        static const uint8_t IO13 = 13;  //!< Arduino micro I/O pin number define
        static const uint8_t MISO = 14;  //!< Arduino micro MISO pin number define
        static const uint8_t SCK  = 15;  //!< Arduino micro SCK  pin number define
        static const uint8_t MOSI = 16;  //!< Arduino micro MOSI pin number define
        static const uint8_t SS   = 17;  //!< Arduino micro SS   pin number define
        static const uint8_t A0   = 18;  //!< Arduino micro Analog pin number define
        static const uint8_t A1   = 19;  //!< Arduino micro Analog pin number define
        static const uint8_t A2   = 20;  //!< Arduino micro Analog pin number define
        static const uint8_t A3   = 21;  //!< Arduino micro Analog pin number define
        static const uint8_t A4   = 22;  //!< Arduino micro Analog pin number define
        static const uint8_t A5   = 23;  //!< Arduino micro Analog pin number define
        
};

class DC_motor:public motorControlLibraryBase
{
    public:
        DC_motor(){}
        ~DC_motor(){}
        int dc_init();
        int initLibPortSet(byte in1,byte in2,byte in3,byte in4);
        int setPWMFrequency(byte hz);
        int setCtlVoltage(byte motorNo,byte duty);
        int setRotation(byte motorNo,byte select);
        int setStartFlag(byte motorNo,byte select);
        void displayAllforTest();
        void set_dc_motor_params(char motor_num,char freq,char direc,char speed,char start_flag);
    protected:

        uint8_t motor1RotateMode = ROTATION_FW_BK;
        uint8_t motor2RotateMode = ROTATION_FW_BK;
        float motor1Duty = DUTY_MIN;
        float motor2Duty = DUTY_MIN;
        uint8_t motor1StartFlag = FLAG_OFF;
        uint8_t motor2StartFlag = FLAG_OFF;
        uint8_t inPin[4];
        byte dutyRateToValue(int rate);
        byte dutyRateToReverseValue(byte rate);
        void timeoutPol();
        int motorRotation(byte motorNo);
        int setTCCR0();
        int setTCCR1();
        int setTCCR3();
        int setTCCR4();
        int setPinMode(byte pin){pinMode(pin, OUTPUT);};
        static const byte PWM_FREQ_DividingRatio_8    = 0;  //division ratio 1/8 7.813kHz
        static const byte PWM_FREQ_DividingRatio_64   = 1;  //division ratio 1/64 0.977kHz
        static const byte PWM_FREQ_DividingRatio_256  = 2;  //division ratio 1/256 0.244kHz
        static const byte PWM_FREQ_DividingRatio_1024 = 3;  //division ratio 1/1024 0.061kHz
        uint8_t pwmFreq = 1;

    private:
        static const uint8_t DUTY_MIN       = 0;     //!< duty rate 0% = 0
        static const uint8_t DUTY_MAX       = 100;   //!< duty rate 100% = 255

        
        static const uint8_t FLAG_OFF       = 0;     //!< set start flag off.
        static const uint8_t FLAG_ON        = 1;     //!< set start flag on.
        static const uint8_t FLAG_BRAKE     = 2;     //!< set start flag brake.

        static const uint8_t ROTATION_FW_BK = 1;     //!< set foward & brake rotation.
        static const uint8_t ROTATION_FW_SB = 0;     //!< set foward & standby rotation.
        static const uint8_t ROTATION_RV_BK = 3;     //!< set reverse & brake rotation.
        static const uint8_t ROTATION_RV_SB = 2;     //!< set reverse & standby rotation.
        static const uint8_t ROTATION_SB_SB = 4;     //!< set stanby & standby rotation.(Stop)
        static const uint8_t ROTATION_BK_BK = 5;     //!< set brake & brake rotation.(Stop)

        static const uint8_t MOTOR_NO1      = 0;     //!< set motor number1.
        static const uint8_t MOTOR_NO2      = 1;     //!< set motor number2.
        

        
        // Serial message identiffer types
        static const byte SRMES_GET_ID      = 0x03;         //  3
        static const byte SRMES_POLLING_ID  = 0x04;         //  4
        static const byte SRMES_CTL_VOLTAGE = 0x41;         // 'A'
        static const byte SRMES_ROTATION    = 0x44;         // 'D'
        static const byte SRMES_PWM_FREQSET = 0x67;         // 'g'
        static const byte SRMES_START_FLAG  = 0x68;         // 'h'

        
        // Segment definition of serial message
        typedef struct
        {
        byte identiffer;
        byte motorNo;
        byte duty;
        } SrMesDivCtlVoltage;

        typedef struct
        {
        byte identiffer;
        byte motorNo;
        byte select;
        } SrMesDivStartFlag;

        typedef struct
        {
        byte identiffer;
        byte motorNo;
        byte select;
        } SrMesDivRotation;
        
        typedef struct
        {
        byte identiffer;
        byte hz;
        } SrMesDivPwmFreqset;
        
};


class STEP_motor:public motorControlLibraryBase
{
    public:
        STEP_motor(){StepDeg=7.5;}
        ~STEP_motor(){}
        void step_init();
        int setStepAngle(float deg);
        int motorRotationDeg(float freq, float deg, byte cwccw, byte exc);
        int motorRotationTime(float freq, uint16_t time, byte cwccw, byte exc);
        int motorRotationStep(float freq, uint32_t step, byte cwccw, byte exc);
        void motorRotationStop();
        void motorRotationFree();
        void timerFire(long tm);
        void displayAllforTest();
        inline float show_step_deg(void){return StepDeg;}
    protected:
        byte inPin[4];
        float StepDeg = ANGLE_MIN;
        float StepFrequency = FREQ_MIN;
        byte Excitation = FULLSTEP;
        byte CwCcw = ROTATION_CW;
        uint32_t Target_Step = 0;
        float TimerCounter = 0;
        int8_t PhaseCounter = 0;
        uint32_t Now_Step = 0;
        bool isRotation = false;
        int setPinMode(byte pin){pinMode(pin, OUTPUT);}
        float freqChange(float freq, byte exp);
        //void timeoutPol();
    private:
        static const uint8_t ROTATION_CW = 0;
        static const uint8_t ROTATION_CCW = 1;
        static const uint8_t FULLSTEP = 0;
        static const uint8_t HALFSTEP = 1;
        static const uint16_t FREQ_MIN = 1;
        static const uint16_t FREQ_MAX = 4800;
        static const uint16_t DEG_MIN = 0;
        static const uint16_t DEG_MAX = 65535;
        static const uint16_t TIME_MIN = 0;
        static const uint16_t TIME_MAX = 65535;
        static const uint16_t STEP_MIN = 0;
        static const uint16_t STEP_MAX = 65535;
        static const uint16_t ANGLE_MIN = 0.01;
        static const uint16_t ANGLE_MAX = 360;

        // Serial message identiffer types
        static const byte SRMES_GET_ID = 0x03;		   // 3
        static const byte SRMES_POLLING_ID  = 0x04;         //  4
        static const byte SRMES_STEP_ANGLE = 0x69;     // 'i'
        static const byte SRMES_ROTATION_ANGLE = 0x6A; // 'j'
        static const byte SRMES_ROTATION_TIME = 0x6B;  // 'k'
        static const byte SRMES_ROTATION_STEP = 0x6C;  // 'l'
        static const byte SRMES_ROTATION_STOP = 0x6E;  // 'n'
        static const byte SRMES_ROTATION_FREE = 0x6F;  // 'o'

        static const byte PWM_FREQ_DividingRatio_1 = 0;///1 62.5kHz
        static const byte PWM_FREQ_DividingRatio_8 = 1;///8 7.813kHz
        static const byte PWM_FREQ_DividingRatio_64 = 2;///64 0.977kHz
        static const byte PWM_FREQ_DividingRatio_256 = 3;///256 0.244kHz
        static const byte PWM_FREQ_DividingRatio_1024 = 4;///1024 0.061kHz

        typedef struct
        {
            byte identiffer;
            byte angle[2];
        } SrMesDivSetStepAngle;

        typedef struct
        {
            byte identiffer;
            byte frequency[2];
            byte deg[4];
            byte rotation;
            byte exp;
        } SrMesDivRotationDeg;

        typedef struct
        {
            byte identiffer;
            byte frequency[2];
            byte time[2];
            byte rotation;
            byte exp;
        } SrMesDivRotationTime;

        typedef struct
        {
            byte identiffer;
            byte frequency[2];
            byte step[4];
            byte rotation;
            byte exp;
        } SrMesDivRotationStep;

        typedef struct
        {
            byte identiffer;
        } SrMesDivRotationStop;

        typedef struct
        {
            byte identiffer;
        } SrMesDivRotationFree;
        const byte FullstepForward[4][4] = { {HIGH,LOW,HIGH,LOW},{LOW,HIGH,HIGH,LOW},{LOW,HIGH,LOW,HIGH},{HIGH,LOW,LOW,HIGH} };
        const byte HalfstepForward[8][4] = {{HIGH,LOW,LOW,LOW},{HIGH,LOW,HIGH,LOW},{LOW,LOW,HIGH,LOW},
								{LOW,HIGH,HIGH,LOW},{LOW,HIGH,LOW,LOW},{LOW,HIGH,LOW,HIGH},{LOW,LOW,LOW,HIGH},{HIGH,LOW,LOW,HIGH}};

};


class LV8548_Motor:public STEP_motor,public DC_motor
{
    public:
        void set_motor_type(u8 type);    //motor type:Step or DC
        u8 get_motor_type();
        char* get_motor_id(){if(motor_type==DC_MOTOR)return DC_motor_ID;else return STEP_motor_ID;}
        void motor_init();

        void recv_buf_parse(unsigned char* buf,unsigned int len);

    private:
        u8 motor_type;
        const char *DC_motor_ID="LV8548DC_Ver.9.0.0";
        const char *STEP_motor_ID="LV8548Step_Ver.9.0.0";

        typedef enum
        {
            SELECT_MOTOR_TYPE=0X1,
            GET_ID=2,
            SET_DC_ROTATION_DIRECTION=3,
            SET_DC_MOTOR_VOLTAGE=4,
            SET_DC_PWM_FREQ=5,
            SET_DC_MOTOR_START_FLAG=6,

            SET_STEP_ANGLE=7,
            SET_STEP_RUN_WITH_ENDOF_DEGREE=8,
            SET_STEP_RUN_WITH_ENDOF_STEPS=9,
            SET_STEP_RUN_WITH_ENDOF_TIME=10,
            RETAIN=11,
            SET_STEP_STOP=12,

            SET_DC_MOTOR_PARAMS=13,
        }protocol_enum_t;
};


void wire_init();
u8 get_motor_type_from_input();
void commucation_parse();
void Serial_init();
u8 get_user_choise();
void uart_recv_data();
extern LV8548_Motor motor;

#endif
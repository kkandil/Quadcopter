#include "RFH.h"
#include "INPUTH.h"
#include <SPI.h>
#include <NRFLite.h>

//========================================================================================================
//========================================================================================================
// Macros
#define PIN_RADIO_MOSI  PA7
#define PIN_RADIO_MISO  PA6
#define PIN_RADIO_SCK   PA5
#define PIN_RADIO_CE    PB10
#define PIN_RADIO_CSN   PA4

//========================================================================================================
//========================================================================================================
// Global Variables


//========================================================================================================
//========================================================================================================
// Locak Variables
const static uint8_t RADIO_ID = 1;             // Our radio's id.
const static uint8_t DESTINATION_RADIO_ID = 0; // Id of the radio we will transmit to.
NRFLite _radio;

struct strOutputRadioPacket // Any packet up to 32 bytes can be sent.
{ 
    uint32_t Throttle ;  
    int16_t RollAngle ;  
    int16_t PitchAngle ;  
    int16_t YawAngle ;  
    int16_t REncoderCount;
};


struct strInputRadioPacket
{ 
    //char Message[32];    // Note the max packet size is 32, so 31 is all we can use here.
    int16_t Motor_F_R;
    int16_t Motor_F_L;
    int16_t Motor_B_R;
    int16_t Motor_B_L;
    int16_t Angle_X;
    int16_t Angle_Y;
    int16_t Angle_Z;
    int16_t ExecutionTime;
    int16_t Motion_X;
    int16_t Motion_Y;
    int16_t PID_roll;
    int16_t PID_pitch;
    int16_t MotorsSetPower;
    int16_t elapsedTime;
    int16_t error_roll;
    int16_t pid_p_roll; 
};


strInputRadioPacket InputRadioPacket;  
strOutputRadioPacket OutputRadioPacket;

//========================================================================================================
//========================================================================================================
// Global Functions
void RFH_Init(void)
{
    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN))
    {
        Serial1.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    }
    else
    {
        Serial1.println("communicate with radio"); 
    }
}

void RFH_SendData(void)
{
    String msg = "" ;
    
    OutputRadioPacket.Throttle = INPUTH_Inputs.thruttle ;
    OutputRadioPacket.RollAngle = INPUTH_Inputs.rollangle ;
    OutputRadioPacket.PitchAngle = INPUTH_Inputs.pitchangle ;
    OutputRadioPacket.YawAngle = INPUTH_Inputs.yawangle ;
    OutputRadioPacket.REncoderCount = 0; //INPUTH_Inputs.REncoderCount;

    if (_radio.send(DESTINATION_RADIO_ID, &OutputRadioPacket, sizeof(OutputRadioPacket))) // Note how '&' must be placed in front of the variable name.
    {
       
      while (_radio.hasData())
      { 
          _radio.readData(&InputRadioPacket); 

          msg =  String((InputRadioPacket.Angle_X)/100.0) + "," + String(InputRadioPacket.Angle_Y) + "," + String(InputRadioPacket.Angle_Z) + "," + String(InputRadioPacket.ExecutionTime) 
                    + "," + String(InputRadioPacket.Motor_F_R) + "," + String(InputRadioPacket.Motor_F_L) + "," + String(InputRadioPacket.Motor_B_R) + "," + String(InputRadioPacket.Motor_B_L) 
                    + "," + String((InputRadioPacket.Motion_X)/100.0) + "," + String((InputRadioPacket.Motion_Y)/100.0) + "," + String(InputRadioPacket.PID_roll) + "," + String(InputRadioPacket.PID_pitch) + "," + String(InputRadioPacket.MotorsSetPower)
                    + "," + String(InputRadioPacket.elapsedTime) + "," + String((InputRadioPacket.error_roll)/100.0) + "," + String((InputRadioPacket.pid_p_roll)/100.0) ;//+ "," + String((InputRadioPacket.pid_d_roll)/100.0)  ;  
          Serial1.println(msg);  

      }
    }
}

//========================================================================================================
//========================================================================================================
// Local Functions

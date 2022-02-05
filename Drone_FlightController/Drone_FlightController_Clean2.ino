
//----------------------------------------------------------------------------------------------------------------------
#include <SimpleKalmanFilter.h>
#include <Servo.h> //Using servo library to control ESC  
#include <PIDController.h> 
#include <FastPID.h> 

#include <SPI.h>        // RF Transciever
#include <NRFLite.h>    // RF Transciever

 




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// RF Transciever
const static uint8_t RADIO_ID = 0;       // Our radio's id.  The transmitter will send to this id.
const static uint8_t DESTINATION_RADIO_ID = 1; 
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket // Any packet up to 32 bytes can be sent.
{ 
    uint32_t Throttle ;  
    int16_t RollAngle ;  
    int16_t PitchAngle ;  
    int16_t YawAngle ;  
    int16_t REncoderCount;
};

struct RadioPacket2
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
RadioPacket2 _radioData2;

NRFLite _radio;
RadioPacket _radioData;

#define RADIO_ENABLED
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// ADNS3080
#define PIN_MOUSECAM_RESET     A1
#define PIN_MOUSECAM_CS        A0

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17 

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];

struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};

int Motion_X = 0;
int Motion_Y = 0 ;


float PID_motionx = 0.0, error_motionx=0.0, previous_error_motionx=0.0, pid_i_motionx=0.0;
float ki_motionx=0.002;
float kp_motionx = 2.6;
float kd_motionx = 20.1; 

float PID_motiony = 0.0, error_motiony=0.0, previous_error_motiony=0.0, pid_i_motiony=0.0; 
float ki_motiony = ki_motionx;
float kp_motiony = kp_motionx;
float kd_motiony = kd_motionx; 
int motionUpdateTimer = 0 ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// Motors
#define MOTOR_ENABLE
Servo Motor_F_R;  
Servo Motor_F_L;  
Servo Motor_B_R;  
Servo Motor_B_L; 

const static int  Motor_F_R_Pin  =  3 ;
const static int  Motor_F_L_Pin  =  4 ;
const static int  Motor_B_R_Pin  =  5 ;
const static int  Motor_B_L_Pin  =  6 ;
 
int Motor_F_R_Power = 1000 ;
int Motor_F_L_Power = 1000 ;
int Motor_B_R_Power = 1000 ;
int Motor_B_L_Power = 1000 ;

int Motor_F_R_Power_Prev = 1000 ;
int Motor_F_L_Power_Prev = 1000 ;
int Motor_B_R_Power_Prev = 1000 ;
int Motor_B_L_Power_Prev = 1000 ;

int MotorsSetPower = 1000 ;
#define MOTOR_POWER_MAX   1450

float elapsedTime, time1, timePrev;
unsigned long loop_timer;
float executionTime= 0.0, executionTime_Prev=0.0;
unsigned long starttime1 = 0, endtime1=0, starttime2=0, endtime2=0;
 
 
#define SERIAL_ENABLED
//#define DEBUG_TIME


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// MPU6050
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
 
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
 

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards  

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

  
// INTERRUPT DETECTION ROUTINE 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


float Total_angleX=0.0, Total_angleY=0.0, Total_angleZ=0.0 ;

float pid_p_roll=0.0,pid_i_roll=0.0, pid_d_roll=0.0, PID_roll=0.0;
float error_roll=0.0, previous_error_roll=0.0 ; 
float kp_roll=2.5;//2.5 , 3.5
float ki_roll=0.01;//0.1
float kd_roll=50.0;//550
int roll_desired_angle_count = 0 ;
float PID_Roll_Max = 300;
float roll_desired_angle = 0.0;

float pid_p_pitch=0.0, pid_i_pitch=0.0, pid_d_pitch=0.0, PID_pitch=0.0 ;
float error_pitch=0.0, previous_error_pitch=0.0;
float kp_pitch=kp_roll; 
float ki_pitch=ki_roll; 
float kd_pitch=kd_roll; 
int pitch_desired_angle_count = 0 ;
float PID_Pitch_Max = PID_Roll_Max;
float pitch_desired_angle = 0.0;

float pid_i_yaw=0.0, PID_yaw=0.0, error_yaw=0.0, previous_error_yaw=0.0 ;
float pid_d_yaw=0.0;
float kp_yaw=3.0;//3.55
float ki_yaw=ki_roll;//0.003
float kd_yaw=50.0;//2.05
int yaw_desired_angle_count = 0 ;
float PID_Yaw_Max = PID_Roll_Max;

float period = 5.0;

float UpdateTimer = 0 ;
int UpdatePIDTunning = 0;
float delta_time = 0.0;
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() { 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    #ifdef SERIAL_ENABLED
    Serial.begin(250000);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    #endif

    //--------------------------------------------------------------------------------------------------------------------------------
    pinMode(Motor_F_R_Pin, OUTPUT);
    pinMode(Motor_F_L_Pin, OUTPUT);
    pinMode(Motor_B_R_Pin, OUTPUT);
    pinMode(Motor_B_L_Pin, OUTPUT);
    
    Motor_F_R.attach(Motor_F_R_Pin);  
    Motor_F_L.attach(Motor_F_L_Pin);
    Motor_B_R.attach(Motor_B_R_Pin);  
    Motor_B_L.attach(Motor_B_L_Pin);
    
    Motor_F_R.writeMicroseconds(1000); //initialize the signal to 1000
    Motor_F_L.writeMicroseconds(1000); //initialize the signal to 1000
    Motor_B_R.writeMicroseconds(1000); //initialize the signal to 1000
    Motor_B_L.writeMicroseconds(1000); //initialize the signal to 1000

    //myESC_F_R.arm();  
    //myESC_F_L.arm();  
    //myESC_B_R.arm();  
    //myESC_B_L.arm();  

    
   #ifdef RADIO_ENABLED
    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN))
    {
      #ifdef SERIAL_ENABLED
        Serial.println("Cannot communicate with radio");
      #endif
        while (1); // Wait here forever.
    }
    #endif  
 
  //--------------------------------------------------------------------------------------------------------------------------------


  

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    #ifdef SERIAL_ENABLED
    Serial.println(F("Initializing I2C devices..."));
    #endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    #ifdef SERIAL_ENABLED
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    #endif
    
    // load and configure the DMP
    #ifdef SERIAL_ENABLED
    Serial.println(F("Initializing DMP..."));
    #endif
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        #ifdef SERIAL_ENABLED
        Serial.println(F("Enabling DMP..."));
        #endif
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        #ifdef SERIAL_ENABLED
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        #endif
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        #ifdef SERIAL_ENABLED
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        #endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        #ifdef SERIAL_ENABLED
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        #endif
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////// ADNS3080
    #ifdef CAMERA_ENABLED
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    if(mousecam_init()==-1)
    {
      Serial.println("Mouse cam failed to init");
      while(1);
    }  
    #endif
   //delay(25000) ;
  delay(5000) ;
  
  time1 = micros(); //Start counting time in milliseconds
  UpdateTimer = millis(); 
  delta_time = (float)micros();
  executionTime_Prev = millis(); 
}

 
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {

  // Calculate Execution Time
  //timePrev = time1;  // the previous time is stored before the actual time read
  //time1 = micros();  // actual time read
  //elapsedTime = (time1 - timePrev)/100000.0 ;  
  executionTime = (millis() - executionTime_Prev) ;
  executionTime_Prev = millis();
                                                  

  /*
  if( (millis() - UpdateTimer) >= 4 )
  {
    UpdatePIDTunning = 1;
    UpdateTimer = millis();
  } 
  */
  //////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////// MPU6050 
    
  // Update MPU6050 x,y and z angels   
  UpdateGyroAngles();                              

  


  //////////////////////////////////////////////////////////////////////////////
  /*
  ////////////////////////////////// ADNS3080 
  //if( motionUpdateTimer >= 25 )
  { 
    // Update Optical Flow X and Y values
    UpdateOpticalFlow();

    // Update Optical Flow X and Y PID values
    UpdateOpticalFlowPID();

   
    motionUpdateTimer = 0;
  }
  //else
  //{
  //  motionUpdateTimer++;
 // }
*/
  //////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////// Radio Transceiver
  #ifdef RADIO_ENABLED
  while (_radio.hasData())
  { 
      _radio.readData(&_radioData); // Note how '&' must be placed in front of the variable name.  
      if(_radioData.RollAngle>10)
      {
        roll_desired_angle =  -10.0;
      }
      else if( _radioData.RollAngle < -10 )
      {
        roll_desired_angle =  10.0; 
      }
      else
      {
        roll_desired_angle =  (float)(-1*_radioData.RollAngle); 
      }
      if( _radioData.Throttle > 1000 && _radioData.Throttle > MotorsSetPower)
      { 
          MotorsSetPower = _radioData.Throttle ;
          if( MotorsSetPower > MOTOR_POWER_MAX ) 
          {
            MotorsSetPower = MOTOR_POWER_MAX ;
          }
  
         // Motor_F_R_Power = MotorsSetPower ; 
         // Motor_F_L_Power = MotorsSetPower ;
          //Motor_B_R_Power = MotorsSetPower ; 
         // Motor_B_L_Power = MotorsSetPower ; 
      }
      else if ( (_radioData.Throttle == 1000 || _radioData.Throttle == 999) && MotorsSetPower > 1000 ) 
      {
        
      }
      else if ( _radioData.Throttle > 1000 && _radioData.Throttle < MotorsSetPower ) 
      {
        
      }
      else if ( _radioData.Throttle < 1000 ) 
      { 
          MotorsSetPower = map(_radioData.Throttle,1000,0,MotorsSetPower,1000); 
          if( MotorsSetPower < 1000 )
          {
            MotorsSetPower = 1000 ;
          }
         // Motor_F_R_Power = MotorsSetPower ; 
         // Motor_F_L_Power = MotorsSetPower ;
         // Motor_B_R_Power = MotorsSetPower ; 
         // Motor_B_L_Power = MotorsSetPower ; 
      }  
      else
      {
         // MotorsSetPower = 1000 ;
          //Motor_F_R_Power = 1000 ; 
          //Motor_F_L_Power = 1000 ;
         // Motor_B_R_Power = 1000 ; 
         // Motor_B_L_Power = 1000 ; 
      } 

      #ifdef ENCODER_THROTLE_CONTROL_ENABLED
      MotorsSetPower = MotorsSetPower + _radioData.REncoderCount;
      if( MotorsSetPower > MOTOR_POWER_MAX )
      {
        MotorsSetPower = MOTOR_POWER_MAX;
      }
      else if( MotorsSetPower<1000)
      {
        MotorsSetPower = 1000;
      }
      #endif
       
       _radioData2.Motor_F_R = Motor_F_R_Power;
        _radioData2.Motor_F_L = Motor_F_L_Power;
        _radioData2.Motor_B_R = Motor_B_R_Power;
        _radioData2.Motor_B_L = Motor_B_L_Power;
        _radioData2.Angle_X = (int16_t)(Total_angleX*100.0);
        _radioData2.Angle_Y = (int16_t)(Total_angleY*100.0);
        _radioData2.Angle_Z = (int16_t)(Total_angleZ*100.0);
        _radioData2.ExecutionTime = (int16_t) executionTime;
        _radioData2.Motion_X = (int16_t)(error_roll * 100.0); //Motion_X;
        _radioData2.Motion_Y = (int16_t)(pid_i_roll * 100.0); //Motion_Y;
        _radioData2.PID_roll = PID_roll;
        _radioData2.PID_pitch = roll_desired_angle;
        _radioData2.MotorsSetPower = MotorsSetPower;
        _radioData2.elapsedTime = (int16_t) (elapsedTime);
        _radioData2.error_roll = (int16_t)(pid_p_roll * 100.0);
        _radioData2.pid_p_roll = (int16_t)(pid_d_roll * 100.0);
        //_radioData2.pid_d_roll = (int16_t)(pid_d_roll * 100.0);
        
        if (_radio.send(DESTINATION_RADIO_ID, &_radioData2, sizeof(_radioData2)))
        {
            //Serial.println("...Success");
        }
        else
        {
            //Serial.println("...Failed");
        }  
        
  }  
  #endif
    

  if (millis() > UpdateTimer+period)
  {
    UpdateTimer = millis(); 
    timePrev = time1;  // the previous time is stored before the actual time read
    time1 = micros();  // actual time read
    elapsedTime = (time1 - timePrev)/1000.0 ;   
    
    UpdateGyroAnglesPID2();   
     if(MotorsSetPower < 1100)
     {
        Motor_F_R_Power = MotorsSetPower ; 
        Motor_F_L_Power = MotorsSetPower ;
        Motor_B_R_Power = MotorsSetPower ; 
        Motor_B_L_Power = MotorsSetPower ;  
     }
     else  
     {  
        Motor_F_R_Power = MotorsSetPower - ((int)PID_pitch) + ((int)PID_roll) ;//+ PID_yaw ;//- PID_motionx; 
        Motor_F_L_Power = MotorsSetPower - ((int)PID_pitch) - ((int)PID_roll) ;//- PID_yaw ;//+ PID_motionx; 
        Motor_B_R_Power = MotorsSetPower + ((int)PID_pitch) + ((int)PID_roll) ;//- PID_yaw ;//- PID_motionx; 
        Motor_B_L_Power = MotorsSetPower + ((int)PID_pitch) - ((int)PID_roll) ;//+ PID_yaw ;//+ PID_motionx; 
        UpdatePIDTunning = 0;
    
     } 
    if(Motor_F_R_Power < 1000)
    {
      Motor_F_R_Power= 1000;
    } 
    else if(Motor_F_R_Power > 1900)
    {
      Motor_F_R_Power=1900;
    }   
    if( Motor_F_R_Power < 1100 && MotorsSetPower >= 1100 )
    {
      Motor_F_R_Power = 1100 ; 
    } 
    //-----------------
    if(Motor_F_L_Power < 1000)
    {
      Motor_F_L_Power= 1000;
    }
    else if(Motor_F_L_Power > 1900)
    {
      Motor_F_L_Power=1900;
    } 
    if( Motor_F_L_Power < 1100 && MotorsSetPower >= 1100 )
    {
      Motor_F_L_Power = 1100 ; 
    }
    //-----------------
    if(Motor_B_R_Power < 1000)
    {
      Motor_B_R_Power= 1000;
    }
    else if(Motor_B_R_Power > 1900)
    {
      Motor_B_R_Power=1900;
    }
    if( Motor_B_R_Power < 1100 && MotorsSetPower >= 1100 )
    {
      Motor_B_R_Power = 1100 ; 
    }
    //-----------------
    if(Motor_B_L_Power < 1000)
    {
      Motor_B_L_Power= 1000;
    }
    else if(Motor_B_L_Power > 1900)
    {
      Motor_B_L_Power=1900;
    } 
    if( Motor_B_L_Power < 1100 && MotorsSetPower >= 1100 )
    {
      Motor_B_L_Power = 1100 ; 
    } 
    #ifdef MOTOR_ENABLE
    Motor_F_R.writeMicroseconds(Motor_F_R_Power);  
    Motor_F_L.writeMicroseconds(Motor_F_L_Power); 
    Motor_B_R.writeMicroseconds(Motor_B_R_Power); 
    Motor_B_L.writeMicroseconds(Motor_B_L_Power);
    #endif
  }

  //Serial.print(Motor_F_L_Power);Serial.print(", ");Serial.println(Motor_F_R_Power);
  //Serial.print(Motor_B_L_Power);Serial.print(", ");Serial.println(Motor_B_R_Power);
  //Serial.println("----------------------" );

   delay(5);
  
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// MPU6050
void UpdateGyroAngles()
{
  #if 0
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop 
        fifoCount = mpu.getFIFOCount();
      }  
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
  }
  #endif
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize)
  {
      //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) 
  {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
      Serial.println(F("FIFO overflow!")); 
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
  { 
    
      // read a packet from FIFO
      while(fifoCount >= packetSize)
      { // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
      }
       
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Total_angleZ = ypr[0] * 180/M_PI ;
      Total_angleY = ypr[1] * 180/M_PI ;
      Total_angleX = ypr[2] * 180/M_PI ; 

      //Serial.print(Total_angleX);Serial.print(", ");Serial.print(Total_angleY);Serial.print(", ");Serial.println(Total_angleZ ); 
 
 }  
 
}

void UpdateGyroAnglesPID2()
{
  int angleTemp = 0;
  //angleTemp = (int) Total_angleX;
  error_roll = roll_desired_angle - Total_angleX; 
  pid_p_roll = kp_roll*error_roll;

  if(  error_roll > -1.0 && error_roll < 1.0)
  {
    pid_i_roll = 0.0; 
  }
  if( error_roll >= -3.0 && error_roll <= 3.0)     //( abs(error_roll) >= 1.0)
  {
    pid_i_roll = pid_i_roll+(ki_roll*error_roll);  
  } 
  /*
  else if( abs(error_roll) < 1.0)
  {
    pid_i_roll = 0; 
  }
  */
  
  pid_d_roll = kd_roll*((error_roll - previous_error_roll)/elapsedTime);
  
  PID_roll = pid_p_roll + pid_i_roll + pid_d_roll;

  if(PID_roll < (-1*PID_Roll_Max))
  {
    PID_roll=-1*PID_Roll_Max;
  }
  if(PID_roll > PID_Roll_Max)
  {
    PID_roll= PID_Roll_Max;
  }
  previous_error_roll = error_roll;


  angleTemp = (int) Total_angleY;
  error_pitch = pitch_desired_angle - Total_angleY; 
  pid_p_pitch = kp_pitch*error_pitch;

  if(  error_pitch > -1.0 && error_pitch < 1.0)
  {
    pid_i_pitch = 0.0; 
  }
  if( error_pitch >= -3.0 && error_pitch <= 3.0)     //( abs(error_pitch) >= 1.0)
  {
    pid_i_pitch = pid_i_pitch+(ki_pitch*error_pitch);  ;  
  }  
  /*
  else if( abs(error_pitch) < 1.0)
  {
    error_pitch = 0; 
  }
  */
  pid_d_pitch = kd_pitch*((error_pitch - previous_error_pitch)/elapsedTime);
  
  PID_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  
  if(PID_pitch < (-1*PID_Pitch_Max))
  {
    PID_pitch=-1*PID_Pitch_Max;
  }
  if(PID_pitch > PID_Pitch_Max)
  {
    PID_pitch= PID_Pitch_Max;
  }
  previous_error_pitch = error_pitch;

 
 //Serial.print(Total_angleX);Serial.print(", ");Serial.print(error_roll);Serial.print(", ");Serial.print(PID_roll);Serial.print(", ");
 //Serial.print(pid_p_roll);Serial.print(", ");Serial.print(pid_i_roll);Serial.print(", ");Serial.print(pid_d_roll);Serial.print(", ");Serial.print(elapsedTime,5);
 //Serial.println(""); 
}

void UpdateGyroAnglesPID()
{
  int angletemp = 0;

  angletemp = Total_angleZ;
  error_yaw = 0.0 - angletemp ;
  pid_i_yaw += ki_yaw * error_yaw;
  if(pid_i_yaw > PID_Yaw_Max)pid_i_yaw = PID_Yaw_Max;
  else if(pid_i_yaw < (-1*PID_Yaw_Max))pid_i_yaw = -1*PID_Yaw_Max;
  PID_yaw = kp_yaw * error_yaw + pid_i_yaw + kd_yaw * (error_yaw - previous_error_yaw); 
  previous_error_yaw = error_yaw;
  if(PID_yaw < (-1*PID_Yaw_Max))
  {
    PID_yaw = -1*PID_Yaw_Max;
  }
  if(PID_yaw > PID_Yaw_Max)
  {
    PID_yaw = PID_Yaw_Max;
  } 

  if( PID_yaw <= 0.9 && PID_yaw >= -0.9 )
  {
    PID_yaw = 0.0 ;
  }

  /*  
  angletemp = Total_angleX;
  error_roll = desired_angle - angletemp ;
  pid_i_roll += ki_roll * error_roll;
  if(pid_i_roll > PID_Roll_Max)pid_i_roll = PID_Roll_Max;
  else if(pid_i_roll < (-1*PID_Roll_Max))pid_i_roll = -1*PID_Roll_Max;
  PID_roll = kp_roll * error_roll + pid_i_roll + kd_roll * (error_roll - previous_error_roll); 
  previous_error_roll = error_roll;
  if(PID_roll < (-1*PID_Roll_Max))
  {
    PID_roll=-1*PID_Roll_Max;
  }
  if(PID_roll > PID_Roll_Max)
  {
    PID_roll= PID_Roll_Max;
  } 

  if( PID_roll <= 0.9 && PID_roll >= -0.9 )
  {
    PID_roll = 0.0 ;
  }
  */
  delta_time = (float)((float)micros() - delta_time)/1000000.0 ;
  angletemp = Total_angleX;
  error_roll = roll_desired_angle - ((float)angletemp) ;  
  pid_i_roll += error_roll * delta_time; 
  PID_roll = (error_roll * kp_roll) + (pid_i_roll * ki_roll) + (((error_roll - previous_error_roll) / delta_time) * kd_roll);
  previous_error_roll = error_roll;
  if(PID_roll < (-1*PID_Roll_Max))
  {
    PID_roll=-1*PID_Roll_Max;
  }
  if(PID_roll > PID_Roll_Max)
  {
    PID_roll= PID_Roll_Max;
  } 
  //Serial.print("AngleX: ");Serial.print(Total_angleX); Serial.print(", time: "); Serial.print(delta_time,6); Serial.print(", error: "); Serial.print(error_roll); Serial.print(", I: "); Serial.print(pid_i_roll); Serial.print(", PID: "); Serial.println(PID_roll);
  
  


   /*
  angletemp = Total_angleY;
  error_pitch = desired_angle - angletemp ;
  pid_i_pitch += ki_pitch * error_pitch;
  if(pid_i_pitch > PID_Pitch_Max)pid_i_pitch = PID_Pitch_Max;
  else if(pid_i_pitch < (-1*PID_Pitch_Max))pid_i_pitch = -1*PID_Pitch_Max;
  PID_pitch = kp_pitch * error_pitch + pid_i_pitch + kd_pitch * (error_pitch - previous_error_pitch); 
  previous_error_pitch = error_pitch;
  if(PID_pitch < (-1*PID_Pitch_Max))
  {
    PID_pitch = -1*PID_Pitch_Max;
  }
  if(PID_pitch > PID_Pitch_Max)
  {
    PID_pitch = PID_Pitch_Max;
  } 

  if( PID_pitch <= 0.9 && PID_pitch >= -0.9 )
  {
    PID_pitch = 0.0 ;
  }
  PID_pitch = 0.0;
  */ 
  angletemp = Total_angleY;
  error_pitch = pitch_desired_angle - ((float)angletemp) ;  
  pid_i_pitch += error_pitch * delta_time; 
  PID_pitch = (error_pitch * kp_pitch) + (pid_i_pitch * ki_pitch) + (((error_pitch - previous_error_pitch) / delta_time) * kd_pitch);
  previous_error_pitch = error_pitch;
  if(PID_pitch < (-1*PID_Pitch_Max))
  {
    PID_pitch=-1*PID_Pitch_Max;
  }
  if(PID_pitch > PID_Pitch_Max)
  {
    PID_pitch= PID_Pitch_Max;
  } 
  
  delta_time = (float)micros();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// ADNS3080
void UpdateOpticalFlow()
{
  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);
   
  Motion_X += md.dx;
  Motion_Y += md.dy;   
}

void UpdateOpticalFlowPID()
{
  error_motionx = roll_desired_angle - Motion_X ;
  pid_i_motionx += ki_motionx * error_motionx;
  if(pid_i_motionx > 300.0)pid_i_motionx = 300;
  else if(pid_i_motionx < -300.0)pid_i_motionx = -300;
  PID_motionx = kp_motionx * error_motionx + pid_i_motionx + kd_motionx * (error_motionx - previous_error_motionx); 
  previous_error_motionx = error_motionx;  
  
  
  error_motiony = pitch_desired_angle - Motion_Y ;
  pid_i_motiony += ki_motiony * error_motiony;
  if(pid_i_motiony > 300.0)pid_i_motiony = 300;
  else if(pid_i_motiony < -300.0)pid_i_motiony = -300;
  PID_motiony = kp_motiony * error_motiony + pid_i_motiony + kd_motiony * (error_motiony - previous_error_motiony); 
  previous_error_motiony = error_motiony;
}


void mousecam_reset()
{
  digitalWrite(PIN_MOUSECAM_RESET,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET,LOW);
  delay(35); // 35ms from reset to functional
}

int mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);
  
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  
  mousecam_reset();
  
  int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  if(pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);

  return 0;
}

void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(1);
  return ret;
}

void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(5);
}

char asciiart(int k)
{
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4];
}

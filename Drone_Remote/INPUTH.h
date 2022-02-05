#include <Arduino.h>

//========================================================================================================
//========================================================================================================
// Macros
//RF Transiver 
#define PIN_RADIO_CE  PB10
#define PIN_RADIO_CSN  PA4
 
// LCD
#define PIN_LCD_SCK   PB6  
#define PIN_LCD_SDA   PB7
#define PIN_LCD_UP    PC14         
#define PIN_LCD_DOWN  PC15      
#define PIN_LCD_SEL   PA11   

// Encoder
#define PIN_ENCODER_SW     PB12 
#define PIN_ENCODER_DT     PB13    
#define PIN_ENCODER_CLK    PB14   

// Left Joystick
#define PIN_LEFT_JOYSTICK_Rx    PA3     // ↑↓ mid: 2096, max: 4095, min: 0
#define PIN_LEFT_JOYSTICK_Ry    PA2     //    mid: 2037, max: 4095, min: 0
#define PIN_LEFT_JOYSTICK_SW    PB15     

// Right Joystick
#define PIN_RIGHT_JOYSTICK_Rx    PA1    // ↑↓ mid: 2075, max: 4095, min: 0
#define PIN_RIGHT_JOYSTICK_Ry    PA0    //    mid: 2045, max: 4095, min: 0
#define PIN_RIGHT_JOYSTICK_SW    PA8     

// Trigger Pot
#define PIN_LEFT_POT      PB1   // not working
#define PIN_RIGHT_POT     PB0   // Min: 750, Max: 3870

// Trigger Switchs
#define PIN_TRIG_LEFT     PB3   
#define PIN_TRIG_RIGHT    PB4   // not working
#define PIN_TRIG_Front_L  PB5   // ↑: 1, ↓:0 
#define PIN_TRIG_Front_R  PB8   // ↑: 0, ↓:1

#define PIN_BUZZER    PB9

#define RX_FILTER   30
#define RY_FILTER   30
#define LX_FILTER   30
#define LY_FILTER   30

#define BUTTON_RELEASED   0
#define BUTTON_PRESSED    1

#define ENCODER_DIRECTION_CW  0
#define ENCODER_DIRECTION_CCW 1

//========================================================================================================
//========================================================================================================
// Global Variables
typedef struct{
  int button_curr_read ;
  int button_curr_state;
  int button_prev_state;
  unsigned long button_tickstart;
  bool is_updated; 
  int Pin;
}strButtonStatus;


typedef enum{
  PRESSED = 0,
  RELEASED = 1
}enButton_Status;

typedef enum{  
  lcd_button_up = 0,
  lcd_button_down = 1,
  lcd_button_select = 2, 
  encoder_button = 3,
  left_joystick_button = 4,
  right_joystick_button = 5,
  trigger_button_left = 6,
  trigger_button_right = 7,
  trigger_button_front_left = 8,
  trigger_button_fron_right = 9
}enButtons;

struct strInputs{
  // LCD Switchs
  int lcd_button_up;
  int lcd_button_down;
  int lcd_button_select;  

  // Rotary Encoder
  bool encoder_rotated;
  int encoder_direction;
  int encoder_button;

  // Left Joystick
  int left_joystick_x;
  int left_joystick_y;
  int left_joystick_button;

  // right Joystick
  int right_joystick_x;
  int right_joystick_y;
  int right_joystick_button;

  // Trigger Pot
  int left_trigger_pot;
  int right_trigger_pot;

  // Trigger Switchs
  int trigger_button_left;
  int trigger_button_right;
  int trigger_button_front_left;
  int trigger_button_fron_right;

  // Drone controls
  int rollangle;
  int pitchangle;
  int yawangle;
  int thruttle;
};

extern strInputs INPUTH_Inputs;

//========================================================================================================
//========================================================================================================
// Locak Variables

//========================================================================================================
//========================================================================================================
// Global Functions
extern void INPUTH_Init(void);
extern void INPUTH_ReadInputs(void);
extern void INPUTH_OutputTone(int duration_ms);

//========================================================================================================
//========================================================================================================
// Local Functions
static void ResetInputs(void);
static void Encoder_ISR_A();
static void Encoder_ISR_B();
static void GetRollPitchYawAngles(int Rx, int Ry, int Ly, int *rollangle,int *pitchangle, int *yawangle);
static void GetThruttle(int Lx, int *Thruttle);
static void GetButtonStatus(int buttonIndex);

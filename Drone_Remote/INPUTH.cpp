 
#include "INPUTH.h" 
#include "LCDH.h" 

//========================================================================================================
//========================================================================================================
// Macros
unsigned long BUTTONS_DEBUNCE_TIME = 50;

//========================================================================================================
//========================================================================================================
// Global Variables

strInputs INPUTH_Inputs;

strButtonStatus button_status[10] = {{0, LOW, LOW, 0, false, PIN_LCD_UP},
                                    {0, LOW, LOW, 0, false, PIN_LCD_DOWN},
                                    {0, LOW, LOW, 0, false, PIN_LCD_SEL},
                                    {0, LOW, LOW, 0, false, PIN_ENCODER_SW},
                                    {0, LOW, LOW, 0, false, PIN_LEFT_JOYSTICK_SW},
                                    {0, LOW, LOW, 0, false, PIN_RIGHT_JOYSTICK_SW},
                                    {0, LOW, LOW, 0, false, PIN_TRIG_LEFT},
                                    {0, LOW, LOW, 0, false, PIN_TRIG_RIGHT},
                                    {0, LOW, LOW, 0, false, PIN_TRIG_Front_L},
                                    {0, LOW, LOW, 0, false, PIN_TRIG_Front_R}};
                                    
//========================================================================================================
//========================================================================================================
// Local Variables

#define DIR_CW  0
#define DIR_CCW 1
volatile int master_count = 0; // universal count
volatile byte INTFLAG1 = 0; // interrupt status flag
int EncoderDir = DIR_CW;

//========================================================================================================
//========================================================================================================
// Global Functions
void INPUTH_Init(void)
{
    // Set the default values for all inputs signals
    ResetInputs();
 
    // Configure all DIO pins
    // LCD
    pinMode(PIN_LCD_UP, INPUT_PULLDOWN);
    pinMode(PIN_LCD_DOWN, INPUT_PULLDOWN);
    pinMode(PIN_LCD_SEL, INPUT_PULLDOWN); 
    // Encoder
    pinMode(PIN_ENCODER_SW, INPUT);
    pinMode(PIN_ENCODER_DT, INPUT);
    pinMode(PIN_ENCODER_CLK, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_DT), Encoder_ISR_A, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_CLK), Encoder_ISR_B, RISING); 
    
    // Left Joystick
    pinMode(PIN_LEFT_JOYSTICK_Rx, INPUT_ANALOG);
    pinMode(PIN_LEFT_JOYSTICK_Ry, INPUT_ANALOG);
    pinMode(PIN_LEFT_JOYSTICK_SW, INPUT_PULLUP); 
    // Right Joystick
    pinMode(PIN_RIGHT_JOYSTICK_Rx, INPUT_ANALOG);
    pinMode(PIN_RIGHT_JOYSTICK_Ry, INPUT_ANALOG);
    pinMode(PIN_RIGHT_JOYSTICK_SW, INPUT_PULLUP); 
    // Trigger Pot
    pinMode(PIN_LEFT_POT, INPUT_ANALOG);
    pinMode(PIN_RIGHT_POT, INPUT_ANALOG);  
    // Trigger Switchs
    pinMode(PIN_TRIG_LEFT, INPUT_PULLDOWN);
    pinMode(PIN_TRIG_RIGHT, INPUT_PULLDOWN);
    pinMode(PIN_TRIG_Front_L, INPUT_PULLDOWN);
    pinMode(PIN_TRIG_Front_R, INPUT_PULLDOWN); 
    // Output Buzzer
    pinMode(PIN_BUZZER, OUTPUT); 
    
    delay(1000);
}
 
void INPUTH_ReadInputs(void)
{   

    for(int index=lcd_button_up ; index<=trigger_button_fron_right ; index++)
    {
      GetButtonStatus(index);
    }

    if(  button_status[lcd_button_up].is_updated == true && button_status[lcd_button_up].button_curr_state == PRESSED)
    {
      LCDH_LCDUpButtonPressed(lcd_button_up, true);
    }
    else if(  button_status[lcd_button_down].is_updated == true && button_status[lcd_button_down].button_curr_state == PRESSED)
    {
      LCDH_LCDUpButtonPressed(lcd_button_down, true);
    }
    else if(  button_status[lcd_button_select].is_updated == true && button_status[lcd_button_select].button_curr_state == PRESSED)
    {
      LCDH_LCDUpButtonPressed(lcd_button_select, true);
    }
  else if(  button_status[encoder_button].is_updated == true && button_status[encoder_button].button_curr_state == PRESSED)
    {
      LCDH_LCDUpButtonPressed(lcd_button_select, true);
      //Serial1.println("Pressed");
    }
    /*
    Serial1.println("lcd_button_up: " + String(button_status[lcd_button_up].button_curr_state));
    Serial1.println("lcd_button_down: " + String(button_status[lcd_button_down].button_curr_state));
    Serial1.println("lcd_button_select: " + String(button_status[lcd_button_select].button_curr_state));
    Serial1.println("encoder_button: " + String(button_status[encoder_button].button_curr_state));
    Serial1.println("left_joystick_button: " + String(button_status[left_joystick_button].button_curr_state));
    Serial1.println("right_joystick_button: " + String(button_status[right_joystick_button].button_curr_state));
    Serial1.println("trigger_button_left: " + String(button_status[trigger_button_left].button_curr_state));
    Serial1.println("trigger_button_right: " + String(button_status[trigger_button_right].button_curr_state));
    Serial1.println("trigger_button_front_left: " + String(button_status[trigger_button_front_left].button_curr_state));
    Serial1.println("trigger_button_fron_right: " + String(button_status[trigger_button_fron_right].button_curr_state));
    Serial1.println("---------------------");
    */
  /*
    // Read LCD Switchs 
    INPUTH_Inputs.lcd_button_up = digitalRead(PIN_LCD_UP);
    INPUTH_Inputs.lcd_button_down = digitalRead(PIN_LCD_DOWN);
    INPUTH_Inputs.lcd_button_select = digitalRead(PIN_LCD_SEL);

    // Read Encoder switch
    INPUTH_Inputs.encoder_button = digitalRead(PIN_ENCODER_SW); 
    
    // Read left and right joystick data
    INPUTH_Inputs.right_joystick_x = analogRead(PIN_RIGHT_JOYSTICK_Rx);  
    INPUTH_Inputs.right_joystick_y = analogRead(PIN_RIGHT_JOYSTICK_Ry);
    INPUTH_Inputs.left_joystick_x = analogRead(PIN_LEFT_JOYSTICK_Rx);  
    INPUTH_Inputs.left_joystick_y = analogRead(PIN_LEFT_JOYSTICK_Ry);
    INPUTH_Inputs.left_joystick_button = digitalRead(PIN_LEFT_JOYSTICK_SW);
    INPUTH_Inputs.right_joystick_button = digitalRead(PIN_RIGHT_JOYSTICK_SW); 
    
    GetThruttle(INPUTH_Inputs.left_joystick_x, &INPUTH_Inputs.thruttle) ;
    GetRollPitchYawAngles(INPUTH_Inputs.right_joystick_x, INPUTH_Inputs.right_joystick_y, INPUTH_Inputs.left_joystick_y, &INPUTH_Inputs.rollangle, &INPUTH_Inputs.pitchangle, &INPUTH_Inputs.yawangle);

     // Trigger Pot
    INPUTH_Inputs.left_trigger_pot = analogRead(PIN_LEFT_POT);
    INPUTH_Inputs.right_trigger_pot = analogRead(PIN_RIGHT_POT); 

    // Trigger Switchs
    INPUTH_Inputs.trigger_button_left = digitalRead(PIN_TRIG_LEFT);
    INPUTH_Inputs.trigger_button_right = digitalRead(PIN_TRIG_RIGHT);
    INPUTH_Inputs.trigger_button_front_left = digitalRead(PIN_TRIG_Front_L);
    INPUTH_Inputs.trigger_button_fron_right = digitalRead(PIN_TRIG_Front_R);
*/ 
}

void GetButtonStatus(int buttonIndex)
{
  button_status[buttonIndex].is_updated = false;

  button_status[buttonIndex].button_curr_read = digitalRead(button_status[buttonIndex].Pin) ;

  if (button_status[buttonIndex].button_curr_read != button_status[buttonIndex].button_prev_state) {
    button_status[buttonIndex].button_tickstart = millis();
  }

  if ((millis() - button_status[buttonIndex].button_tickstart) > BUTTONS_DEBUNCE_TIME) {
    if (button_status[buttonIndex].button_curr_read != button_status[buttonIndex].button_curr_state) {
      button_status[buttonIndex].button_curr_state = button_status[buttonIndex].button_curr_read;
      button_status[buttonIndex].is_updated = true;
    }
  }

  button_status[buttonIndex].button_prev_state = button_status[buttonIndex].button_curr_read; 
}

void INPUTH_OutputTone(int duration_ms)
{
    tone(PIN_BUZZER, 4000, duration_ms);
}

//========================================================================================================
//========================================================================================================
// Local Functions

void ResetInputs(void)
{ 
  // LCD Switchs
  INPUTH_Inputs.lcd_button_up = BUTTON_RELEASED;
  INPUTH_Inputs.lcd_button_down = BUTTON_RELEASED;
  INPUTH_Inputs.lcd_button_select = BUTTON_RELEASED;  

  // Rotary Encoder
  INPUTH_Inputs.encoder_rotated = false;
  INPUTH_Inputs.encoder_direction = ENCODER_DIRECTION_CW;
  INPUTH_Inputs.encoder_button = BUTTON_RELEASED;

  // Left Joystick
  INPUTH_Inputs.left_joystick_x = 0;
  INPUTH_Inputs.left_joystick_y = 0;
  INPUTH_Inputs.left_joystick_button = BUTTON_RELEASED;

  // right Joystick
  INPUTH_Inputs.right_joystick_x = 0;
  INPUTH_Inputs.right_joystick_y = 0;
  INPUTH_Inputs.right_joystick_button = BUTTON_RELEASED;

  // Trigger Pot
  INPUTH_Inputs.left_trigger_pot = 0;
  INPUTH_Inputs.right_trigger_pot = 0;

  // Trigger Switchs
  INPUTH_Inputs.trigger_button_left = BUTTON_RELEASED;
  INPUTH_Inputs.trigger_button_right = BUTTON_RELEASED;
  INPUTH_Inputs.trigger_button_front_left = BUTTON_RELEASED;
  INPUTH_Inputs.trigger_button_fron_right = BUTTON_RELEASED;

  // Drone controls
  INPUTH_Inputs.rollangle = 0;
  INPUTH_Inputs.pitchangle = 0;
  INPUTH_Inputs.yawangle = 0;
  INPUTH_Inputs.thruttle = 1000;
}

int A,B,prev_A=1, prev_B=1;
void Encoder_ISR_A() {  
 
  B = digitalRead(PIN_ENCODER_CLK); 
  if (B != prev_B)
  {
    if( B == 1)
    {
      if( A == 0)
      { 
        //Serial1.println("DIR_CW");
        INPUTH_Inputs.encoder_rotated = true;
        INPUTH_Inputs.encoder_direction = ENCODER_DIRECTION_CW;
        LCDH_LCDUpButtonPressed(lcd_button_down, true);
      }
    }
    prev_B = B;
  } 
}

void Encoder_ISR_B() { 
  A = digitalRead(PIN_ENCODER_DT); 
  if (A != prev_A)
  {
    if( A== 1)
    {
      if( B == 0)
      { 
        //Serial1.println("DIR_CCW");
        INPUTH_Inputs.encoder_rotated = true;
        INPUTH_Inputs.encoder_direction = ENCODER_DIRECTION_CCW;
        LCDH_LCDUpButtonPressed(lcd_button_up, true);
      }
    }
    prev_A = A;
  }
}

 

void GetRollPitchYawAngles(int Rx, int Ry, int Ly, int *rollangle,int *pitchangle, int *yawangle)
{
  if( Ry > 2045 + RY_FILTER )
  {
    *rollangle = map(Ry, 2045 + RY_FILTER, 4095,0,10);  
  }
  else if( Ry < 2037 - RY_FILTER )
  {
    *rollangle = map(Ry, 2045 - RY_FILTER, 0,0,-10);   
  }
  else 
  {
    *rollangle = 0 ;
  }  


  if( Rx > 2075 + RX_FILTER)
  {
    *pitchangle = map(Rx, 2075 + RX_FILTER, 4095,0,10);  
  }
  else if( Rx < 2075 - RX_FILTER )
  {
    *pitchangle = map(Rx, 2075 - RX_FILTER, 0,0,-10);   
  }
  else 
  {
    *pitchangle = 0 ;
  }  

  if( Ly > 2037 + LY_FILTER)
  {
    *yawangle = map(Ly, 2037 + LY_FILTER, 4095,0,179);  
  }
  else if( Ly < 2037 - RY_FILTER )
  {
    *yawangle = map(Ly, 2037 - LY_FILTER, 0,0,-179);   
  }
  else 
  {
    *yawangle = 0 ;
  }  
 
}


void GetThruttle(int Lx, int *Thruttle)
{  
    if( Lx > 2096 + LX_FILTER )
    {
      *Thruttle = map(Lx, 2096 + LX_FILTER, 4095,1000,2000);
    }
    else if( Lx < 2037 - LX_FILTER )
    {
      *Thruttle = map(Lx, 2096 - LX_FILTER, 0,1000,0);
    }
    else
    {
      *Thruttle = 1000; 
    } 
}
 

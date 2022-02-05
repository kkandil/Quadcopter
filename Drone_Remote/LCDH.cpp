#include "LCDH.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
#include <Fonts/FreeMono9pt7b.h>


//========================================================================================================
//========================================================================================================
// Macros
#define OLED_RESET 4

#define lcd_button_up 0
#define lcd_button_down 1
#define lcd_button_select 2

#define PRESSED true
#define RELEASED false

//========================================================================================================
//========================================================================================================
// Global Variables


//========================================================================================================
//========================================================================================================
// Locak Variables
Adafruit_SSD1306 display(OLED_RESET);

bool lcd_buttons_state[3] = {false,false,false};



#define NUMBER_MAIN_MENU_ITEMS  6
strMainMenu main_Menu[NUMBER_MAIN_MENU_ITEMS] = {{"Main", MainMenuHandler_Main, -1, false},
						{"Telm", MainMenuHandler_Telm, -1, false},
                        {"Config", MainMenuHandler_Config, -1, false},
                        {"Drone", MainMenuHandler_Drone, -1, false},
                        {"Item1", MainMenuHandler_Item1, -1, false},
                        {"Item2", MainMenuHandler_Item2, -1, false}
                         };
int config_menu_current_index = 0;
#define NUMBER_CONFIG_MENU_ITEMS  5
strMainMenu config_Menu[NUMBER_CONFIG_MENU_ITEMS] = {{"Config", ConfigMenuHandler_Config, -1, false},
                           {"Joystick", ConfigMenuHandler_Joystick, -1, false},
                           {"PotL", ConfigMenuHandler_PotL_Min, 10, false},
                           {"PotR", ConfigMenuHandler_PotL_Max, 100, false},
                           {"Back", ConfigMenuHandler_Back, -1, false}};

int joystick_menu_current_index = 0;
#define NUMBER_JOYSTICK_MENU_ITEMS  6
strMainMenu Joystick_Menu[NUMBER_JOYSTICK_MENU_ITEMS] = {{"Joystick", JoystickMenuHandler_Joystick, -1, false},
                           {"Lx", JoystickMenuHandler_Lx, 10, false},
                           {"Ly", JoystickMenuHandler_Ly, 20, false},
                           {"Rx", JoystickMenuHandler_Rx, 30, false},
                           {"Ry", JoystickMenuHandler_Ry, 40, false},
                           {"Back", JoystickMenuHandler_Back, -1, false}};


int main_menu_current_index = 0;
int main_menu_first = 1;
int main_menu_last = 3;
int highlight_item_index = 0;

//========================================================================================================
//========================================================================================================
// Global Functions
void LCDH_Init(void)
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.setFont(&FreeMono9pt7b);

  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,9);
  display.println("  " + main_Menu[0].name);
  display.println(main_Menu[1].name);
  display.println(main_Menu[2].name);
  display.println(main_Menu[3].name);

  display.display(); 
}

void LCDH_Update(void)
{
  DisplayHandler();
}

void LCDH_LCDUpButtonPressed(int button_index, bool button_pressed)
{
  lcd_buttons_state[button_index] = button_pressed;
}
//========================================================================================================
//========================================================================================================
// Local Functions

void DisplayHandler(void)
{
  main_Menu[main_menu_current_index].FunctionHandler(0);
}
bool is_lcd_updated = false;
static void HandleOutput(strMainMenu* menu, int number_of_menu_items, int *selected_menu_item_index)
{
  if( is_lcd_updated == true)
  {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0,9);
    display.println("  " + menu[0].name);
    String line = "";
    for(int i=main_menu_first ; i<=main_menu_last; i++)
    {
      line = "";
      if( highlight_item_index == i)
      {
        line += ">";
        //display.setTextColor(BLACK, WHITE);
      }
      else
      {
        //display.setTextColor(WHITE);
      }
      line += menu[i].name; 
      if( menu[i].value != -1)
      {
        line += ": " + String(menu[i].value,1);
      }
      display.println(line);
    }
    display.display(); 
  }

  is_lcd_updated = HandleMenuItemsIndex(menu, number_of_menu_items, selected_menu_item_index);
}
int prev_selected_menu_item = 0;
static bool HandleMenuItemsIndex(strMainMenu* menu, int number_of_items, int *selected_menu_item_index)
{
  bool is_lcd_updated = false;

  main_menu_last = main_menu_first + 2;
  if( main_menu_last > number_of_items-1)
  {
    main_menu_last = number_of_items-1;
  }

  if( lcd_buttons_state[lcd_button_up] == PRESSED && highlight_item_index != 1)
  {
    if( highlight_item_index == *selected_menu_item_index && ( menu[*selected_menu_item_index].value != -1))
    {
      menu[*selected_menu_item_index].value += 1;
    }
    else
    {
      highlight_item_index -= 1;
      if( highlight_item_index < main_menu_first && main_menu_first != 1)
      {
        main_menu_first -= 1;
        main_menu_last -= 1;
      }
      
    }
//    Serial1.print("highlight_item_index: " + String(highlight_item_index) + ", " );
//    Serial1.print("main_menu_first: " + String(main_menu_first) + ", " );
//    Serial1.print("main_menu_last: " + String(main_menu_last) + ", " );
//    Serial1.println("selected_menu_item_index: " + String(*selected_menu_item_index) + ", " );
    lcd_buttons_state[lcd_button_up] = RELEASED;
    is_lcd_updated = true;
  }
  else if( lcd_buttons_state[lcd_button_up] == PRESSED )
  {
    highlight_item_index = 1;
    is_lcd_updated = true;
    lcd_buttons_state[lcd_button_up] = RELEASED;
    //Serial1.println("state 2");
  }
  else if( lcd_buttons_state[lcd_button_down] == PRESSED  && highlight_item_index != number_of_items - 1)
  { 
    if( highlight_item_index == *selected_menu_item_index && ( menu[*selected_menu_item_index].value != -1))
    {
      menu[*selected_menu_item_index].value -= 1;
    }
    else
    {
      highlight_item_index += 1;
      if( highlight_item_index > main_menu_last && main_menu_last != number_of_items - 1)
      {
        main_menu_first += 1;
        main_menu_last += 1;
      }
    }
    is_lcd_updated = true;
    lcd_buttons_state[lcd_button_down] = RELEASED; 
  }
  else if( lcd_buttons_state[lcd_button_select] == PRESSED && (menu[highlight_item_index].value == -1))//&& (menu[*selected_menu_item_index].value == -1))
  {
    *selected_menu_item_index = highlight_item_index;
    if( menu[highlight_item_index].name != "Back")
      main_menu_first = 1;
    highlight_item_index = 0;
    lcd_buttons_state[lcd_button_select] = RELEASED;
    is_lcd_updated = true;
  }
  else if( lcd_buttons_state[lcd_button_select] == PRESSED  && (menu[highlight_item_index].value != (float)-1))
  {
    if( highlight_item_index != *selected_menu_item_index)
    {
      prev_selected_menu_item = *selected_menu_item_index;
      *selected_menu_item_index = highlight_item_index;
    }
    else
    {
      *selected_menu_item_index = prev_selected_menu_item;
    }
    lcd_buttons_state[lcd_button_select] = RELEASED;
    is_lcd_updated = true;
  }
  else
  {
    if( lcd_buttons_state[lcd_button_up] == PRESSED )
      lcd_buttons_state[lcd_button_up] = RELEASED;
    if( lcd_buttons_state[lcd_button_down] == PRESSED )
      lcd_buttons_state[lcd_button_down] = RELEASED;
    if( lcd_buttons_state[lcd_button_select] == PRESSED )
      lcd_buttons_state[lcd_button_select] = RELEASED;
  }

  return is_lcd_updated;
}


//------------------------------------------------------------------------
// Main menu handlers
static void MainMenuHandler_Main (int value)
{
  HandleOutput(main_Menu, NUMBER_MAIN_MENU_ITEMS, &main_menu_current_index);
}
static void MainMenuHandler_Telm (int value)
{
}
static void MainMenuHandler_Config (int value)
{
  config_Menu[config_menu_current_index].FunctionHandler(0);
}
static void MainMenuHandler_Drone (int value)
{
	HandleOutput(main_Menu, NUMBER_MAIN_MENU_ITEMS, &main_menu_current_index);
}
static void MainMenuHandler_Item1 (int value)
{
	HandleOutput(main_Menu, NUMBER_MAIN_MENU_ITEMS, &main_menu_current_index);
}
static void MainMenuHandler_Item2 (int value)
{
	HandleOutput(main_Menu, NUMBER_MAIN_MENU_ITEMS, &main_menu_current_index);
}

//------------------------------------------------------------------------
// Config menu handlers
static void ConfigMenuHandler_Config (int value)
{
  HandleOutput(config_Menu, NUMBER_CONFIG_MENU_ITEMS, &config_menu_current_index);
}
static void ConfigMenuHandler_Joystick (int value)
{
  Joystick_Menu[joystick_menu_current_index].FunctionHandler(0);
}
static void ConfigMenuHandler_PotL_Min (int value)
{
  HandleOutput(config_Menu, NUMBER_CONFIG_MENU_ITEMS, &config_menu_current_index);
}
static void ConfigMenuHandler_PotL_Max (int value)
{
  HandleOutput(config_Menu, NUMBER_CONFIG_MENU_ITEMS, &config_menu_current_index);
}
static void ConfigMenuHandler_Back (int value)
{
  config_menu_current_index = 0;
  main_menu_current_index = 0;
  highlight_item_index = 0;
  main_menu_first = 1;
}


//------------------------------------------------------------------------
// Joystick menu handlers
static void JoystickMenuHandler_Joystick (int value)
{
  HandleOutput(Joystick_Menu, NUMBER_JOYSTICK_MENU_ITEMS, &joystick_menu_current_index);
}
static void JoystickMenuHandler_Lx (int value)
{
	HandleOutput(Joystick_Menu, NUMBER_JOYSTICK_MENU_ITEMS, &joystick_menu_current_index);
}
static void JoystickMenuHandler_Ly (int value)
{
	HandleOutput(Joystick_Menu, NUMBER_JOYSTICK_MENU_ITEMS, &joystick_menu_current_index);
}
static void JoystickMenuHandler_Rx (int value)
{
	HandleOutput(Joystick_Menu, NUMBER_JOYSTICK_MENU_ITEMS, &joystick_menu_current_index);
}
static void JoystickMenuHandler_Ry (int value)
{
	HandleOutput(Joystick_Menu, NUMBER_JOYSTICK_MENU_ITEMS, &joystick_menu_current_index);
}
static void JoystickMenuHandler_Back (int value)
{
  joystick_menu_current_index = 0;
  config_menu_current_index = 0;
  highlight_item_index = 0;
  main_menu_first = 1;
}

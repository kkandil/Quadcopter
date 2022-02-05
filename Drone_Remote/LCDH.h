//#include "INPUTH.h"
#include <WString.h>  

//========================================================================================================
//========================================================================================================
// Macros

typedef struct{
  String name;
  void (*FunctionHandler) (int value);
  float value;
  bool is_updated;
}strMainMenu;

//========================================================================================================
//========================================================================================================
// Global Variables


//========================================================================================================
//========================================================================================================
// Locak Variables


//========================================================================================================
//========================================================================================================
// Global Functions
extern void LCDH_Init(void);
extern void LCDH_LCDUpButtonPressed(int button_index, bool button_pressed);

extern void LCDH_Update(void);

//========================================================================================================
//========================================================================================================
// Local Functions
static void DisplayHandler(void);

static void MainMenuHandler_Main (int value) ;
static void MainMenuHandler_Config (int value) ;
static void MainMenuHandler_Telm (int value) ;
static void MainMenuHandler_Drone (int value) ;
static void MainMenuHandler_Item1 (int value) ;
static void MainMenuHandler_Item2 (int value) ;

static void ConfigMenuHandler_Config (int value) ;
static void ConfigMenuHandler_Joystick (int value) ;
static void ConfigMenuHandler_PotL_Min (int value) ;
static void ConfigMenuHandler_PotL_Max (int value) ;
static void ConfigMenuHandler_Back (int value) ;


static void JoystickMenuHandler_Joystick (int value) ;
static void JoystickMenuHandler_Lx (int value) ;
static void JoystickMenuHandler_Ly (int value) ;
static void JoystickMenuHandler_Rx (int value) ;
static void JoystickMenuHandler_Ry (int value) ;
static void JoystickMenuHandler_Back (int value) ;


static void HandleOutput(strMainMenu* menu, int number_of_menu_items, int *selected_menu_item_index);
static bool HandleMenuItemsIndex(strMainMenu* menu,int number_of_items, int *selected_menu_item_index);

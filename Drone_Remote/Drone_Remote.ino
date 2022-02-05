#include "INPUTH.h"
#include "RFH.h"
#include "LCDH.h"
#include <MapleFreeRTOS821.h>

static void Task1(void *pvParameters) {
    for (;;) {
        //Serial1.println(millis() - task1_timer);
        //task1_timer = millis();
        
        //RFH_SendData(); 
        vTaskDelay(10); 
    }
}

void setup() { 
    Serial1.begin(250000);

    
    INPUTH_Init();
    LCDH_Init();

    xTaskCreate(Task1, "Task1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    vTaskStartScheduler();
    //RFH_Init();
}

void loop() { 
    INPUTH_ReadInputs();
    LCDH_Update();
    //RFH_SendData();

    delay(10);
}

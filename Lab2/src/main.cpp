#include <sam.h>
//include <FreeRTOS_SAMD21.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Define priorities using const int */
static const int LED_ON_TASK_PRIORITY   = 2;
static const int LED_OFF_TASK_PRIORITY  = 1;
static const int STATUS_TASK_PRIORITY   = 1;

/* Mutex handle for the shared LED GPIO pin */
static SemaphoreHandle_t xLedMutex = NULL;

/* Helper function for active delay (polling-based wait) */
void vActiveWaitMs(uint32_t ulMs) {
    // Approximate delay loop running at 48MHz core clock
    volatile uint32_t ulCycles = (48000000UL / 1000UL / 4UL) * ulMs;
    while(ulCycles--) {
        __asm__("nop");
    }
}

/* UART Initialization for status printing (Sercom / UART configuration) */
void UART_Init(void) {
    // Enable peripheral clock for SERCOM0 (example mapping for debugging UART)
    // Adjust depending on the specific pin configuration of your board
    SYSCTRL->OSC8M.bit.PRESC = 0; // 8MHz or DFLL4M at 48MHz
    // Standard bare-metal UART configuration goes here if physical printing is required.
}

/* Task 1: Turns LED ON, actively waits 0.5s, then yields */
static void vLedOnTask(void *pvParameters) {
    for( ;; ) {
        // Attempt to take the mutex before accessing the shared LED pin
        if(xLedMutex != NULL && xSemaphoreTake(xLedMutex, portMAX_DELAY) == pdTRUE) {
            
            /* Turn LED ON (Example: PORT->Group[0].OUTSET.reg = (1 << 17); ) */
            PORT->Group[0].OUTSET.reg = (1 << 17); 

            // Active wait for 0.5 seconds
            vActiveWaitMs(500);

            // Release the mutex
            xSemaphoreGive(xLedMutex);
        }

        // Yield the processor to other tasks of equal priority
        taskYIELD();
    }
}

/* Task 2: Turns LED OFF, and task-delays for 1 second */
static void vLedOffTask(void *pvParameters) {
    for( ;; ) {
        if(xLedMutex != NULL && xSemaphoreTake(xLedMutex, portMAX_DELAY) == pdTRUE) {
            
            /* Turn LED OFF */
            PORT->Group[0].OUTCLR.reg = (1 << 17);

            xSemaphoreGive(xLedMutex);
        }

        // Task delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Task 3: Prints a status message via serial UART and task-delays for 1 second */
static void vStatusTask(void *pvParameters) {
    for( ;; ) {
        // Print status message over UART (placeholder transmission check)
        // while(!(SERCOM0->USART.INTFLAG.bit.RECV)); // Example TX handling
        
        // Task delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    // Initialize system core and clock via CMSIS
    SystemInit();
    UART_Init();

    // Configure LED pin (PA17) as digital output
    PORT->Group[0].DIRSET.reg = (1 << 17);

    // Create the Mutex for the shared GPIO pin
    xLedMutex = xSemaphoreCreateMutex();

    if(xLedMutex != NULL) {
        // Create the three tasks with individual priorities defined by const ints
        xTaskCreate(vLedOnTask, "LED_On", configMINIMAL_STACK_SIZE, NULL, LED_ON_TASK_PRIORITY, NULL);
        xTaskCreate(vLedOffTask, "LED_Off", configMINIMAL_STACK_SIZE, NULL, LED_OFF_TASK_PRIORITY, NULL);
        xTaskCreate(vStatusTask, "Status", configMINIMAL_STACK_SIZE, NULL, STATUS_TASK_PRIORITY, NULL);

        // Start the FreeRTOS scheduler
        vTaskStartScheduler();
    }

    // Execution should never reach here
    for( ;; );
}

extern "C" void loop() {
  // Intentionally empty; required by FreeRTOS_SAMD21's idle_hook.c
}
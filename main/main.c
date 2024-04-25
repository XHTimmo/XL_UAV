#include <stdio.h>
#include <IIC.h>
#include <freertos/FreeRTOS.h>
void app_main(void)
{
    printf("XL_UAV\n");
    i2c_bus_init();
    while (1)
    {
        
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    
}


#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#include <hardware/gpio.h>
#include <hardware/i2c.h>

void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        usb_serial_print("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            usb_serial_print("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        /*int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        usb_serial_print ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        usb_serial_print ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        usb_serial_print ("Accel return:  %d\n", _accel);*/
    } else {
        usb_serial_print("Failed to initialize ICM-42670P.\n");
    }
    // Start collection data here. Infinite loop. 
    uint8_t buf[BUFFER_SIZE];
    while (1)
    {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
            printf("Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d", ax, ay, az, gx, gy, gzc);

        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(60));
    }

}

//reading the IMU sensor data

int ICM42670_read_sensor_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    uint8_t raw[12]; // length of the data from gyroscope and acceleration
    if (ICM42670_read_bytes(0x0B, raw, 12) != 0) //if something wrong i2c reads an error
        return -1;
    //bitwise operations to get the data in 8 bits
    int16_t raw_ax = (raw[0] << 8) | raw[1];
    int16_t raw_ay = (raw[2] << 8) | raw[3];
    int16_t raw_az = (raw[4] << 8) | raw[5];
    int16_t raw_gx = (raw[6] << 8) | raw[7];
    int16_t raw_gy = (raw[8] << 8) | raw[9];
    int16_t raw_gz = (raw[10] << 8) | raw[11];
    //as instructed, making the data human-readable:
    //accelerator
    *ax = raw_ax / 8192.0f;
    *ay = raw_ay / 8192.0f;
    *az = raw_az / 8192.0f;
    //gyroscope
    *gx = raw_gx / 131.0f;
    *gy = raw_gy / 131.0f;
    *gz = raw_gz / 131.0f;
    
    return 0;
}



int main() {
    stdio_init_all();
    sleep_ms(2000); //Wait to see the output.
    init_hat_sdk();
    printf("Start tests\n");
    
    // Initialize LED
    init_red_led();
    printf("Initializing red led\r\n");
    
    //Testing RED LED
    // printf("Testing red led should be on\r\n");
   


    // Test SW1 
    //init_sw1();
    //printf("Initializing switch 1\r\n");

    //Test SW2
    //init_sw2();
    //printf("Initializing switch 2\r\n");

    //Test RGB
    //init_rgb_led();
    //printf("Initializing RGB LED\r\n");


   // Initialize Buzzer
   //init_buzzer();
   //printf("Initializing the buzzer\n");

   
    // Initialize I2C
    init_i2c_default();
    printf("Initializing the i2c\n");

    //Initialize Light Sesnsor VEML6030
    //veml6030_init();
    //printf("Initializing the light sensor\n");

    //Initialize the Temp and Humidity Sensor
    //hdc2021_init();
    //printf("Initializing the temp/humidity sensor\n");

    //Initialize the display
    //init_display();
    //printf("Initializating display\n");

    //Initialize the microphone
    //Microhpone test in test_microphone.c

    //Initialize IMU
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT) != 0){
            printf("Wrong values to init the accelerometer in ICM-42670P.\n");
        }
        if (ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT) != 0){
            printf("Wrong values to init the gyroscope in ICM-42670P.\n");
        };
        ICM42670_enable_accel_gyro_ln_mode();
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    float ax, ay, az, gx, gy, gz, t;

    while(true){
        set_red_led_status(true);
        //printf("SW1 state: %d\n", gpio_get(SW1_PIN));
        //printf("SW2 state: %d\n", gpio_get(SW2_PIN));
        //buzzer_play_tone(440, 500);
        /*rgb_led_write(255,0,0);
        sleep_ms(1000);
        rgb_led_write(0,255,0);
        sleep_ms(1000);
        rgb_led_write(0,0,255);*/
        //uint16_t reg = _veml6030_read_register(VEML6030_CONFIG_REG);
        //printf("Register: 0x%04X\n",(unsigned int)reg);
        //uint32_t light = veml6030_read_light();
        //printf("Light level: %u\n", light);
        // clear_display();
        

        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            
            printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);

        } else {
            printf("Failed to read imu data\n");
        }
        sleep_ms(1000);
    }// end infite loop
        

 


    
    

   

    // Create tasks
    
    // xTaskCreate(sw2_task, "SW2Task", 256, NULL, 1, NULL);
    // xTaskCreate(rgb_task, "RGBTask", 256, NULL, 1, NULL);
    // xTaskCreate(buzzer_task, "BuzzerTask", 256, NULL, 4, NULL);
    // xTaskCreate(light_sensor_task, "LightSensorTask", 256, NULL, 3, NULL);
    // xTaskCreate(ths_task, "THSTask", 256, NULL, 1, NULL);
    xTaskCreate(imu_task, "IMUTask", 256, NULL, 1, NULL);
    // xTaskCreate(led_simple_task, "LEDSimpleTask", 64, NULL, 1, NULL);
    // xTaskCreate(sw1_task, "SW1Task", 64, NULL, 1, NULL);
    // xTaskCreate(led_task, "LEDTask", 64, NULL, 2, NULL);
 

    //  xTaskCreate(display_task, "DisplayTask", 256, NULL, 2, NULL);
    // // xTaskCreate(mic_task, "MicTask", 256, NULL, 1, NULL);
    
    // Start the FreeRTOS scheduler
    //vTaskStartScheduler();

    return 0;
}

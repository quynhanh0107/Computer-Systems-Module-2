
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#include <hardware/gpio.h>
#include <hardware/i2c.h>



#define BUFFER_SIZE 40

// Introducing state
enum state {IDLE=1, READ_SENSOR, UPDATE, NEW_MSG};

// Global state variable, initialized to waiting state
enum state myState = IDLE;

// Timer interrupt once per second
void clkFxn(void *pvParameters) {
    // We change the state to a desired one
    // If-clause is used to check, that the state transition is possible
    // Now we allow only the state transition IDLE -> READ_SENSOR
    if (myState == IDLE) {
    
        // State transition IDLE -> READ_SENSOR
        myState = READ_SENSOR;
    }
}

// Communications task
void commTask(void *pvParameters) {

    while (1) { 
        // Function is_message_waiting is used to check
        // are there new messages in the buffer
        // Additionally, we allow only the state transition IDLE -> NEW_MSG
        if (is_message_waiting() == TRUE && myState == IDLE) {

            // State transition IDLE -> NEW_MSG
            myState = NEW_MSG;
        
            // Functionality of state
            handle_message();
            send_reply();			        
            
            // State transition NEW_MSG -> IDLE
            myState = IDLE;
        }
    }
}

// Handling sensors
void sensorTask(void *pvParameters) {

    while (1) {
    
        if (myState == READ_SENSOR) {
        
            // Functionality of state
            read_sensor_values();
            
            // State transition READ_SENSOR -> UPDATE
            myState = UPDATE;				        
        }
    
        vTaskDelay(..);
    }
}

// Handling display update
void displayTask(void *pvParameters) {

    while (1) {
    
        if (myState == UPDATE) {
        
            // Functionality of state
            update_screen();
            
            // State transition UPDATE -> IDLE
            myState = IDLE;				        
        }
    
        vTaskDelay(..);
    }
}

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
           
            printf("Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d \n", ax, ay, az, gx, gy, gz);

        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(60));
    }

}

//reading the IMU sensor data

int ICM42670_read_sensor_data(float *ax, float *ay, float *az,
    float *gx, float *gy, float *gz) {
        
        uint8_t raw[14]; // 14 bytes total from TEMP to GYRO Z

        int rc = icm_i2c_read_bytes(ICM42670_SENSOR_DATA_START_REG, raw, sizeof(raw));
        if (rc != 0) return rc;

        // Convert to signed 16-bit integers (big-endian)
        int16_t ax_raw = ((int16_t)raw[2] << 8) | raw[3];
        int16_t ay_raw = ((int16_t)raw[4] << 8) | raw[5];
        int16_t az_raw = ((int16_t)raw[6] << 8) | raw[7];
        int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
        int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

        *ax =  (float)ax_raw / aRes; 
        *ay =  (float)ay_raw / aRes; 
        *az =  (float)az_raw / aRes;
        *gx =  (float)gx_raw / gRes; 
        *gy =  (float)gy_raw / gRes; 
        *gz =  (float)gz_raw / gRes;
        return 0; // success
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
    vTaskStartScheduler();

    return 0;
}

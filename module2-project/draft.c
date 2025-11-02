#include <math.h> // We need this for fabs() (absolute value for floats)

// Define the states our sensor can be in
enum MorseState {
    STATE_IDLE,    // In-between positions
    STATE_DOT,     // Flat on table
    STATE_DASH,    // Tilted 90 degrees
    STATE_SPACE    // Being shaken
};

// A global (static) variable to remember the last state
static enum MorseState g_last_state = STATE_IDLE;
static int g_space_count = 0;

void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    float ax, ay, az, gx, gy, gz;

    // IMU IS NOW INITIALIZED IN main()
    usb_serial_print("IMU Task Started...\n");

    while (1)
    {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
           
            enum MorseState current_state = STATE_IDLE;

            // 1. Determine current state based on sensor data
            // These thresholds may need tuning!
            
            // STATE_DOT: Flat on table (az is ~1.0g, others are ~0)
            if (az > 0.9 && fabs(ax) < 0.2 && fabs(ay) < 0.2) {
                current_state = STATE_DOT;
            }
            // STATE_DASH: Tilted 90 degrees forward (ay is ~1.0g, others are ~0)
            else if (ay > 0.9 && fabs(ax) < 0.2 && fabs(az) < 0.2) {
                current_state = STATE_DASH;
            }
            // STATE_SPACE: Shaking (high rotation detected by gyro)
            else if (fabs(gx) > 150.0 || fabs(gy) > 150.0 || fabs(gz) > 150.0) {
                current_state = STATE_SPACE;
            }

            // 2. Act only when the state *changes*
            if (current_state != g_last_state) {
                switch (current_state) {
                    case STATE_DOT:
                        printf("."); // Send DOT to terminal
                        g_space_count = 0; // Reset space count
                        break;
                    case STATE_DASH:
                        printf("-"); // Send DASH to terminal
                        g_space_count = 0; // Reset space count
                        break;
                    case STATE_SPACE:
                        printf(" "); // Send SPACE to terminal
                        g_space_count++;
                        break;
                    case STATE_IDLE:
                        // Don't print anything when returning to idle
                        break;
                }
                
                // Check for end of message
                if (g_space_count >= 3) {
                    printf("\n--- END OF MESSAGE ---\n");
                    g_space_count = 0;
                }
            }
            
            // 3. Update the last state
            g_last_state = current_state;

        } else {
            printf("Failed to read imu data\n");
        }
        
        // Poll at a reasonable rate (e.g., 5 times a second)
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}

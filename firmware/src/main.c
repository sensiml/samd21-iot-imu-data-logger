/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "buffer.h"
#include "sensor.h"
#include "app_config.h"

#if SENSIML_SIMPLE_STREAM_BUILD
#include "ssi_comms.h"
#endif //SENSIML_SIMPLE_STREAM_BUILD

#define SYSTICK_FREQ_IN_MHZ 48 // !NB! This must be changed if the processor clock changes

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

static volatile uint32_t tickcounter = 0;

static struct sensor_device_t sensor;
static struct sensor_buffer_t snsr_buffer;

#if SENSIML_SIMPLE_STREAM_BUILD
static SNSR_DATA_TYPE packet_data_buffer[SNSR_NUM_AXES * SNSR_SAMPLES_PER_PACKET];
#define PACKET_BUFFER_BYTE_LEN (SNSR_NUM_AXES * SNSR_SAMPLES_PER_PACKET * sizeof(SNSR_DATA_TYPE))
static int num_packets = 0;
static char json_config_str[512];

static void build_json_config(void)
{
    int written=0;
    int snsr_index = 0;
    written += sprintf(json_config_str, "{\"version\":%d, \"sample_rate\":%d,"
                        "\"samples_per_packet\":%d,"
                        "\"column_location\":{"
                        , SSI_JSON_CONFIG_VERSION, SNSR_SAMPLE_RATE, SNSR_SAMPLES_PER_PACKET);
    #if SNSR_USE_ACCEL_X
    written += sprintf(json_config_str+written, "\"AccelerometerX\":%d,", snsr_index++);
    #endif
    #if SNSR_USE_ACCEL_Y
    written += sprintf(json_config_str+written, "\"AccelerometerY\":%d,", snsr_index++);
    #endif
    #if SNSR_USE_ACCEL_Z
    written += sprintf(json_config_str+written, "\"AccelerometerZ\":%d,", snsr_index++);
    #endif
    #if SNSR_USE_GYRO_X
    written += sprintf(json_config_str+written, "\"GyroscopeX\":%d,", snsr_index++);
    #endif
    #if SNSR_USE_GYRO_Y
    written += sprintf(json_config_str+written, "\"GyroscopeY\":%d,", snsr_index++);
    #endif
    #if SNSR_USE_GYRO_Z
    written += sprintf(json_config_str+written, "\"GyroscopeZ\":%d", snsr_index++);
    #endif
    if(json_config_str[written-1] == ',')
    {
        written--;
    }
    sprintf(json_config_str+written, "}}");
}

#endif //SENSIML_SIMPLE_STREAM_BUILD

void SYSTICK_Callback(uintptr_t context) {
    static int mstick = 0;
    int tickrate = *((int *) context);

    ++tickcounter;
    if (tickrate == 0) {
        mstick = 0;
    }
    else if (++mstick == tickrate) {
        LED_GREEN_Toggle();
        mstick = 0;
    }
}

uint64_t read_timer_ms(void) {
    return tickcounter;
}

uint64_t read_timer_us(void) {
    return tickcounter * 1000 + SYSTICK_TimerCounterGet() / SYSTICK_FREQ_IN_MHZ;
}

void sleep_ms(uint32_t ms) {
    SYSTICK_TimerStop();
    tickcounter = 0;
    SYSTICK_TimerStart();
    while (read_timer_ms() < ms) {};
}

void sleep_us(uint32_t us) {
    SYSTICK_TimerStop();
    tickcounter = 0;
    SYSTICK_TimerStart();
    while (read_timer_us() < us) {};
}


// For handling read of the sensor data
void SNSR_ISR_HANDLER(uintptr_t context) {
    struct sensor_device_t *sensor = (struct sensor_device_t *) context;

    /* Check if any errors we've flagged have been acknowledged */
    if (sensor->status != SNSR_STATUS_OK) {
        return;
    }

    sensor->status = sensor_read(sensor, &snsr_buffer);
}

int main ( void )
{
    int tickrate = 0;

    /* Initialize all modules */
    SYS_Initialize ( NULL );

    /* Register and start the LED ticker */
    SYSTICK_TimerCallbackSet(SYSTICK_Callback, (uintptr_t) &tickrate);
    SYSTICK_TimerStart();

    /* Activate External Interrupt Controller for sensor capture */
    EIC_CallbackRegister(MIKRO_EIC_PIN, SNSR_ISR_HANDLER, (uintptr_t) &sensor);
    EIC_InterruptEnable(MIKRO_EIC_PIN);

    /* Initialize our data buffer */
    buffer_init(&snsr_buffer);

    printf("\r\n");

    while (1)
    {
        if (sensor_init(&sensor) != SNSR_STATUS_OK) {
            printf("sensor init result = %d\r\n", sensor.status);
            break;
        }

        if (sensor_set_config(&sensor) != SNSR_STATUS_OK) {
            printf("sensor configuration result = %d\r\n", sensor.status);
            break;
        }

        printf("sensor type is %s\r\n", SNSR_NAME);
        printf("sensor sample rate set at %d%s\r\n", SNSR_SAMPLE_RATE, SNSR_SAMPLE_RATE_UNIT_STR);
#if SNSR_USE_ACCEL
        printf("accelerometer axes %s%s%s enabled with range set at +/-%dGs\r\n", SNSR_USE_ACCEL_X ? "x" : "", SNSR_USE_ACCEL_Y ? "y" : "", SNSR_USE_ACCEL_Z ? "z" : "", SNSR_ACCEL_RANGE);
#else
        printf("accelerometer disabled\r\n");
#endif
#if SNSR_USE_GYRO
        printf("gyrometer axes %s%s%s enabled with range set at %dDPS\r\n", SNSR_USE_GYRO_X ? "x" : "", SNSR_USE_GYRO_Y ? "y" : "", SNSR_USE_GYRO_Z ? "z" : "", SNSR_GYRO_RANGE);
#else
        printf("gyrometer disabled\r\n");
#endif

#if SENSIML_SIMPLE_STREAM_BUILD
        build_json_config();
        SERCOM5_USART_ReceiverEnable();
        ssi_init(SERCOM5_USART_Read, SERCOM5_USART_Write);

        uint32_t time = read_timer_ms();
        ssi_try_connect();
        while(!ssi_connected()) {
            if ((read_timer_ms() - time) >= 1000) {
                time = read_timer_ms();
                printf("%s\r\n", json_config_str);
            }
        }
        ssi_try_disconnect();
#endif //SENSIML_SIMPLE_STREAM_BUILD
        buffer_reset(&snsr_buffer);
        break;
    }

    while (1)
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );

        if (sensor.status != SNSR_STATUS_OK) {
            printf("Got a bad sensor status: %d\r\n", sensor.status);
            break;
        }
        else if (snsr_buffer.overrun == true) {
            printf("\r\n\r\n\r\nOverrun!\r\n\r\n\r\n");

            // Light the LEDs to indicate overflow
            tickrate = 0;
            LED_ALL_Off();
            LED_YELLOW_On(); LED_RED_On();  // Indicate OVERFLOW
            sleep_ms(5000U);
            LED_ALL_Off(); // Clear OVERFLOW

            buffer_reset(&snsr_buffer);
            continue;
        }
        else {
            // Feed temp buffer
            buffer_data_t *ptr;
            int rdcnt = buffer_get_read_buffer(&snsr_buffer, &ptr);

            while ( --rdcnt >= 0 ) {
#if SENSIML_SIMPLE_STREAM_BUILD
                for(int i = 0; i < SNSR_NUM_AXES; i++)
                {
                    packet_data_buffer[i+(num_packets * SNSR_NUM_AXES)]=ptr[i];
                }
                num_packets++;
                if(num_packets == SNSR_SAMPLES_PER_PACKET)
                {
                    #if SSI_JSON_CONFIG_VERSION==2
                    ssiv2_publish_sensor_data(0, (uint8_t*) packet_data_buffer, PACKET_BUFFER_BYTE_LEN);
                    #elif SSI_JSON_CONFIG_VERSION==1
                    ssiv1_publish_sensor_data((uint8_t*) packet_data_buffer, PACKET_BUFFER_BYTE_LEN);
                    #endif
                    num_packets = 0;
                }
#elif DATA_VISUALIZER_BUILD
                uint8_t headerbyte = MPDV_START_OF_FRAME;

                SERCOM5_USART_Write(&headerbyte, 1);

                SERCOM5_USART_Write(ptr, SNSR_NUM_AXES*sizeof(buffer_data_t));

                headerbyte = ~headerbyte;
                SERCOM5_USART_Write(&headerbyte, 1);
                headerbyte = ~headerbyte;
#elif DATA_LOGGER_BUILD
                printf("%d", ptr[0]);
                for (int j=1; j < SNSR_NUM_AXES; j++) {
                    printf(" %d", ptr[j]);
                }
                printf("\r\n");
#endif
                buffer_advance_read_index(&snsr_buffer, 1);
                ptr += SNSR_NUM_AXES;
            }
        }

    }
    tickrate = 0;
    LED_GREEN_Off();
    LED_RED_On();

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/


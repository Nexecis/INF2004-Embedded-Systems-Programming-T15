#include <stdio.h>                                  // Include the standard I/O library.
#include "pico/cyw43_arch.h"                        // Include the CYW43 architecture-specific header.
#include "pico/stdlib.h"                            // Include the Pico standard library.
#include "FreeRTOS.h"                               // Include the FreeRTOS header.
#include "task.h"                                   // Include the task management header.
#include "queue.h"                                  // Include the queue header.
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <string.h>                                 // Include for TCP connection
#include <stdlib.h>                                 // Include for TCP connection
#include "lwip/pbuf.h"                              // Include for TCP connection
#include "lwip/tcp.h"                               // Include for TCP connection

#define QUEUE_SIZE 10                               // Define the size of the queues.

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0                      // Define the default core to run FreeRTOS on.
#endif

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL) // Define the priority for the test task.

static QueueHandle_t xTriggerQueue;
static QueueHandle_t xSpeedAQueue;
static QueueHandle_t xSpeedBQueue;
static QueueHandle_t xSpeedQueue;


// Define the GPIO pins for motor control
#define MOTOR_ENA 0  // Enable motor A
#define MOTOR_IN1 2  // Input 1 for motor A
#define MOTOR_IN2 3  // Input 2 for motor A

#define MOTOR_ENB 1  // Enable motor B
#define MOTOR_IN3 5  // Input 1 for motor B
#define MOTOR_IN4 4  // Input 2 for motor B

#define PI 3.14159
#define WHEEL_DIAMETER 6.5
#define DEBOUNCE_DELAY_MS 3

#define TCP_PORT 4242
#define BUF_SIZE 6
#define TEST_ITERATIONS 1
#define WIFI_SSID "Nexecis"
#define WIFI_PASSWORD "nrro6661"

// Define a global variable to store the last event time for debouncing
static uint32_t last_event_time = 0;

// Define a structure to represent the TCP data
typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    uint8_t buffer_sent[BUF_SIZE];
    uint8_t buffer_recv[BUF_SIZE];
    int sent_len;
    int recv_len;
    int run_count;
} TCP_SERVER_T;

// Define a structure to represent the encoder data
typedef struct {
    uint64_t start_time;
    float quarter_rotation;
    float arch;
    int pinhole;
    float total_distance;
    uint64_t milliseconds_per_cycle;
    float speed;
} Encoder;

// Initialize an instance of the Encoder structure
Encoder encoder = {0, 0.0, 0.0, 0, 0.0, 0, 0.0};

err_t tcp_on_recv_action(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err){
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    if (p->tot_len > 0) {
        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - state->recv_len;
        state->recv_len += pbuf_copy_partial(p, state->buffer_recv + state->recv_len,
                                             p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    uint8_t received_byte;
    printf("Received data: ");
    for (int i = 0; i < state->recv_len; i++) {
        received_byte = state->buffer_recv[i];
        printf("%c", received_byte);

        //do something with the received data
    }
    printf("\n");

    return ERR_OK;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_recv(client_pcb, tcp_on_recv_action);

    return ERR_OK;
}

static void tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    ip4addr_ntoa(netif_ip4_addr(netif_list));

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    tcp_bind(pcb, NULL, TCP_PORT);
    state->server_pcb = tcp_listen_with_backlog(pcb, 1);

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);
}

// Callback function to handle GPIO interrupt events
void check_speed_a_callback(uint gpio, uint32_t events) {

    // Get the current time in milliseconds since boot
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Check if enough time has passed to debounce the signal
    if (current_time - last_event_time >= DEBOUNCE_DELAY_MS) {

        if (encoder.pinhole == 0) {
            // Record the start time for the cycle
            encoder.start_time = time_us_64();
            encoder.pinhole++;
        } else if (encoder.pinhole > 0 && encoder.pinhole < 4) {
            // Increment the pinhole count
            encoder.pinhole++;
        } else {
            // Calculate the total distance, milliseconds per cycle and speed
            encoder.total_distance = encoder.total_distance + encoder.quarter_rotation;
            encoder.milliseconds_per_cycle = (time_us_64() - encoder.start_time) / 1000;
            encoder.speed = ((encoder.quarter_rotation / 100) * (1000 / encoder.milliseconds_per_cycle));

            // Print the results
            printf("%.2f cm\n", encoder.total_distance);
            printf("%lld milliseconds\n", encoder.milliseconds_per_cycle);
            printf("%.5f m/s\n\n", encoder.speed);

            // Reset the pinhole count
            encoder.pinhole = 0;
        }

        // Update the last event time for debouncing
        last_event_time = current_time;
    }
    
    xQueueSend(xSpeedAQueue, &encoder.speed, portMAX_DELAY);
}

// Callback function to handle GPIO interrupt events
void check_speed_b_callback(uint gpio, uint32_t events) {

    // Get the current time in milliseconds since boot
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Check if enough time has passed to debounce the signal
    if (current_time - last_event_time >= DEBOUNCE_DELAY_MS) {

        if (encoder.pinhole == 0) {
            // Record the start time for the cycle
            encoder.start_time = time_us_64();
            encoder.pinhole++;
        } else if (encoder.pinhole > 0 && encoder.pinhole < 4) {
            // Increment the pinhole count
            encoder.pinhole++;
        } else {
            // Calculate the total distance, milliseconds per cycle and speed
            encoder.total_distance = encoder.total_distance + encoder.quarter_rotation;
            encoder.milliseconds_per_cycle = (time_us_64() - encoder.start_time) / 1000;
            encoder.speed = ((encoder.quarter_rotation / 100) * (1000 / encoder.milliseconds_per_cycle));

            // Print the results
            printf("%.2f cm\n", encoder.total_distance);
            printf("%lld milliseconds\n", encoder.milliseconds_per_cycle);
            printf("%.5f m/s\n\n", encoder.speed);

            // Reset the pinhole count
            encoder.pinhole = 0;
        }

        // Update the last event time for debouncing
        last_event_time = current_time;
    }

    xQueueSend(xSpeedBQueue, &encoder.speed, portMAX_DELAY);
}

void motor_setup() {
    // Configure GPIO pins for motor control as outputs
    gpio_init(MOTOR_ENA); // Initialize the motor A enable pin
    gpio_init(MOTOR_IN1); // Initialize motor A input 1
    gpio_init(MOTOR_IN2); // Initialize motor A input 2

    gpio_init(MOTOR_ENB); // Initialize the motor B enable pin
    gpio_init(MOTOR_IN3); // Initialize motor B input 1
    gpio_init(MOTOR_IN4); // Initialize motor B input 2

    gpio_set_dir(MOTOR_ENA, GPIO_OUT); // Set motor A enable pin as an output
    gpio_set_dir(MOTOR_IN1, GPIO_OUT); // Set motor A input 1 as an output
    gpio_set_dir(MOTOR_IN2, GPIO_OUT); // Set motor A input 2 as an output

    gpio_set_dir(MOTOR_ENB, GPIO_OUT); // Set motor B enable pin as an output
    gpio_set_dir(MOTOR_IN3, GPIO_OUT); // Set motor B input 1 as an output
    gpio_set_dir(MOTOR_IN4, GPIO_OUT); // Set motor B input 2 as an output

    // Initialize motor control (set ENA and ENB HIGH for full speed)
    gpio_put(MOTOR_ENA, 1); // Enable motor A
    gpio_put(MOTOR_IN1, 0); // Set motor A input 1 to low
    gpio_put(MOTOR_IN2, 0); // Set motor A input 2 to low

    gpio_put(MOTOR_ENB, 1); // Enable motor B
    gpio_put(MOTOR_IN3, 0); // Set motor B input 1 to low
    gpio_put(MOTOR_IN4, 0); // Set motor B input 2 to low
}

void move_motor_a_forward(uint speed) {
    // Set motor A direction for forward motion
    gpio_put(MOTOR_IN1, 1); // Set motor A input 1 to high
    gpio_put(MOTOR_IN2, 0); // Set motor A input 2 to low

    // Adjust the speed of motor A using PWM (e.g., speed 0-255)
    pwm_set_wrap(0, 255);  // Set the PWM period for motor speed control
    pwm_set_chan_level(0, PWM_CHAN_A, speed);  // Set the motor A speed
}

void move_motor_a_backward(uint speed) {
    // Set motor A direction for backward motion
    gpio_put(MOTOR_IN1, 0); // Set motor A input 1 to low
    gpio_put(MOTOR_IN2, 1); // Set motor A input 2 to high

    // Adjust the speed of motor A using PWM (e.g., speed 0-255)
    pwm_set_wrap(0, 255);  // Set the PWM period for motor speed control
    pwm_set_chan_level(0, PWM_CHAN_A, speed);  // Set the motor A speed
}

void move_motor_b_forward(uint speed) {
    // Set motor B direction for forward motion
    gpio_put(MOTOR_IN3, 1); // Set motor B input 1 to high
    gpio_put(MOTOR_IN4, 0); // Set motor B input 2 to low

    // Adjust the speed of motor B using PWM (e.g., speed 0-255)
    pwm_set_wrap(1, 255);  // Set the PWM period for motor speed control
    pwm_set_chan_level(1, PWM_CHAN_A, speed);  // Set the motor B speed
}

void move_motor_b_backward(uint speed) {
    // Set motor B direction for backward motion
    gpio_put(MOTOR_IN3, 0); // Set motor B input 1 to low
    gpio_put(MOTOR_IN4, 1); // Set motor B input 2 to high

    // Adjust the speed of motor B using PWM (e.g., speed 0-255)
    pwm_set_wrap(1, 255);  // Set the PWM period for motor speed control
    pwm_set_chan_level(1, PWM_CHAN_A, speed);  // Set the motor B speed
}

void stop() {
    // Stop both motors
    gpio_put(MOTOR_IN1, 0); // Set motor A input 1 to low
    gpio_put(MOTOR_IN2, 0); // Set motor A input 2 to low
    pwm_set_chan_level(0, PWM_CHAN_A, 0);  // Set PWM duty cycle for motor A to 0

    gpio_put(MOTOR_IN3, 0); // Set motor B input 1 to low
    gpio_put(MOTOR_IN4, 0); // Set motor B input 2 to low
    pwm_set_chan_level(1, PWM_CHAN_A, 0);  // Set PWM duty cycle for motor B to 0
}

void check_speed_a_task(__unused void *params) {

    // Calculate the properties of the encoder movement
    encoder.quarter_rotation = (PI * WHEEL_DIAMETER) / 4;

    // Set up GPIO interrupt on pin 2 with a rising edge trigger and callback function
    gpio_set_irq_enabled_with_callback(8, GPIO_IRQ_EDGE_RISE, true, &check_speed_a_callback);

    while (true);

}

void check_speed_b_task(__unused void *params) {

    // Calculate the properties of the encoder movement
    encoder.quarter_rotation = (PI * WHEEL_DIAMETER) / 4;

    // Set up GPIO interrupt on pin 2 with a rising edge trigger and callback function
    gpio_set_irq_enabled_with_callback(9, GPIO_IRQ_EDGE_RISE, true, &check_speed_b_callback);

    while (true);

}

void check_speed_task(__unused void *params) {
    float speedA = 0.0;
    float speedB = 0.0;
    float difference = 0.0;
    int send_speed = 0;

    while (true) {
        if (xQueueReceive(xSpeedAQueue, &speedA, portMAX_DELAY) &&
            xQueueReceive(xSpeedBQueue, &speedB, portMAX_DELAY)) {
                difference = speedA - speedB;

                if (difference > 0 ) {
                    send_speed = 1;
                    xQueueSend(xSpeedQueue, &send_speed, portMAX_DELAY);
                } else if (difference < 0) {
                    send_speed = 2;
                    xQueueSend(xSpeedQueue, &send_speed, portMAX_DELAY);
                } else if (difference == 0) {
                    send_speed = 0;
                    xQueueSend(xSpeedQueue, &send_speed, portMAX_DELAY);
                }
                
        }
    }
}

void move_forward_task(__unused void *params) {
    int received_message = 0;
    int received_difference = 0;
    int speedA = 0;
    int speedB = 0;

    // Configure BUTTON_PIN as an input with pull-up enabled
    gpio_set_dir(15, GPIO_IN);
    gpio_set_pulls(15, true, false);


    xQueueReceive(xTriggerQueue, &received_message, portMAX_DELAY);
    xQueueReceive(xSpeedQueue, &received_difference, portMAX_DELAY);

    while (true) {
        if (gpio_get(15)) {
            speedA = speedB = 100;
            move_motor_a_forward(speedA);
            move_motor_b_forward(speedB);
            gpio_put(8, 1);
            gpio_put(9, 1);
        } else if (received_message == 1 && received_difference == 1) {
            speedA += 20;
            move_motor_a_forward(speedA);
        } else if (received_message == 1 && received_difference == 2) {
            speedB += 20;
            move_motor_b_forward(speedB);
        } else if (received_message == 0) {
            stop();
            gpio_put(8, 0);
            gpio_put(9, 0);
        }
    }
}

void run_tcp_server_task(__unused void *params) {
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    tcp_server_open(state);
    while(!state->complete);
    free(state);
}


void vLaunch(void) {
    // Create queues.
    xTriggerQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));
    xSpeedAQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));
    xSpeedBQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));
    xSpeedQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));

    // Create tasks.
    xTaskCreate(run_tcp_server_task, "TCPServerThread", configMINIMAL_STACK_SIZE, NULL, 9, NULL);
    //xTaskCreate(move_forward_task, "ForwardThread", configMINIMAL_STACK_SIZE, NULL, 9, NULL);
    //xTaskCreate(check_speed_a_task, "SpeedAThread", configMINIMAL_STACK_SIZE, NULL, 9, NULL);
    //xTaskCreate(check_speed_b_task, "SpeedBThread", configMINIMAL_STACK_SIZE, NULL, 9, NULL);
    //xTaskCreate(check_speed_task, "SpeedThread", configMINIMAL_STACK_SIZE, NULL, 9, NULL);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    vTaskCoreAffinitySet(task, 1);                // Set core affinity for the main task.
#endif

    vTaskStartScheduler();                         // Start the FreeRTOS scheduler.
}

int main(void) {
    stdio_init_all();                              // Initialize standard I/O.
    cyw43_arch_init();
    cyw43_arch_enable_sta_mode();
    cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 0x00400004, 30000);

    const char *rtos_name;
#if (portSUPPORT_SMP == 1)
    rtos_name = "FreeRTOS SMP";                   // Define the name for SMP-enabled FreeRTOS.
#else
    rtos_name = "FreeRTOS";                       // Define the name for regular FreeRTOS.
#endif

#if (portSUPPORT_SMP == 1) && (configNUM_CORES == 2)
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();                                     // Start FreeRTOS on both cores.
#elif (RUN_FREERTOS_ON_CORE == 1)
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch());             // Start FreeRTOS on core 1.
    while (true);
#else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();                                     // Start FreeRTOS on core 0.
#endif
    return 0;                                      // Return from the main function.
}

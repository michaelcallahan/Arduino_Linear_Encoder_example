#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <gpio.h>
#include <usart.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define STRING_BUFFER_LEN 50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

/* Pin Definitions */
#define DIR_PIN GPIO_PIN_10   // D2 -> GPIOA_PIN_10
#define DIR_PORT GPIOA
#define STEP_PIN GPIO_PIN_3  // D3 -> GPIOA_PIN_3
#define STEP_PORT GPIOB
#define EN_PIN GPIO_PIN_5    // D4 -> GPIOA_PIN_5
#define EN_PORT GPIOB

#define DATA_PIN GPIO_PIN_9  // D8 -> GPIOB_PIN_9
#define DATA_PORT GPIOA
#define CK_PIN GPIO_PIN_7    // D9 -> GPIOA_PIN_7
#define CK_PORT GPIOC
#define REQ_PIN GPIO_PIN_7   // D11 -> GPIOA_PIN_7
#define REQ_PORT GPIOA

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Float32 encoder_msg;
std_msgs__msg__Float32 position_msg;

int device_id;
int seq_no;

/* PID constants */
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0.0;

/* PID variables */
float setpoint = 7.5;  // Desired position in mm (initial value)
float input, output;
float lastInput;
float ITerm;
float speedMult = 300;

/* Encoder variables */
volatile long encoderData = 0;
volatile bool newDataAvailable = false;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) {

		seq_no = rand();

		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
    rcl_publish(&publisher, &encoder_msg, NULL);
	}
}

void positionControlCallback(const void *msg_in)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    setpoint = msg->data;
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Received new setpoint: %f\r\n", setpoint);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void setupPID()
{
    // Implement PID setup here
    lastInput = 0;
    ITerm = 0;
}

void computePID()
{
    float error = setpoint - input;
    ITerm += (Ki * error);
    if (ITerm > 255) ITerm = 255;
    else if (ITerm < -255) ITerm = -255;
    float dInput = (input - lastInput);

    output = Kp * error + ITerm - Kd * dInput;
    if (output > 255) output = 255;
    else if (output < -255) output = -255;

    lastInput = input;

    char buffer[50];
    snprintf(buffer, sizeof(buffer), "PID Output: %f\r\n", output);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    if (output > 0) {
        motorSetDirection(0); // Set direction to forward
        motorStep();
    } else {
        motorSetDirection(1); // Set direction to backward
        motorStep();
    }
}

float readEncoder()
{
    uint8_t data[13];
    int signCh = 8;
    int sign = 0;
    int decimal;
    float dpp;
    int units;
    char value_str[7];
    long value_int;
    float value;

    HAL_GPIO_WritePin(REQ_PORT, REQ_PIN, GPIO_PIN_RESET);

    for (int i = 0; i < 13; i++) {
        char value = 0;
        for (int bit = 0; bit < 4; bit++) {
            /** //For Debugging Purposes
            while (HAL_GPIO_ReadPin(CK_PORT, CK_PIN) == GPIO_PIN_RESET); // Wait until clock is high
            while (HAL_GPIO_ReadPin(CK_PORT, CK_PIN) == GPIO_PIN_SET); // Wait until clock is low
            */
            value |= (HAL_GPIO_ReadPin(DATA_PORT, DATA_PIN) & 0x1) << bit;
        }
        data[i] = value;
    }

    sign = data[4];
    snprintf(value_str, sizeof(value_str), "%c%c%c%c%c%c", data[5], data[6], data[7], data[8], data[9], data[10]);
    decimal = data[11];
    units = data[12];
    value_int = atoi(value_str);

    switch (decimal) {
        case 0: dpp = 1.0; break;
        case 1: dpp = 10.0; break;
        case 2: dpp = 100.0; break;
        case 3: dpp = 1000.0; break;
        case 4: dpp = 10000.0; break;
        case 5: dpp = 100000.0; break;
        default: dpp = 1.0; break;
    }

    value = value_int / dpp;
    value = input + output/500; //For Simulation Purposes
    HAL_GPIO_WritePin(REQ_PORT, REQ_PIN, GPIO_PIN_SET);

    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Encoder value: %f\r\n", value);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    return value;
}


void motorSetDirection(uint8_t direction)
{
    if (direction == 0) {
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET); // Forward
    } else {
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);   // Backward
    }
}

void motorEnable(uint8_t enable)
{
    if (enable == 0) {
        HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET); // Disable
    } else {
        HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_SET);   // Enable
    }
}

void motorStep(void)
{
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
    HAL_Delay(1); // Adjust delay as needed for step signal
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
}

void appMain(void *argument)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "microros_node", "", &support));

	// Create a reliable publisher
	RCCHECK(rclc_publisher_init_default(&publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/microROS/encoder"));

	// Create a best effort subscriber
	RCCHECK(rclc_subscription_init_best_effort(&subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/microROS/setpoint"));


	// Create a 3 seconds ping timer timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), timer_callback));


	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &position_msg,
		&positionControlCallback, ON_NEW_DATA));

	device_id = rand();

	while(1){

        /* Read encoder data */
        input = readEncoder();
        encoder_msg.data = input;
        rcl_publish(&publisher, &encoder_msg, NULL);

        /* Compute PID */
        computePID();
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(100000);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}

#include <micro_ros_arduino.h>
#include <TMCStepper.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32.h>

// Define pin connections
#define ENABLE_PIN 2
#define DIR_PIN 3
#define STEP_PIN 4
#define SW_SCK 5
#define SW_TX 6
#define SW_RX 7

#define REQ_PIN 11
#define DATA_PIN 8
#define CK_PIN 9

#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f

// PID constants
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0.0;

// PID variables
float setpoint = 7.5; // Desired position in mm (initial value)
float input, output;
float lastInput;
float ITerm;
float speedMult = 300;

// Encoder variables
volatile long encoderData = 0;
volatile bool newDataAvailable = false;

// Create TMC2209 driver instance
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

// micro-ROS variables
rcl_publisher_t publisher;
std_msgs__msg__Float32 encoder_msg;
rcl_subscription_t subscriber;
std_msgs__msg__Float32 position_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

float readEncoder();
void setupPID();
void computePID();
void positionControlCallback(const void *msg_in);

void setup()
{
  // Initialize serial communication
  Serial.begin(9600);
  SERIAL_PORT.begin(9600);

  // Set pin modes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT_PULLUP);
  pinMode(CK_PIN, INPUT_PULLUP);
  pinMode(REQ_PIN, OUTPUT);

  // Enable driver
  digitalWrite(ENABLE_PIN, LOW);

  // Initialize CK and REQ pins
  digitalWrite(REQ_PIN, LOW);

  // Initialize the TMC2209 driver
  driver.begin();
  driver.toff(5);
  driver.rms_current(600);
  driver.microsteps(256);
  driver.en_spreadCycle(false);
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.pwm_autoscale(true);
  driver.shaft(HIGH);
  driver.VACTUAL(0);

  // Initialize PID variables
  lastInput = 0;
  ITerm = 0;

  // Initialize micro-ROS
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "arduino_node", "", &support);

  rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/microROS/encoder");

  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/microROS/setpoint");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &position_msg, &positionControlCallback, ON_NEW_DATA);

  encoder_msg.data = 0.0;
}

void loop()
{
  // Read encoder data
  input = readEncoder();
  encoder_msg.data = input;
  rcl_publish(&publisher, &encoder_msg, NULL);

  // Compute PID
  computePID();

  // Spin micro-ROS executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100);
}

float readEncoder()
{
  byte data[14];
  int signCh = 8;
  int sign = 0;
  int decimal;
  float dpp;
  int units;
  String value_str;
  long value_int;
  float value;

  digitalWrite(REQ_PIN, LOW);
  for (int i = 0; i < 13; i++)
  {
    char value = 0;
    for (int bit = 0; bit < 4; bit++)
    {
      while (digitalRead(CK_PIN) == LOW)
        ;
      while (digitalRead(CK_PIN) == HIGH)
        ;
      bitWrite(value, bit, (digitalRead(DATA_PIN) & 0x1));
    }
    data[i] = value;
  }

  sign = data[4];
  value_str = String(data[5]) + String(data[6]) + String(data[7]) + String(data[8]) + String(data[9]) + String(data[10]);
  decimal = data[11];
  units = data[12];
  value_int = value_str.toInt();

  switch (decimal)
  {
  case 0:
    dpp = 1.0;
    break;
  case 1:
    dpp = 10.0;
    break;
  case 2:
    dpp = 100.0;
    break;
  case 3:
    dpp = 1000.0;
    break;
  case 4:
    dpp = 10000.0;
    break;
  case 5:
    dpp = 100000.0;
    break;
  default:
    dpp = 1.0;
    break;
  }

  value = value_int / dpp;
  digitalWrite(REQ_PIN, HIGH);

  return value;
}

void positionControlCallback(const void *msg_in)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
  setpoint = msg->data;
}

void setupPID()
{
  // Implement PID setup here
}

void computePID()
{
  float error = setpoint - input;
  ITerm += (Ki * error);
  if (ITerm > 255)
    ITerm = 255;
  else if (ITerm < -255)
    ITerm = -255;
  float dInput = (input - lastInput);

  output = Kp * error + ITerm - Kd * dInput;
  if (output > 255)
    output = 255;
  else if (output < -255)
    output = -255;

  lastInput = input;

  if (output > 0)
  {
    driver.shaft(LOW);
    driver.VACTUAL(speedMult * output);
  }
  else
  {
    driver.shaft(HIGH);
    driver.VACTUAL(-speedMult * output);
  }
}
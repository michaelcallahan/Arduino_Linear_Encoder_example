#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <Streaming.h>

// Define pin connections
#define ENABLE_PIN 2 // Pin connected to EN on TMC2209
#define DIR_PIN 3    // Pin connected to DIR on TMC2209
#define STEP_PIN 4   // Pin connected to STEP on TMC2209
#define SW_SCK 5     // Software Slave Clock (SCK)
#define SW_TX 6      // SoftwareSerial receive pin
#define SW_RX 7      // SoftwareSerial transmit pin

#define REQ_PIN 11 // Pin connected to REQ on Encoder
#define DATA_PIN 8 // Pin connected to DATA on Encoder
#define CK_PIN 9   // Pin connected to CLK on Encoder

SoftwareSerial SERIAL_PORT(SW_RX, SW_TX); // Software serial port for UART communication
#define DRIVER_ADDRESS 0b00               // TMC2209 Driver address
#define R_SENSE 0.11f                     // Sense resistor value

// PID constants
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0.0;

// PID variables
float setpoint = 7.5; // Desired position in mm
float input, output;
float lastInput;
float ITerm;
float speedMult = 300;

// Encoder variables
volatile long encoderData = 0;
volatile bool newDataAvailable = false;

// Create TMC2209 driver instance
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

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
    driver.rms_current(600);      // Set motor current
    driver.microsteps(256);       // Set microstepping resolution
    driver.en_spreadCycle(false); // Use stealthChop for quiet operation
    driver.pdn_disable(true);
    driver.I_scale_analog(false);
    driver.pwm_autoscale(true); // Needed for stealthChop
    driver.shaft(HIGH);
    driver.VACTUAL(0);

    // Initialize PID variables
    lastInput = 0;
    ITerm = 0;
}

void loop()
{
    // Read encoder data
    input = readEncoder();

    // Compute PID
    float error = setpoint - input;
    ITerm += (Ki * error);
    if (ITerm > 255)
        ITerm = 255;
    else if (ITerm < -255)
        ITerm = -255;
    float dInput = (input - lastInput);

    // PID Output
    output = Kp * error + ITerm - Kd * dInput;
    if (output > 255)
        output = 255;
    else if (output < -255)
        output = -255;

    // Apply PID control to motor
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

    lastInput = input;

    // Print encoder data and PID output for debugging
    Serial.print("Encoder Data: ");
    Serial.print(input);
    Serial.print(" Output: ");
    Serial.println(output);

    delay(100); // Update every 100 ms
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
                ; // Hold until clock is high
            while (digitalRead(CK_PIN) == HIGH)
                ; // Hold until clock is low
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
        break; // Default to 1.0 if unexpected decimal place
    }

    value = value_int / dpp;
    digitalWrite(REQ_PIN, HIGH);

    return value; // Return value in mm (assuming encoder resolution)
}

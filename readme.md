# Linear CNC Stage Control with Arduino and TMC2209

This project demonstrates how to control a linear CNC stage using an Arduino, a TMC2209 stepper motor driver, and a Mitutoyo LGS-1012P linear encoder. The system employs a PID control loop to accurately position the stage. Additionally, this project demonstrates how to make use of the Mitutoyo digimatic serial interface for reading precise linear position measurements.

## Components

- **Arduino (e.g., Arduino Uno, Mega)**
- **TMC2209 Stepper Motor Driver**
- **Stepper Motor**
- **Mitutoyo LGS-1012P Linear Encoder**
- **Power Supply (appropriate for your stepper motor)**
- **Connecting Wires**
- **Breadboard (optional for prototyping)**

## Wiring

### Arduino to TMC2209
- **ENABLE_PIN** (D2) -> TMC2209 EN
- **DIR_PIN** (D3) -> TMC2209 DIR
- **STEP_PIN** (D4) -> TMC2209 STEP
- **SW_TX** (D6) -> TMC2209 PDN_UART
- **SW_RX** (D7) -> TMC2209 PDN_UART
- **GND** -> TMC2209 GND
- **5V** -> TMC2209 VCC

### TMC2209 to Stepper Motor
- **A1** -> Stepper Motor Coil A1
- **A2** -> Stepper Motor Coil A2
- **B1** -> Stepper Motor Coil B1
- **B2** -> Stepper Motor Coil B2

### Arduino to Mitutoyo Linear Encoder
- **REQ_PIN** (D11) -> Encoder REQ
- **DATA_PIN** (D8) -> Encoder DATA
- **CK_PIN** (D9) -> Encoder CLK
- **GND** -> Encoder GND
- **5V** -> Encoder VCC

### Power Supply
- **Power Supply Positive (e.g., 12V)** -> TMC2209 VM
- **Power Supply Ground** -> TMC2209 GND

## Setup Image

![Motor Linear Encoder Setup](./pictures/motor_linear_encoder_setup.jpg)

## Arduino Code

The Arduino code for this project can be found in the file: [`/motor_linear_encoder_control/motor_linear_encoder_control.ino`](./motor_linear_encoder_control/motor_linear_encoder_control.ino). This code implements a PID control loop that reads the encoder value and adjusts the motor position to achieve the desired setpoint.

## Usage

1. **Wiring**: Follow the wiring instructions provided above to connect your Arduino, TMC2209 driver, stepper motor, and Mitutoyo linear encoder.

2. **Upload Code**: Open the Arduino IDE, navigate to the provided Arduino code file (`/motor_linear_encoder_control/motor_linear_encoder_control.ino`), and upload it to your Arduino board.

3. **Monitor Output**: Open the Serial Monitor (set to 9600 baud rate) to observe the encoder readings and PID control output. Adjust the PID constants (`Kp`, `Ki`, `Kd`) as needed to achieve desired performance.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

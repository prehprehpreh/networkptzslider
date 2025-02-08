#include <AccelStepper.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <MultiStepper.h>
#include <SPI.h>
#include <LiquidCrystal.h>

// Ethernet settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x03, 0x30 };
IPAddress ip(192, 168, 0, 225);
unsigned int localPort = 44158;  // UDP port
EthernetUDP Udp;
char packetBuffer[255];

// Define LCD pins
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

// Stepper motor pins
const int stepPin1 = 3, dirPin1 = 2, ms1Pin1 = 10, ms2Pin1 = 11;
const int stepPin2 = 31, dirPin2 = 30, ms1Pin2 = 32, ms2Pin2 = 33;
const int stepPin3 = 45, dirPin3 = 44, ms1Pin3 = 40, ms2Pin3 = 41;

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3);
MultiStepper multiStepper;

float x_axis_value_left, x_axis_value_right, y_axis_value;
float msSpeed = 800, slideSpeed = 20;
int saved_position1 = 0, saved_position2 = 0, saved_position3 = 0;
bool loopPresets = false;
long presetAPositions[] = {0, 0, 0}, presetBPositions[] = {0, 0, 0};

void setup() {
    pinMode(ms1Pin1, OUTPUT); pinMode(ms2Pin1, OUTPUT);
    pinMode(ms1Pin2, OUTPUT); pinMode(ms2Pin2, OUTPUT);
    pinMode(ms1Pin3, OUTPUT); pinMode(ms2Pin3, OUTPUT);

    digitalWrite(ms1Pin1, HIGH); digitalWrite(ms2Pin1, HIGH);
    digitalWrite(ms1Pin2, HIGH); digitalWrite(ms2Pin2, HIGH);
    digitalWrite(ms1Pin3, HIGH); digitalWrite(ms2Pin3, HIGH);

    stepper1.setMaxSpeed(msSpeed);
    stepper2.setMaxSpeed(msSpeed);
    stepper3.setMaxSpeed(msSpeed);

    multiStepper.addStepper(stepper1);
    multiStepper.addStepper(stepper2);
    multiStepper.addStepper(stepper3);

    lcd.begin(16, 2);
    Ethernet.begin(mac, ip);
    Udp.begin(localPort);
    Serial.begin(9600);

    lcd.setCursor(0, 0);
    lcd.print("IP:");
    lcd.print(Ethernet.localIP());
}

void loop() {
    handleUDPRequests();
    controlStepperMotors();


  
}

void handleUDPRequests() {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        int len = Udp.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;
        String request = String(packetBuffer);

        if (request.startsWith("SET_JOYSTICK")) {
            // Parse joystick values by commas
            int firstComma = request.indexOf(',');
            int secondComma = request.indexOf(',', firstComma + 1);
            int thirdComma = request.indexOf(',', secondComma + 1);

            // Extract joystick values dynamically
            String x_axis_left_str = request.substring(13, firstComma);  // From "SET_JOYSTICK" to first comma
            String x_axis_right_str = request.substring(firstComma + 1, secondComma);  // Between commas
            String y_axis_str = request.substring(secondComma + 1, thirdComma);  // Between second and third commas

            // Convert to float
            x_axis_value_left = x_axis_left_str.toFloat();
            x_axis_value_right = x_axis_right_str.toFloat();
            y_axis_value = y_axis_str.toFloat();
        } else if (request.startsWith("SAVE_POS")) {
            saveMotorPositions();
        } else if (request.startsWith("RECALL_POS")) {
            recallMotorPositions();
        } else if (request.startsWith("SAVE_A")) {
            savePresetA();
        } else if (request.startsWith("SAVE_B")) {
            savePresetB();
        } else if (request.startsWith("RECALL_A")) {
            recallPresetA();
        } else if (request.startsWith("RECALL_B")) {
            recallPresetB();
        } else if (request.startsWith("UPDATE_LOOP")) {
            loopPresets = request.substring(12) == "true";
        }
    }
}

// Define constants for max speed of each motor
const int max_speed_left = 0;  // Max speed for left motor
const int max_speed_right = 0; // Max speed for right motor
const int max_speed_y = 0;     // Max speed for tilt motor

// Define acceleration/deceleration rates per motor
const float acceleration_rate_left = 0.3;
const float deceleration_rate_left = 0.3;

const float acceleration_rate_right = 0.1;
const float deceleration_rate_right = 0.1;

const float acceleration_rate_y = 0.1;
const float deceleration_rate_y = 0.1;

// Deadzone threshold (range where input is ignored)
const int deadzone = 10; // Adjust this as needed

// Store the current speeds of each motor
float current_speed_left = 0;
float current_speed_right = 0;
float current_speed_y = 0;

// Function to apply a deadzone and remap values
int applyDeadzoneAndRemap(int input, int minInput, int maxInput, int minOutput, int maxOutput) {
    if (abs(input) < deadzone) return 0; // If within deadzone, return zero

    // Remap values smoothly outside the deadzone
    int adjustedInput = (input > 0) ? input - deadzone : input + deadzone;
    int adjustedMaxInput = maxInput - deadzone;

    return map(adjustedInput, -adjustedMaxInput, adjustedMaxInput, minOutput, maxOutput);
}

void controlStepperMotors() {
    // Apply deadzone and remap joystick values
    int target_speed_left = applyDeadzoneAndRemap(x_axis_value_left * 100, -100, 100, -2000, 2000);
    int target_speed_right = applyDeadzoneAndRemap(x_axis_value_right * 100, -100, 100, -500, 500);
    int target_speed_y = applyDeadzoneAndRemap(y_axis_value * 100, -100, 100, -500, 500);

    // Left motor acceleration & deceleration
    if (target_speed_left > current_speed_left) {
        current_speed_left += acceleration_rate_left;
        if (current_speed_left > target_speed_left) {
            current_speed_left = target_speed_left;
        }
    } else if (target_speed_left < current_speed_left) {
        current_speed_left -= deceleration_rate_left;
        if (current_speed_left < target_speed_left) {
            current_speed_left = target_speed_left;
        }
    }

    // Right motor acceleration & deceleration
    if (target_speed_right > current_speed_right) {
        current_speed_right += acceleration_rate_right;
        if (current_speed_right > target_speed_right) {
            current_speed_right = target_speed_right;
        }
    } else if (target_speed_right < current_speed_right) {
        current_speed_right -= deceleration_rate_right;
        if (current_speed_right < target_speed_right) {
            current_speed_right = target_speed_right;
        }
    }

    // Tilt motor acceleration & deceleration
    if (target_speed_y > current_speed_y) {
        current_speed_y += acceleration_rate_y;
        if (current_speed_y > target_speed_y) {
            current_speed_y = target_speed_y;
        }
    } else if (target_speed_y < current_speed_y) {
        current_speed_y -= deceleration_rate_y;
        if (current_speed_y < target_speed_y) {
            current_speed_y = target_speed_y;
        }
    }

    // Set the final motor speeds (using setSpeed)
    stepper1.setSpeed(current_speed_left);  // Left motor speed
    stepper2.setSpeed(current_speed_right); // Right motor speed
    stepper3.setSpeed(current_speed_y);     // Tilt motor speed

    // Run the stepper motors at the current speed
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
}



void saveMotorPositions() {
    saved_position1 = stepper1.currentPosition();
    saved_position2 = stepper2.currentPosition();
    saved_position3 = stepper3.currentPosition();
}

void recallMotorPositions() {
    long targetPositions[] = {saved_position1, saved_position2, saved_position3};
    multiStepper.moveTo(targetPositions);
    multiStepper.runSpeedToPosition();
}

void savePresetA() {
    presetAPositions[0] = stepper1.currentPosition();
    presetAPositions[1] = stepper2.currentPosition();
    presetAPositions[2] = stepper3.currentPosition();
}

void savePresetB() {
    presetBPositions[0] = stepper1.currentPosition();
    presetBPositions[1] = stepper2.currentPosition();
    presetBPositions[2] = stepper3.currentPosition();
}

const int tolerance = 200;  // Tolerance in steps

void recallPresetA() {
    // Check if the stepper motors are already at preset A positions, within the tolerance
    if (abs(stepper1.currentPosition() - presetAPositions[0]) <= tolerance &&
        abs(stepper2.currentPosition() - presetAPositions[1]) <= tolerance &&
        abs(stepper3.currentPosition() - presetAPositions[2]) <= tolerance) {
        
        // If within tolerance of preset A, just send the "done" message
        sendUDPMessage("PRESET_A_DONE");
        return;  // Exit the function early
    }

    // Otherwise, move to preset A positions
    long targetPositions[] = { presetAPositions[0], presetAPositions[1], presetAPositions[2] };

    stepper1.moveTo(targetPositions[0]);
    stepper2.moveTo(targetPositions[1]);
    stepper3.moveTo(targetPositions[2]);

    syncMove(4000, 2000);

    // Send UDP confirmation
    sendUDPMessage("PRESET_A_DONE");
}

void recallPresetB() {
    // Check if the stepper motors are already at preset B positions, within the tolerance
    if (abs(stepper1.currentPosition() - presetBPositions[0]) <= tolerance &&
        abs(stepper2.currentPosition() - presetBPositions[1]) <= tolerance &&
        abs(stepper3.currentPosition() - presetBPositions[2]) <= tolerance) {
        
        // If within tolerance of preset B, just send the "done" message
        sendUDPMessage("PRESET_B_DONE");
        return;  // Exit the function early
    }

    // Otherwise, move to preset B positions
    long targetPositions[] = { presetBPositions[0], presetBPositions[1], presetBPositions[2] };

    stepper1.moveTo(targetPositions[0]);
    stepper2.moveTo(targetPositions[1]);
    stepper3.moveTo(targetPositions[2]);

    syncMove(4000, 2000);

    // Send UDP confirmation
    sendUDPMessage("PRESET_B_DONE");
}

void sendUDPMessage(const char *message) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(message);
    Udp.endPacket();
}

// Function to calculate synchronized speeds and move steppers
void syncMove(float maxSpeed, float accel)
{
    float allSpeeds[3];

    // Get all distances for each stepper
    allSpeeds[0] = fabs(stepper1.distanceToGo());
    allSpeeds[1] = fabs(stepper2.distanceToGo());
    allSpeeds[2] = fabs(stepper3.distanceToGo());

    // Find the longest travel distance
    float biggestDistance = findBiggestNumFloatArray(allSpeeds, "max");

    // Set synchronized speeds and accelerations
    stepper1.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper1));
    stepper1.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper1) / 2);

    stepper2.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper2));
    stepper2.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper2) / 2);

    stepper3.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper3));
    stepper3.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper3) / 2);

    // Move the steppers in sync
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0)
    {
        if (stepper1.distanceToGo() != 0)
            stepper1.run();
        if (stepper2.distanceToGo() != 0)
            stepper2.run();
        if (stepper3.distanceToGo() != 0)
            stepper3.run();
    }
}

// `calcSync()` function to determine proportional speed
float calcSync(float maxSpeed, float maxAccel, float maxDistance, AccelStepper stepper)
{
    if (stepper.distanceToGo() == 0)
        return 0;

    float stepperDis = stepper.distanceToGo();
    float stepperSpeed = fabs(maxSpeed) * (fabs(stepperDis) / fabs(maxDistance));

    return (stepperSpeed < 0) ? fabs(stepperSpeed) : stepperSpeed;
}

// Function to find the largest number in an array
float findBiggestNumFloatArray(float myArray[], String option)
{
    float maxVal = myArray[0];

    for (int i = 0; i < 3; i++)  // Adjusted to fixed array size
        maxVal = max(fabs(myArray[i]), fabs(maxVal));

    return maxVal;
}

#include <AccelStepper.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <MultiStepper.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h> // Added in case TMC2209 is re-enabled

// Ethernet settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x03, 0x30 };
unsigned int localPort = 44158;
EthernetUDP Udp;
char packetBuffer[255];

// Define LCD pins
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

// Stepper motor pins
const int stepPin1 = 3, dirPin1 = 2, ms1Pin1 = 10, ms2Pin1 = 11;
const int stepPin2 = 31, dirPin2 = 30, ms1Pin2 = 32, ms2Pin2 = 33;
const int stepPin3 = 45, dirPin3 = 44, ms1Pin3 = 40, ms2Pin3 = 41;
const int stepPin4 = 48, dirPin4 = 49, ms1Pin4 = 34, ms2Pin4 = 35, tmcEnPin4 = 50;

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4);
MultiStepper multiStepper;

// TMC2209 Configuration (Using Hardware Serial1, commented out)
//#define DRIVER_ADDRESS 0b00
//TMC2209Stepper driver(&Serial1, 0.11, DRIVER_ADDRESS);

// Joystick input variables
float x_axis_value_left, x_axis_value_right, y_axis_value, trigger_left, trigger_right;
float msSpeed = 800, slideSpeed = 20;
int saved_position1 = 0, saved_position2 = 0, saved_position3 = 0;
bool loopPresets = false;
long presetAPositions[] = {0, 0, 0}, presetBPositions[] = {0, 0, 0};

// Homing Parameters
AccelStepper zoomStepper = stepper4; // assign stepper 4 to zoomStepper
const int homingSpeed = 500; // Adjust homing speed
const int homingDirection = 1; // 1 for clockwise, -1 for counterclockwise
const int stallSensitivity = 100; // Adjust stall sensitivity

void setup() {
    pinMode(ms1Pin1, OUTPUT); pinMode(ms2Pin1, OUTPUT);
    pinMode(ms1Pin2, OUTPUT); pinMode(ms2Pin2, OUTPUT);
    pinMode(ms1Pin3, OUTPUT); pinMode(ms2Pin3, OUTPUT);
    pinMode(ms1Pin4, OUTPUT); pinMode(ms2Pin4, OUTPUT);
    pinMode(tmcEnPin4, OUTPUT);

    digitalWrite(ms1Pin1, HIGH); digitalWrite(ms2Pin1, HIGH);
    digitalWrite(ms1Pin2, HIGH); digitalWrite(ms2Pin2, HIGH);
    digitalWrite(ms1Pin3, HIGH); digitalWrite(ms2Pin3, HIGH);
    digitalWrite(ms1Pin4, HIGH); digitalWrite(ms2Pin4, HIGH);
    digitalWrite(tmcEnPin4, LOW);

    stepper1.setMaxSpeed(msSpeed);
    stepper2.setMaxSpeed(msSpeed);
    stepper3.setMaxSpeed(msSpeed);
    stepper4.setMaxSpeed(msSpeed);

    multiStepper.addStepper(stepper1);
    multiStepper.addStepper(stepper2);
    multiStepper.addStepper(stepper3);
    multiStepper.addStepper(stepper4);

    lcd.begin(16, 2);
    Ethernet.begin(mac);
    Udp.begin(localPort);
    Serial.begin(9600); // Initialize Serial Monitor

    lcd.setCursor(0, 0);
    lcd.print("IP:");
    lcd.print(Ethernet.localIP());

    //homeZoom(); // Uncomment when TMC2209 is re-enabled
}

void loop() {
    stepper4.runSpeed(); // Keep Motor 4 running if speed is set
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
            // Parse 5 values from SET_JOYSTICK
            int firstComma = request.indexOf(',');
            int secondComma = request.indexOf(',', firstComma + 1);
            int thirdComma = request.indexOf(',', secondComma + 1);
            int fourthComma = request.indexOf(',', thirdComma + 1);

            String x_axis_left_str = request.substring(13, firstComma);
            String x_axis_right_str = request.substring(firstComma + 1, secondComma);
            String y_axis_str = request.substring(secondComma + 1, thirdComma);
            String trigger_left_str = request.substring(thirdComma + 1, fourthComma);
            String trigger_right_str = request.substring(fourthComma + 1);

            x_axis_value_left = x_axis_left_str.toFloat();
            x_axis_value_right = x_axis_right_str.toFloat();
            y_axis_value = y_axis_str.toFloat();
            trigger_left = trigger_left_str.toFloat();
            trigger_right = trigger_right_str.toFloat();
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
        } else if (request.startsWith("SEND_CURRENT_POS")) {
            sendCurrentMotorPositions();
        } else if (request.startsWith("INCREASE_SPEED")) {
            msSpeed += 200;
            Serial.print("Speed increased to: ");
            Serial.println(msSpeed);
        } else if (request.startsWith("DECREASE_SPEED")) {
            msSpeed -= 200;
            Serial.print("Speed decreased to: ");
            Serial.println(msSpeed);
        }
    }
}

// Motor control constants
const int max_speed_left = 2000;  // Max speed for Motor 1
const int max_speed_right = 500;  // Max speed for Motor 2
const int max_speed_y = 500;      // Max speed for Motor 3
const int max_speed_zoom = 800;   // Max speed for Motor 4 (zoom)

const float acceleration_rate_left = 0.3;
const float deceleration_rate_left = 0.3;
const float acceleration_rate_right = 0.1;
const float deceleration_rate_right = 0.1;
const float acceleration_rate_y = 0.1;
const float deceleration_rate_y = 0.1;
const float acceleration_rate_zoom = 0.2; // For Motor 4
const float deceleration_rate_zoom = 0.2; // For Motor 4

const int deadzone = 10; // Deadzone threshold

float current_speed_left = 0;
float current_speed_right = 0;
float current_speed_y = 0;
float current_speed_zoom = 0; // For Motor 4

int applyDeadzoneAndRemap(float input, int minInput, int maxInput, int minOutput, int maxOutput) {
    if (abs(input) < deadzone) return 0;
    int adjustedInput = (input > 0) ? input - deadzone : input + deadzone;
    int adjustedMaxInput = maxInput - deadzone;
    return map(adjustedInput, -adjustedMaxInput, adjustedMaxInput, minOutput, maxOutput);
}

void controlStepperMotors() {
    // Apply deadzone and remap joystick values
    int target_speed_left = applyDeadzoneAndRemap(x_axis_value_left * 100, -100, 100, -max_speed_left, max_speed_left);
    int target_speed_right = applyDeadzoneAndRemap(x_axis_value_right * 100, -100, 100, -max_speed_right, max_speed_right);
    int target_speed_y = applyDeadzoneAndRemap(y_axis_value * 100, -100, 100, -max_speed_y, max_speed_y);
    int target_speed_zoom = applyDeadzoneAndRemap((trigger_right + trigger_left) * 100, -100, 100, -max_speed_zoom, max_speed_zoom);

    // Motor 1 (Left)
    if (target_speed_left > current_speed_left) {
        current_speed_left += acceleration_rate_left;
        if (current_speed_left > target_speed_left) current_speed_left = target_speed_left;
    } else if (target_speed_left < current_speed_left) {
        current_speed_left -= deceleration_rate_left;
        if (current_speed_left < target_speed_left) current_speed_left = target_speed_left;
    }

    // Motor 2 (Right)
    if (target_speed_right > current_speed_right) {
        current_speed_right += acceleration_rate_right;
        if (current_speed_right > target_speed_right) current_speed_right = target_speed_right;
    } else if (target_speed_right < current_speed_right) {
        current_speed_right -= deceleration_rate_right;
        if (current_speed_right < target_speed_right) current_speed_right = target_speed_right;
    }

    // Motor 3 (Tilt)
    if (target_speed_y > current_speed_y) {
        current_speed_y += acceleration_rate_y;
        if (current_speed_y > target_speed_y) current_speed_y = target_speed_y;
    } else if (target_speed_y < current_speed_y) {
        current_speed_y -= deceleration_rate_y;
        if (current_speed_y < target_speed_y) current_speed_y = target_speed_y;
    }

    // Motor 4 (Zoom)
    if (target_speed_zoom > current_speed_zoom) {
        current_speed_zoom += acceleration_rate_zoom;
        if (current_speed_zoom > target_speed_zoom) current_speed_zoom = target_speed_zoom;
    } else if (target_speed_zoom < current_speed_zoom) {
        current_speed_zoom -= deceleration_rate_zoom;
        if (current_speed_zoom < target_speed_zoom) current_speed_zoom = target_speed_zoom;
    }

    // Set motor speeds
    stepper1.setSpeed(current_speed_left);
    stepper2.setSpeed(current_speed_right);
    stepper3.setSpeed(current_speed_y);
    stepper4.setSpeed(current_speed_zoom);

    // Run motors
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    stepper4.runSpeed();
}

void sendCurrentMotorPositions() {
    String message = "CURRENT_POS:";
    message += String(stepper1.currentPosition()) + ",";
    message += String(stepper2.currentPosition()) + ",";
    message += String(stepper3.currentPosition());
    sendUDPMessage(message.c_str());
}

void sendPresetPositions() {
    String message = "PRESET_POS_A:";
    message += String(presetAPositions[0]) + ",";
    message += String(presetAPositions[1]) + ",";
    message += String(presetAPositions[2]) + ",";
    message += "PRESET_POS_B:";
    message += String(presetBPositions[0]) + ",";
    message += String(presetBPositions[1]) + ",";
    message += String(presetBPositions[2]);
    sendUDPMessage(message.c_str());
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
    sendPresetPositions();
}

void savePresetB() {
    presetBPositions[0] = stepper1.currentPosition();
    presetBPositions[1] = stepper2.currentPosition();
    presetBPositions[2] = stepper3.currentPosition();
    sendPresetPositions();
}

const int tolerance = 1000;

void recallPresetA() {
    if (abs(stepper1.currentPosition() - presetAPositions[0]) <= tolerance &&
        abs(stepper2.currentPosition() - presetAPositions[1]) <= tolerance &&
        abs(stepper3.currentPosition() - presetAPositions[2]) <= tolerance) {
        sendUDPMessage("PRESET_A_DONE");
        return;
    }
    long targetPositions[] = { presetAPositions[0], presetAPositions[1], presetAPositions[2] };
    stepper1.moveTo(targetPositions[0]);
    stepper2.moveTo(targetPositions[1]);
    stepper3.moveTo(targetPositions[2]);
    lcd.setCursor(0, 0);
    lcd.print("PresetADone");
    syncMove(msSpeed, 400);
    sendUDPMessage("PRESET_A_DONE");
}

void recallPresetB() {
    if (abs(stepper1.currentPosition() - presetBPositions[0]) <= tolerance &&
        abs(stepper2.currentPosition() - presetBPositions[1]) <= tolerance &&
        abs(stepper3.currentPosition() - presetBPositions[2]) <= tolerance) {
        sendUDPMessage("PRESET_B_DONE");
        return;
    }
    long targetPositions[] = { presetBPositions[0], presetBPositions[1], presetBPositions[2] };
    stepper1.moveTo(targetPositions[0]);
    stepper2.moveTo(targetPositions[1]);
    stepper3.moveTo(targetPositions[2]);
    syncMove(msSpeed, 400);
    sendUDPMessage("PRESET_B_DONE");
}

void sendUDPMessage(const char *message) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(message);
    Udp.endPacket();
}

void syncMove(float maxSpeed, float accel) {
    lcd.setCursor(0, 0);
    lcd.print("SyncMove_____");
    lcd.setCursor(0, 1);
    lcd.print("Getting Distance");
    float allSpeeds[3];
    allSpeeds[0] = fabs(stepper1.distanceToGo());
    allSpeeds[1] = fabs(stepper2.distanceToGo());
    allSpeeds[2] = fabs(stepper3.distanceToGo());
    lcd.setCursor(0, 0);
    lcd.print("SyncMove_____");
    lcd.setCursor(0, 1);
    lcd.print("Finding Longest");
    float biggestDistance = findBiggestNumFloatArray(allSpeeds, "max");
    lcd.setCursor(0, 0);
    lcd.print("SyncMove_____");
    lcd.setCursor(0, 1);
    lcd.print("Setting Speeds");
    stepper1.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper1));
    stepper1.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper1) / 2);
    stepper2.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper2));
    stepper2.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper2) / 2);
    stepper3.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper3));
    stepper3.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper3) / 2);
    lcd.setCursor(0, 0);
    lcd.print("SyncMove_____");
    lcd.setCursor(0, 1);
    lcd.print("Moving Steppers");
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        if (stepper1.distanceToGo() != 0) stepper1.run();
        if (stepper2.distanceToGo() != 0) stepper2.run();
        if (stepper3.distanceToGo() != 0) stepper3.run();
    }
    lcd.setCursor(0, 0);
    lcd.print("SyncMove_____");
    lcd.setCursor(0, 1);
    lcd.print("ResetSpeedAccel");
    stepper1.setMaxSpeed(msSpeed);
    stepper2.setMaxSpeed(msSpeed);
    stepper3.setMaxSpeed(msSpeed);
    stepper1.setAcceleration(400);
    stepper2.setAcceleration(400);
    stepper3.setAcceleration(400);
    lcd.setCursor(0, 0);
    lcd.print("SyncMove_____");
    lcd.setCursor(0, 1);
    lcd.print("Done_____");
}

float calcSync(float maxSpeed, float maxAccel, float maxDistance, AccelStepper stepper) {
    lcd.setCursor(0, 0);
    lcd.print("calcSync_____");
    lcd.setCursor(0, 1);
    lcd.print("CalculatingSPeed");
    if (stepper.distanceToGo() == 0) return 0;
    float stepperDis = stepper.distanceToGo();
    float stepperSpeed = fabs(maxSpeed) * (fabs(stepperDis) / fabs(maxDistance));
    return (stepperSpeed < 0) ? fabs(stepperSpeed) : stepperSpeed;
}

float findBiggestNumFloatArray(float myArray[], String option) {
    float maxVal = myArray[0];
    for (int i = 0; i < 3; i++)
        maxVal = max(fabs(myArray[i]), fabs(maxVal));
    return maxVal;
}

void homeZoom() {
    // Placeholder until TMC2209 is re-enabled
    Serial.println("Homing Zoom...");
    zoomStepper.setSpeed(homingSpeed * homingDirection);
    delay(1000); // Simulate homing for now
    zoomStepper.stop();
    zoomStepper.setCurrentPosition(0);
    Serial.println("Zoom homed.");
}
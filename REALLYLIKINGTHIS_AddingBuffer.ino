#include <AccelStepper.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <MultiStepper.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// Ethernet settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x03, 0x30 };
unsigned int localPort = 44158;
EthernetUDP Udp;
char packetBuffer[255];

// Define OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Stepper motor pins
const int stepPin1 = 3, dirPin1 = 2, ms1Pin1 = 10, ms2Pin1 = 11;
const int stepPin2 = 31, dirPin2 = 30, ms1Pin2 = 32, ms2Pin2 = 33;
const int stepPin3 = 45, dirPin3 = 44, ms1Pin3 = 40, ms2Pin3 = 41;
const int stepPin4 = 48, dirPin4 = 49, ms1Pin4 = 37, ms2Pin4 = 35, tmcEnPin4 = 50;

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4);
MultiStepper multiStepper;

// Joystick input variables (buffered)
float x_axis_value_left = 0.0, x_axis_value_right = 0.0, y_axis_value = 0.0, trigger_left = 0.0, trigger_right = 0.0;
float msSpeed = 800, slideSpeed = 20;
int saved_position1 = 0, saved_position2 = 0, saved_position3 = 0;
bool loopPresets = false;
long presetAPositions[] = {0, 0, 0}, presetBPositions[] = {0, 0, 0};

// Motor control constants
const int max_speed_left = 600;
const int max_speed_right = 100;
const int max_speed_y = 100;
const int max_speed_zoom = 75;

const float acceleration_rate_left = 10;  // Increased for faster ramp-up
const float deceleration_rate_left = 10;  // Matching deceleration
const float acceleration_rate_right = 0.6;
const float deceleration_rate_right = 0.6;
const float acceleration_rate_y = 0.6;
const float deceleration_rate_y = 0.6;
const float acceleration_rate_zoom = 0.75;
const float deceleration_rate_zoom = 0.75;

const int deadzone = 10;

float current_speed_left = 0;
float current_speed_right = 0;
float current_speed_y = 0;
float current_speed_zoom = 0;

void setup() {
    pinMode(ms1Pin1, OUTPUT); pinMode(ms2Pin1, OUTPUT);
    pinMode(ms1Pin2, OUTPUT); pinMode(ms2Pin2, OUTPUT);
    pinMode(ms1Pin3, OUTPUT); pinMode(ms2Pin3, OUTPUT);
    pinMode(ms1Pin4, OUTPUT); pinMode(ms2Pin4, OUTPUT);
    pinMode(tmcEnPin4, OUTPUT);
    digitalWrite(ms1Pin1, HIGH); digitalWrite(ms2Pin1, HIGH);
    digitalWrite(ms2Pin2, HIGH); digitalWrite(ms2Pin2, HIGH);
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

    Ethernet.begin(mac);
    Udp.begin(localPort);

    Wire.begin();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("IP:");
    display.println(Ethernet.localIP());
    display.display();
}

void loop() {
    handleUDPRequests();
    controlStepperMotors();
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    stepper4.runSpeed();
    delayMicroseconds(3000);  // 1kHz loop rate
}

void handleUDPRequests() {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        int len = Udp.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;
        String request = String(packetBuffer);

        if (request.startsWith("SET_JOYSTICK")) {
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
        } else if (request.startsWith("SEND_CURRENT_POS")) {
            sendCurrentMotorPositions();
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
        } else if (request.startsWith("INCREASE_SPEED")) {
            msSpeed += 200;
            sendMsSpeed();
        } else if (request.startsWith("DECREASE_SPEED")) {
            msSpeed -= 200;
            sendMsSpeed();
        }
    }
}

void controlStepperMotors() {
    int target_speed_left = applyDeadzoneAndRemap(x_axis_value_left * 100, -100, 100, -max_speed_left, max_speed_left);
    int target_speed_right = applyDeadzoneAndRemap(x_axis_value_right * 100, -100, 100, -max_speed_right, max_speed_right);
    int target_speed_y = applyDeadzoneAndRemap(y_axis_value * 100, -100, 100, -max_speed_y, max_speed_y);
    int target_speed_zoom = applyDeadzoneAndRemap((trigger_right + trigger_left) * 100, -100, 100, -max_speed_zoom, max_speed_zoom);

    // Smooth acceleration for slider (Motor 1)
    float speed_diff_left = target_speed_left - current_speed_left;
    if (speed_diff_left > 0) {
        current_speed_left += min(acceleration_rate_left, speed_diff_left * 0.002 + 0.02);  // Proportional smoothing
        if (current_speed_left > target_speed_left) current_speed_left = target_speed_left;
    } else if (speed_diff_left < 0) {
        current_speed_left -= min(deceleration_rate_left, -speed_diff_left * 0.002 + 0.02);
        if (current_speed_left < target_speed_left) current_speed_left = target_speed_left;
    }

    // Other motors (unchanged)
    if (target_speed_right > current_speed_right) {
        current_speed_right += acceleration_rate_right;
        if (current_speed_right > target_speed_right) current_speed_right = target_speed_right;
    } else if (target_speed_right < current_speed_right) {
        current_speed_right -= deceleration_rate_right;
        if (current_speed_right < target_speed_right) current_speed_right = target_speed_right;
    }

    if (target_speed_y > current_speed_y) {
        current_speed_y += acceleration_rate_y;
        if (current_speed_y > target_speed_y) current_speed_y = target_speed_y;
    } else if (target_speed_y < current_speed_y) {
        current_speed_y -= deceleration_rate_y;
        if (current_speed_y < target_speed_y) current_speed_y = target_speed_y;
    }

    if (target_speed_zoom > current_speed_zoom) {
        current_speed_zoom += acceleration_rate_zoom;
        if (current_speed_zoom > target_speed_zoom) current_speed_zoom = target_speed_zoom;
    } else if (target_speed_zoom < current_speed_zoom) {
        current_speed_zoom -= deceleration_rate_zoom;
        if (current_speed_zoom < target_speed_zoom) current_speed_zoom = target_speed_zoom;
    }

    stepper1.setSpeed(current_speed_left);
    stepper2.setSpeed(current_speed_right);
    stepper3.setSpeed(current_speed_y);
    stepper4.setSpeed(current_speed_zoom);
}

int applyDeadzoneAndRemap(float input, int minInput, int maxInput, int minOutput, int maxOutput) {
    if (abs(input) < deadzone) return 0;
    int adjustedInput = (input > 0) ? input - deadzone : input + deadzone;
    int adjustedMaxInput = maxInput - deadzone;
    return map(adjustedInput, -adjustedMaxInput, adjustedMaxInput, minOutput, maxOutput);
}

void sendCurrentMotorPositions() {
    String message = "CURRENT_POS:";
    message += String(stepper1.currentPosition()) + ",";
    message += String(stepper2.currentPosition()) + ",";
    message += String(stepper3.currentPosition());
    sendUDPMessage(message.c_str());
}

void sendMsSpeed() {
    String message = "MSSPEED:" + String(msSpeed);
    sendUDPMessage(message.c_str());
}

void sendUDPMessage(const char *message) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(message);
    Udp.endPacket();
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
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Preset A Done");
    display.display();
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
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Preset B Done");
    display.display();
    syncMove(msSpeed, 400);
    sendUDPMessage("PRESET_B_DONE");
}

void syncMove(float maxSpeed, float accel) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SyncMove Running");
    display.display();

    float allSpeeds[3];
    allSpeeds[0] = fabs(stepper1.distanceToGo());
    allSpeeds[1] = fabs(stepper2.distanceToGo());
    allSpeeds[2] = fabs(stepper3.distanceToGo());
    float biggestDistance = findBiggestNumFloatArray(allSpeeds, "max");

    stepper1.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper1));
    stepper1.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper1) / 2);
    stepper2.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper2));
    stepper2.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper2) / 2);
    stepper3.setMaxSpeed(calcSync(maxSpeed, accel, biggestDistance, stepper3));
    stepper3.setAcceleration(calcSync(maxSpeed, accel, biggestDistance, stepper3) / 2);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        if (stepper1.distanceToGo() != 0) stepper1.run();
        if (stepper2.distanceToGo() != 0) stepper2.run();
        if (stepper3.distanceToGo() != 0) stepper3.run();
    }

    stepper1.setMaxSpeed(msSpeed);
    stepper2.setMaxSpeed(msSpeed);
    stepper3.setMaxSpeed(msSpeed);
    stepper1.setAcceleration(400);
    stepper2.setAcceleration(400);
    stepper3.setAcceleration(400);
}

float calcSync(float maxSpeed, float maxAccel, float maxDistance, AccelStepper stepper) {
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
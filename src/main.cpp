#include <AccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Bounce2.h>
#include <TMCStepper.h>
#include <Wire.h>
#include </home/david/platform-io/PlaywriteGBS_VariableFont_wght9pt7b.h>
#include <cmath>

#include "esp_system.h"
#include "freertos/FreeRTOS.h"

// Function declarations for multitaskingmotorTask
void motorTask(void *parameter);
void displayTask(void *parameter);
void disableMotor();
void cleanup();
void enableMotor(int);
void handleEncoder();
void displayMessage(const char *title, const char *subtitle);
void runMotorTask();
void stopMotor();
void setupMotor(int microstepsSetting, int maxSpeedSetting, int distance);
void processMotorPosition();
void displayProgressBar(int progress);
int calculateMotorTime(int steps_per_revolution, int speed, int distance, int acceleration);

// Pin definitions for ESP32 and TMC2209
#define stepPin 27           // Step pin
#define dirPin 26            // Direction pin
#define enablePin 25         // Enable pin
#define R_SENSE 0.11f        // Sense resistor value
#define SERIAL_PORT Serial2  // UART communication

// Pin definitions for the rotary encoder
#define encoderAPin 34  // Change to your actual pin for A
#define encoderBPin 35  // Change to your actual pin for B
#define buttonPin 32    // Encoder Button pin

#define DRIVER_ADDRESS 0b00  // For standalone configuration
const long SERIAL_BAUD_RATE = 115200;

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

// Create AccelStepper object
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

//HardwareSerial SERIAL_PORT(2);

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// hello 123
// Global variables
static bool motorEnabled = false;
int countdownValue = 0;        // Global countdown value
bool countdownActive = false;  // Flag to indicate if countdown is active
int encoderValue = 0;
int selectedOption = 0;
unsigned long lastMotorUpdateTime = 0;
int animationStep = 0;
bool repeat = false;
int totalTimeProg = 0;

int targetPosition = 2000;      // Forward target position
int repeatCount = 5;            // Number of times to repeat forward and backward motion
int currentIteration = 0;       // Track how many times the motion has repeated
bool motorTaskUpdated = false;  // Flag to indicate if the motor task needs to update

int *positions = nullptr;  // Array of target positions
int currentPosIndex = 0;   // Track the current position index
int numPositions = 0;      // Number of positions

bool eStop = false;

int maxSpeed;
int microsteps;

int hello;




Bounce debouncedButton = Bounce();
Bounce debouncedEncoderA = Bounce();
Bounce debouncedEncoderB = Bounce();

void setup() {
    Serial.begin(115200);
    SERIAL_PORT.begin(115200, SERIAL_8N1, 16, 17);
    //TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

    // Check communication with TMC2209 by reading a register
    uint32_t drv_status = driver.DRV_STATUS();

    // Print the raw status value
    Serial.print("DRV_STATUS: ");
    Serial.println(drv_status, HEX);  // Print in hexadecimal for clarity

    uint8_t microsteps = driver.microsteps();
    Serial.print("Microstepping set to: ");
    Serial.println(microsteps);  // Should print 16

    // Check if the communication response is valid
    if (drv_status == 0xFFFFFFFF) {
        Serial.println("Communication failed. No valid response from TMC2209.");
    } else {
        Serial.println("Communication successful!");
    }

    //Serial.println("Serial connection established!");

    // Set button pin as input
    pinMode(buttonPin, INPUT_PULLUP);  // Button connected to ground and pin 2 with internal pull-up enabled
    pinMode(encoderAPin, INPUT_PULLUP);
    pinMode(encoderBPin, INPUT_PULLUP);

    // Attach the debounced objects to the encoder pins
    debouncedButton.attach(buttonPin);
    debouncedButton.interval(25);  // Debounce time for the button

    debouncedEncoderA.attach(encoderAPin);
    debouncedEncoderA.interval(5);  // Debounce time in milliseconds

    debouncedEncoderB.attach(encoderBPin);
    debouncedEncoderB.interval(5);

    delay(1000);

    driver.begin();
    driver.toff(5);
    driver.rms_current(1700);
    driver.microsteps(2);
    driver.en_spreadCycle(true);  // Disable SpreadCycle to use StealthChop
    driver.pwm_autoscale(false);  // Autoscale PWM for StealthChop

    // Initialize stepper settings
    stepper.setMaxSpeed(1000);      // Lower maximum speed for testing
    stepper.setAcceleration(2000);  // Set acceleration (steps/s²)

    // Enable the driver
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);  // Disable the motor

    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    display.clearDisplay();
    display.setFont(&PlaywriteGBS_VariableFont_wght9pt7b);  // Set the desired font
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(35, 15);
    display.print("Saving");
    display.setCursor(25, 35);
    display.print("Time");

    display.setFont();  // Set the desired font
    display.setCursor(55, 50);
    display.print("Cleaner V1.0");
    display.display();
    // Start the motor control task on Core 0
    xTaskCreatePinnedToCore(
        motorTask,    // Task function
        "MotorTask",  // Task name
        10000,        // Stack size
        NULL,         // Task parameter
        2,            // Priority
        NULL,         // Task handle
        0             // Core ID (0 for Core 0)
    );

    // Start the display control task on Core 1
    xTaskCreatePinnedToCore(
        displayTask,    // Task function
        "DisplayTask",  // Task name
        10000,          // Stack size
        NULL,           // Task parameter
        1,              // Priority
        NULL,           // Task handle
        1               // Core ID (1 for Core 1)
    );
}

void motorTask(void *parameter) {
    // FreeRTOS Task loop for motor control
    disableCore0WDT();
    for (;;) {
        for (;;) {
            if (stepper.distanceToGo() == 0) {
                if (motorEnabled) {
                    processMotorPosition();
                }
            } else {
                runMotorTask();
            }
        }
    }
}

void displayTask(void *parameter) {
    for (;;) {
        if (motorEnabled && !eStop) {
            if (countdownActive) {
                display.clearDisplay();

                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(10, 0);

                const char *menuOptions[] = {"Cleaning Cycle", "Quick Dry", "Quick Clean", "Drying Cycle"};
                display.print(menuOptions[selectedOption]);

                // Display RPM

                // Get the current speed in steps per second
                float currentSpeed = stepper.speed();

                // Calculate RPM
                uint8_t microsteps = driver.microsteps();
                int stepsNow = microsteps * 200;
                float rpm = (fabs(currentSpeed) * 60.0) / stepsNow;

                display.setCursor(0, 20);
                display.print("RPM: ");
                display.print(rpm);
                display.setCursor(0, 30);
                display.print("Microsteps: ");
                display.print(stepsNow);
                display.setCursor(0, 40);
                display.print("Power: ");
                display.print(driver.cs_actual());
                Serial.print(driver.cs2rms(driver.cs_actual()),DEC);
                Serial.println(" MA MOTOR CURRENT"); 

                // Spinning circle animation
                int radius = 10;
                int centerX = 110;
                int centerY = 30;
                display.fillRect(centerX - radius, centerY - radius, radius * 2, radius * 2, SSD1306_BLACK);
                int angle = (animationStep % 8) * 45;
                int x = centerX + radius * cos(angle * PI / 180);
                int y = centerY + radius * sin(angle * PI / 180);
                display.drawCircle(x, y, 3, SSD1306_WHITE);
                animationStep++;
                displayProgressBar(animationStep);
                display.display();
            }
            vTaskDelay(50 / portTICK_PERIOD_MS);
        } else if (eStop) {
          display.clearDisplay();

                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(0, 10);
                display.print("Gental stop if i'm");
                display.setCursor(0, 20);
                display.print("exploding try");
                display.setCursor(0, 30);
                display.print("pulling out the plug!");
              
                display.display();


        } else { 
            animationStep = 0;
            handleEncoder();
        }
        taskYIELD();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void displayProgressBar(int progress) {
    // Define progress bar position and size
    int barX = 10;
    int barY = 54;
    int barWidth = 100;
    int barHeight = 10;

    // Draw the border of the progress bar
    display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);

    int totalTimeLeft = (totalTimeProg * 4) * 10;
    Serial.println(totalTimeLeft);

    // Calculate the fill width based on the progress percentage
    int fillWidth = (progress * barWidth) / totalTimeLeft;

    // Draw the filled part of the progress bar
    display.fillRect(barX + 1, barY + 1, fillWidth - 2, barHeight - 2, SSD1306_WHITE);
}

void processMotorPosition() {
    // Move to the next position if more are available
    if (currentPosIndex < numPositions) {
        if (eStop || currentPosIndex >= numPositions) {
            stepper.moveTo(stepper.currentPosition());
            stopMotor();
        } else {
            stepper.moveTo(positions[currentPosIndex]);
            currentPosIndex++;
        }
    } else {
        stopMotor();
    }
}

void stopMotor() {
    vTaskDelay(20 / portTICK_PERIOD_MS);
eStop = false;
    disableMotor();
    motorEnabled = false; 
    currentPosIndex = 0;
    cleanup();
}

void runMotorTask() {
    if (!motorEnabled) {
        enableMotor(23);
        motorEnabled = true;
    }
    debouncedButton.update();

    stepper.run();

    if (debouncedButton.fell()) {
        eStop = true;
    }
}

void handleEncoder() {
    //  Serial.println("Motor disabled, handling encoder");
    debouncedEncoderA.update();
    debouncedEncoderB.update();
    debouncedButton.update();
    // Check if the encoder has changed

    //uint8_t microsteps = 0;
    //int maxSpeed = 0;

    if (debouncedButton.fell()) {
        switch (selectedOption) {
            case 0:  // Cleaning Cycle full 3 mins
                setupMotor(8, 8000, 188000);
                break;

            case 1:  // Quick Dry 30 seconds
                setupMotor(2, 7500, 88000);
                break;

            case 2:  // Quick Clean
                setupMotor(2, 4000, 68000);
                break;

            case 3:  // Drying cycle
                setupMotor(2, 4000, 8000);
                break;

            default:
                // Handle unexpected selectedOption value
                break;
        }
    }

    if (debouncedEncoderA.changed() && debouncedEncoderA.read() == LOW) {
        if (debouncedEncoderB.read() == HIGH) {
            encoderValue++;  // Clockwise rotation
                             //Serial.println("Clockwise rotation");
        } else {
            encoderValue--;  // Counterclockwise rotation
                             //Serial.println("Counterclockwise rotation");
        }
        // Update menu option based on encoder value

        // Debug print for encoder value
        //Serial.print("Encoder Value: ");
        //Serial.println(encoderValue);

        // Update menu option based on encoder value
        selectedOption = encoderValue % 4;  // Ensure the range is between 0 and 3
        if (selectedOption < 0) {
            selectedOption += 4;  // Correct for negative values
        }
        Serial.println(selectedOption);
        // Clear the display
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        // Menu options
        const char *menuOptions[] = {"Cleaning Cycle", "Quick Dry", "Quick Clean", "Drying Cycle"};

        // Display all menu options and highlight the selected one
        for (int i = 0; i < 4; i++) {
            if (i == selectedOption) {
                display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  // Highlight the selected option
                display.setCursor(10, i * 13);                       // Adjust cursor position for each option
                display.print("* ");                                 // Print asterisk for the selected option
            } else {
                display.setTextColor(SSD1306_WHITE);  // Normal text for other options
                display.setCursor(10, i * 13);        // Adjust cursor position for each option
            }

            display.print(menuOptions[i]);  // Print the menu option
        }

        // Display everything on the screen
        display.display();
    }
}

void setupMotor(int microstepsSetting, int maxSpeedSetting, int distance) {
    driver.microsteps(microstepsSetting);
    stepper.setMaxSpeed(maxSpeedSetting);
    stepper.setCurrentPosition(0);
    totalTimeProg = 0;

    // Define positions array for each movement
    numPositions = 3;
    positions = new int[numPositions];  // Ensure old memory is freed if dynamic
    positions[0] = 0;
    positions[1] = distance;
    positions[2] = 0;

    maxSpeed = stepper.maxSpeed();
    microsteps = driver.microsteps() * 200;

    totalTimeProg = calculateMotorTime(microstepsSetting, maxSpeedSetting, distance, 2000);

    stepper.moveTo(distance);
    Serial.println(motorEnabled);
}

void enableMotor(int countdownTime) {
    digitalWrite(enablePin, LOW);  // Enable the motor
    Serial.println("Motor enabled");

    // Set countdown and activate the countdown flag
    countdownValue = countdownTime;
    countdownActive = true;
}

void disableMotor() {
    delay(100);
    countdownActive = false;
    digitalWrite(enablePin, HIGH);  // Disable the motor
    //Serial.println("Motor disabled");

    switch (selectedOption) {
        case 0:  // Cleaning Cycle full 3 mins
            displayMessage("Full Clean", "Finished");
            break;

        case 1:  // Quick Dry 30 seconds
            displayMessage("Quick Dry", "Finished");
            break;

        case 2:  // Quick Clean
            displayMessage("Quick Clean", "Finished");
            break;

        case 3:  // Drying cycle
            displayMessage("Full Drying", "Finished");
            break;

        default:
            // Handle unexpected menuOption value
            break;
    }
}

void displayMessage(const char *title, const char *subtitle) {
    display.clearDisplay();
    delay(50);
    display.setFont(&PlaywriteGBS_VariableFont_wght9pt7b);  // Set the desired font
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(15, 15);
    display.print(title);
    display.setCursor(28, 35);
    display.print(subtitle);
    display.setFont();  // Reset to default font
    display.setCursor(55, 50);
    display.print("Cleaner V1.0");
    display.display();
}

int calculateMotorTime(int steps_per_revolution, int speed, int distance, int acceleration) {
    // Step 1: Calculate time to reach maximum speed
    float t_acc = static_cast<float>(speed) / acceleration;  // Time to reach max speed

    // Step 2: Calculate distance covered during acceleration
    float d_acc = 0.5f * acceleration * (t_acc * t_acc);

    // Step 3: Check if the distance can be completed within the acceleration phase
    float t_total;  // Total time
    if (distance <= d_acc) {
        // If the total distance is less than the distance covered during acceleration
        t_total = sqrt(2.0f * distance / acceleration);
    } else {
        // If the distance exceeds the distance covered during acceleration
        // Calculate the constant speed distance
        float d_const = distance - 2 * d_acc;
        float t_const = d_const / speed;
        // Total time
        t_total = t_acc + t_const + t_acc;
    }

    // Return the total time as an integer (in seconds)
    return static_cast<int>(t_total);
}

void cleanup() {
    delete[] positions;   // Free allocated memory
    positions = nullptr;  // Prevent dangling pointer
}

void loop() {
}



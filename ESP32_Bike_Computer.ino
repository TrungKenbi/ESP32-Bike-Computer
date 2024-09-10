#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define HALL_EFFECT_PIN 13
#define WAKE_THRESHOLD 10000 // 10 seconds of inactivity
#define MAX_SENSOR_ERRORS 5
#define MAX_DISPLAY_ERRORS 3

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Global variables
volatile float speed = 0.0;
volatile float distance = 0.0;
volatile unsigned long rideTime = 0;
volatile unsigned long lastWakeTime = 0;
volatile int sensorErrorCount = 0;
volatile int displayErrorCount = 0;

void goToSleep() {
  display.clearDisplay();
  display.display();
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  
  esp_sleep_enable_ext0_wakeup((gpio_num_t)HALL_EFFECT_PIN, 0);
  esp_deep_sleep_start();
}

void checkSleep() {
  if (millis() - lastWakeTime > WAKE_THRESHOLD) {
    goToSleep();
  }
}

bool checkSensor() {
  if (digitalRead(HALL_EFFECT_PIN) == HIGH) {
    sensorErrorCount++;
    if (sensorErrorCount >= MAX_SENSOR_ERRORS) {
      Serial.println("Hall effect sensor error detected");
      return false;
    }
  } else {
    sensorErrorCount = 0;
  }
  return true;
}

bool updateDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    displayErrorCount++;
    if (displayErrorCount >= MAX_DISPLAY_ERRORS) {
      Serial.println("Display error detected");
      return false;
    }
  } else {
    displayErrorCount = 0;
  }
  return true;
}

// Task for measuring speed and distance
void speedTask(void *pvParameters) {
  const float wheelDiameter = 0.686; // 622mm rim + 2 * 32mm tire
  const float wheelCircumference = PI * wheelDiameter; // Wheel circumference (m)
  
  unsigned long lastTime = 0;
  unsigned long debounceDelay = 10;

  rtc_gpio_pullup_en((gpio_num_t)HALL_EFFECT_PIN);
  rtc_gpio_hold_en((gpio_num_t)HALL_EFFECT_PIN);

  while (1) {
    if (checkSensor() && digitalRead(HALL_EFFECT_PIN) == LOW) {
      unsigned long currentTime = millis();
      lastWakeTime = currentTime; // Reset sleep timer
      if (currentTime - lastTime > debounceDelay) {
        float timeDiff = (currentTime - lastTime) / 1000.0; // Convert to seconds
        speed = wheelCircumference / timeDiff * 3.6; // km/h
        distance += wheelCircumference / 1000.0; // km
        lastTime = currentTime;
      }
    }
    checkSleep();
    vTaskDelay(1); // Yield CPU to other tasks
  }
}

// Task for display update
void displayTask(void *pvParameters) {
  while (1) {
    if (updateDisplay()) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      
      display.setCursor(0, 0);
      display.print("Speed: ");
      display.print(speed, 1);
      display.print(" km/h");

      display.setCursor(0, 16);
      display.print("Distance: ");
      display.print(distance, 2);
      display.print(" km");

      display.setCursor(0, 32);
      display.print("Time: ");
      display.print(rideTime / 3600); // Hours
      display.print(":");
      display.print((rideTime % 3600) / 60); // Minutes
      display.print(":");
      display.print(rideTime % 60); // Seconds

      display.display();
      rideTime++;
    } else {
      // Handle display error (e.g., try to reinitialize or notify user)
      Serial.println("Attempting to reinitialize display...");
      display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    }
    checkSleep();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
  }
}

void setup() {
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    // In a production environment, you might want to implement a fallback or error notification here
  }

  xTaskCreate(speedTask, "SpeedTask", 2048, NULL, 1, NULL);
  xTaskCreate(displayTask, "DisplayTask", 2048, NULL, 1, NULL);
}

void loop() {
  // Main loop is empty because we're using FreeRTOS tasks
}

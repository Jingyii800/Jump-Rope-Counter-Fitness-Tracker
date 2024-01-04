#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display width, height, and address
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

// Set the pins for the I2C interface
#define MPU6050_SDA 4
#define MPU6050_SCL 5
#define BME280_SDA 4
#define BME280_SCL 5

// Digital pin for the vibration actuator
#define VIBRATION_PIN 1

// Jump detection parameters
const float JUMP_THRESHOLD = 1.5; // Acceleration threshold for jump detection
const unsigned long DEBOUNCE_TIME = 500; // Debounce time in milliseconds
unsigned long lastJumpTime = 0;
int jumpCounter = 0;
const int JUMP_THRESHOLD_COUNT = 5; // Number of jumps to trigger the actuator

// MPU6050 calibration offsets
float accelXOffset = 0.0, accelYOffset = 0.0, accelZOffset = 0.0;

// Environmental data update rate and filtering
const long ENV_UPDATE_INTERVAL = 2000; // 2 seconds
unsigned long lastEnvUpdateTime = 0;
const int MOVING_AVERAGE_SIZE = 5;
// moving average filtering
float tempReadings[MOVING_AVERAGE_SIZE] = {0};
float humidityReadings[MOVING_AVERAGE_SIZE] = {0};
int readingIndex = 0;

void calibrateMPU6050();
void displayJumpCount(int jumpCount);
void triggerVibration();
void updateReadingsArray(float newTemperature, float newHumidity);
float calculateMovingAverage(float readings[], int size);
void updateEnvironmentalData(float temp, float hum);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  calibrateMPU6050();

  // Initialize BME280
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1) {
      delay(10);
    }
  }

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  // rotate the screen
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("Hello! Jingyi"));
  display.setCursor(10, 10);
  display.print(F("Goal: "));
  display.print(JUMP_THRESHOLD_COUNT);
  // Display static "Jumps:" text
  display.setCursor(10, 20); 
  display.print(F("Jumps: 0")); 
  display.setCursor(10, 30);
  display.print(F("Temp: "));
  display.setCursor(10, 40);
  display.print(F("Hum: "));
  display.display();

  // Initialize vibration actuator
  pinMode(VIBRATION_PIN, OUTPUT);
}

bool jumpDetected = false;

void loop() {
  unsigned long currentMillis = millis();

  // Read and process MPU6050 data for jump detection
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply calibration offsets to the acceleration values
  float calibratedZ = a.acceleration.z - accelZOffset;

  // Detect jump: look for a high acceleration followed by a low one
  if (currentMillis - lastJumpTime > DEBOUNCE_TIME) {
    if (!jumpDetected && calibratedZ > JUMP_THRESHOLD) {
      jumpDetected = true;
    } else if (jumpDetected && calibratedZ < -JUMP_THRESHOLD) { // Note the direction change
      jumpCounter++;
      jumpDetected = false;
      lastJumpTime = currentMillis;

      // Update jump count on OLED display
      Serial.print("Jump detected! Total jumps: ");
      Serial.println(jumpCounter);
      displayJumpCount(jumpCounter);

      // Actuate when jump count threshold is reached
      if (jumpCounter >= JUMP_THRESHOLD_COUNT) {
        triggerVibration();
        jumpCounter = 0; // Reset counter after reaching the goal
        displayJumpCount(jumpCounter);
      }
    }
  }

  // Update environmental data in every 2 seconds
  if (currentMillis - lastEnvUpdateTime >= ENV_UPDATE_INTERVAL) {
    lastEnvUpdateTime = currentMillis;
    updateReadingsArray(bme.readTemperature(), bme.readHumidity());
    float avgTemp = calculateMovingAverage(tempReadings, MOVING_AVERAGE_SIZE);
    float avgHumidity = calculateMovingAverage(humidityReadings, MOVING_AVERAGE_SIZE);
    updateEnvironmentalData(avgTemp, avgHumidity);

    // Print environmental data to Serial
    Serial.print("Temperature: ");
    Serial.print(avgTemp, 1);
    Serial.print(" C, Humidity: ");
    Serial.print(avgHumidity, 1);
    Serial.println(" %");

  }
}

void calibrateMPU6050() {
    const int numReadings = 100;
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < numReadings; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumX += a.acceleration.x;
        sumY += a.acceleration.y;
        sumZ += a.acceleration.z;
        delay(10);
    }

    accelXOffset = sumX / numReadings;
    accelYOffset = sumY / numReadings;
    accelZOffset = sumZ / numReadings - 9.81; // Assuming GRAVITY is defined as 9.81 or 1g
}

void updateReadingsArray(float newTemperature, float newHumidity) {
  tempReadings[readingIndex] = newTemperature;
  humidityReadings[readingIndex] = newHumidity;
  
  // Increment the index and wrap around if necessary
  readingIndex = (readingIndex + 1) % MOVING_AVERAGE_SIZE;
}

// Function to calculate the moving average of the values in an array
float calculateMovingAverage(float readings[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += readings[i];
  }
  return sum / size;
}

void triggerVibration() {
  digitalWrite(VIBRATION_PIN, HIGH);
  delay(4000); // Vibrate for 4 second
  digitalWrite(VIBRATION_PIN, LOW);
}

void displayJumpCount(int jumpCount) {
  display.fillRect(0, 20, SCREEN_WIDTH, 10, SSD1306_BLACK); // Clear the jump count area
  display.setCursor(10, 20); // Reset cursor to the line with "Jumps: "
  display.print(F("Jumps: "));
  display.print(jumpCount); // Display the current jump count
  display.display(); // Update the display
}

void updateEnvironmentalData(float temp, float hum) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Clear the previous temperature and humidity by drawing a black rectangle
  display.fillRect(0, 30, SCREEN_WIDTH, 18, SSD1306_BLACK);

  // Set the cursor to the start of the third line for temperature
  display.setCursor(10, 30);
  display.print("Temp: ");
  display.print(temp, 1);
  display.print(" C");

  // Set the cursor to the start of the fourth line for humidity
  display.setCursor(10, 40);
  display.print("Hum: ");
  display.print(hum, 1);
  display.print("%");

  // Update the display
  display.display();
}
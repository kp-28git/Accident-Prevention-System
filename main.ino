#include <Arduino.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <SoftwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Instantiate GPS, WiFi, and ThingSpeak objects
TinyGPSPlus gps;
HardwareSerial SerialGPS(1); // Using Serial1 for ESP32
SoftwareSerial SIM808_SERIAL(22, 23); // RX pin: D22, TX pin: D23

// WiFi credentials and ThingSpeak API key
const char* ssid = "Omen 15"; 
const char* password = "kpatel.."; 
const char* thingSpeakApiKey = "8RY0JRJEMKMP2FWL";
const int channel_ID = 2739260;

float xAccel, yAccel, zAccel; // Variables for accelerometer values
int alcoholValue; // Variable for alcohol sensor reading
WiFiClient client; // Client to communicate with ThingSpeak

// Pin definitions
#define LED 2
#define ALCOHOL_SENSOR_PIN 35
#define X_AXIS_PIN 32
#define Y_AXIS_PIN 33
#define Z_AXIS_PIN 34

// Threshold values for alcohol sensor and accelerometer readings
#define ALCOHOL_THRESHOLD 1020
#define ACCIDENT_THRESHOLD_X 1.5
#define ACCIDENT_THRESHOLD_Y 1.5
#define ACCIDENT_THRESHOLD_Z 1.5
#define ACCIDENT_THRESHOLD_XN -1.5
#define ACCIDENT_THRESHOLD_YN -1.5
#define ACCIDENT_THRESHOLD_ZN -1.5

// AT commands for SIM808 to send SMS
String AT = "AT\r\n";
String AT_CMGF = "AT+CMGF=1\r\n"; 
String AT_CMGS = "AT+CMGS=\"7284890760\"\r\n"; 
String SMS_TEXT = "Accident Detected at here - ";

// Timing and flags
unsigned long previousMillis = 0; 
const long interval = 10000; // Set interval for tasks like sending data
volatile bool accidentDetected = false; // Flag to indicate accident detection

// FreeRTOS Queue handles for communication between tasks
QueueHandle_t gpsQueue;
QueueHandle_t accelQueue;
QueueHandle_t alcoholQueue;

// Structure to hold GPS data
struct GpsData {
  float latitude;
  float longitude;
};

// Structure to hold accelerometer data
struct AccelData {
  float xAccel;
  float yAccel;
  float zAccel;
};

// Function to send AT command to SIM808 module
void sendATCommand(String cmd) {
  SIM808_SERIAL.println(cmd); // Send the command
  delay(500); // Wait for response
  while (SIM808_SERIAL.available()) {
    Serial.write(SIM808_SERIAL.read()); // Print response to Serial monitor
  }
}

// Function to send SMS with latitude, longitude, and Google Maps link
void sendSMS(String text, float lat, float lon) {
  String message = text + " Latitude: " + String(lat, 6) + " Longitude: " + String(lon, 6);
  message += " Google Maps link: https://www.google.com/maps?q=" + String(lat, 6) + "," + String(lon, 6);
  sendATCommand(AT_CMGF); // Set SMS mode
  delay(500);
  sendATCommand(AT_CMGS); // Set recipient phone number
  delay(500);
  SIM808_SERIAL.print(message); // Send the message
  SIM808_SERIAL.write(0x1A); // End message with Ctrl+Z
  delay(5000); // Wait for message to send
}

// Function to read accelerometer values from a given pin
float readAccel(int pin) {
  int rawValue = analogRead(pin); // Read raw value from pin
  return ((rawValue * 3.3 / 4096) - 1.65) / 0.3; // Convert raw value to acceleration in Gs
}

// Function to send data to ThingSpeak
void sendToThingSpeak(float latitude, float longitude, int alcoholVal, float x, float y, float z) {
  ThingSpeak.setField(1, latitude); // Set field 1 to latitude
  ThingSpeak.setField(2, longitude); // Set field 2 to longitude
  ThingSpeak.setField(3, alcoholVal); // Set field 3 to alcohol sensor value
  ThingSpeak.setField(4, x); // Set field 4 to x-axis acceleration
  ThingSpeak.setField(5, y); // Set field 5 to y-axis acceleration
  ThingSpeak.setField(6, z); // Set field 6 to z-axis acceleration
  ThingSpeak.writeFields(channel_ID, thingSpeakApiKey); // Send the data to ThingSpeak
  Serial.println("Data sent to ThingSpeak."); // Confirm data sent
}

// Task to read GPS data
void gpsTask(void* parameter) {
  GpsData gpsData; // Structure to hold GPS data
  while (1) { // Infinite loop to continuously read GPS data
    while (SerialGPS.available() > 0) { // If data is available on GPS serial port
      if (gps.encode(SerialGPS.read())) { // Parse the incoming GPS data
        if (gps.location.isValid()) { // Check if the location data is valid
          gpsData.latitude = gps.location.lat(); // Get latitude
          gpsData.longitude = gps.location.lng(); // Get longitude
          xQueueSend(gpsQueue, &gpsData, portMAX_DELAY); // Send GPS data to the queue
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to avoid hogging the CPU
  }
}

// Task to read alcohol sensor data
void alcoholSensorTask(void* parameter) {
  while (1) { // Infinite loop to continuously read alcohol sensor data
    alcoholValue = analogRead(ALCOHOL_SENSOR_PIN); // Read alcohol sensor value
    xQueueSend(alcoholQueue, &alcoholValue, portMAX_DELAY); // Send alcohol data to the queue
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second between readings
  }
}

// Task to read accelerometer data and detect accidents
void accelerometerTask(void* parameter) {
  AccelData accelData; // Structure to hold accelerometer data
  while (1) { // Infinite loop to continuously read accelerometer data
    accelData.xAccel = readAccel(X_AXIS_PIN); // Read x-axis acceleration
    accelData.yAccel = readAccel(Y_AXIS_PIN); // Read y-axis acceleration
    accelData.zAccel = readAccel(Z_AXIS_PIN); // Read z-axis acceleration
    xQueueSend(accelQueue, &accelData, portMAX_DELAY); // Send accelerometer data to the queue

    // Check for accident detection based on acceleration thresholds
    if (accelData.xAccel > ACCIDENT_THRESHOLD_X || accelData.yAccel > ACCIDENT_THRESHOLD_Y || accelData.zAccel > ACCIDENT_THRESHOLD_Z || 
        accelData.xAccel < ACCIDENT_THRESHOLD_XN || accelData.yAccel < ACCIDENT_THRESHOLD_YN || accelData.zAccel < ACCIDENT_THRESHOLD_ZN) {
      accidentDetected = true; // Set accident detected flag
      sendSMS(SMS_TEXT, accelData.xAccel, accelData.yAccel); // Send SMS with dummy coordinates for now
      while (1); // Halt execution after accident detection
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to avoid hogging the CPU
  }
}

// Task to send data to ThingSpeak at regular intervals
void thingSpeakTask(void* parameter) {
  GpsData gpsData; // Structure to hold GPS data
  AccelData accelData; // Structure to hold accelerometer data
  int alcoholVal; // Variable to hold alcohol sensor value

  while (1) { // Infinite loop to continuously send data to ThingSpeak
    if (xQueueReceive(gpsQueue, &gpsData, portMAX_DELAY) == pdTRUE) { // Receive GPS data from the queue
      if (xQueueReceive(alcoholQueue, &alcoholVal, portMAX_DELAY) == pdTRUE) { // Receive alcohol data from the queue
        if (xQueueReceive(accelQueue, &accelData, portMAX_DELAY) == pdTRUE) { // Receive accelerometer data from the queue
          sendToThingSpeak(gpsData.latitude, gpsData.longitude, alcoholVal, accelData.xAccel, accelData.yAccel, accelData.zAccel); // Send all data to ThingSpeak
        }
      }
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay for 10 seconds before sending the next batch of data
  }
}

void setup() {
  pinMode(LED, OUTPUT); // Set LED pin as output
  digitalWrite(LED, LOW); // Turn off the LED initially

  Serial.begin(921600); // Start the serial communication at 921600 baud rate for debugging
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // Start GPS serial at 9600 baud on RX2 (pin 16) and TX2 (pin 17)
  SIM808_SERIAL.begin(9600); // Start SIM808 serial at 9600 baud
  sendATCommand(AT); // Initialize SIM808 with an AT command

  // Connect to WiFi
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000); // Wait until WiFi is connected
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Create FreeRTOS queues for GPS, accelerometer, and alcohol sensor data
  gpsQueue = xQueueCreate(5, sizeof(GpsData)); 
  accelQueue = xQueueCreate(5, sizeof(AccelData)); 
  alcoholQueue = xQueueCreate(5, sizeof(int));

  // Create FreeRTOS tasks for GPS, accelerometer, alcohol sensor, and ThingSpeak data handling
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 1, NULL, 0); 
  xTaskCreatePinnedToCore(accelerometerTask, "AccelerometerTask", 4096, NULL, 1, NULL, 1); 
  xTaskCreatePinnedToCore(alcoholSensorTask, "AlcoholSensorTask", 2048, NULL, 1, NULL, 0); 
  xTaskCreatePinnedToCore(thingSpeakTask, "ThingSpeakTask", 4096, NULL, 1, NULL, 1); 
}

void loop() {
  // Main loop does nothing, all work is done in FreeRTOS tasks
}

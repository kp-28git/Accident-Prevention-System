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
const char* ssid = "kushalpatel"; 
const char* password = "qwertyuiop"; 
const char* thingSpeakApiKey = "****************";
const int channel_ID = 7364012;

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
String AT_CMGS = "AT+CMGS=\"8726361782\"\r\n"; 
String SMS_TEXT = "Accident Detected at here - ";

// Timing and flags
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

// Task definitions are omitted for brevity; refer to the earlier implementation with task priorities and SMS logic fixed.

void setup() {
  pinMode(LED, OUTPUT); // Set LED pin as output
  digitalWrite(LED, LOW); // Turn off the LED initially

  Serial.begin(921600); // Start serial communication
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // GPS on Serial1
  SIM808_SERIAL.begin(9600);

  sendATCommand(AT); // Initialize SIM808

  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000); 
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");

  ThingSpeak.begin(client);

  gpsQueue = xQueueCreate(5, sizeof(GpsData)); 
  accelQueue = xQueueCreate(5, sizeof(AccelData)); 
  alcoholQueue = xQueueCreate(5, sizeof(int));

  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 2, NULL, 0); 
  xTaskCreatePinnedToCore(accelerometerTask, "AccelerometerTask", 4096, NULL, 3, NULL, 1); 
  xTaskCreatePinnedToCore(alcoholSensorTask, "AlcoholSensorTask", 2048, NULL, 1, NULL, 0); 
  xTaskCreatePinnedToCore(thingSpeakTask, "ThingSpeakTask", 4096, NULL, 1, NULL, 1); 
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
}

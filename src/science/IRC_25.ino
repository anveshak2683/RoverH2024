#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <Adafruit_BMP085_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>

// Sensor Pins
#define DHTPIN 32
#define SOIL_MOISTURE_PIN_A 33
#define SOIL_MOISTURE_PIN_B 25
#define SDA_PIN 26
#define SCL_PIN 27
#define ONE_WIRE_BUS 13

// Motor Pump Pins
#define MOTOR1_PIN1 19
#define MOTOR1_PIN2 18
#define MOTOR2_PIN1 5
#define MOTOR2_PIN2 17

// Servo Pins
#define SERVO1_PIN 2
#define SERVO2_PIN 15
#define SERVO3_PIN 4

// Stepper Motor Pins
#define STEP_PIN 23
#define DIR_PIN 22
#define ENABLE_PIN 18

// Stepper Motor Configuration
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Servo Instances
Servo servo1, servo2, servo3;

// Sensor Instances
DHT_Unified dht(DHTPIN, DHT11);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// ROS NodeHandle
ros::NodeHandle nh;

// ROS Messages
std_msgs::Float64MultiArray sensor_data;
ros::Publisher sensor_pub("sensor_data", &sensor_data);

// Callback Function Declarations
void pump1_callback(const std_msgs::Bool &msg);
void pump2_callback(const std_msgs::Bool &msg);
void servo1_callback(const std_msgs::Float64 &msg);
void servo2_callback(const std_msgs::Float64 &msg);
void servo3_callback(const std_msgs::Float64 &msg);
void stepper_callback(const std_msgs::Float64 &msg); // Stepper control

// ROS Subscribers
ros::Subscriber<std_msgs::Bool> pump1_sub("pump1_control", &pump1_callback);
ros::Subscriber<std_msgs::Bool> pump2_sub("pump2_control", &pump2_callback);
ros::Subscriber<std_msgs::Float64> servo1_sub("servo1_control", &servo1_callback);
ros::Subscriber<std_msgs::Float64> servo2_sub("servo2_control", &servo2_callback);
ros::Subscriber<std_msgs::Float64> servo3_sub("servo3_control", &servo3_callback);
ros::Subscriber<std_msgs::Float64> stepper_sub("stepper_control", &stepper_callback);

// Variables
uint32_t dhtDelayMS;
float sensor_readings[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};

// Setup Function
void setup() {
  // Initialize ROS
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(sensor_pub);
  nh.subscribe(pump1_sub);
  nh.subscribe(pump2_sub);
  nh.subscribe(servo1_sub);
  nh.subscribe(servo2_sub);
  nh.subscribe(servo3_sub);
  nh.subscribe(stepper_sub);
  
  // Initialize Sensors
  dht.begin();
  /*Wire.begin(SDA_PIN, SCL_PIN);
  if (!bmp.begin()) {
    Serial.println("Error initializing BMP180!");
  }*/
  ds18b20.begin();

  // Servo Initialization
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  // Motor Pin Modes
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  // Stepper Initialization
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable the stepper motor
  stepper.setMaxSpeed(200);
  stepper.setAcceleration(100);

  // ROS Sensor Data Array Initialization
  sensor_data.data = sensor_readings;
  sensor_data.data_length = 9;
}

// Loop Function
void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastUpdate >= 2000) {
    lastUpdate = currentTime;
    updateSensorData();
    sensor_pub.publish(&sensor_data);
  }

  // Continuously run the stepper
  stepper.run();

  nh.spinOnce();
}

// Sensor Data Update
void updateSensorData() {
  sensors_event_t event;

  // Read DHT Temperature and Humidity
  dht.temperature().getEvent(&event);
  sensor_readings[0] = isnan(event.temperature) ? NAN : event.temperature;

  dht.humidity().getEvent(&event);
  sensor_readings[1] = isnan(event.relative_humidity) ? NAN : event.relative_humidity;

  // Read Soil Moisture
  sensor_readings[2] = 100 - (analogRead(SOIL_MOISTURE_PIN_A) / 4095.0 * 100);
  sensor_readings[3] = 100 - (analogRead(SOIL_MOISTURE_PIN_B) / 4095.0 * 100);

  // Read BMP180 Pressure and Altitude
  /*bmp.getEvent(&event);
  if (event.pressure) {
    sensor_readings[4] = event.pressure;
    sensor_readings[5] = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure);
    float temperature;
    bmp.getTemperature(&temperature);
    sensor_readings[6] = temperature;
  }*/

  // Read DS18B20 Temperature
  ds18b20.requestTemperatures();
  sensor_readings[7] = ds18b20.getTempCByIndex(0);
  sensor_readings[8] = ds18b20.getTempCByIndex(1);
}

// Callback Functions
void pump1_callback(const std_msgs::Bool &msg) {
  digitalWrite(MOTOR1_PIN1, msg.data ? HIGH : LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
}

void pump2_callback(const std_msgs::Bool &msg) {
  digitalWrite(MOTOR2_PIN1, msg.data ? HIGH : LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void servo1_callback(const std_msgs::Float64 &msg) {
  servo1.write(constrain(msg.data, 0, 180));
}

void servo2_callback(const std_msgs::Float64 &msg) {
  servo2.write(constrain(msg.data, 0, 180));
}

void servo3_callback(const std_msgs::Float64 &msg) {
  servo3.write(constrain(msg.data, 0, 180));
}

void stepper_callback(const std_msgs::Float64 &msg) {
  long steps = 200 * msg.data / 360; // Calculate steps for the given angle (200 steps/rev)
  stepper.moveTo(stepper.currentPosition() + steps);
}

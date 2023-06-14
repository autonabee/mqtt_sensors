// MQTT Broker
#include "TinyMqtt.h"
#include <string.h>
#include <stdbool.h> 
#include <WiFi.h>
#include <Adafruit_Sensor.h>                                                    
#include <Adafruit_BNO055.h>   

#define PORT 1883

#define CLOSE 0
#define OPEN 1
#define FLEX_LOW_LIMIT 1300
#define FLEX_HIGH_LIMIT 1400

/* ESP IMU configuration */
struct esp_config {
    char* ssid;
    char* password;
    char* mqtt_server;
    /* main and suffix used for topic */
    char* topic_main;
    char* topic_suffix;
    /* delay in MS between two samples*/
    int delay;
    /* Gyro Z */
    bool rot_speed;
    /* Orientation Y */
    bool tilt;
    /* Euler orientation */
    bool orientation;
    /* Quaternion orientation */
    bool quaternion;
    /* EMG sensor */
    bool emg;
    /* Buttons */
    bool flex;
    /* Display data on serial line */
    bool display;
    /* Hosts Wi-Fi hotspot and MQTT broker */
    bool master;
};

struct esp_config cfg = {"fit_and_fun_kids", "fitandfun", "192.168.4.1", "fit_and_fun", "", 100, false, false, true, false, false, false, true, true};

MqttBroker broker(PORT);
MqttClient client(&broker);

/* Hardware pin for definition and leds */
const int emgPin = 36;
const int flexPin = 33;

/* Check I2C device address */                                                          
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); 
/* Mqtt string messages */
#define MSG_BUFFER_SINGLE_SIZE (10)
#define MSG_BUFFER_VECTOR3_SIZE (30)
#define MSG_BUFFER_VECTOR4_SIZE (40)
char msg_single[MSG_BUFFER_SINGLE_SIZE];
char msg_vector3[MSG_BUFFER_VECTOR3_SIZE];
char msg_vector4[MSG_BUFFER_VECTOR4_SIZE];
/* Mqtt string topics */
#define TOPIC_SIZE (30)
char topic_orientation[TOPIC_SIZE];
char topic_quaternion[TOPIC_SIZE];
char topic_tilt[TOPIC_SIZE];
char topic_rot_speed[TOPIC_SIZE];
char topic_emg[TOPIC_SIZE];
/* Sensor variables initialisation */
float emg = 0.0F;
int flexValue = 0;
bool flexState = false;

/* Network connection */
void setupWifi();
void reConnect();

/* Arduino code initialisation (run once) */
void setup() {
  Serial.begin(115200);

  setupWifi(); 
  
  if (cfg.master){
    /* Start the broker */
    broker.begin();
    Serial.print("Starting MQTT broker on address : ");
    Serial.println(WiFi.softAPIP()); // Ip of the broker 
  } else {
    /* Connect client to specified broker adress */
    client.connect(cfg.mqtt_server, PORT);
  }
  
  /* construct topics complete name */
  snprintf(topic_orientation, TOPIC_SIZE, "%s/orientation%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_quaternion, TOPIC_SIZE, "%s/quaternion%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_tilt, TOPIC_SIZE, "%s/tilt%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_rot_speed, TOPIC_SIZE, "%s/rot_speed%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_emg, TOPIC_SIZE, "%s/emg%s", cfg.topic_main, cfg.topic_suffix);

  /* Initialise the sensor */                                                   
  if (!bno.begin())                                                             
  {                                                                             
    /* There was a problem detecting the BNO055 ... check your connections */   
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");                                                                              
    while (1);                                                                  
  }

  delay(1000);                                                                                   
                  
}

/* Arduino code run continuously */
void loop() {

  /* MQTT */
  client.loop();
  if (cfg.master){
    broker.loop(); 
  }
  
  /* Get a new sensor event */
  sensors_event_t eventOrientation, eventGyro;
  if ((cfg.orientation) ||  (cfg.quaternion) ||  (cfg.tilt) ) {
    /* Get a new sensor event */
    bno.getEvent(&eventOrientation, Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();

    /* Print plotable ide arduino data on serial port */
    if (cfg.display) {
      if (cfg.quaternion) {
        /* Quaternion */
        Serial.print("qW: ");
        Serial.print(quat.w(), 4);
        Serial.print(" qX: ");
        Serial.print(quat.x(), 4);
        Serial.print(" qY: ");
        Serial.print(quat.y(), 4);
        Serial.print(" qZ: ");
        Serial.print(quat.z(), 4);
        Serial.println("");
      }   
      if (cfg.orientation) { 
        /* Euler */
        Serial.print("or_x:");
        Serial.print((float)eventOrientation.orientation.x);
        Serial.print(", or_y:");
        Serial.print((float)eventOrientation.orientation.y);
        Serial.print(", or_z:");
        Serial.print((float)eventOrientation.orientation.z);
        Serial.println("");
      }
       if (cfg.tilt) { 
        /* Tilt */
        Serial.print("tilt:");
        Serial.print((float)eventOrientation.orientation.y);
      }
    }
    /* Creation and sending mqtt messages */
    if (cfg.orientation) {
      snprintf(msg_vector3, MSG_BUFFER_VECTOR3_SIZE, "%6.2f %6.2f %6.2f", eventOrientation.orientation.x, eventOrientation.orientation.y, eventOrientation.orientation.z );
      client.publish(topic_orientation,msg_vector3);
    }
     if (cfg.quaternion) {
      snprintf(msg_vector4, MSG_BUFFER_VECTOR4_SIZE, "%2.6f %2.6f %2.6f %2.6f", quat.w(), quat.x(), quat.y(), quat.z() );
      client.publish(topic_quaternion,msg_vector4);
    }
    if (cfg.tilt) {
      snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%6.2f", (float)eventOrientation.orientation.y);
      client.publish(topic_tilt, msg_single);
    }
  }

  if (cfg.rot_speed) {
    /* Get a new sensor event */
    bno.getEvent(&eventGyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    /* Print plotable ide arduino data on serial port */ 
    if (cfg.display) {
      Serial.print("gyr_x:");
      Serial.print((float)eventGyro.gyro.x);
      Serial.print(", gyr_y:");
      Serial.print((float)eventGyro.gyro.y);
      Serial.print(", gyr_z:");
      Serial.print((float)eventGyro.gyro.z);
      Serial.println("");
    }
    /* Creation and sending mqtt messages */
    snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%6.2f", (float)eventGyro.gyro.z);
    client.publish(topic_rot_speed, msg_single);
  }

  if (cfg.emg) {
     /* Get emg value */
    emg = analogRead(emgPin); 
    /* Print plotable ide arduino data on serial port */
    if (cfg.display) {
      Serial.print("emg:");
      Serial.print(emg);
      Serial.println("");
    }
    /* Creation and sending mqtt messages */
    snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%6.2f", emg);
    client.publish(topic_emg, msg_single);
  }

  if(cfg.flex){
    /* Get flex value */
    flexValue = analogRead(flexPin); 
    /* Creation and sending mqtt messages */
    if ((flexValue < FLEX_LOW_LIMIT) && (flexState == OPEN)) {
      flexState = CLOSE; 
      snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%d", flexState);
      client.publish(topic_emg, msg_single);
      if (cfg.display){
        Serial.print("Sending message on topic ");
        Serial.println(topic_emg);
      }
    } else if ((flexValue > FLEX_HIGH_LIMIT) && (flexState == CLOSE)) {
      flexState = OPEN;
      snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%d", flexState);
      client.publish(topic_emg, msg_single);
      if (cfg.display){
        Serial.print("Sending message on topic ");
        Serial.println(topic_emg);
      }
    }
    /* Print plotable ide arduino data on serial port */
    if (cfg.display) {
      Serial.print("flex:");
      Serial.print(flexValue);
      Serial.print(" ----- ");
      Serial.println((flexState == OPEN ? "OPEN" : "CLOSE"));
    }
  }

  delay(cfg.delay);
}

void setupWifi() {
    delay(10);
    if (cfg.master){
      /* Start Wifi connection */
      WiFi.mode(WIFI_AP);
      WiFi.softAP(cfg.ssid, cfg.password);
      Serial.print("Hosting Wi-Fi hotspot : ");
      Serial.print(cfg.ssid);
      Serial.print(" with password ");
      Serial.println(cfg.password);
      delay(200);
      Serial.print("Local ESP32 IP: ");
      Serial.println(WiFi.localIP());
    } else {
      /* Connect to specified Wi-Fi network */
      WiFi.mode(WIFI_STA);
      WiFi.begin(cfg.ssid, cfg.password);
      Serial.print("Connecting to Wi-Fi ");
      Serial.println(cfg.ssid);
      while (WiFi.status() != WL_CONNECTED)
	      { delay(500); Serial << '.'; }
      Serial.print("Connected to Wi-Fi ");
      Serial.println(cfg.ssid);
    }
}

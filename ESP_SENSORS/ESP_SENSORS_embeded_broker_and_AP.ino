// Debug topic : 0: No 1: Yes , If set to 1 topic names, and send values are displayed 
#define DEBUG 0

// included libs
#include <Arduino.h>
// MQTT Broker, huge adventage, can publish esealy on topic
#include <PicoMQTT.h>
// Message size
#define MSG_BUFFER_SIZE (10)
// IMU declaration
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// Wifi AP
#include <WiFi.h>
//
#include <string.h>
#include <stdbool.h>

// Declaration mqtt server
PicoMQTT::Server mqtt;


///////////////////////////////////////////

/* ESP IMU configuration */
struct esp_config {
  char* ssid; // SSID of the Access point 
  char* password; // Password of the access point
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
  bool buttons;
  /* Display data on serial line */
  bool display;
};

//struct esp_config cfg = {"fit_and_fun", "", "10.42.0.1","fit_and_fun", "", 100, true, false, false, false, false, false, true};
struct esp_config cfg = {"Fit_and_fun_kid", "kid_fit_and_fun", "192.168.4.1", "fit_and_fun", "_1", 100, false, false, true, true, false, false, true};


/* Hardware pin for definition and leds */
const int buttonOnePin = 14;
const int buttonTwoPin = 15;
const int ledPin =  13;
const int emgPin = 36;

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
char topic_button1[TOPIC_SIZE];
char topic_button2[TOPIC_SIZE];
/* Sensor variables initialisation */
int buttonOneState = 0;
int buttonTwoState = 0;
float emg = 0.0F;

///////////////////////////////////////////

void setup() {
  // Setting up the serial communication
  Serial.begin(115200);
  // Setting up the board as Wifi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(cfg.ssid, cfg.password);
  delay(200);
  Serial.println(WiFi.softAPIP()); // Ip of the broker 
  // Start the broker
  mqtt.begin();  
  // Subscribe to a topic pattern and attach a callback
  mqtt.subscribe("#", [](const char * topic, const char * payload) {
    Serial.printf("Received message in topic '%s': %s\n", topic, payload);
  });

  /////////////////////////////////////////////

  /* construct topics complete name */
  snprintf(topic_orientation, TOPIC_SIZE, "%s/orientation%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_quaternion, TOPIC_SIZE, "%s/quaternion%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_tilt, TOPIC_SIZE, "%s/tilt%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_rot_speed, TOPIC_SIZE, "%s/rot_speed%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_emg, TOPIC_SIZE, "%s/emg%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_button1, TOPIC_SIZE, "%s/button1%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_button2, TOPIC_SIZE, "%s/button2%s", cfg.topic_main, cfg.topic_suffix);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialize the two button inputs */
  if (cfg.buttons) {
    pinMode(buttonOnePin, INPUT_PULLUP);
    pinMode(buttonTwoPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
  }

  delay(1000);

  ////////////////////////////////////////////
}

void loop() {
  // This will automatically handle client connections.  By default, all clients are accepted.
  mqtt.loop();

  //////////////////////////////////////////////////

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
        if(DEBUG) {
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
      }
      if (cfg.quaternion) {
        /* Euler */
        if(DEBUG) {
          Serial.print("or_x:");
          Serial.print((float)eventOrientation.orientation.x);
          Serial.print(", or_y:");
          Serial.print((float)eventOrientation.orientation.y);
          Serial.print(", or_z:");
          Serial.print((float)eventOrientation.orientation.z);
          Serial.println("");
        }
      }
      if (cfg.tilt) {
        /* Tilt */
        if(DEBUG) {
          Serial.print("tilt:");
          Serial.print((float)eventOrientation.orientation.y);
        }
      }
    }
    /* Creation and sending mqtt messages */
    if (cfg.orientation) {
      snprintf(msg_vector3, MSG_BUFFER_VECTOR3_SIZE, "%6.2f %6.2f %6.2f", eventOrientation.orientation.x, eventOrientation.orientation.y, eventOrientation.orientation.z );
      mqtt.publish(topic_orientation, msg_vector3);
      //////////////////////
      // Debug
      if (DEBUG) {
        Serial.print("topic varriable : topic_orientation => ");
        Serial.print(topic_orientation);
        Serial.print(" topic message : msg_vector3 => ");
        Serial.println(msg_vector3);
      }
      
    }
    if (cfg.quaternion) {
      snprintf(msg_vector4, MSG_BUFFER_VECTOR4_SIZE, "%2.6f %2.6f %2.6f %2.6f", quat.w(), quat.x(), quat.y(), quat.z() );
      mqtt.publish(topic_quaternion, msg_vector4);
      //////////////////////
      // Debug
      if (DEBUG) {
        Serial.print("topic varriable : topic_quaternion => ");
        Serial.print(topic_orientation);
        Serial.print(" topic message : msg_vector4 => ");
        Serial.println(msg_vector4);
      }
    }
    if (cfg.tilt) {
      snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%6.2f", (float)eventOrientation.orientation.y);
      mqtt.publish(topic_tilt, msg_single);
      //////////////////////
      // Debug
      if (DEBUG) {
        Serial.print("topic varriable : topic_tilt => ");
        Serial.print(topic_tilt);
        Serial.print(" topic message : msg_vector3 => ");
        Serial.println(msg_single);
      }
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
    mqtt.publish(topic_rot_speed, msg_single);
  }
  if (cfg.buttons) {
    /* Get buttons one and two states */
    buttonOneState = digitalRead(buttonOnePin);
    buttonTwoState = digitalRead(buttonTwoPin);
    /* Print plotable ide arduino data on serial port */
    if (cfg.display) {
      Serial.print("button1:");
      Serial.print(buttonOneState);
      Serial.print(", button2:");
      Serial.print(buttonTwoState);
      Serial.println("");
    }
    /* Creation and sending mqtt messages */
    snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%s", buttonOneState == HIGH ? "false" : "true");
    mqtt.publish(topic_button1, msg_single);
    snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%s", buttonTwoState == HIGH ? "false" : "true");
    mqtt.publish(topic_button2, msg_single);
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
    mqtt.publish(topic_rot_speed, msg_single);
  }
  delay(cfg.delay);
}

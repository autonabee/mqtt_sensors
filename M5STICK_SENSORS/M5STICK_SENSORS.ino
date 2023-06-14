
// MQTT Broker
#include "TinyMqtt.h"
#include <string.h>
#include <stdbool.h> 
#include <WiFi.h>
#include <M5StickCPlus.h>

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
    /* EMG sensor */
    bool emg;
    /* Flex-force sensor */
    bool flex;
    /* Display data on serial line */
    bool display;
    /* Hosts Wi-Fi hotspot and MQTT broker */
    bool master;
};

struct esp_config cfg = {"fit_and_fun_kids", "fitandfun", "192.168.4.1", "fit_and_fun", "", 100, false, false, false, false, true, true, true};

MqttBroker broker(PORT);
MqttClient client(&broker);

const int ledPin =  13; 
const int emgPin = 36;
const int flexPin = 33;

/* Mqtt string messages */
#define MSG_BUFFER_SINGLE_SIZE (10)
#define MSG_BUFFER_VECTOR_SIZE (30)
char msg_single[MSG_BUFFER_SINGLE_SIZE];
char msg_vector[MSG_BUFFER_VECTOR_SIZE];
/* Mqtt string topics */
#define TOPIC_SIZE (30)
char topic_orientation[TOPIC_SIZE];
char topic_tilt[TOPIC_SIZE];
char topic_rot_speed[TOPIC_SIZE];
char topic_emg[TOPIC_SIZE];
/* Sensor variables initialisation */
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float orientX = 0.0F;
float orientY = 0.0F;
float orientZ = 0.0F;
float emg = 0.0F;
int flexValue = 0;
bool flexState = false;

/* Network connection */
void setupWifi();
void reConnect();

/* Arduino code initialisation (run once) */
void setup() {
  Serial.begin(115200);  
  M5.begin();
  M5.Lcd.setRotation(3);

  setupWifi(); 
  
  /* IMU init */
  M5.Imu.Init(); 
  
  if (cfg.master){
    /* Start the broker */
    broker.begin();
    Serial.print("Starting MQTT broker on address : ");
    Serial.println(WiFi.softAPIP()); // Ip of the broker 
  } else {
    /* Connect client to specified broker adress */
    client.connect(cfg.mqtt_server, PORT);
  }
  
  snprintf(topic_orientation, TOPIC_SIZE, "%s/orientation%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_tilt, TOPIC_SIZE, "%s/tilt%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_rot_speed, TOPIC_SIZE, "%s/rot_speed%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_emg, TOPIC_SIZE, "%s/emg%s", cfg.topic_main, cfg.topic_suffix);

  if (cfg.emg) {
    pinMode(36, ANALOG);
  }

  if (cfg.flex){
    pinMode(33, ANALOG);
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
  if ((cfg.orientation) ||  (cfg.tilt)) {
    /* Get a new sensor event */
    M5.IMU.getAhrsData(&orientX, &orientY, &orientZ);
    /* Print plotable ide arduino data on serial port */
    if (cfg.display) {
      Serial.print("or_x:");
      Serial.print(orientX);
      Serial.print(", or_y:");
      Serial.print(orientX);
      Serial.print(", or_z:");
      Serial.print(orientZ);
      Serial.println("");
      M5.Lcd.setCursor(0, 20);
      M5.Lcd.printf("o_X:%5.2f/no_Y:%5.2f/no_Z:%5.2f ", orientX, orientY, orientZ);
    }
    /* Creation and sending mqtt messages */
    if (cfg.orientation) {
      snprintf(msg_vector, MSG_BUFFER_VECTOR_SIZE, "%6.2f %6.2f %6.2f",orientX, orientY, orientZ );
      client.publish(topic_orientation, msg_vector);
    }
    if (cfg.tilt) {
      snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%6.2f", orientY);
      client.publish(topic_tilt, msg_single);
    }
  }

  if (cfg.rot_speed) {
    /* Get a new sensor event */
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    /* Print plotable ide arduino data on serial port */ 
    if (cfg.display) {
      Serial.print("gyr_x:");
      Serial.print(gyroX);
      Serial.print(", gyr_y:");
      Serial.print(gyroY);
      Serial.print(", gyr_z:");
      Serial.print(gyroZ);
      Serial.println("");
      M5.Lcd.setCursor(0, 30);
      M5.Lcd.printf("g_X:%5.2f/g_Y:%5.2f/g_Z:%5.2f ", gyroX, gyroY, gyroZ);
    }
    /* Creation and sending mqtt messages */
    snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%6.2f", gyroZ);
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
      M5.Lcd.setCursor(0, 40);
      M5.Lcd.printf("emg:%5.2f", emg);
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
      M5.Lcd.setCursor(0, 40);
      M5.Lcd.printf("flex:%d ----- %s", flexValue, (flexState == OPEN ? "OPEN" : "CLOSE"));
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
      Serial.print("Local M5Stick IP: ");
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

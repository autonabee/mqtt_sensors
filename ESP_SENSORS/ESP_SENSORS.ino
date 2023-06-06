#include <string.h>
#include <stdbool.h> 
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>                                                    
#include <Adafruit_BNO055.h>   

WiFiClient espClient;
PubSubClient client(espClient);

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
    /* Buttons */
    bool buttons;
    /* Display data on serial line */
    bool display;
};

//struct esp_config cfg = {"fit_and_fun", "", "10.42.0.1","fit_and_fun", "", 100, true, false, false, false, false, true};
struct esp_config cfg = {"honor5_roger", "", "192.168.43.78", "fit_and_fun", "_1", 100, false, false, true, false, false, true};

/* Hardware pin for definition and leds */
const int buttonOnePin = 14;
const int buttonTwoPin = 15;
const int ledPin =  13; 
const int emgPin = 36;

/* Check I2C device address */                                                          
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); 
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
char topic_button1[TOPIC_SIZE];
char topic_button2[TOPIC_SIZE]; 
/* Sensor variables initialisation */
int buttonOneState = 0;
int buttonTwoState = 0;
float emg = 0.0F;

/* Network connection */
void setupWifi();
void reConnect();

/* Arduino code initialisation (run once) */
void setup() {
  Serial.begin(115200); 
  setupWifi(); 
  /* Mqtt init */
  client.setServer(cfg.mqtt_server, 1883); 
  
  snprintf(topic_orientation, TOPIC_SIZE, "%s/orientation%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_tilt, TOPIC_SIZE, "%s/tilt%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_rot_speed, TOPIC_SIZE, "%s/rot_speed%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_button1, TOPIC_SIZE, "%s/button1%s", cfg.topic_main, cfg.topic_suffix);
  snprintf(topic_button2, TOPIC_SIZE, "%s/button2%s", cfg.topic_main, cfg.topic_suffix);

  /* Initialise the sensor */                                                   
  if (!bno.begin())                                                             
  {                                                                             
    /* There was a problem detecting the BNO055 ... check your connections */   
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"\
);                                                                              
    while (1);                                                                  
  }       

  /* Initialize the two button inputs */
  if (cfg.buttons) {
    pinMode(buttonOnePin, INPUT_PULLUP);
    pinMode(buttonTwoPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
  }

  delay(1000);                                                                                   
                  
}

/* Arduino code run continuously */
void loop() {
  if (!client.connected()) {
        reConnect();
    }
  client.loop(); 
  
  /* Get a new sensor event */
  sensors_event_t eventOrientation, eventGyro;
  if ((cfg.orientation) ||  (cfg.tilt)) {
    /* Get a new sensor event */
    bno.getEvent(&eventOrientation, Adafruit_BNO055::VECTOR_EULER);
    /* Print plotable ide arduino data on serial port */
    if (cfg.display) {
      Serial.print("or_x:");
      Serial.print((float)eventOrientation.orientation.x);
      Serial.print(", or_y:");
      Serial.print((float)eventOrientation.orientation.y);
      Serial.print(", or_z:");
      Serial.print((float)eventOrientation.orientation.z);
      Serial.println("");
    }
    /* Creation and sending mqtt messages */
    if (cfg.orientation) {
      snprintf(msg_vector, MSG_BUFFER_VECTOR_SIZE, "%6.2f %6.2f %6.2f", eventOrientation.orientation.x, eventOrientation.orientation.y, eventOrientation.orientation.z );
      client.publish(topic_orientation, msg_vector);
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
    client.publish(topic_button1, msg_single);
    snprintf(msg_single, MSG_BUFFER_SINGLE_SIZE, "%s", buttonTwoState == HIGH ? "false" : "true");
    client.publish(topic_button2, msg_single);
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
    client.publish(topic_rot_speed, msg_single);
  }
  delay(cfg.delay);
}

void setupWifi() {
    delay(10);
    /* Start Wifi connection */
    WiFi.begin(cfg.ssid);
    Serial.print("Connecting to ");
    Serial.println(cfg.ssid);
    if ( strlen(cfg.password) == 0 ) {
      WiFi.begin(cfg.ssid);
    } else {
      WiFi.begin(cfg.ssid, cfg.password);
    }
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}

void reConnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID.  
        String clientId = "ESP_IMU-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect. 
        if (client.connect(clientId.c_str())) {
            Serial.println("\nSuccess\n");
            // Once connected, publish an announcement to the topic.
            client.publish(cfg.topic_main, "ESP_IMU init");
            // ... and resubscribe.
            client.subscribe(cfg.topic_main);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println("try again in 5 seconds");
            delay(5000);
        }
    }
}

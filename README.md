# mqtt_sensors

DIY sensors with mqtt communication

Used in Fit_and_fun and Fit_and_fun_kids


## Hardware 

The node can be an Adafruit ESP32 Feather or a M5STICKC-Plus. Sensors which can be used are:
* an IMU (Adafruit BNO055 for ESP32 Feather or built-in 6-Axis IMU  M5STICKC-Plus)
* a Myoware V2 EMG
* two buttons

## Arduino code

* ESP_SENSORS/ESP_SENSORS.ino
* M5STICK_SENSORS/M5STICK_SENSORS.ino

It implements the same behavior for 2 differents hardware.
Depending on configuration, the node can publish data on mqtt topics:
*  `fit_and_fun/rot_speed`
*  `fit_and_fun/orientation`
*  `fit_and_fun/tilt`
*  `fit_and_fun/emg`
*  `fit_and_fun/button1` and `fit_and_fun/button2`

## Tests

The `mqtt_subscriber.py` allows to test mqtt connection with a PC.

### Requirements

* `Ubuntu 22.04`
* python >3.10

```bash
> pip3 install -r requirements.txt
```

### Mqtt broker config

Add at this end of the file: `/etc/mosquitto/mosquitto.conf`

```bash
listener 1883 localhost 
listener 1883 10.42.0.1
allow_anonymous true
```

Restart the service

```bash
> systemctl restart mosquitto
```



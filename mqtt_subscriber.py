from enum import Enum
import threading
import time
import sys
import paho.mqtt.client as mqtt


class mqtt_subscriber():
    """ Mqtt subscriber to receive a raw rotational speed"""
    def __init__(self, on_message, synchro, topics, broker_addr='localhost'):
        """
        Class constructor

        Parameter:
        ---------
        on_message: callback function
            mqtt callback subscriber to get the value

        synchro: threading.Lock
            to synchronize the end of the client subscriber
        
        broker_addr: string
            broker address (name or IP) in action

        topics: [string]
            Mqtt topics to be subscribed
        
        """
        self.on_message=on_message
        self.mqttBroker = broker_addr
        self.topics = topics
        # Lock thread synchronization
        self.lock = synchro
        self.lock.acquire()
        self.debug=True


    def subscribe_connect(self):
        """ client function launched in a thread
        """
        if self.debug==True: print("Start mqtt subscriber")
        # Broker connection
        client = mqtt.Client("Console")
        client.connect(self.mqttBroker) 
        # Topics 'fit_and_fun/speed' subscription
        client.loop_start()
        for topic in self.topics:
            client.subscribe(topic)
        client.on_message=self.on_message 
        # Wait for the end
        self.lock.acquire()
        client.loop_stop()
        if self.debug==True: print("End mqtt subscriber")
   
    def run(self):
        """ Start the thread """
        self.t1=threading.Thread(target=self.subscribe_connect)
        self.t1.start()

    def stop(self):
        """ Stop the thread """
        self.lock.release()
        self.t1.join()

class SENSOR_TYPE(Enum):
    ROT_SPEED   = 1
    ORIENTATION = 2
    TILT        = 3

class mqtt_imu_example():

    def __init__(self, topic_main, topics_list):
        """
        Constructor
        """
        self.lock = threading.Lock()
        self.reception=False
        # Topics list name contruction associated with sensor type
        self.topics_list=[]
        self.sensors_list=[]
        for topic in topics_list: 
            # Dynamic attribute creation with the name of the topic
            self.__dict__[topic[0]] = None
            self.topics_list.append(topic_main+'/'+topic[0])
            self.sensors_list.append(topic[1])
        # Mqtt subscribing
        self.mqtt_sub=mqtt_subscriber(self.message_callback, self.lock, self.topics_list)

    def get_topic(self, abs_topic_name):
        topic_name=abs_topic_name.split('/')[1]
        return(self.__getattribute__(topic_name))

    def set_topic(self, abs_topic_name, value):
        topic_name=abs_topic_name.split('/')[1]
        return(self.__setattr__(topic_name, value))

    def get_topic_sensor_type(self, topic_name):
        """
        """
        index=-1
        sensor_type=None
        try:
            index = self.topics_list.index(topic_name)
            sensor_type=self.sensors_list[index]
        except Exception:
            sensor_type=None
        return sensor_type

    def get_rot_speed(self, client, userdata, message):
        """
        Callback for rot_speed mqtt topic
        """
        try:
            self.set_topic(message.topic, float(str(message.payload.decode("utf-8"))))
            self.reception=True
            #print('GET ', self.get_topic(message.topic))
        except Exception:
            print('Error in mqtt message')
            self.reception=False
            self.set_topic(message.topic, 0)
            
    def get_tilt(self, client, userdata, message):
        """
        Callback for tilt mqtt topic
        """
        try:
            self.set_topic(message.topic, float(str(message.payload.decode("utf-8"))))
            self.reception=True
            #print('GET ', self.get_topic(message.topic))
        except Exception:
            print('Error in mqtt message')
            self.reception=False
            self.set_topic(message.topic, 0)

    def get_orientation(self, client, userdata, message):
        """
        Callback for orientation mqtt topic
        """
        try:
            orientation_str=str(message.payload.decode("utf-8"))
            orientation = [float(x) for x in orientation_str.split()]
            self.set_topic(message.topic, orientation)
            self.reception=True
            #print('GET ', self.get_topic(message.topic) )
        except Exception:
            print('Error in mqtt message')
            self.reception=False
            self.set_topic(message.topic, [0,0,0])
  
    def message_callback(self, client, userdata, message):
        """ 
        Mqtt callback treating all topics
        """  
        sensor_type=self.get_topic_sensor_type(message.topic)   
        match sensor_type:
            case SENSOR_TYPE.ROT_SPEED:
                self.get_rot_speed(client, userdata, message)
            case SENSOR_TYPE.ORIENTATION:
                self.get_orientation(client, userdata, message)
            case SENSOR_TYPE.TILT:
                self.get_tilt(client, userdata, message)
            case _:
                print("WARNING: topic " + message.topic + " unknown\n")

    def run(self):
        self.mqtt_sub.run()
   
    def stop(self):
        self.mqtt_sub.stop()

def example_imu():
    topics_list=[ ['rot_speed', SENSOR_TYPE.ROT_SPEED],
                  ['orientation', SENSOR_TYPE.ORIENTATION],
                  ['tilt', SENSOR_TYPE.TILT]
                ]
    mqtt_imu=mqtt_imu_example('fit_and_fun', topics_list)
    
    mqtt_imu.run()
    try:    
        while True:
            if mqtt_imu.reception:
                print('ORIENTATION', mqtt_imu.orientation)
                print('ROT_SPEED', mqtt_imu.rot_speed)
            else:
                print('Mqtt sensors not detected')
            time.sleep(0.2)
    except KeyboardInterrupt:
        # User interrupt the program with ctrl+c
        print('CTL C') 
        mqtt_imu.stop()
        sys.exit()

def example_joint():
    topics_list=[ ['orientation', SENSOR_TYPE.ORIENTATION],
                  ['orientation_1', SENSOR_TYPE.ORIENTATION]
                ]
    mqtt_imu=mqtt_imu_example('fit_and_fun', topics_list)
    
    mqtt_imu.run()
    try:    
        while True: 
            if mqtt_imu.reception:
                angle=0
                if mqtt_imu.orientation_1 is not None:
                    angle=angle+1
                    print('OR1', mqtt_imu.orientation_1[1], end="")
                if mqtt_imu.orientation is not None:
                    angle=angle+1 
                    print('OR', mqtt_imu.orientation[1], end="")
                if angle == 2:
                    print('ANGLE',  mqtt_imu.orientation_1[1] - mqtt_imu.orientation[0])
                else:
                    print("")
            else:
                print('Mqtt sensors not detected')
            time.sleep(0.2)
    except KeyboardInterrupt:
        # User interrupt the program with ctrl+c
        print('CTL C') 
        mqtt_imu.stop()
        sys.exit()

if __name__ == '__main__':
    #example_imu()
    example_joint()
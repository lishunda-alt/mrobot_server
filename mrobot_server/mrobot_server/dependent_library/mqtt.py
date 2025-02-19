from paho.mqtt import client as mqtt_client
from .log import log

import json
import threading
import time

class MQTT:
    def __init__(self, _broker, _port, _client_id, subscribe_topic, _fun_name) -> None:
       self.broker = _broker
       self.port = _port
       self.client_id = _client_id
       self.subscribe_topic = subscribe_topic
       self.subscribe_function = _fun_name
       self.is_connected = False
       self.connect_mqtt()
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.is_connected = True
            log.info("Connected to MQTT Broker!")
            t = threading.Thread(target=self.subscribe, name="subscribe", args=(self.subscribe_topic, self.subscribe_function))
            t.start()

    def on_disconnect(self, client, userdata, rc):
        self.is_connected = False
        log.info("mqtt the disconnection result: {}".format(rc))
        
    def connect_mqtt(self):
        try:
            msg = {"id":str(self.client_id), "cmd":"/api/v1/offline/"}
            self.client = mqtt_client.Client(self.client_id) 
            self.client.username_pw_set("mrbrain", "mr@2023")
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.reconnect_delay_set(1)
            self.client.will_set("mrclient/" + str(self.client_id) + "/status", json.dumps(msg), 0, False)
            # self.client.connect(self.broker, self.port)     # # 阻塞连接
            self.client.connect_async(self.broker, self.port)
            self.loop_start()
        except Exception as e:
            print("failed to connect server!", e)
    
    def publish(self, topic, msg):
        try:
            result = self.client.publish(topic, str(msg))[0]
            if result != 0:
                print(f"Failed to send message to topic {topic}")
        except Exception as e:
            print(e)
            
    def loop_start(self):
        try:
            self.client.loop_start()
        except Exception as e:
            print(e)
    
    def loop_stop(self):
        try:
            self.client.loop_start()
        except Exception as e:
            print(e)

    def subscribe(self, topic, _fun_name):
        try:
            def on_message(client, userdata, msg):
                self.msg = msg.payload.decode()
                # print("msg: ", self.msg)
                _fun_name(self.msg, msg.topic)
            self.client.subscribe(topic)
            self.client.on_message = on_message
        except Exception as e:
            print(e)


# def sub(msg, topic):
#     print(msg, topic)

# def msub(client):
#     client.subscribe("mtopic", sub)

# broker = '111.230.7.154'
# port = 1883
# topic = "mrrserver/1"
# client_id = "0001"
# print(client_id)
# msg = {"a":1}
# m = {"b":1}
# msg = json.dumps(msg)
# client = MQTT(broker, port, client_id, "mtopic", sub)
# time.sleep(1)
# # t4 = threading.Thread(target=msub, name='msub')
# # t4.start()
# while True:
#     client.publish("topic", "msg")
#     # client.subscribe("mtopic", sub)
#     time.sleep(1)

# def sub_print(msg, topic):
#     print(msg)
#     client.publish("topic", m)




# print("sub")
# while True:
#     client.publish(topic, msg)
#     time.sleep(1)
#     print("pub")
    

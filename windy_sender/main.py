import logging
import os
import time
from collections import defaultdict
import statistics

import requests
from paho.mqtt import client as mqtt_client

broker = os.environ["MQTT_BROKER"]
port = 1883
username = os.environ["MQTT_USERNAME"]
password = os.environ["MQTT_PASSWORD"]
client_id = 'weather-sender'

temp_topic = 'outside_temp'
humidity_topic = 'outside_humidity'
pressure_topic = 'outside_pressure'
wind_speed_topic = 'outside_wind_speed'
wind_direction_topic = 'outside_wind_direction'

wind_gust_aggregate = 'gust'

all_numeric_topics = [temp_topic, humidity_topic, pressure_topic, wind_speed_topic]
topic_to_windy = {temp_topic: 'temp',
                  humidity_topic: 'rh',
                  pressure_topic: 'pressure',
                  wind_speed_topic: 'wind',
                  wind_direction_topic: 'winddir',
                  wind_gust_aggregate: 'gust'
                  }

data = {}
totals = {}

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

wind_map = {"N": 0,
            "NE": 45,
            "E": 90,
            "SE": 135,
            "S": 180,
            "SW": 225,
            "W": 270,
            "NW": 315,
            "-1": -1
            }

data = defaultdict(list)


def connect_mqtt():
    def on_connect(_client, userdata, flags, rc, properties=None):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id=client_id,
                                callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)

    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def on_disconnect(client, userdata, rc):
    logging.info("Disconnected with result code: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        logging.info("Reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            logging.info("Reconnected successfully!")
            return
        except Exception as err:
            logging.error("%s. Reconnect failed. Retrying...", err)

        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)


def subscribe(client: mqtt_client):
    def on_message(_client, userdata, msg):
        category = msg.topic
        if msg.topic in all_numeric_topics:
            payload = float(msg.payload.decode())
            data[category].append(payload)
        if category == wind_direction_topic:
            payload = msg.payload.decode()
            value = wind_map[payload]
            data[category].append(value)

    for topic in all_numeric_topics:
        client.subscribe(topic)
    client.subscribe(wind_direction_topic)
    client.on_message = on_message


def process_means():
    for category, data_values in data.items():
        if data_values:
            median = statistics.median(data_values)
            totals[category] = median
            if category == wind_speed_topic:
                mx = max(data[category])
                totals[wind_gust_aggregate] = mx
    # print(totals)
    transformed_totals = {}
    for old_category, value in totals.items():
        new_category = topic_to_windy.get(old_category, old_category)
        transformed_totals[new_category] = value

    # print(transformed_totals)
    url = "https://stations.windy.com/pws/update/" + os.environ["WINDY_API_KEY"]
    response = requests.post(url, json=transformed_totals)
    if response.status_code == 200:
        print("Result sent successfully")
    else:
        print("Failed to send result")


def main():
    client = connect_mqtt()
    client.on_disconnect = on_disconnect
    subscribe(client)
    client.loop_start()
    while True:
        process_means()
        time.sleep(60)  # Sleep for 5 minutes


if __name__ == "__main__":
    main()

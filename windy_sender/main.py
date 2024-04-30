import json
import logging
import os
import subprocess
import time
from collections import defaultdict
import statistics
from datetime import datetime, timedelta

from dotenv import load_dotenv

import requests
from paho.mqtt import client as mqtt_client

load_dotenv()

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

data_filename = "weather_data.json"
cycle_length = 300  # 5 minutes
github_cycles = 6  # 30 minutes

all_numeric_topics = [temp_topic, humidity_topic, pressure_topic, wind_speed_topic]
topic_to_windy = {temp_topic: 'temp',
                  humidity_topic: 'rh',
                  pressure_topic: 'pressure',
                  wind_speed_topic: 'wind',
                  wind_direction_topic: 'winddir',
                  wind_gust_aggregate: 'gust'
                  }

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

mqtt_data = defaultdict(list)


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
            mqtt_data[category].append(payload)
        if category == wind_direction_topic:
            payload = msg.payload.decode()
            value = wind_map[payload]
            mqtt_data[category].append(value)

    for topic in all_numeric_topics:
        client.subscribe(topic)
    client.subscribe(wind_direction_topic)
    client.on_message = on_message


def update_github_repository(weather_data):
    # Commit and push changes to GitHub
    subprocess.run(['git', 'add', data_filename])
    subprocess.run(['git', 'commit', '-m', 'Update weather data'])
    subprocess.run(['git', 'push'])


def send_to_windy(data):
    url = "https://stations.windy.com/pws/update/" + os.environ["WINDY_API_KEY"]
    print(data)
    response = requests.post(url, json=data)
    if response.status_code == 200:
        print("sent ok")
    else:
        print("Failed to send result")


def process_means():
    totals = {}
    for category, data_values in mqtt_data.items():
        if data_values:
            totals[category] = round(statistics.median(data_values), 2)
            if category == wind_speed_topic:
                mx = max(mqtt_data[category])
                totals[wind_gust_aggregate] = mx
    transformed_totals = {}
    for old_category, value in totals.items():
        new_category = topic_to_windy.get(old_category, old_category)
        transformed_totals[new_category] = value
    mqtt_data.clear()
    return transformed_totals


def main():
    client = connect_mqtt()
    client.on_disconnect = on_disconnect
    subscribe(client)
    client.loop_start()
    cycle_counter = 0
    try:
        with open(data_filename, 'r') as f:
            weather_history = json.load(f)
    except FileNotFoundError:
        weather_history = []
    while True:
        time.sleep(cycle_length)
        processed = process_means()
        if len(processed):
            try:
                send_to_windy(processed)
            except Exception as e:
                print("Cannot send to windy", e)
            processed["timestamp"] = int(time.time())
            weather_history.append(processed)
            weather_history = [data for data in weather_history
                               if data['timestamp'] >= time.time() - (12 * 60 * 60)]
            with open(data_filename, 'w') as f:
                json.dump(weather_history, f, indent=4)
        cycle_counter += 1
        if cycle_counter % github_cycles == 0:
            try:
                update_github_repository(weather_history)
            except Exception as e:
                print("Cannot update github", e)


if __name__ == "__main__":
    main()

import asyncio
import csv
import logging
import os
import time
import boto3

from collections import defaultdict

from dotenv import load_dotenv

from paho.mqtt import client as mqtt_client

load_dotenv()

s3 = boto3.client('s3', aws_access_key_id=os.environ["AWS_KEY_ID"], aws_secret_access_key=os.environ["AWS_ACCESS_KEY"],
                  region_name=os.environ["AWS_REGION"])

broker = os.environ["MQTT_BROKER"]
port = 1883
username = os.environ["MQTT_USERNAME"]
password = os.environ["MQTT_PASSWORD"]
client_id = 'weather-sender'

air_quality_topic = 'weather-station/sensor/air_quality/state'
rain_topic = 'weather-station/sensor/rain/state'
temp_topic = 'weather-station/sensor/temperature/state'
humidity_topic = 'weather-station/sensor/humidity/state'
pressure_topic = 'weather-station/sensor/outside_pressure/state'
wind_speed_topic = 'weather-station/sensor/wind_speed/state'
wind_direction_topic = 'weather-station/sensor/wind_direction/state'
wind_gusts_topic = 'weather-station/sensor/wind_gusts/state'

all_topics = [air_quality_topic, rain_topic, temp_topic, humidity_topic, pressure_topic, wind_speed_topic,
              wind_direction_topic, wind_gusts_topic]

data_filename = "weather_data.csv"

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

upload_interval = 60 * 30  # 30 mins


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


def on_disconnect(client, userdata, rc, arg4, arg5):
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


def subscribe(client: mqtt_client, mqtt_data, weather_history):
    def on_message(_client, userdata, msg):
        payload_str = msg.payload.decode('utf-8')

        # Convert payload to proper type based on application logic
        try:
            # Try converting to integer
            payload_int = int(payload_str)
            mqtt_data[msg.topic] = payload_int
        except ValueError:
            try:
                payload_float = float(payload_str)
                mqtt_data[msg.topic] = payload_float
            except ValueError:
                # Keep payload as string if not convertible to int or float
                mqtt_data[msg.topic] = payload_str
        check_and_upload(mqtt_data, weather_history)

    for topic in all_topics:
        client.subscribe(topic)
    client.on_message = on_message


def check_and_upload(mqtt_data, weather_history):
    complete = True
    for topic in all_topics:
        if topic not in mqtt_data:
            complete = False
    # not ideal obviously, but esphome dumps all data at about the same time
    if complete:
        row = [round(time.time())]
        # TODO: s3 does not allow appending lines -- switch storage mechanism
        for topic in all_topics:
            row.append(mqtt_data[topic])
        weather_history.append(row)
        one_week_ago = time.time() - (7 * 24 * 60 * 60)  # seconds in a week
        for i in range(len(weather_history) - 1, -1, -1):
            if float(weather_history[i][0]) < one_week_ago:
                del weather_history[i]
        with open(data_filename, 'w') as f:
            writer = csv.writer(f)
            for row in weather_history:
                writer.writerow(row)
        mqtt_data.clear()
    try:
        s3.upload_file(data_filename, os.environ["BUCKET_NAME"], data_filename)
    except Exception as e:
        print(f'An error occurred: {str(e)}')


def main():
    client = connect_mqtt()
    client.on_disconnect = on_disconnect
    client.loop_start()
    weather_history = []
    try:
        with open(data_filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                weather_history.append(row)
    except FileNotFoundError:
        pass

    mqtt_data = defaultdict(list)

    subscribe(client, mqtt_data, weather_history)
    loop = asyncio.get_event_loop()
    try:
        loop.run_forever()
    finally:
        loop.close()


if __name__ == "__main__":
    main()

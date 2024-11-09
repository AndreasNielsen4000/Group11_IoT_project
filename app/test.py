import paho.mqtt.client as mqtt
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# MQTT Broker Configuration
BROKER = "eu1.cloud.thethings.network"  # Your TTN cluster region
PORT = 8883
APP_ID = "an-an-dtu-34365@ttn"  # Replace with your TTN Application ID
ACCESS_KEY = "NNSXS.XKAUT5YGQ6CMQ7XJPX5VSJIOJGZ2WCCRCLO6ECA.FLUZPEZ7SNN6T4QCB32WVLXRNA6HTGQDRX4FFH7PRJESOOUR3CAQ"  # Replace with your TTN Access Key

# Define on_connect callback
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("Connected successfully to MQTT broker")
        client.subscribe("v3/+/devices/+/up")  # Adjust the topic as needed
    else:
        logger.error(f"Failed to connect, return code {rc}")

# Define on_disconnect callback
def on_disconnect(client, userdata, rc):
    logger.warning("Disconnected from MQTT broker with return code %d", rc)

# Define on_message callback
def on_message(client, userdata, msg):
    logger.debug(f"Message received on topic {msg.topic}")
    try:
        payload = msg.payload.decode()
        logger.info(f"Payload: {payload}")
    except Exception as e:
        logger.error("Failed to decode message payload: %s", e)

# Initialize MQTT client with updated API
client = mqtt.Client()
client.username_pw_set(APP_ID, ACCESS_KEY)
client.tls_set()  # Enable TLS for secure connection

# Assign callbacks
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message

# Connect to broker and start the loop
client.connect(BROKER, PORT, 60)
client.loop_forever()  # Starts the network loop for continuous connection

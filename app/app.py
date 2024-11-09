from flask import Flask, jsonify, request
from flask_socketio import SocketIO
from datetime import datetime
import json
import logging
import threading
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import sqlite3
import base64

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


# TTN V3 MQTT Configuration
APP_ID = "an-an-dtu-34365@ttn"  # Replace with your TTN Application ID
MQTT_APP_ID = "an-an-dtu-34365"  # Replace with your TTN Application ID
ACCESS_KEY = "NNSXS.XKAUT5YGQ6CMQ7XJPX5VSJIOJGZ2WCCRCLO6ECA.FLUZPEZ7SNN6T4QCB32WVLXRNA6HTGQDRX4FFH7PRJESOOUR3CAQ"  # Replace with your TTN Access Key
BROKER = "eu1.cloud.thethings.network"  # Replace with your TTN cluster region (e.g., eu1, nam1)
PORT = 8883  # MQTT over TLS
# TOPIC = f"v3/{APP_ID}/devices/+/up"  # Subscribe to all devices for this app
DEVICE_TOPIC = f"v3/{APP_ID}/devices/+/+"
# USER_LOCATION_TOPIC = f"v3/{APP_ID}/devices/user-location/up"
USER_LOCATION_TOPIC = f"v3/{APP_ID}/devices/dummy-device/down/push"


# Initialize MQTT client
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(APP_ID, ACCESS_KEY)
mqtt_client.tls_set()  # Enable TLS for secure connection

# Store received points
points = []

# MQTT connection success callback
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("Connected successfully to MQTT broker")
        client.subscribe([(DEVICE_TOPIC, 0), (USER_LOCATION_TOPIC, 0)])
    else:
        logger.error(f"Failed to connect, return code {rc}")

# MQTT disconnection callback
def on_disconnect(client, userdata, rc):
    logger.warning("Disconnected from MQTT broker with return code %d", rc)

# Start the MQTT client loop in a separate thread
def start_mqtt():
    mqtt_client.connect(BROKER, PORT, 60)
    mqtt_client.loop_forever()

mqtt_thread = threading.Thread(target=start_mqtt)
mqtt_thread.start()

@app.route('/api/points', methods=['GET'])
def get_points():
    return jsonify(points)

# @socketio.on("send_location")
# def handle_send_location(data):
#     data["time"] = datetime.utcnow().isoformat()  # Add server time
#     data["port"] = 1
#     publish_data = {
#         "downlinks": [
#             {
#                 "f_port": data["port"],
#                 "frm_payload": "vu8=",
#                 "priority": "NORMAL"
#             }
#         ]
#     }
#     publish.single(USER_LOCATION_TOPIC, json.dumps(publish_data), BROKER, port=1883, auth={'username': MQTT_APP_ID, 'password': ACCESS_KEY})
#     ispublish = mqtt_client.publish(USER_LOCATION_TOPIC, json.dumps(publish_data))  # Publish to TTN via MQTT
#     ispublish.wait_for_publish()
#     if ispublish.rc == mqtt.MQTT_ERR_SUCCESS:
#         logger.info("MQTT message published successfully")
#     else:
#         logger.error(f"Failed to publish MQTT message, return code {ispublish.rc}")
#     # publish.single(USER_LOCATION_TOPIC, '{"downlinks":[{"f_port": 1,"frm_payload":"vu8=","priority": "NORMAL"}]}', BROKER, port=8883, auth={'username':MQTT_APP_ID,'password':ACCESS_KEY})
#     logger.info(f"Sent location to MQTT: {data}")

@socketio.on("send_location")
def handle_send_location(data):
    data["time"] = datetime.utcnow().isoformat()  # Add server time
    data["port"] = 1
    save_point_to_db(data)
    # Convert data to JSON, then encode it as Base64
    json_data = json.dumps(data)
    print(json_data)
    encoded_data = base64.b64encode(json_data.encode()).decode()  # Base64 encoding

    # encoded_data = base64.b64encode("test".encode()).decode()  # Base64 encoding

    publish_data = {
        "downlinks": [
            {
                "f_port": 2,
                "frm_payload": encoded_data,
                "priority": "NORMAL"
            }
        ]
    }

    # print(encoded_data)
    # print(base64.b64decode(encoded_data).decode())
    # print(json.dumps(publish_data['downlinks'][0]['frm_payload']))
    # print(base64.b64decode(json.dumps(publish_data['downlinks'][0]['frm_payload'])).decode())  

    publish.single(
        USER_LOCATION_TOPIC,
        json.dumps(publish_data),
        hostname=BROKER,
        port=8883,  # Secure MQTT port
        auth={'username': APP_ID, 'password': ACCESS_KEY},
        tls={'ca_certs': None}  # Enable TLS
    )


# Initialize SQLite database
def init_db():
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    # Create a table if it doesn’t already exist
    c.execute('''CREATE TABLE IF NOT EXISTS points (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    time TEXT,
                    latitude REAL,
                    longitude REAL
                )''')
    conn.commit()
    conn.close()

# Function to save points to SQLite
def save_point_to_db(point):
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute('INSERT INTO points (time, latitude, longitude) VALUES (?, ?, ?)', 
              (point['time'], point['location']['lat'], point['location']['lng']))
    conn.commit()
    conn.close()


# Function to handle incoming MQTT messages
def on_message(client, userdata, msg):
    try:
        # Decode the message payload
        payload = json.loads(msg.payload.decode())
        logger.debug(f"Message payload: {payload}")

        # Extract time and location from the payload
        device_time = payload.get("received_at", datetime.utcnow().isoformat())
        location_data = payload.get("uplink_message", {}).get("decoded_payload", {}).get("location", {})

        # Check if location data exists in the payload
        if "lat" in location_data and "lng" in location_data:
            point = {
                "time": device_time,
                "location": {
                    "lat": location_data["lat"],
                    "lng": location_data["lng"]
                }
            }
            logger.info(f"Received new point: {point}")

            # Save to the in-memory list and the database
            points.append(point)
            save_point_to_db(point)

            # Emit new point to WebSocket clients
            socketio.emit("new_point", point)
        else:
            logger.warning("Location data not found in payload")

    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode JSON payload: {e}")
    except Exception as e:
        logger.error(f"Error processing message: {e}")


@app.route('/api/points/history', methods=['GET'])
def get_history():
    # Extract sorting parameters from query string
    sort_by = request.args.get('sort_by', 'time')  # Default to sorting by time
    order = request.args.get('order', 'asc')       # Default to ascending order
    
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    
    # Allow sorting by 'time', 'latitude', or 'longitude'
    if sort_by in ('time', 'latitude', 'longitude') and order in ('asc', 'desc'):
        query = f'SELECT * FROM points ORDER BY {sort_by} {order.upper()}'
    else:
        query = 'SELECT * FROM points ORDER BY time ASC'
    
    c.execute(query)
    rows = c.fetchall()
    conn.close()

    # Format rows for JSON response
    points_history = [{'id': row[0], 'time': row[1], 'latitude': row[2], 'longitude': row[3]} for row in rows]
    return jsonify(points_history)

# Set MQTT client callbacks
mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect
mqtt_client.on_message = on_message

if __name__ == '__main__':
    # Call init_db to initialize the database on startup
    init_db()
    socketio.run(app, host="0.0.0.0", port=5000)
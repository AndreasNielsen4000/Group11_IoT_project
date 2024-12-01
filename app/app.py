from flask import Flask, jsonify, request, send_from_directory, redirect
from flask_socketio import SocketIO
from datetime import datetime
import json
import logging
import threading
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import sqlite3
from pyngrok import ngrok, exception
import base64
import requests
import socket
from queue import Queue
import time

'''
The app.py and index.html files are inspired by previous projects not related to DTU.
They are based on projects created at our workplace (Alexander and Andreas), and the code has been modified to fit the requirements of this project.

Github Copilot has been used to assist with the code in these files.

The functionallity of the app.py file is to create a webserver that can receive data from TTN via MQTT and store the data in a SQLite database.
The data is then displayed on a map in the index.html file.

The index.html file is a simple webpage that displays a map with markers for each data point received from TTN.
It is also possible to filter, export and delete data points from the map and database.
On the webpage, it is also possible to send a location to TTN via MQTT.

To run the app, the following non-standard packages are required:
- Flask
- Flask-SocketIO
- paho-mqtt
- pyngrok
- requests

In addition, a registered Ngrok account is required to run the app in production mode.
Ngrok is used to create a secure tunnel to the local server, this provides HTTPS and a public URL for the webserver.
'''

app = Flask(__name__, static_folder="static", template_folder="templates")
socketio = SocketIO(app, cors_allowed_origins="*")

DEBUG_MODE = True  # Set to True to disable ngrok for debugging

# Attempt to start ngrok and set the WebSocket URL
def get_local_ip():
    # Retrieve the local IP address for fallback
    hostname = socket.gethostname()
    return socket.gethostbyname(hostname)

if not DEBUG_MODE:
    try:
        public_url = ngrok.connect(5000, "http").public_url
        print(" * ngrok tunnel available at:", public_url)
        websocket_url = public_url
    except exception.PyngrokNgrokError:
        print(" * ngrok failed to start, using localhost IP")
        websocket_url = f"http://{get_local_ip()}:5000"
else:
    websocket_url = f"http://{get_local_ip()}:5000"
    print(" * Debug mode enabled, using localhost IP:", websocket_url)

@app.route('/')
def index():
    # Check if the request was made to localhost or the local IP; if so, redirect to the Ngrok URL
    if not DEBUG_MODE and "ngrok" in websocket_url and (request.host.startswith("localhost") or request.host.startswith("127.") or request.host.startswith("192.168")):
        return redirect(websocket_url, code=302)
    else:
        # If already on the Ngrok URL or no Ngrok URL, serve index.html directly from the static folder
        return send_from_directory(app.static_folder, 'index.html')

@app.route('/ws_url')
def ws_url():
    # Serve the WebSocket URL (either ngrok or local IP) as JSON
    return jsonify({'ws_url': websocket_url})

@app.route('/ngrokdir')
def ngrokdir():
    # Redirect to the ngrok URL if available, otherwise redirect to local IP
    return redirect(websocket_url, code=302)

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


# TTN V3 MQTT Configuration
APP_ID = "an-an-dtu-34365@ttn"  # TTN Application ID
ACCESS_KEY = "NNSXS.XKAUT5YGQ6CMQ7XJPX5VSJIOJGZ2WCCRCLO6ECA.FLUZPEZ7SNN6T4QCB32WVLXRNA6HTGQDRX4FFH7PRJESOOUR3CAQ"  # TTN Access Key
BROKER = "eu1.cloud.thethings.network"  # Replace with your TTN cluster region (e.g., eu1, nam1)
PORT = 8883  # MQTT over TLS
DEVICE_TOPIC = f"v3/{APP_ID}/devices/+/+"
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

# Show all points on the map
@app.route('/api/points/all', methods=['GET'])
def get_all_points():
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute('SELECT * FROM points')
    rows = c.fetchall()
    conn.close()

    points = [{'id': row[0], 'time': row[1], 'latitude': row[2], 'longitude': row[3], 'address': row[4]} for row in rows]
    return jsonify(points)

@socketio.on("send_location")
def handle_send_location(data):
    logger.info(f"Received location data: {data}")  # Debug log
    data["time"] = datetime.utcnow().isoformat()  # Add server time
    data["port"] = 1
    save_point_to_db(data)
    # Convert data to JSON, then encode it as Base64
    json_data = json.dumps(data)
    encoded_data = base64.b64encode(json_data.encode()).decode()  # Base64 encoding

    publish_data = {
        "downlinks": [
            {
                "f_port": 2,
                "frm_payload": encoded_data,
                "priority": "NORMAL"
            }
        ]
    }

    logger.info(f"Publishing data to MQTT: {publish_data}")  # Debug log
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
    # Create a table with an address column if it doesn’t already exist
    c.execute('''CREATE TABLE IF NOT EXISTS points (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    time TEXT,
                    latitude REAL,
                    longitude REAL,
                    address TEXT
                )''')
    conn.commit()
    conn.close()


import requests
import logging

logger = logging.getLogger(__name__)

def get_address(lat, lon):
    try:
        # Set a custom user-agent to comply with Nominatim’s usage policy
        headers = {
            "User-Agent": "Checker/1.0 (s176361@dtu.dk)"  # Replace with your app name and email
        }
        
        # Send request with user-agent
        response = requests.get(
            'https://nominatim.openstreetmap.org/reverse',
            params={
                'lat': lat,
                'lon': lon,
                'format': 'json'
            },
            headers=headers
        )
        
        # Check for a successful response
        if response.status_code == 200:
            data = response.json()
            address = data.get("display_name", "Address not found")
            return address
        else:
            logger.error(f"Failed to fetch address, status code: {response.status_code}")
            return "Address not found"
    except Exception as e:
        logger.error(f"Error fetching address: {e}")
        return "Address not found"


# Create a queue for address lookups
address_queue = Queue()

# Define a global lock for database access
db_lock = threading.Lock()

def save_point_to_db(point):
    with db_lock:  # Ensure only one thread accesses the database at a time
        conn = sqlite3.connect('data.db')
        c = conn.cursor()

        # Insert the point with a placeholder address
        c.execute('INSERT INTO points (time, latitude, longitude, address) VALUES (?, ?, ?, ?)', 
                  (point['time'], point['location']['lat'], point['location']['lng'], "Fetching address..."))
        point_id = c.lastrowid
        # Add to address queue for lookup
        address_queue.put((point_id, point['location']['lat'], point['location']['lng']))
        # Emit new point to WebSocket clients
        socketio.emit("new_point", point)
        
        conn.commit()
        conn.close()

def address_worker():
    while True:
        point_id, lat, lon = address_queue.get()
        try:
            address = get_address(lat, lon)
            with db_lock:  # Use the lock when updating the database
                conn = sqlite3.connect('data.db')
                c = conn.cursor()
                c.execute('UPDATE points SET address = ? WHERE id = ?', (address, point_id))
                conn.commit()
                conn.close()
            logger.info(f"Updated address for point {point_id}")
        except Exception as e:
            logger.error(f"Error updating address for point {point_id}: {e}")
        finally:
            address_queue.task_done()
        time.sleep(5)  # Avoid hitting rate limits


def check_for_missing_addresses():
    while True:
        with db_lock:  # Lock the database access
            conn = sqlite3.connect('data.db')
            c = conn.cursor()
            # Find all points with "Fetching address..." as the address
            c.execute('SELECT id, latitude, longitude FROM points WHERE address = "Fetching address..."')
            rows = c.fetchall()
            conn.close()

        # Add each point needing an address to the address queue
        for row in rows:
            point_id, lat, lon = row
            address_queue.put((point_id, lat, lon))
            logger.info(f"Enqueued point {point_id} for address update")

        # Sleep for 30 minutes (1800 seconds) before the next check
        time.sleep(1800)

# Function to handle incoming MQTT messages
def on_message(client, userdata, msg):
    try:
        # Decode the message payload
        payload = json.loads(msg.payload.decode())
        logger.debug(f"Message payload: {payload}")

        # Extract time and location from the payload
        device_time_str = payload.get("uplink_message", {}).get("decoded_payload", {}).get("time", {})
        location_data = payload.get("uplink_message", {}).get("decoded_payload", {}).get("location", {})

        # Convert the timestring to ISO format
        if device_time_str:
            device_time = datetime.strptime(device_time_str, "%Y%m%dT%H%M%S").isoformat()
        else:
            device_time = datetime.utcnow().isoformat()

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
    sort_by = request.args.get('sort_by', 'time_asc')  # Default to sorting by time ascending
    start_date = request.args.get('start_date')
    end_date = request.args.get('end_date')

    conn = sqlite3.connect('data.db')
    c = conn.cursor()

    query = 'SELECT * FROM points'
    params = []

    # Add date filters if specified
    if start_date:
        query += ' WHERE time >= ?'
        params.append(start_date)
    if end_date:
        if 'WHERE' in query:
            query += ' AND time <= ?'
        else:
            query += ' WHERE time <= ?'
        params.append(end_date)

    # Allow sorting by 'time', 'latitude', or 'longitude'
    if sort_by == 'time_asc':
        query += ' ORDER BY time ASC'
    elif sort_by == 'time_desc':
        query += ' ORDER BY time DESC'
    elif sort_by == 'latitude':
        query += ' ORDER BY latitude ASC'
    elif sort_by == 'longitude':
        query += ' ORDER BY longitude ASC'
    else:
        query += ' ORDER BY time ASC'

    c.execute(query, params)
    rows = c.fetchall()
    conn.close()

    # Include address in the JSON response
    points_history = [{'id': row[0], 'time': row[1], 'latitude': row[2], 'longitude': row[3], 'address': row[4]} for row in rows]
    return jsonify(points_history)


@app.route('/api/points/<int:id>', methods=['DELETE'])
def delete_point(id):
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute('DELETE FROM points WHERE id = ?', (id,))
    conn.commit()
    conn.close()
    return '', 204


# Set MQTT client callbacks
mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect
mqtt_client.on_message = on_message

if __name__ == '__main__':
    # Call init_db to initialize the database on startup
    init_db()
    
    # Start the address worker thread to process the address queue
    address_worker_thread = threading.Thread(target=address_worker, daemon=True)
    address_worker_thread.start()
    
    # Start the missing address checker thread
    missing_address_thread = threading.Thread(target=check_for_missing_addresses, daemon=True)
    missing_address_thread.start()

    socketio.run(app, host="0.0.0.0", port=5000)
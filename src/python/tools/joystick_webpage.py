import logging
import os
from logging import handlers

from flask import Flask, render_template
from flask_cors import CORS
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config["SECRET_KEY"] = "secret"
CORS(app)
socketio = SocketIO(app)

log_file = "/var/log/joystick_webpage.log"
# Ensure log directory exists
if not os.path.exists(os.path.dirname(log_file)):
    os.makedirs(os.path.dirname(log_file))

# Set up logging
handler = handlers.RotatingFileHandler(log_file, maxBytes=10000, backupCount=1)
handler.setLevel(logging.INFO)

# Logging format
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)

# Add the handler to the Flask logger and to the root logger
app.logger.addHandler(handler)
logging.getLogger().addHandler(handler)


@app.route("/")
def index() -> None:
    app.logger.error("Serving the webpage.")
    return render_template("index.html")


@socketio.on("connect")
def handle_connect() -> None:
    app.logger.error("Client connected.")
    emit("response", {"message": "Connected successfully!"})


@socketio.on("disconnect")
def handle_disconnect() -> None:
    app.logger.error("Client disconnected.")


@socketio.on("message")
def handle_message(data) -> None:
    app.logger.error(f"Received message: {data}")
    emit("response", {"message": "Message received!"})


@socketio.on_error_default
def default_error_handler(e) -> None:
    app.logger.error(f"ERROR: {e}")


@socketio.on("control", namespace="/control")
def control(message) -> None:
    data = message["data"]
    if "left" in data.keys():
        x = data["left"][0]
        y = data["left"][1]
        app.logger.error(f"[Server] Left: {x=}, {y=}")
    elif "right" in data.keys():
        x = data["right"][0]
        y = data["right"][1]
        app.logger.error("[Server] Right: {x=}, {y=}")
    elif "A" in data.keys():
        app.logger.error("[Server] A, 1, 0")
    elif "B" in data.keys():
        app.logger.error("[Server] B, 1, 0")


if __name__ == "__main__":
    socketio.run(app, host="100.79.164.2", debug=True, use_reloader=False)

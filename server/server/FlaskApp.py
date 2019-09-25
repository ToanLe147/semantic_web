#!/usr/bin/env python
from flask import Flask, render_template, Response
import sys
modpath = '/home/nico/catkin_ws/src/semantic_web/src'
sys.path.insert(0, modpath)
from flask_socketio import SocketIO


app = Flask(__name__)
socketio = SocketIO(app)


@app.route('/')
def index():
    return render_template('index.html', test=test)


@socketio.on('connect')
def handle_my_custom_event():
    global test
    socketio.emit('my response', test, callback="Message Received")


if __name__ == '__main__':
    socketio.run(app, host="192.168.100.16")

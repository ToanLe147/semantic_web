#!/usr/bin/env python
from flask import Flask, render_template
import sys
modpath = '/home/nico/catkin_ws/src/semantic_web/src'
sys.path.insert(0, modpath)
from flask_socketio import SocketIO
from uploader import ontology

app = Flask(__name__)
socketio = SocketIO(app)
KnowledgeBase = ontology()


@app.route('/', methods=['GET', 'POST'])
def index():
    return render_template('index.html')


@socketio.on('handle_instance')
def update_instance(msg):
    name = msg["name"]
    type = msg["type"]
    action = msg["action"]
    print ("{} instance name {} {}".format(action, name, type))
    KnowledgeBase.handle_instance(name, type, action)
    socketio.emit('server_response', "Updated", callback="Message Received")


@socketio.on('handle_data')
def update_instance_data(msg):
    instance = msg["name"]
    property = msg["type"]
    value = msg["value"]
    print ("{} instance type {} {}".format(instance, property, value))
    KnowledgeBase.update_property(instance, property, value)
    socketio.emit('server_response', "Updated", callback="Message Received")


if __name__ == '__main__':
    # socketio.run(app, host="192.168.100.16")
    socketio.run(app, host="localhost")

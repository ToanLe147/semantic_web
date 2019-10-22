#!/usr/bin/env python
# ######### Edit modpath to get correct directory
from flask import Flask, render_template
import sys
modpath = '/home/nico/catkin_ws/src/semantic_web/src'
sys.path.insert(0, modpath)
from flask_socketio import SocketIO
from uploader import Ontology
import roslibpy
from reasoner import Reasoner


app = Flask(__name__)

# Initialize SocketIO
socketio = SocketIO(app)
KnowledgeBase = Ontology()

# Connect with ROS
client = roslibpy.Ros(host='localhost', port=9090)
client.run()

reasoner = Reasoner(client)
camera_scan = reasoner.camera_scan
robot_move = reasoner.robot_move
gripper_grasp = reasoner.gripper_grasp


@app.route('/')
def index():
    return render_template('tasks.html')


@socketio.on('refresh_data')
def load_data():
    data = KnowledgeBase.get_instances()
    if data:
        for i in data:
            socketio.emit("refresh_page", i, callback="Update Page")


@socketio.on('handle_instance')
def update_instance(msg):
    name = msg["name"]
    type = msg["type"]
    action = msg["action"]
    print ("{} instance name {} {}".format(action, name, type))
    new_msg = KnowledgeBase.handle_instance(name, type, action)
    if action == "delete":
        new_msg = "Instance deleted"
    socketio.emit('server_response', new_msg, callback="Message Received")


@socketio.on('handle_data')
def update_instance_data(msg):
    instance = msg["name"]
    property = msg["type"]
    value = msg["value"]
    print ("{} instance type {} {}".format(instance, property, value))
    res = KnowledgeBase.update_property(instance, property, value)
    socketio.emit('server_response', res, callback="Message Received")


@socketio.on('refresh_relationship')
def update_relationship():
    data = KnowledgeBase.get_relationship()
    if data:
        for i in data:
            socketio.emit("refresh_rel_list", i, callback="Update Page")


@socketio.on('handle_relationship')
def handle_relationship(msg):
    data = []
    if len(msg) > 1:
        data = msg["relates"]
        name1 = msg["name1"]
        name2 = msg["name2"]
        res = KnowledgeBase.handle_relationship(data, name1, name2)
    else:
        data = msg["relates"]
        res = KnowledgeBase.handle_relationship(data)
    if data:
        socketio.emit('server_response', res, callback="Message Received")


@socketio.on('learning_trigger')
def learning_trigger(trigger):
    msg = roslibpy.Message({'data': str(trigger["trigger"])})
    camera_scan.publish(msg)
    print("Btn pressed")


@socketio.on('generate_scene')
def generate_scene(msg):
    instance = msg["instance"]
    property = msg["property"]
    update = msg["update"]
    if update == "update_scene_tree":
        data = KnowledgeBase.get_property(instance, property)
        res = eval(data)
        data = {}
        # Later change to call Reasoner.py to generate tasks
        for shape in res:
            # print(shape)
            data["shape"] = shape[0]
            socketio.emit(update, data, callback="Message Received")
    else:
        res = reasoner.generate_task()
        # print(res)
        for shape in res:
            data = res[shape]
            socketio.emit(update, data, callback="Message Received")
        print


@socketio.on('perform_task')
def perform_task(msg):
    reasoner.perform_task()
    print("perform pressed")


if __name__ == '__main__':
    # socketio.run(app, host="192.168.100.16")
    socketio.run(app, host="localhost")

#!/usr/bin/env python
import os
import pygal
from pygal.style import DarkStyle
from flask import Flask, request, render_template, url_for
import roslibpy
# import pathmagic

# Test
from ontology_manager import ontology
# from views import visual, perception


db = ontology()
image = ""
# Flask
app = Flask(__name__)
app.config["CACHE_TYPE"] = "null"
client = roslibpy.Ros(host='localhost', port=9090)
client.run()

talker = roslibpy.Topic(client, '/observe_table', 'std_msgs/String')
ur5_move = roslibpy.Topic(client, '/UR5_command', 'geometry_msgs/Pose')
ur5_move.advertise()
talker.advertise()
listener = roslibpy.Topic(client, '/objects_detected', 'std_msgs/String')
img_listener = roslibpy.Topic(client, '/image_detected', 'std_msgs/String')
listener.advertise()
img_listener.advertise()


def callback(msg):
    db.update_ontology(msg['data'], "INSERT")


def callback_img(msg):
    global image

    image = 'images/'+msg['data']


img_listener.subscribe(callback_img)

# prevent cached responses
if app.config["DEBUG"]:
    @app.after_request
    def after_request(response):
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate, public, max-age=0"
        response.headers["Expires"] = 0
        response.headers["Pragma"] = "no-cache"
        return response


@app.route("/")
def index():
    return render_template("home.html")


@app.route("/perception")
def move_to_knowleadge_base():
    return render_template("actions.html")


@app.route("/visual")
def move_to_visual():
    names = db.get_name()
    graph = pygal.XY(stroke=False, style=DarkStyle, inverse_y_axis=True, range=(0, 1), xrange=(0, 1))
    graph.title = 'Object Position'

    for name in names:
        if name == "object":
            continue
        xmin, ymin, xmax, ymax = db.get_info(name)[name]["status"]
        # x, y, _ = db.get_info(name)[name]["location"]
        # graph.add(name, [{'value': (x, y), 'node': {'r': 6}}])
        graph.add(name, [(None, None), (xmin, ymin), (xmin, ymax), (xmax, ymax), (xmax, ymin), (xmin, ymin), (None, None)], stroke=True, fill=True)

    graph_data = graph.render_data_uri()
    return render_template("visual.html", graph_data=graph_data)


@app.route("/perception/knowleadge", methods=['POST'])
def actions():
    listener.subscribe(callback)
    user = request.form['ref_object']
    btn = request.form['btn_object']
    names = request.form.getlist('handles[]')

    if btn == "Check":
        # Input somethinglike LEFT OF TV
        if user:
            side, _, name = user.split(" ")
            results = db.get_relative_position(name)
            if len(results[side.upper()]) > 0:
                names = results[side.upper()]

    if btn == "Observe":
        talker.publish(roslibpy.Message({'data': 'ok'}))

    if btn == "Update":
        names = db.get_name()

    return render_template("actions.html", names=names, image_detected=image)


@app.route("/perception/knowleadge/<name>", methods=['GET'])
def show_pos(name):
    pos = request.form.getlist('pos[]')
    status = request.form.getlist('status[]')
    names = db.get_name()
    neighbors = db.get_relative_position(name)
    actions = ['pick', 'place']
    pos = db.get_info(name)[name]["location"]
    status = db.get_info(name)[name]["status"]
    return render_template("actions.html", names=names, pos=pos, status=status, actions=actions, selected_object=name, image_detected=image, neighbors=neighbors)


@app.route("/perception/knowleadge/<name>/<action>", methods=['GET'])
def show_action(name, action):
    pos = request.form.getlist('pos[]')
    status = request.form.getlist('status[]')
    names = db.get_name()
    neighbors = db.get_relative_position(name)
    actions = ['pick', 'place']
    pos = db.get_info(name)[name]["location"]
    destination = [0.4, -0.5, 0.1]  # OPtional position
    status = db.get_info(name)[name]["status"]

    if action == "pick":
        ur5_move.publish(roslibpy.Message({"position": {"x": pos[0], "y": pos[1], "z": pos[2]}}))

    if action == 'place':
        ur5_move.publish(roslibpy.Message({"position": {"x": destination[0], "y": destination[1], "z": destination[2]}}))

    return render_template("actions.html", names=names, pos=pos, status=status, actions=actions, selected_object=name, image_detected=image, neighbors=neighbors)


if __name__ == '__main__':
    app.run(host="localhost", port=8080, debug="true")

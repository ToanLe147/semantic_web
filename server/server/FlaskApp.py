#!/usr/bin/env python

from flask import Flask, render_template
import roslibpy


app = Flask(__name__)
client = roslibpy.Ros(host='localhost', port=9090)
client.run()


@app.route('/')
def index():
    return render_template('index.html')


if __name__ == '__main__':
    app.run(host="192.168.100.16")

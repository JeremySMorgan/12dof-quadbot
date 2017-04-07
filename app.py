#!/usr/bin/env python
from flask import Flask,  render_template,  session,  request,  send_from_directory,  send_file
from flask_socketio import SocketIO,  emit,  join_room,  leave_room,  close_room,  rooms,  disconnect
import time
import json
import datetime
import logging
import platform
import os
import sys
from bColors import bcolors
from RobotSystem.Hypervisor import Hypervisor
from RobotSystem.Services.Utilities.RobotUtils import RobotUtils

async_mode = None
app = Flask(__name__,  static_url_path='/static')
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app,  async_mode=async_mode)
log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

thread = None
connections = 0

@app.route('/',  methods=['GET',  'POST'])
def index():
    return render_template('index.html',  async_mode=socketio.async_mode)

def background_thread():
    while True:
        socketio.sleep(1)

@socketio.on('valueUpdate')
def valueUpdateHandler(message):
    RobotUtils.ColorPrinter("app.py",'Value update fired ', 'OKGREEN')
    quadbot.inputData(message)
    data = {}
    data['Recieved'] = True
    return json.dumps(data)


@socketio.on('connect')
def test_connect():
    global connections
    connections+=1
    print_str = "Client connected. "+ str(connections)+  " current connections"
    RobotUtils.ColorPrinter("app.py",print_str, 'OKGREEN')

    global thread, quadbotThread
    if thread is None:
        print "init"
        thread = socketio.start_background_task(target=background_thread)

@socketio.on('disconnect')
def test_disconnect():
    global connections
    connections -= 1
    RobotUtils.ColorPrinter("app.py",str( 'Client disconnected. ' +str(connections)+ " current connections" ), 'OKGREEN')

if __name__ == '__main__':
    global quadbot
    quadbot = Hypervisor()
    try:
        socketio.run(app,  debug=True,use_reloader=False)
    except KeyboardInterrupt:
        RobotUtils.ColorPrinter("app.py", "Server shutting down", 'FAIL')
        quadbot.EndHypervisor()
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

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
import io
from bColors import bcolors
from RobotSystem.Hypervisor import Hypervisor
from RobotSystem.Services.Utilities.RobotUtils import RobotUtils
from threading import Thread

async_mode = None

app = Flask(__name__, static_url_path='/static')

app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app,  async_mode=async_mode)
log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

connections = 0
not_shutdown = True

@app.route('/',  methods=['GET',  'POST'])
def index():
    return render_template('index.html',  async_mode=socketio.async_mode)

def background_thread():
    
    if RobotUtils.VIDEO_STEAMING and RobotUtils.LIVE_TESTING:
        import base64
        import picamera

        with picamera.PiCamera() as c:
            #c = picamera.PiCamera()
            c.resolution = (200,200)
            #c.framerate = 80
            c.hflip = False
            c.vflip = True
            
            time.sleep(2)     
            now = time.time()   
            sum_t = 0
            taken = 1
            print "loop starting"
            while True:
                if connections > 0:
                    

                    c.capture('image.png',use_video_port=True)
                    
                    with file('image.png') as f:
                        data = f.read()
                        socketio.emit('image',{'image':True,'buffer':data.encode('base64')})       
                        #elapsed_t = time.time() - now                     
                        #sum_t += elapsed_t
                        #ave = sum_t / taken
                        #print "image sent in: ",elapsed_t,"s\t\tave:",ave," s"
                        #taken += 1
                        #now = time.time()


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

@socketio.on('disconnect')
def test_disconnect():
    global connections
    connections -= 1
    RobotUtils.ColorPrinter( "app.py", str( 'Client disconnected. ' + str(connections)+ " current connections" ), 'OKGREEN' )

if __name__ == '__main__':
    global quadbot
    quadbot = Hypervisor()
    
    #command = "TURN"
    #quadbot.testSuite(command)
    
    try:
        global thread
        thread = Thread(target = background_thread)
        thread.start()
        socketio.run(app, host='0.0.0.0',  debug=True,use_reloader=False)
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        RobotUtils.ColorPrinter("app.py", "Server shutting down", 'FAIL')
        socketio.stop()
        not_shutdown = False
        quadbot.endHypervisor()
        try: 
            sys.exit(0)
        except SystemExit:
            os._exit(0)
    

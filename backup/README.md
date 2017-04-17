# ProjectPrism

## Description
Autonomous and tele-operated quad-bot, with live streaming video and thermal imaging footage

## Hardware
The robot is made of 3d prints designed using AutoCAD for Mac 2016. The frame hold 12 Standard Servo motors which are powered through an external power supply which is regulated by the DROK® LM2596 Digital Control Voltage Regulator DC Buck Converter. The servos are driven by Adafruit's  16-Channel PWM / Servo HAT .<br>
Software runs off a Raspberry Pi 2

## Web Interface
 Flask server  with  socket.io  python plugin provide client <-> server communication over  websocket protocol .<br>
Front end presented with  bootstrap ,  jQuery  and  noioslider .<br>
User input gathered with  nouislider.js , and  GamepadAPI  for ps3/xbox controller input

## Robot Controls
Flask server creates Robot object which acts as executive of the system. <br>
Robot.py  takes user input and controls Legs  <br>
Leg.py  takes commands from Robot class and commands motors <br>
Motor.py commands individual Servos <br>
RobotUtil.py  holds universal variables and static methods <br>
pwm.py | I2c.py  provide  i2c interface  <br>


## User Input Schema

JS in webpage polls user input every 15 milliseconds and sends input after every 750 milliseconds (average time for robot to execute segment of step/turn), if input exists. Flask server reads incoming data, and adds it to a queue stored in the Robot class. An agenda thread in the robot class polls the queue every 15 milliseconds looking for data to interpret - if new data is found, the queue is dequeued and the resulting command (forward, backward, left,right, stop, autonomous) is executed.

## todo
<ul>
  <li>Xbox Kinect on robot</li>
  <li>Onboard IMU on robot </li>
  <li>Force Sensors on feet of robot </li>
  <li>Allow users to input data through keyboard</li>
  <li>~~the web interface should only send significant data - currently the js polls the gamecontroller/sliders every x milliseconds and sends data, regardless of whether of the data will be useful to the robot. instead, the js should only send on value change, and the robot will understand nothing has changed without new data~~</li>
</ul>

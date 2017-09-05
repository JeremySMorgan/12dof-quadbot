# ProjectPrism

## Description
12 dof quadruped, controlled through web interface with live streaming video

## Hardware
The robot is made of 3d prints designed using AutoCAD for Mac 2016. The frame hold 12 Standard Servo motors which are powered through an external power supply which is regulated by the DROK® LM2596 Digital Control Voltage Regulator DC Buck Converter. The servos are driven by Adafruit's  16-Channel PWM / Servo HAT .<br>
Software runs off a Raspberry Pi

## Web Interface
Flask server  with  socket.io  python plugin provide client <-> server communication over  websocket protocol .<br>
Front end presented with  bootstrap ,  jQuery  and  noioslider .<br>
User input gathered with  nouislider.js , and  GamepadAPI  for ps3/xbox controller input

## Robot Software Structure
Uses a microservice achitecture. Robot is controlled by a Hypervisor which delegates tasks to each service.

## todo
<ul>
  <li>Onboard IMU on robot </li>
  <li>Force Sensors on feet of robot </li>
</ul>

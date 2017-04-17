#!/usr/bin/python
# -*- coding: utf-8 -*-

from RobotSystem.Services.Utilities.RobotUtils import RobotUtils

class LaserDriver(object):

    def __init__(self,laser_pin,laser_enabled,print_color):
        
        self.laser_pin = laser_pin
        
        self.firing = False 
        
        self.laser_enabled = laser_enabled
        
        self.print_color = print_color
        
        self.GPIO = None
            
        if self.laser_enabled:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            
        self.setup()
        
    def setup(self):
        if self.laser_enabled:
            self.GPIO.setmode(self.GPIO.BCM)
            self.GPIO.setwarnings(True)
            self.GPIO.setup(self.laser_pin,self.GPIO.OUT)
    
    def On(self):
        self.firing = True 
        if self.laser_enabled:
            self.GPIO.output(self.laser_pin,self.GPIO.HIGH)
            RobotUtils.ColorPrinter(self.__class__.__name__,'Laser HIGH', self.print_color)
            
        else:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,'Laser simultation HIGH', self.print_color)

    def Off(self):
        self.firing = False 
        if self.laser_enabled:
            self.GPIO.output(self.laser_pin,self.GPIO.LOW)
            RobotUtils.ColorPrinter(self.__class__.__name__,'Laser LOW', self.print_color)
            
        else:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,'Laser simultation LOW', self.print_color)

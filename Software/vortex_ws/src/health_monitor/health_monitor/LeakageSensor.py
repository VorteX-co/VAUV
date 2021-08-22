#!/usr/bin/env python

# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.


import Jetson.GPIO as GPIO
import time

class LeakageSensor:
    def __init__(self):
        self.leak_pin = 22   #Board pin
        GPIO.setmode(GPIO.BOARD)  
        GPIO.setup(self.leak_pin, GPIO.IN)  # set pin as an input pin
        
    def takeValues(self):
        value = GPIO.input(Leak_pin)
        if value == GPIO.HIGH:
                value_str = "HIGH"
        else:
                value_str = "LOW"
        print("Value read from pin is {}".format(value_str))
        return value_str
    
    def cleanUP(self):
        GPIO.cleanUP(self.leak_pin)
    
# def main():
#     leakage_Sensor = LeakageSensor()
#     leakage_Sensor.takeValues()
#     leakage_Sensor.cleanUP()
    
# if __name__== '__main__':
#     main()
    
    
    
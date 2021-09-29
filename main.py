# ////////////////////////////////////////////////////////////////
# //                     IMPORT STATEMENTS                      //
# ////////////////////////////////////////////////////////////////

import math
import sys
import time

from kivy.app import App
from kivy.lang import Builder
from kivy.core.window import Window
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.button import Button
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import *
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.uix.slider import Slider
from kivy.uix.image import Image
from kivy.uix.behaviors import ButtonBehavior
from kivy.clock import Clock
from kivy.animation import Animation
from functools import partial
from kivy.config import Config
from kivy.core.window import Window
from kivy.properties import ObjectProperty
from pidev.kivy import DPEAButton
from pidev.kivy import PauseScreen
from time import sleep
import RPi.GPIO as GPIO
from pidev.stepper import stepper
from pidev.Cyprus_Commands import Cyprus_Commands_RPi as cyprus
from threading import Thread

# ////////////////////////////////////////////////////////////////
# //                      GLOBAL VARIABLES                      //
# //                         CONSTANTS                          //
# ////////////////////////////////////////////////////////////////
START = True
STOP = False
UP = False
DOWN = True
ON = True
OFF = False
YELLOW = .180, 0.188, 0.980, 1
BLUE = 0.917, 0.796, 0.380, 1
CLOCKWISE = 0
COUNTERCLOCKWISE = 1
ARM_SLEEP = 2.5
DEBOUNCE = 0.10

lowerTowerPosition = 60
upperTowerPosition = 76


# ////////////////////////////////////////////////////////////////
# //            DECLARE APP CLASS AND SCREENMANAGER             //
# //                     LOAD KIVY FILE                         //
# ////////////////////////////////////////////////////////////////
class MyApp(App):

    def build(self):
        self.title = "Robotic Arm"
        return sm


Builder.load_file('main.kv')
Window.clearcolor = (.1, .1, .1, 1)  # (WHITE)

cyprus.open_spi()

# ////////////////////////////////////////////////////////////////
# //                    SLUSH/HARDWARE SETUP                    //
# ////////////////////////////////////////////////////////////////

sm = ScreenManager()
arm = stepper(port=0, speed=10)


# ////////////////////////////////////////////////////////////////
# //                       MAIN FUNCTIONS                       //
# //             SHOULD INTERACT DIRECTLY WITH HARDWARE         //
# ////////////////////////////////////////////////////////////////

class MainScreen(Screen):
    version = cyprus.read_firmware_version()
    armPosition = 0
    arm_direction = 0
    lastClick = time.clock()
    armControl = ObjectProperty(None)
    magnetControl = ObjectProperty(None)

    def __init__(self, **kwargs):
        super(MainScreen, self).__init__(**kwargs)
        self.initialize()

    def debounce(self):
        processInput = False
        currentTime = time.clock()
        if (currentTime - self.lastClick) > DEBOUNCE:
            processInput = True
        self.lastClick = currentTime
        return processInput

    def toggleArm(self):
        if self.arm_status:
            compare = 0
            self.armControl.text = "Lower Arm"
        else:
            compare = 50000
            self.armControl.text = "Raise Arm"
        self.cyprus.set_pwm_values(1, period_value=100000,
                                   compare_value=compare,
                                   compare_mode=self.cyprus.LESS_THAN_OR_EQUAL)
        self.arm_status = 1 - self.arm_status

    def toggleMagnet(self):
        self.cyprus.set_servo_position(2, self.electromagnet_status)
        self.electromagnet_status = .5 - self.electromagnet_status
        if self.electromagnet_status == .5:
            self.magnetControl.text = "Release Ball"
        else:
            self.magnetControl.text = "Hold Ball"

    def auto(self):
        print("Run the arm automatically here")

    def moveArmWithThread(self, direction):
        Thread(target=lambda: self.moveArm(direction)).start()

    def moveArm(self, text):
        if text == "CCW":
            direction = 0
        else:
            direction = 1

        self.is_s0_moving = 1
        self.s0.go_until_press(direction, 7500)
        while True:
            if not self.is_s0_moving:
                self.s0.hardStop()
            sleep(.075)

    def stopArm(self):
        self.is_s0_moving = 0

    def homeArm(self):
        arm.home(self.homeDirection)

    def isBallOnTallTower(self):
        print("Determine if ball is on the top tower")

    def isBallOnShortTower(self):
        print("Determine if ball is on the bottom tower")


    def reset(self):
        # Talon on port 2.
        # Must be 0 or .5
        self.electromagnet_status = .5
        self.toggleMagnet()

        # Stepper
        # self.is_s0_moving = 0
        self.s0.hardStop()

        # 1 = up and 0 = down
        # Initially, the arm defaults to staying up. By setting self.arm_status = 1, the next time
        # the arm toggles, it will go down.
        self.arm_status = 1
        self.toggleArm()



    def initialize(self):
        print("Home arm and turn off magnet")
        self.cyprus = cyprus
        self.cyprus.initialize()

        self.s0 = stepper(port=0, micro_steps=32, hold_current=20, run_current=20, accel_current=20, deaccel_current=20,
                          steps_per_unit=200, speed=2)

        # It really doesn't matter which motor controller you use in this case. Both act as switches.

        # Cytron Motor Controller, which is connected to a device that electronically presses a button that controlls air pressure.
        # Use set_pwm_values. 0 means that the arm is raised and 50000 means that it's lowered.
        self.cyprus.setup_servo(1)

        # Talon Motor Controller, which is connected to the electromagnet.
        # Use set_servo_position. 1 and 0 mean on and .5 means off.
        # Will use 0 and .5 to easily toggle between states without a condition.
        self.cyprus.setup_servo(2)

        self.reset()




    def resetColors(self):
        self.ids.armControl.color = YELLOW
        self.ids.magnetControl.color = YELLOW
        self.ids.auto.color = BLUE

    def quit(self):
        self.stopArm()
        MyApp().stop()


sm.add_widget(MainScreen(name='main'))

# ////////////////////////////////////////////////////////////////
# //                          RUN APP                           //
# ////////////////////////////////////////////////////////////////

MyApp().run()
cyprus.close_spi()

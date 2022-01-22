#!/usr/bin/env python3

import os
import signal
import threading
import logging
import traceback
import pathlib
import socket
import fcntl
import struct

import roslaunch
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from slam_toolbox_msgs.srv import SerializePoseGraph, DeserializePoseGraph

from flask import Flask, render_template, make_response, jsonify

from driver import Driver
from supervisorstate import SupervisorState

from luma.core.render import canvas
from luma.core.interface.serial import i2c
from luma.emulator.device import pygame
from luma.oled.device import ssd1306
from PIL import ImageFont

class Supervisor:

    def __init__(self):
        self.app = None
        self.wsport = None
        self.wsinterface = None
        self.nodelaunchfile = None
        self.mapname = 'map01'

        self.state = SupervisorState()

        self.driver = None

        self.processwakeup = False
        self.processshutdown = False

        self.displaydevice = None
        self.font8 = None

    def startWebServer(self):
        rospy.loginfo('Starting supervisor at %s:%s', self.wsinterface, self.wsport)
        self.app.run(host=self.wsinterface, port=self.wsport)

    def deliverStart(self):
        return make_response(render_template('index.html'), 200)

    def wakeup(self):
        self.state.syncLock.acquire()

        self.driver.stop()

        data = {
        }

        rospy.loginfo('Request to wakeup')
        self.processwakeup = True

        self.state.syncLock.release()
        return make_response(jsonify(data), 200)

    def systemstate(self):
        self.state.syncLock.acquire()

        data = self.state.gathersystemstate()

        self.state.syncLock.release()

        return make_response(jsonify(data), 200)

    def shutdown(self):
        self.state.syncLock.acquire()

        self.driver.stop()

        data = {
        }

        rospy.loginfo('Request to shutdown')
        self.processshutdown = True

        self.state.syncLock.release()
        return make_response(jsonify(data), 200)

    def turnleft(self):

        data = {
        }

        self.driver.drive(.0, .4)
        self.state.lastcommand = 'turnleft'
        return make_response(jsonify(data), 200)

    def turnright(self):

        data = {
        }

        self.driver.drive(.0, -.4)
        self.state.lastcommand = 'turnright'
        return make_response(jsonify(data), 200)

    def forward(self):

        data = {
        }

        self.driver.drive(.20, .0)
        self.state.lastcommand = 'forward'
        return make_response(jsonify(data), 200)

    def stop(self):

        data = {
        }

        self.driver.stop()
        self.state.lastcommand = 'stop'
        return make_response(jsonify(data), 200)

    def updateDisplay(self):
        device = self.displaydevice
        state = self.state.gathersystemstate()
        with canvas(device) as draw:
            draw.rectangle(device.bounding_box, outline='white', fill='black')
            draw.text((3, 1), 'IP: ' + self.wsinterface + ':' + str(self.wsport), fill='white', font=self.font8)
            if state['awake']:
                device.contrast(255)
                draw.text((3, 11), 'Bat: ' + str(state['batteryCharge']) + '/' + str(state['batteryCapacity']) + ' mAH', fill='white', font=self.font8)
            else:
                device.contrast(128)
                draw.text((3, 11), 'Sleeping...', fill='white', font=self.font8)

    def ipforinterface(self, ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname.encode('utf_8'))
        )[20:24])

    def bindinginterface(self, default):
        try:
            return str(self.ipforinterface('wlan0'))
        except Exception as e:
            try:
                return str(self.ipforinterface('eth0'))
            except Exception as e2:
                return default

    def restorestateformap(self):
        rospy.loginfo('Loading pose graph %s from file...', self.mapname)
        try:
            rospy.loginfo('Waiting for service...')
            rospy.wait_for_service('/slam_toolbox/deserialize_map')
            rospy.loginfo('Calling...')
            service = rospy.ServiceProxy('/slam_toolbox/deserialize_map', DeserializePoseGraph)

            service(self.mapname, int(8), Pose2D())
        except Exception as e:
            logging.error(traceback.format_exc())
            rospy.logerr('Error loading pose graph : %s', e)

        rospy.loginfo('Done')

    def savestateformap(self):
        try:
            rospy.loginfo('Saving pose graph %s to file...', self.mapname)
            rospy.loginfo('Waiting for service')
            rospy.wait_for_service('/slam_toolbox/serialize_map')
            rospy.loginfo('Calling...')
            service = rospy.ServiceProxy('/slam_toolbox/serialize_map', SerializePoseGraph)
            service(self.mapname)
        except Exception as e:
            logging.error(traceback.format_exc())
            rospy.logerr('Error saving pose graph : %s', e)

        rospy.loginfo('Done')

    def start(self):
        rospy.init_node('supervisor', anonymous=True)

        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '40'))
        self.wsport = int(rospy.get_param('~wsport', '8080'))
        self.wsinterface = self.bindinginterface('0.0.0.0')

        staticContentFolder = str(pathlib.Path(__file__).parent.resolve().joinpath('web'))
        rospy.loginfo('Using %s as static content folder', staticContentFolder)

        self.nodelaunchfile = str(pathlib.Path(__file__).parent.resolve().parent.joinpath('launch', rospy.get_param('~launchfile', 'slamtoolbox.launch')))
        rospy.loginfo('Using %s as the launch file for nodes', self.nodelaunchfile)

        rospy.loginfo('Checking system state with %s hertz', pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size=10))

        self.app = Flask(__name__, static_url_path='', static_folder=staticContentFolder, template_folder=staticContentFolder)
        self.app.add_url_rule('/', view_func=self.deliverStart)
        self.app.add_url_rule('/index.html', view_func=self.deliverStart)
        self.app.add_url_rule('/index.htm', view_func=self.deliverStart)

        self.app.add_url_rule('/systemstate.json', view_func=self.systemstate)
        self.app.add_url_rule('/actions/wakeup', view_func=self.wakeup)
        self.app.add_url_rule('/actions/shutdown', view_func=self.shutdown)
        self.app.add_url_rule('/actions/turnleft', view_func=self.turnleft)
        self.app.add_url_rule('/actions/turnright', view_func=self.turnright)
        self.app.add_url_rule('/actions/forward', view_func=self.forward)
        self.app.add_url_rule('/actions/stop', view_func=self.stop)

        wsThread = threading.Thread(target=self.startWebServer)
        wsThread.start()

        rospy.Subscriber("batteryCharge", Int16, self.state.newBatteryCharge)
        rospy.Subscriber("batteryCapacity", Int16, self.state.newBatteryCapacity)
        rospy.Subscriber("bumperLeft", Int16, self.state.newBumperLeft)
        rospy.Subscriber("bumperRight", Int16, self.state.newBumperRight)
        rospy.Subscriber("wheeldropLeft", Int16, self.state.newWheeldropLeft)
        rospy.Subscriber("wheeldropRight", Int16, self.state.newWheeldropRight)

        rospy.Subscriber("lightBumperLeft", Int16, self.state.newLightBumperLeft)
        rospy.Subscriber("lightBumperFrontLeft", Int16, self.state.newLightBumperFrontLeft)
        rospy.Subscriber("lightBumperCenterLeft", Int16, self.state.newLightBumperCenterLeft)
        rospy.Subscriber("lightBumperCenterRight", Int16, self.state.newLightBumperCenterRight)
        rospy.Subscriber("lightBumperFrontRight", Int16, self.state.newLightBumperFrontRight)
        rospy.Subscriber("lightBumperRight", Int16, self.state.newLightBumperRight)

        rospy.Subscriber("odom", Odometry, self.state.newOdometry)

        odometrylogfilename = str(pathlib.Path(__file__).parent.resolve().parent.joinpath('maps', 'odometry.txt'))
        rospy.loginfo('Writing odometry debug to %s', odometrylogfilename)
        odometrylogfile = open(odometrylogfilename, 'w')
        self.state.initOdometryLog(odometrylogfile)

        rospy.loginfo('Initializing physical display')
        font0path = str(pathlib.Path(__file__).resolve().parent.joinpath('ttf', 'C&C Red Alert [INET].ttf'))
        self.font8 = ImageFont.truetype(font0path, 12)

        if rospy.get_param('~displaytype', 'demo') == 'demo':
            rospy.loginfo('Using emulator')
            self.displaydevice = pygame()
        else:
            i2cport = int(rospy.get_param('~i2cport', '1'))
            rospy.loginfo('Using i2c connection to ssd1306 on port %s', i2cport)
            interface = i2c(port=i2cport, address=0x3C)
            self.displaydevice = ssd1306(interface)

        displayUpdateCounter = 0

        # Processing the sensor polling in an endless loop until this node goes to die
        rospy.loginfo('Starting main loop')
        while not rospy.is_shutdown():
            self.state.syncLock.acquire()

            displayUpdateCounter = (displayUpdateCounter + 1) % pollingRateInHertz
            if displayUpdateCounter == 0:
                self.updateDisplay()

            if self.processwakeup and self.state.robotnode is None:
                try:
                    rospy.loginfo('Starting new node with launch file %s', self.nodelaunchfile)
                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    self.state.robotnode = roslaunch.parent.ROSLaunchParent(uuid, [self.nodelaunchfile])
                    self.state.robotnode.start()

                    # self.restorestateformap()
                except Exception as e:
                    self.state.robotnode = None

                    logging.error(traceback.format_exc())
                    rospy.logerr('Error spawning nodes : %s', e)

                self.processwakeup = False

            if self.processshutdown and self.state.robotnode is not None:

                # self.savestateformap()

                try:
                    rospy.loginfo('Shutting down running node')
                    self.state.robotnode.shutdown()
                except Exception as e:
                    logging.error(traceback.format_exc())
                    rospy.logerr('Error shutting down node : %s', e)

                self.processshutdown = False
                self.state.robotnode = None
                self.latestbatterycapacity = None
                self.latestbatterycharge = None


            self.state.syncLock.release()
            rate.sleep()

        rospy.loginfo('Clearing status display')
        self.displaydevice.clear()

        rospy.loginfo('Closing odometry log')
        odometrylogfile.close()

        rospy.loginfo('Suicide to stop all services')

        os.kill(os.getpid(), signal.SIGKILL)

if __name__ == '__main__':
    try:
        supervisor = Supervisor()
        supervisor.start()
    except rospy.ROSInterruptException:
        pass

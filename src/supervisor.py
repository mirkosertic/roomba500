#!/usr/bin/env python3
import math
import os
import signal
import threading
import logging
import traceback
import pathlib
import socket
import fcntl
import struct
import time
import shutil

import tf
import roslaunch
import rospy

from std_srvs.srv import Empty
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Pose2D, Point32, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud

from flask import Flask, render_template, make_response, jsonify

from driver import Driver
from supervisorstate import SupervisorState

from luma.core.render import canvas
from luma.core.interface.serial import i2c
from luma.emulator.device import pygame
from luma.oled.device import ssd1306
from PIL import ImageFont

from roomba500.msg import RoombaSensorFrame, NavigationInfo
from roomba500.srv import Clean, CleanResponse


class Supervisor:

    def __init__(self):
        self.mapframe = None
        self.app = None
        self.wsport = None
        self.wsinterface = None
        self.nodelaunchfile = None
        self.roomname = 'map01'

        self.state = None

        self.driver = None

        self.processwakeup = False
        self.processshutdown = False

        self.displaydevice = None
        self.font8 = None

        self.forwardspeed = .2
        self.rotationspeed = .6  # 0.4 is bare minimum

        self.roomsdirectory = None

        self.bumperspub = None
        self.transformlistener = None

    def startWebServer(self):
        rospy.loginfo('Starting supervisor at %s:%s', self.wsinterface, self.wsport)
        self.app.run(host=self.wsinterface, port=self.wsport)

    def deliverStart(self):
        return make_response(render_template('index.html'), 200)

    def startnewroom(self):
        self.state.syncLock.acquire()

        self.roomname = str(round(time.time() * 1000))

        mapfile = str(pathlib.Path(self.roomsdirectory).joinpath(self.roomname))
        os.mkdir(mapfile)

        self.driver.stop()

        data = {
        }

        rospy.loginfo('Request to wakeup in new room %s', self.roomname)
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

    def command(self, velx, veltheta):
        self.driver.drive(velx, veltheta)

    def turnleft(self):

        data = {
        }

        self.command(.0, self.rotationspeed)
        return make_response(jsonify(data), 200)

    def turnright(self):

        data = {
        }

        self.command(.0, -self.rotationspeed)
        return make_response(jsonify(data), 200)

    def forward(self):

        data = {
        }

        self.command(self.forwardspeed, .0)
        return make_response(jsonify(data), 200)

    def forwardleft(self):

        data = {
        }

        self.command(self.forwardspeed, self.rotationspeed / 2)
        return make_response(jsonify(data), 200)

    def forwardright(self):

        data = {
        }

        self.command(self.forwardspeed, -(self.rotationspeed / 2))
        return make_response(jsonify(data), 200)

    def backward(self):

        data = {
        }

        self.command(-self.forwardspeed, .0)
        return make_response(jsonify(data), 200)

    def stop(self):

        data = {
        }

        self.command(.0, .0)
        return make_response(jsonify(data), 200)

    def relocalization(self):

        data = {
        }

        self.driver.stop()

        try:
            rospy.loginfo('Waiting for service global_localization')
            rospy.wait_for_service('global_localization')
            rospy.loginfo('Invoking service')
            service = rospy.ServiceProxy('global_localization', Empty)
            service()
            rospy.loginfo('Ready to rumble!')
        except rospy.ServiceException as e:
            logging.error(traceback.format_exc())
            rospy.logerr('Error initiating relocalization : %s', e)

        return make_response(jsonify(data), 200)

    def clean(self):

        data = {
        }

        servicename = 'clean'
        rospy.loginfo('Waiting for service %s', servicename)
        rospy.wait_for_service(servicename)
        rospy.loginfo('Invoking service')
        service = rospy.ServiceProxy(servicename, Clean)
        response = service()
        rospy.loginfo('Service called!')

        return make_response(jsonify(data), 200)

    def cancel(self):

        data = {
        }

        servicename = 'cancel'
        rospy.loginfo('Waiting for service %s', servicename)
        rospy.wait_for_service(servicename)
        rospy.loginfo('Invoking service')
        service = rospy.ServiceProxy(servicename, Clean)
        response = service()
        rospy.loginfo('Service called!')

        return make_response(jsonify(data), 200)

    def deleteroom(self, room):

        rospy.loginfo('Deleting room %s', room)

        self.state.syncLock.acquire()

        directory = str(pathlib.Path(self.roomsdirectory).joinpath(room))
        shutil.rmtree(directory)

        data = {
        }

        self.state.syncLock.release()
        return make_response(jsonify(data), 200)

    def startroom(self, room):

        rospy.loginfo('Starting room %s', room)
        self.state.syncLock.acquire()

        self.driver.stop()

        data = {
        }

        self.processwakeup = True
        self.roomname = room

        self.state.syncLock.release()
        return make_response(jsonify(data), 200)

    def simulateObstacle(self):

        rospy.loginfo('Simulating obstacle')
        self.state.syncLock.acquire()

        data = {
        }

        try:
            distance = 0.5
            dx = math.cos(self.state.latestyawonmap) * distance
            dy = math.sin(self.state.latestyawonmap) * distance

            msg = PointCloud()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.points.append(Point32(self.state.latestpositiononmap.x + dx, self.state.latestpositiononmap.y + dy, 0.05))

            self.bumperspub.publish(msg)

        except Exception as e:
            rospy.logerr(traceback.format_exc())
            rospy.logerr('Error generating obstacle : %s', e)

        self.state.syncLock.release()

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
                draw.text((3, 22), 'Dist: ' + "{:.2f}".format(state['distanceToTargetInMeters']) + ' m / ' + "{:.2f}".format(state['angleToTargetInDegrees']) + ' deg', fill='white', font=self.font8)
                draw.text((3, 33), 'Nav: ' + str(state['currentWaypoint']) + ' / ' + str(state['numWaypoints']), fill='white', font=self.font8)
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

    def savestateformap(self):
        try:
            mapfile = str(pathlib.Path(self.roomsdirectory).joinpath(self.roomname, 'map'))

            rospy.loginfo('Saving map %s to file %s...', self.roomname, mapfile)

            node = roslaunch.core.Node('map_server', 'map_saver', args='-f ' + mapfile)
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            rospy.loginfo('Launching save process and waiting to finish')
            process = launch.launch(node)
            time.sleep(5)
            process.stop()

            latestposfilename = str(pathlib.Path(self.roomsdirectory).joinpath(self.roomname, 'latest.position'))
            rospy.loginfo('Saving latest position to %s', latestposfilename)
            handle = open(latestposfilename, 'w')
            handle.write('{:.6f}'.format(self.state.latestpositiononmap.x) + '\n')
            handle.write('{:.6f}'.format(self.state.latestpositiononmap.y) + '\n')
            handle.write('{:.6f}'.format(self.state.latestyawonmap) + '\n')
            handle.flush()
            handle.close()

        except Exception as e:
            logging.error(traceback.format_exc())
            rospy.logerr('Error saving map : %s', e)

        rospy.loginfo('Done')

    def start(self):
        rospy.init_node('supervisor', anonymous=True)

        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '5'))
        self.wsport = int(rospy.get_param('~wsport', '8080'))
        self.wsinterface = self.bindinginterface('0.0.0.0')

        staticContentFolder = str(pathlib.Path(__file__).parent.resolve().joinpath('web'))
        rospy.loginfo('Using %s as static content folder', staticContentFolder)

        self.nodelaunchfile = str(pathlib.Path(__file__).parent.resolve().parent.joinpath('launch', rospy.get_param('~launchfile', 'highlevelsimulation.launch')))
        rospy.loginfo('Using %s as the launch file for nodes', self.nodelaunchfile)

        rospy.loginfo('Checking system state with %s hertz', pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.roomsdirectory = str(pathlib.Path(__file__).parent.resolve().parent.joinpath('rooms'))
        rospy.loginfo('Using %s as the rooms directory', self.roomsdirectory)

        self.mapframe = rospy.get_param('~mapframe', 'map')
        self.transformlistener = tf.TransformListener()
        self.state = SupervisorState(self.transformlistener, self.mapframe, self.roomsdirectory)

        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size=10))

        self.bumperspub = rospy.Publisher('bumpers', PointCloud, queue_size=10)

        self.app = Flask(__name__, static_url_path='', static_folder=staticContentFolder, template_folder=staticContentFolder)
        self.app.add_url_rule('/', view_func=self.deliverStart)
        self.app.add_url_rule('/index.html', view_func=self.deliverStart)
        self.app.add_url_rule('/index.htm', view_func=self.deliverStart)

        self.app.add_url_rule('/systemstate.json', view_func=self.systemstate)

        self.app.add_url_rule('/actions/startnewroom', view_func=self.startnewroom)
        self.app.add_url_rule('/actions/shutdown', view_func=self.shutdown)
        self.app.add_url_rule('/actions/turnleft', view_func=self.turnleft)
        self.app.add_url_rule('/actions/turnright', view_func=self.turnright)
        self.app.add_url_rule('/actions/forwardleft', view_func=self.forwardleft)
        self.app.add_url_rule('/actions/forward', view_func=self.forward)
        self.app.add_url_rule('/actions/forwardright', view_func=self.forwardright)
        self.app.add_url_rule('/actions/stop', view_func=self.stop)
        self.app.add_url_rule('/actions/backward', view_func=self.backward)
        self.app.add_url_rule('/actions/relocalization', view_func=self.relocalization)
        self.app.add_url_rule('/actions/clean', view_func=self.clean)
        self.app.add_url_rule('/actions/cancel', view_func=self.cancel)
        self.app.add_url_rule('/actions/room/<room>/delete', view_func=self.deleteroom)
        self.app.add_url_rule('/actions/room/<room>/start', view_func=self.startroom)
        self.app.add_url_rule('/actions/simulateObstacle', view_func=self.simulateObstacle)

        wsThread = threading.Thread(target=self.startWebServer)
        wsThread.start()

        shutdownTopic = rospy.Publisher('shutdown', Int16, queue_size=10)

        rospy.Subscriber("sensorframe", RoombaSensorFrame, self.state.newSensorFrame)
        rospy.Subscriber("odom", Odometry, self.state.newOdometry)
        rospy.Subscriber("cmd_vel", Twist, self.state.newCmdVel)
        rospy.Subscriber("navigation_info", NavigationInfo, self.state.newNavigationInfo)

        rospy.loginfo('Initializing physical display')
        font0path = str(pathlib.Path(__file__).resolve().parent.joinpath('ttf', 'C&C Red Alert [INET].ttf'))
        self.font8 = ImageFont.truetype(font0path, 12)

        if rospy.get_param('~displaytype', 'demo') == 'demo':
            rospy.loginfo('Using display emulator')
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

                    latestposfilename = str(pathlib.Path(self.roomsdirectory).joinpath(self.roomname, 'latest.position'))

                    arguments = []
                    if os.path.exists(latestposfilename):
                        rospy.loginfo('Loading latest position from %s', latestposfilename)
                        handle = open(latestposfilename, 'r')
                        x = float(handle.readline())
                        y = float(handle.readline())
                        yaw = float(handle.readline())
                        handle.close()

                        rospy.loginfo('Latest position is x=%s, y=%s, yaw=%s', x, y, yaw)

                        mapfile = str(pathlib.Path(self.roomsdirectory).joinpath(self.roomname, 'map.yaml'))
                        rospy.loginfo('Using map file %s', mapfile)

                        arguments.append('initialx:=' + str(x))
                        arguments.append('initialy:=' + str(y))
                        arguments.append('initialyaw:=' + str(yaw))
                        arguments.append('loadmap:=true')
                        arguments.append('createmap:=false')
                        arguments.append('mapfile:=' + mapfile)
                    else:
                        rospy.loginfo('Using a new map which will be stored as %s', self.roomname)
                        arguments.append('loadmap:=false')
                        arguments.append('createmap:=true')

                    roombalog = str(pathlib.Path(self.roomsdirectory).joinpath(self.roomname, 'roombalog.txt'))
                    arguments.append('roombalog:=' + roombalog)

                    debugimage = str(pathlib.Path(self.roomsdirectory).joinpath(self.roomname, 'debug.png'))
                    arguments.append('debugimagelocation:=' + debugimage)

                    rospy.loginfo('Launching with arguments %s', str(arguments))
                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)

                    self.state.ready = False

                    self.state.robotnode = roslaunch.parent.ROSLaunchParent(uuid, [(self.nodelaunchfile, arguments)])
                    self.state.robotnode.start()

                    # If the high level service is ready, we are ready to go!
                    rospy.wait_for_service('cancel')

                    self.state.ready = True

                except Exception as e:
                    self.state.robotnode = None

                    logging.error(traceback.format_exc())
                    rospy.logerr('Error spawning nodes : %s', e)

                self.processwakeup = False

            if self.processshutdown and self.state.robotnode is not None:

                # We publish the shutdown command
                # This requests a shutdown of subscribing nodes (Odometry, Highlevel etc.)
                shutdownTopic.publish(Int16(1))

                self.savestateformap()

                try:
                    rospy.loginfo('Shutting down running node')
                    self.state.robotnode.shutdown()
                except Exception as e:
                    logging.error(traceback.format_exc())
                    rospy.logerr('Error shutting down node : %s', e)

                self.processshutdown = False
                self.state.ready = False
                self.state.robotnode = None
                self.state.latestbatterycapacity = None
                self.state.latestbatterycharge = None

            self.state.syncLock.release()
            rate.sleep()

        rospy.loginfo('Clearing status display')
        self.displaydevice.clear()

        rospy.loginfo('Suicide to stop all services')

        os.kill(os.getpid(), signal.SIGKILL)


if __name__ == '__main__':
    try:
        supervisor = Supervisor()
        supervisor.start()
    except rospy.ROSInterruptException:
        pass

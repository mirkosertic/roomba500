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
import asyncio
import json

import tf
import roslaunch
import rospy
import actionlib
import rosservice

from std_srvs.srv import Empty
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Pose2D, Point32, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import PointCloud
from rosgraph_msgs.msg import Log
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import MarkerArray

from tf.transformations import quaternion_from_euler

from starlette.applications import Starlette
from starlette.responses import JSONResponse, FileResponse
from sse_starlette.sse import EventSourceResponse
import uvicorn

from driver import Driver
from supervisorstate import SupervisorState

from luma.core.render import canvas
from luma.core.interface.serial import i2c
from luma.emulator.device import pygame
from luma.oled.device import ssd1306
from PIL import ImageFont

from roomba500.msg import RoombaSensorFrame, NavigationInfo, Area
from roomba500.srv import Clean, CleanResponse


class Supervisor:

    def __init__(self):
        self.mapframe = None
        self.app = None
        self.wsport = None
        self.wsinterface = None
        self.nodelaunchfile = None
        self.roomname = 'map01'
        self.staticfileswebdir = None

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

        self.movebaseclient = None

        self.sidebrushpub = None
        self.mainbrushpub = None
        self.vacuumpub = None

    def startWebServer(self):
        rospy.loginfo('Starting supervisor at %s:%s', self.wsinterface, self.wsport)
        uvicorn.run(self.app, host=self.wsinterface, port=self.wsport, debug=False, access_log=False, reload=False)

    def deliverStart(self, req):
        return FileResponse(self.staticfileswebdir + '/index.html')

    def deliverDummyJS(self, req):
        return FileResponse(self.staticfileswebdir + '/dummy.js')

    def startnewroom(self, req):
        self.state.syncLock.acquire()

        self.roomname = str(round(time.time() * 1000))

        mapfile = str(pathlib.Path(self.roomsdirectory).joinpath(self.roomname))
        os.mkdir(mapfile)

        self.driver.stop()

        rospy.loginfo('Request to wakeup in new room %s', self.roomname)
        self.processwakeup = True

        self.state.syncLock.release()

        return JSONResponse({})

    def shutdown(self, req):
        self.state.syncLock.acquire()

        self.driver.stop()

        rospy.loginfo('Request to shutdown')
        self.processshutdown = True

        self.state.syncLock.release()

        return JSONResponse({})

    def command(self, velx, veltheta):
        self.driver.drive(velx, veltheta)

    def turnleft(self, req):

        self.command(.0, self.rotationspeed)

        return JSONResponse({})

    def turnright(self, req):

        self.command(.0, -self.rotationspeed)

        return JSONResponse({})

    def forward(self, req):

        self.mainbrushpub.publish(Int16(127))
        self.sidebrushpub.publish(Int16(127))
        self.vacuum.publish(Int16(127))

        self.command(self.forwardspeed, .0)

        return JSONResponse({})

    def backward(self, req):

        self.command(-self.forwardspeed, .0)

        return JSONResponse({})

    def stop(self, req):

        self.mainbrushpub.publish(Int16(0))
        self.sidebrushpub.publish(Int16(0))
        self.vacuum.publish(Int16(0))

        self.command(.0, .0)
        return JSONResponse({})

    def relocalization(self, req):

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

        return JSONResponse({})

    async def logstream(self, req):
        queue = asyncio.Queue()
        self.state.loggingqueues.append(queue)

        async def eventpublisher():
            try:
                while True:
                    disconnected = await req.is_disconnected()
                    if disconnected:
                        rospy.loginfo("Disconnecting client %s", req.client)
                        self.state.loggingqueues.remove(queue)
                        break
                    data = await queue.get()
                    yield json.dumps(data)
            except asyncio.CancelledError as e:
                rospy.loginfo("Disconnected from client %s (via refresh/close)", req.client)
                self.state.loggingqueues.remove(queue)
                raise e
        return EventSourceResponse(eventpublisher())

    async def mapstream(self, req):
        queue = asyncio.Queue()
        self.state.mapqueues.append(queue)

        async def eventpublisher():
            try:
                while True:
                    disconnected = await req.is_disconnected()
                    if disconnected:
                        rospy.loginfo("Disconnecting client %s", req.client)
                        self.state.mapqueues.remove(queue)
                        break
                    data = await queue.get()
                    yield json.dumps(data)
            except asyncio.CancelledError as e:
                rospy.loginfo("Disconnected from client %s (via refresh/close)", req.client)
                self.state.mapqueues.remove(queue)
                raise e
        return EventSourceResponse(eventpublisher())

    async def costmapstream(self, req):
        queue = asyncio.Queue()
        self.state.costmapqueues.append(queue)

        async def eventpublisher():
            try:
                while True:
                    disconnected = await req.is_disconnected()
                    if disconnected:
                        rospy.loginfo("Disconnecting client %s", req.client)
                        self.state.costmapqueues.remove(queue)
                        break
                    data = await queue.get()
                    yield json.dumps(data)
            except asyncio.CancelledError as e:
                rospy.loginfo("Disconnected from client %s (via refresh/close)", req.client)
                self.state.costmapqueues.remove(queue)
                raise e
        return EventSourceResponse(eventpublisher())

    async def statestream(self, req):
        queue = asyncio.Queue()
        self.state.statequeues.append(queue)

        async def eventpublisher():
            try:
                while True:
                    disconnected = await req.is_disconnected()
                    if disconnected:
                        rospy.loginfo("Disconnecting client %s", req.client)
                        self.state.statequeues.remove(queue)
                        break
                    data = await queue.get()
                    yield json.dumps(data)
            except asyncio.CancelledError as e:
                rospy.loginfo("Disconnected from client %s (via refresh/close)", req.client)
                self.state.statequeues.remove(queue)
                raise e
        return EventSourceResponse(eventpublisher())

    async def clean(self, req):

        cmd = await req.json()

        servicename = 'clean'
        rospy.loginfo('Waiting for service %s', servicename)
        rospy.wait_for_service(servicename)
        rospy.loginfo('Invoking service')

        cleanarea = Area()
        cleanarea.mapTopLeftX = cmd['topX']
        cleanarea.mapTopLeftY = cmd['topY']
        cleanarea.mapBottomRightX = cmd['bottomX']
        cleanarea.mapBottomRightY = cmd['bottomY']

        service = rospy.ServiceProxy(servicename, Clean)
        service(cleanarea)
        rospy.loginfo('Service called!')

        return JSONResponse({})

    def cancel(self, req):

        servicename = 'cancel'
        rospy.loginfo('Waiting for service %s', servicename)
        rospy.wait_for_service(servicename)
        rospy.loginfo('Invoking service')
        service = rospy.ServiceProxy(servicename, Clean)
        service()
        rospy.loginfo('Service called!')

        return JSONResponse({})

    def deleteroom(self, req):

        room = req.path_params['room']
        rospy.loginfo('Deleting room %s', room)

        self.state.syncLock.acquire()

        directory = str(pathlib.Path(self.roomsdirectory).joinpath(room))
        shutil.rmtree(directory)

        self.state.syncLock.release()

        return JSONResponse({})

    def startroom(self, req):

        room = req.path_params['room']

        rospy.loginfo('Starting room %s', room)
        self.state.syncLock.acquire()

        self.driver.stop()

        self.processwakeup = True
        self.roomname = room

        self.state.syncLock.release()

        return JSONResponse({})

    def simulateObstacle(self, req):

        rospy.loginfo('Simulating obstacle')
        self.state.syncLock.acquire()

        try:
            distance = 0.5
            dx = math.cos(self.state.latestodomonmap["theta"]) * distance
            dy = math.sin(self.state.latestodomonmap["theta"]) * distance

            msg = PointCloud()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.points.append(Point32(self.state.latestodomonmap["x"] + dx, self.state.latestodomonmap["y"] + dy, 0.05))

            self.bumperspub.publish(msg)

        except Exception as e:
            rospy.logerr(traceback.format_exc())
            rospy.logerr('Error generating obstacle : %s', e)

        self.state.syncLock.release()

        return JSONResponse({})

    async def moveToPosition(self, req):

        cmd = await req.json()

        rospy.loginfo("Received move to command %s", str(cmd))

        if not self.movebaseclient.wait_for_server(rospy.Duration(1.0)):
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            q = quaternion_from_euler(0, 0, cmd['theta'])
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = cmd['targetx']
            goal.target_pose.pose.position.y = cmd['targety']
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]

            self.state.latestcleaningpath = {}
            self.movebaseclient.send_goal(goal)

        return JSONResponse({})

    def updateDisplay(self, state):
        device = self.displaydevice
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
        except Exception as e1:
            try:
                return str(self.ipforinterface('wifi0'))
            except Exception as e2:
                try:
                    return str(self.ipforinterface('eth0'))
                except Exception as e3:
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
            handle.write('{:.6f}'.format(self.state.latestodomonmap["x"]) + '\n')
            handle.write('{:.6f}'.format(self.state.latestodomonmap["y"]) + '\n')
            handle.write('{:.6f}'.format(self.state.latestodomonmap["theta"]) + '\n')
            handle.flush()
            handle.close()

        except Exception as e:
            logging.error(traceback.format_exc())
            rospy.logerr('Error saving map : %s', e)

        rospy.loginfo('Done')

    def start(self):
        rospy.init_node('supervisor', anonymous=True)

        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '2'))
        self.wsport = int(rospy.get_param('~wsport', '8080'))
        self.wsinterface = self.bindinginterface('0.0.0.0')

        self.staticfileswebdir = str(pathlib.Path(__file__).parent.resolve().joinpath('web'))
        rospy.loginfo('Using %s as static content folder', self.staticfileswebdir)

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

        self.app = Starlette()
        self.app.add_route('/', self.deliverStart, methods=['GET'])
        self.app.add_route('/index.html', self.deliverStart, methods=['GET'])
        self.app.add_route('/index.htm', self.deliverStart, methods=['GET'])
        self.app.add_route('/dummy.js', self.deliverDummyJS, methods=['GET'])

        self.app.add_route('/logstream', self.logstream, methods=['GET'])
        self.app.add_route('/mapstream', self.mapstream, methods=['GET'])
        self.app.add_route('/costmapstream', self.costmapstream, methods=['GET'])
        self.app.add_route('/statestream', self.statestream, methods=['GET'])

        self.app.add_route('/actions/startnewroom', self.startnewroom, methods=['GET'])
        self.app.add_route('/actions/shutdown', self.shutdown, methods=['GET'])
        self.app.add_route('/actions/turnleft', self.turnleft, methods=['GET'])
        self.app.add_route('/actions/turnright', self.turnright, methods=['GET'])
        self.app.add_route('/actions/forward', self.forward, methods=['GET'])
        self.app.add_route('/actions/stop', self.stop, methods=['GET'])
        self.app.add_route('/actions/backward', self.backward, methods=['GET'])
        self.app.add_route('/actions/relocalization', self.relocalization, methods=['GET'])
        self.app.add_route('/actions/clean', self.clean, methods=['POST'])
        self.app.add_route('/actions/cancel', self.cancel, methods=['GET'])
        self.app.add_route('/actions/room/{room}/delete', self.deleteroom, methods=['GET'])
        self.app.add_route('/actions/room/{room}/start', self.startroom, methods=['GET'])
        self.app.add_route('/actions/simulateObstacle', self.simulateObstacle, methods=['GET'])
        self.app.add_route('/actions/moveTo', self.moveToPosition, methods=['POST'])

        wsThread = threading.Thread(target=self.startWebServer)
        wsThread.start()

        shutdownTopic = rospy.Publisher('shutdown', Int16, queue_size=10)

        rospy.Subscriber("sensorframe", RoombaSensorFrame, self.state.newSensorFrame)
        rospy.Subscriber("odom", Odometry, self.state.newOdometry)
        rospy.Subscriber("cmd_vel", Twist, self.state.newCmdVel)
        rospy.Subscriber("navigation_info", NavigationInfo, self.state.newNavigationInfo)
        rospy.Subscriber("map", OccupancyGrid, self.state.newMap)
        rospy.Subscriber("move_base/local_costmap/costmap", OccupancyGrid, self.state.newCostMap)
        rospy.Subscriber("cleaningpath", Path, self.state.newCleaningPath)
        rospy.Subscriber("cleaningmap", MarkerArray, self.state.newCleaningMap)

        rospy.Subscriber("rosout_agg", Log, self.state.newLogMessage)

        self.mainbrushpub = rospy.Publisher('cmd_mainbrush', Int16, queue_size=10)
        self.sidebrushpub = rospy.Publisher('cmd_sidebrush', Int16, queue_size=10)
        self.vacuum = rospy.Publisher('cmd_vacuum', Int16, queue_size=10)

        # We need the client for move base
        rospy.loginfo("Connecting to move_base action server")
        self.movebaseclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

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
            # self.displaydevice = None

        displayUpdateCounter = 0

        # Processing the sensor polling in an endless loop until this node goes to die
        rospy.loginfo('Starting main loop')
        while not rospy.is_shutdown():

            self.state.syncLock.acquire()

            systemstate = self.state.gathersystemstate()
            for q in self.state.statequeues:
                q.put_nowait(systemstate)

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

            displayUpdateCounter = (displayUpdateCounter + 1) % pollingRateInHertz
            if displayUpdateCounter == 0 and self.displaydevice is not None:
                self.updateDisplay(systemstate)
                self.state.knownservices = rosservice.get_service_list()

        rate.sleep()

        rospy.loginfo('Clearing status display')
        if self.displaydevice is not None:
            self.displaydevice.clear()

        rospy.loginfo('Suicide to stop all services')

        os.kill(os.getpid(), signal.SIGKILL)

if __name__ == '__main__':
    try:
        supervisor = Supervisor()
        supervisor.start()
    except rospy.ROSInterruptException:
        pass

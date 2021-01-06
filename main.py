#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
gyro = GyroSensor(Port.S3)
leftlm = Motor(Port.B, Direction.COUNTERCLOCKWISE)
rightlm = Motor(Port.A, Direction.COUNTERCLOCKWISE)
robot = DriveBase(leftlm, rightlm, 62.4, 105)
# Write your program here.
ev3.speaker.beep()

x=0
y=0

def gyroDrive(angle, distance, speed=180):
    global x
    global y
    start = robot.distance()
    kp = 1
    ki = 0.005
    kd = 0.4
    lastError = 0
    derivative = 0
    integral = 0
    while abs(robot.distance()-start)+15<distance:
        error = (gyro.angle()-angle)*-1
        integral += error
        steering = (kp*error + ki*integral + kd*derivative)
        robot.drive(speed, steering)
        lastError = error
        derivative = error - lastError
    robot.stop()

def startPoint(startCoordinates, robotAngle):
    global x
    global y
    global startx
    global starty

    x = startCoordinates[0]
    y = startCoordinates[1]
    gyro.reset_angle(robotAngle)


def waypoint(targetCoordinates, robotAngle, backwards=False):
    global x
    global y
    if targetCoordinates[0]>x and targetCoordinates[1]>y:
        targetAngle = (math.atan(abs(targetCoordinates[0]-x)/abs(targetCoordinates[1]-y)))*180/math.pi
        calcType = 1
    elif targetCoordinates[0]>x and targetCoordinates[1]<y:
        targetAngle = (math.atan(abs(targetCoordinates[1]-y)/abs(targetCoordinates[0]-x)))*180/math.pi+90
        calcType = 2
    elif targetCoordinates[0]<x and targetCoordinates[1]>y:
        targetAngle = (math.atan(abs(targetCoordinates[0]-x)/abs(targetCoordinates[1]-y)))*-180/math.pi
        calcType = 3
    elif targetCoordinates[0]<x and targetCoordinates[1]<y:
        targetAngle = ((math.atan(abs(targetCoordinates[1]-y)/abs(targetCoordinates[0]-x)))*-180/math.pi)-90
        calcType = 4
    elif targetCoordinates[0]==x and targetCoordinates[1]>y:
        targetAngle = 0
        calcType = 5
    elif targetCoordinates[0]==x and targetCoordinates[1]<y:
        targetAngle = 180
        calcType = 6
    elif targetCoordinates[0]>x and targetCoordinates[1]==y:
        targetAngle = 90
        calcType = 7
    else:
        targetAngle = 270
        calcType = 8
    
    if not backwards:
        a=1
        angleModifier=0
        targetAngle1=targetAngle+360
        targetAngle2=targetAngle-360
    else:
        a=-1
        angleModifier=180
        targetAngle+=180
        targetAngle1 = targetAngle+540
        targetAngle2 = targetAngle-180

    if abs(gyro.angle()-targetAngle)< abs(gyro.angle()-targetAngle1) and abs(gyro.angle()-targetAngle) < abs(gyro.angle()-targetAngle2):
        turnAngle=targetAngle
        angleModifier+=0

    elif abs(gyro.angle()-targetAngle1) < abs(gyro.angle()-targetAngle) and abs(gyro.angle()-targetAngle1)< abs(gyro.angle()-targetAngle2):
        turnAngle=targetAngle1
        angleModifier+=360

    else:
        turnAngle=targetAngle2
        angleModifier-=360

    if gyro.angle()<turnAngle:
        while gyro.angle()<turnAngle-5:
            rightlm.run(-87.5)
            leftlm.run(87.5)
        rightlm.stop()
        leftlm.stop()
    
    else:
        while gyro.angle()>turnAngle+5:
            rightlm.run(87.5)
            leftlm.run(-87.5)
        rightlm.stop()
        leftlm.stop()

    distance = math.sqrt((targetCoordinates[0]-x)**2+(targetCoordinates[1]-y)**2)*a

    if calcType == 1:
        def angleCalc(xLength, yLength):
            angleCorrection=(math.atan(abs(xLength)/abs(yLength)))*180/math.pi+angleModifier
            return angleCorrection

    elif calcType == 2:
        def angleCalc(xLength, yLength):
            angleCorrection=(math.atan(abs(yLength)/abs(xLength)))*180/math.pi+90+angleModifier
            return angleCorrection
    elif calcType == 3:
        def angleCalc(xLength, yLength):
            angleCorrection=(math.atan(abs(xLength)/abs(yLength)))*-180/math.pi+angleModifier
            return angleCorrection
    elif calcType == 4:
        def angleCalc(xLength, yLength):
            angleCorrection=(math.atan(abs(yLength)/abs(xLength)))*-180/math.pi-90+angleModifier
            return angleCorrection
    elif calcType == 5:
        def angleCalc(xLength, yLength):
            angleCorrection=0+angleModifier
            return angleCorrection
    elif calcType == 6:
        def angleCalc(xLength, yLength):
            angleCorrection=180+angleModifier
            return angleCorrection
    elif calcType == 7:
        def angleCalc(xLength, yLength):
            angleCorrection=90+angleModifier
            return angleCorrection
    elif calcType == 8:
        def angleCalc(xLength, yLength):
            angleCorrection=270+angleModifier
            return angleCorrection

    if backwards==False:
        kp = 1
        ki = 0.005
        kd = 0.4
        lastError = 0
        derivative = 0
        integral = 0
        start = robot.distance()
        while robot.distance()-start<distance:
            xAdd = math.sin(math.radians(gyro.angle()))*(robot.distance()-start)
            yAdd = math.cos(math.radians(gyro.angle()))*(robot.distance()-start)
            error = gyro.angle()-angleCalc(targetCoordinates[0]-(x+xAdd), targetCoordinates[1]-(y+yAdd))
            integral+=error
            steering = -1*error*kp+integral*ki+derivative*kd
            robot.drive(190*a, steering)
            lastError = error
            derivative = lastError-error
        robot.stop()
    else:
        kp = 1
        ki = 0.005
        kd = 0.4
        lastError = 0
        derivative = 0
        integral = 0
        start = robot.distance()
        while robot.distance()-start>distance:
            xAdd = math.sin(math.radians(gyro.angle()))*(robot.distance()-start)
            yAdd = math.cos(math.radians(gyro.angle()))*(robot.distance()-start)
            error = gyro.angle()-angleCalc(targetCoordinates[0]-(x+xAdd), targetCoordinates[1]-(y+yAdd))
            integral+=error
            steering = -(error*kp+integral*ki+derivative*kd)
            robot.drive(190*a, steering)
            lastError = error
            derivative = lastError-error
        robot.stop()

    x += xAdd
    y += yAdd

    if gyro.angle()<robotAngle:
        while gyro.angle()<robotAngle-4.5:
            leftlm.run(87.5)
            rightlm.run(-87.5)
        rightlm.stop()
        leftlm.stop()
    else:
        while gyro.angle()>robotAngle+4.5:
            leftlm.run(-87.5)
            rightlm.run(87.5)
        rightlm.stop()
        leftlm.stop()

    robot.reset()


    print(x)
    print(y)

startPoint((121, -1024), 0)
waypoint((607, -401), 0)
waypoint((451, -207), -45)
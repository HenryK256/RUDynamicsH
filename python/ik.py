import sys, time, math
import numpy as np


sys.path.append('../lib')
from unitree_actuator_sdk import *
# all units in meters and radians

class Motor():
    def __init__(self, id, serial):
        self._serial = serial
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.data.motorType = MotorType.GO_M8010_6
        self.cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        self.cmd.id = id
        self.cmd.dq = 0.0
        self.cmd.kp = 0.0
        self.cmd.kd = 0.01
        self.cmd.tau = 0.0

        self.theta = 0

    def moveToPos(self, theta):
        self.theta = theta
        self.cmd.q = self.theta * queryGearRatio(MotorType.GO_M8010_6) / (2 * math.pi)
        self._serial.sendRecv(self.cmd, self.data)


def generatePath():
    t = list(np.linspace(0, 2*3.14, 100))
    bias = 5
    radius = 0.5
    xp = [radius*math.cos(x) + 5 for x in t]
    yp = [radius*math.sin(x) + 5 for x in t]
    return xp, yp

def ik_planar(L2,L3,y, z, knee_sign=+1):
    """Closed-form IK for 2-link in y-z plane → (theta2, theta3) (link angles)."""
    r2 = y*y + z*z
    c  = (r2 - L2*L2 - L3*L3) / (2.0*L2*L3)
    if c < -1.0 or c > 1.0:
        raise ValueError("unreachable")
    s  = knee_sign * math.sqrt(max(0.0, 1.0 - c*c))
    th3 = math.atan2(s, c)
    phi = math.atan2(z, y)
    psi = math.atan2(L3*math.sin(th3), L2 + L3*math.cos(th3))
    th2 = phi - psi
    return th2, th3

def check_joint_constraints(t1,t2):
    if abs(t1) > 130 or abs(t2) > 130:
        return False
    return True

def check_c_space(l1, l2, x, y):
    p = np.array((x,y))
    #case 1: (x,y) are too far
    if np.linalg.norm(p - np.array((0,0))) > l1+l2:
        return False
    #case 2: (x,y) are too close
    elif np.linalg.norm(p - np.array((0,0))) < l2-l1:
        return False
    return True

def funct(x, y):  # Takes in two coordinates => outputs two thetas
    sec1, sec2 = 7.5, 12.9  # Measured lengths of leg segments
    theta0 = math.acos((sec1 * sec1 + x * x + y * y - sec2 * sec2) / (2 * sec1 * math.sqrt(x * x + y * y))) - math.atan(
        x / y)
    theta1 = math.acos((sec1 * sec1 + sec2 * sec2 - x * x - y * y) / (2 * sec1 * sec2))
    return theta0, theta1

def main():
    serial = SerialPort('/dev/ttyUSB0')

    motor0 = Motor(0, serial)
    motor1 = Motor(1, serial)

    pos_x, pos_y = generatePath()
    l1 = 7.5
    l2 = 12.9
    for x, y in zip(pos_x, pos_y):
        t1, t2 = ik_planar(l1, l2, x, y)
        if check_c_space(l1, l2, x, y) and check_joint_constraints(t1, t2):
            motor0.moveToPos(t1)
            motor1.moveToPos(t2)
        else:
            print("bound exceeded check path")
            exit()
        time.sleep(0.01)

def main1():
    serial = SerialPort('/dev/ttyUSB0')

    motor= Motor(0,serial)

    motor.moveToPos(0)
    for i in range(0,60):
        motor.moveToPos(i)

if __name__ == "__main__":
    main1()

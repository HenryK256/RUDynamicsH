import sys, time, math
sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')

IDS = [0, 1]

CMDS = [MotorCmd() for _ in IDS]
DATAS = [MotorData() for _ in IDS]

# If gear ratio is 6.33:1, then 1 * queryGearRatio(MotorType.A1) is 1 rotation
# In order to implement trig functions, we will use a new multiplier TRIGGEAR
# For this model, motor ID 0 is the hip and ID 1 is the knee

TRIGGEAR = queryGearRatio(MotorType.A1) / (2 * math.pi)
thetas = [0.0, 0.0]

for id, cmd, data in IDS, CMDS, DATAS:
    data.motorType = MotorType.A1
    cmd.motorType = MotorType.A1
    cmd.id = id

while True:
    for id, cmd, data in IDS, CMDS, DATAS:
        cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
        cmd.q    = 0.0
        cmd.dq   = theta[id] * TRIGGEAR
        cmd.kp   = 0.0
        cmd.kd   = 0.01
        cmd.tau  = 0.0
    serial.sendRecv(cmd, data)
    print('\n')
    print("theta: " + str(theta[id]))
    print('\n')
    time.sleep(0.1) # 100 ms

# The coordinate system used here has (0, 0) with the knee joint at 90 degrees and
# The hip joint 45 degrees from vertical to the left
# The units are currently arbitrary until measurements of the leg are given

sec1, sec2 = 1, 1 # Assuming they each have a unit length of 1 for now

def funct(x, y): # Takes in two coordinates => outputs two thetas
    theta[0] = math.acos((sec1*sec1+x*x+y*y-sec2*sec2)/(2*sec1*math.sqrt(x*x+y*y))) - math.atan(x/y)
    theta[1] = math.acos((sec1*sec1+sec2*sec2-x*x-y*y)/(2*sec1*sec2))

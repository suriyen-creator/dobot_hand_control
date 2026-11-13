"""A fake vendor module for exercising the real SDK subprocess boundary."""
import time
pose = [200.,0.,50.,0.]
index = 0
pump = False
port_name = ''

class PTPMode:
    PTPMOVLXYZMode = 2

def load(): return object()
def ConnectDobot(api, port, baud):
    global port_name
    port_name = port
    return [1 if port == 'MISSING' else 0]
def DisconnectDobot(api): pass
def SetCmdTimeout(api, ms): pass
def SetQueuedCmdForceStopExec(api): pass
def SetQueuedCmdClear(api): pass
def SetQueuedCmdStartExec(api): pass
def SetPTPCommonParams(api, v, a, isQueued=0): pass
def SetPTPCoordinateParams(api, xyzVelocity, xyzAcceleration, rVelocity, rAcceleration, isQueued=0): pass
def GetPose(api): return pose + [0.,0.,0.,0.]
def SetPTPCmd(api, mode, x, y, z, r, isQueued=1):
    global pose,index
    if port_name == 'HANG': time.sleep(5)
    pose = [x,y,z,r]
    index += 1
    return [index]
def GetQueuedCmdCurrentIndex(api): return [index]
def SetEndEffectorSuctionCup(api, enabled, on, isQueued=0):
    global pump
    pump = on
def GetEndEffectorSuctionCup(api): return [True,pump]

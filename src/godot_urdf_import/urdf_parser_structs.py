from typing import List


class JointLimits:
    def __init__(self):
        self.enabled = False
        self.lower = 0.0
        self.upper = 0.0
        self.effort = 0.0
        self.velocity = 0.0
   

class JointDynamics:
    def __init__(self):
        self.enabled = False
        self.damping = 0.0
        self.friction = 0.0


class JointMimic:
    def __init__(self):
        self.enabled = False
        self.joint = ""
        self.multiplier = 0.0
        self.offset = 0.0

class JointSafetyController:
    def __init__(self):
        self.enabled = False
        self.k_position = 0.0
        self.k_velocity = 0.0
        self.soft_lower_limit = 0.0
        self.soft_upper_limit = 0.0


class JointCalibration:
    def __init__(self):
        self.enabled = False
        self.reference_position = 0.0
        self.rising = 0.0
        self.falling = 0.0


class UrdfPose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0



class JointData:
    def __init__(self):
        self.name = ""
        self.type = ""
        self.parent = ""
        self.child = ""
        self.axis = [0.0, 0.0, 0.0]
        self.origin = UrdfPose()
        self.limit = JointLimits()
        self.dynamics = JointDynamics()
        self.safety_controller = JointSafetyController()
        self.calibration = JointCalibration()
        self.mimic = JointMimic()
        self.transmission : List[JointTransmission] = []



class Material:
    def __init__(self):
        self.name = ""
        self.rgba = [0.0, 0.0, 0.0, 0.0]
        self.texture_filename = ""
  

class Geometry:
    def __init__(self):
        self.type = ""
        self.data = {}
        self.material = Material()
   

class LinkCollision:
    def __init__(self):
        self.name = ""
        self.origin = UrdfPose()
        self.geometry = Geometry()
        self.verbose = ""
  


class LinkVisual:
    def __init__(self):
        self.name = ""
        self.origin = UrdfPose()
        self.geometry = Geometry()
        self.material = Material()
        

class Inertia:
    def __init__(self):
        self.ixx = 0.0
        self.ixy = 0.0
        self.ixz = 0.0
        self.iyy = 0.0
        self.iyz = 0.0
        self.izz = 0.0


class LinkInertial:
    def __init__(self):
        self.mass = 0.0
        self.inertia = Inertia()
        self.origin = UrdfPose()
   

class LinkData:
    def __init__(self):
        self.name = ""
        self.parent = ""
        self.inertial = LinkInertial()
        self.visuals : List[LinkVisual] = []
        self.collisions : List[LinkCollision] = []


class JointTransmission:
    def __init__(self):
        self.name = ""
        self.hardwareInterface : List[str] = []


class JointActuator:
    def __init__(self):
        self.name = ""
        self.mechanicalReduction = 0.0
        self.hardwareInterface : List[str] = []


class Transmission:
    def __init__(self):
        self.name = ""
        self.type = ""
        self.joints : List[JointTransmission] = []
        self.actuators : List[JointActuator] = []




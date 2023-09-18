from typing import TypedDict

class Pose: 
    def __init__(self):
        self.pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.relative_to = ""

class CameraImage:
    def __init__(self):
        self.width = 320
        self.height = 240
        self.format = "R8G8B8"
        self.anti_aliasing = 4

class CameraClip:
    def __init__(self):
        self.enabled = False
        self.near = 0.1
        self.far = 100.0

class CameraSave:
    def __init__(self):
        self.enabled = False
        self.path = ""
 

class CamearDepth:
    def __init__(self):
        self.enabled = False
        self.output = "depth"
        self.clip = CameraClip()

        self.clip.enabled = True
        self.clip.near = 0.1
        self.clip.far = 10.0
   

class SensorNoise:
    def __init__(self):
        self.enabled = False
        self.type = "gaussian"
        self.mean = 0.0
        self.stddev = 0.0


class CameraDistortion:
    def __init__(self):
        self.enabled = False
        self.type = ""
        self.k1 = 0.0
        self.k2 = 0.0
        self.k3 = 0.0
        self.p1 = 0.0
        self.p2 = 0.0
        self.center = [0.5, 0.5]


class LensCustomFunc:
    def __init__(self):
        self.enabled = False
        self.c1 = 1.0
        self.c2 = 1.0
        self.c3 = 0.0
        self.f = 1.0
        self.fun = "tan"
 

class LensIntrinsic:
    def __init__(self):
        self.enabled = False
        self.fx = 277
        self.fy = 277
        self.cx = 160
        self.cy = 120
        self.s = 0.0

class LensProjection:
    def __init__(self):
        self.enabled = False
        self.p_fx = 277
        self.p_fy = 277
        self.p_cx = 160
        self.p_cy = 120
        self.tx = 0.0
        self.ty = 0.0


class CameraLens:
    def __init__(self):
        self.enabled = False
        self.type = "stereographic"
        self.scale_to_hfov = False
        self.custom_function = LensCustomFunc()
        self.cutoff_angle = 1.5707
        self.env_texture_size = 256
        self.intrinsic = LensIntrinsic()
        self.projection = LensProjection()

        self.intrinsic.enabled = False
        self.projection.enabled = False
        self.custom_function.enabled = True
        self.intrinsic.enabled = True




class SDFCamera:
    def __init__(self):
        self.enabled = False
        self.name = ""
        self.camera_info_topic = ""
        self.horizontal_fov = 1.5707
        self.image = CameraImage()
        self.clip = CameraClip()
        self.save = CameraSave()
        self.depth_camera = CamearDepth()
        self.noise = SensorNoise()
        self.distortion = CameraDistortion()
        self.lens = CameraLens()

        self.visibility_mask = 4294967295
        self.optical_frame_id = ""
        self.pose = Pose()







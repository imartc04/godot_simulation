
import xml.etree.ElementTree as ET
import random
import string
from urdf_parser_structs import *
from sdf_1_7_structs import *


"""
Function to generate randrom string with parameter length
and checking generated string is not in list of strings
"""
def generate_random_string(f_length, f_list):

    # Generate random string
    str_out = ''.join(random.choice(string.ascii_letters + string.digits)
                      for i in range(f_length))

    # Check if string is not in list
    if str_out not in f_list:

        # Return string
        return str_out

    # Else
    else:

        # Call function again
        return generate_random_string(f_length, f_list)


def parse_origin_element(f_xml):

    l_out = UrdfPose()

    #Check if f_xml has child origin
    if f_xml.find('origin') is not None:

        #Check if origin has xyz attribute
        if 'xyz' in f_xml.attrib:

            #Set xyz
            l_xyz = f_xml.attrib['xyz']
            l_xyz_vec = [float(x) for x in l_xyz.split()]  
            l_out.x = l_xyz_vec[0]
            l_out.y = l_xyz_vec[1]
            l_out.z = l_xyz_vec[2]


        #Check if origin has rpy attribute
        if 'rpy' in f_xml.attrib:

            #Set rpy
            l_rpy = f_xml.attrib['rpy']
            l_rpy_vec = [float(x) for x in l_rpy.split()]  
            l_out.roll = l_rpy_vec[0]
            l_out.pitch = l_rpy_vec[1]
            l_out.yaw = l_rpy_vec[2]

    return l_out


def parse_joint_links(f_joint_xml):
    
        l_out = {}
    
        #Check if joint has a parent xml child
        l_parent_xml = f_joint_xml.find('parent')
        if l_parent_xml:
            
            #Check if link attribute is set
                if 'link' in l_parent_xml.attrib:
    
                    #Set parent
                    l_out['parent'] = l_parent_xml.attrib['link']
    
                else:
                    raise Exception("Joint has no parent")
    
        else:
            raise Exception("Joint has no parent")
    
    
        #Check if joint has a child xml child
        l_child_xml = f_joint_xml.find('child')
        if l_child_xml:
                
                #Check if link attribute is set
                if 'link' in l_child_xml.attrib:
    
                    #Set child
                    l_out['child'] = l_child_xml.attrib['link']
                else:
                    raise Exception("Joint has no child")
    
        else:
            raise Exception("Joint has no child")
    
        return l_out



def parse_joint_axis(f_joint_xml):
    
        l_out = [0] * 3
    
        #Check if joint has an axis child
        l_axis_xml = f_joint_xml.find('axis')
        if l_axis_xml is not None:
    
            #Check if axis has xyz attribute
            if 'xyz' in l_axis_xml.attrib:
    
                #Set xyz
                l_xyz = l_axis_xml.attrib['xyz']
                l_xyz_vec = [float(x) for x in l_xyz.split()]  
                l_out[0] = l_xyz_vec[0]
                l_out[1] = l_xyz_vec[1]
                l_out[2] = l_xyz_vec[2]
    
            else:
                l_out[0] = 1.0
                l_out[1] = 0.0
                l_out[2] = 0.0
    
        else:
            l_out = [1.0, 0.0, 0.0]
    
        return l_out


def parse_joint_dynamics(f_joint_xml):
    
        l_out = JointDynamics()
    
        #Check if joint has a dynamics child
        l_dynamics_xml = f_joint_xml.find('dynamics')
        if l_dynamics_xml is not None:
    
            l_out.enabled = True

            #Check if damping attribute is set
            if 'damping' in l_dynamics_xml.attrib:
    
                #Set damping
                l_out.damping = float(l_dynamics_xml.attrib['damping'])
    

            #Check if friction attribute is set
            if 'friction' in l_dynamics_xml.attrib:
    
                #Set friction
                l_out.friction = float(l_dynamics_xml.attrib['friction'])


        return l_out


def parse_joint_limits(f_joint_xml, f_joint_type):

    l_out = JointLimits()

    #Check if parsing joint limits is needed
    if f_joint_type == 'revolute' or f_joint_type == 'prismatic':

        #Set enabled to true
        l_out.enabled = True

        #Checck if joint has a limit child
        l_limit_xml = f_joint_xml.find('limit')
        if l_limit_xml is not None:
            
            #Check if lower attribute is set
            if 'lower' in l_limit_xml.attrib:
                
                #Set lower
                l_out.lower = float(l_limit_xml.attrib['lower'])

            #Check if upper attribute is set
            if 'upper' in l_limit_xml.attrib:
                    
                    #Set upper
                    l_out.upper = float(l_limit_xml.attrib['upper'])

            #Check if effort attribute is set
            if 'effort' in l_limit_xml.attrib:
                            
                #Set effort
                l_out.effort = float(l_limit_xml.attrib['effort'])

            else:
                raise Exception("Joint has no effort limit")


            #Check if velocity attribute is set
            if 'velocity' in l_limit_xml.attrib:

                #Set velocity
                l_out.velocity = float(l_limit_xml.attrib['velocity'])

            else:
                raise Exception("Joint has no velocity limit")
            
        else:
            raise Exception("Joint has no limit and is of type " + f_joint_type)

 
    return l_out



def parse_joint_mimic(f_joint_xml):

    l_out = JointMimic()

    #Check if joint has a mimic child
    l_mimic_xml = f_joint_xml.find('mimic')
    if l_mimic_xml is not None:
            
            #Check if joint_name attribute is set
            if 'joint' in l_mimic_xml.attrib:
    
                #Set joint_name
                l_out.joint = l_mimic_xml.attrib['joint']
    
            else:
                raise Exception("Joint has no mimic joint_name")
    
            #Check if multiplier attribute is set
            if 'multiplier' in l_mimic_xml.attrib:
                #Set multiplier
                l_out.multiplier = float(l_mimic_xml.attrib['multiplier'])
    
    
            #Check if offset attribute is set
            if 'offset' in l_mimic_xml.attrib:
    
                #Set offset
                l_out.offset = float(l_mimic_xml.attrib['offset'])
    
    return l_out


def parse_joint_calibration(f_joint_xml):
    l_out = JointCalibration()

    #Check if joint has a calibration child
    l_calibration_xml = f_joint_xml.find('calibration')
    if l_calibration_xml is not None:
                
        #Set enabled to true
        l_out.enabled = True

        #Check if reference_position attribute is set
        if 'reference_position' in l_calibration_xml.attrib:
                
            #Set reference_position
            l_out.reference_position = float(l_calibration_xml.attrib['reference_position'])

        #Check if rising attribute is set
        if 'rising' in l_calibration_xml.attrib:

            #Set rising
            l_out.rising = float(l_calibration_xml.attrib['rising'])

        #Check if falling attribute is set
        if 'falling' in l_calibration_xml.attrib:
            #Set falling
            l_out.falling = float(l_calibration_xml.attrib['falling'])


    return l_out



def pase_joint_safety_controller(f_joint_xml):

    l_out = JointSafetyController()

    #Check if joint has a safety_controller child
    l_safety_controller_xml = f_joint_xml.find('safety_controller')
    if l_safety_controller_xml is not None:
                    
        #Set enabled to true
        l_out.enabled = True

        #Check if k_position attribute is set
        if 'k_position' in l_safety_controller_xml.attrib:

            #Set k_position
            l_out.k_position = float(l_safety_controller_xml.attrib['k_position'])

        #Check if k_velocity attribute is set
        if 'k_velocity' in l_safety_controller_xml.attrib:
            #Set k_velocity
            l_out.k_velocity = float(l_safety_controller_xml.attrib['k_velocity'])

        else:
            raise Exception("Joint has no k_velocity safety_controller")

        #Check if soft_lower_limit attribute is set
        if 'soft_lower_limit' in l_safety_controller_xml.attrib:
            #Set soft_lower_limit
            l_out.soft_lower_limit = float(l_safety_controller_xml.attrib['soft_lower_limit'])

        #Check if soft_upper_limit attribute is set
        if 'soft_upper_limit' in l_safety_controller_xml.attrib:
            #Set soft_upper_limit
            l_out.soft_upper_limit = float(l_safety_controller_xml.attrib['soft_upper_limit'])

    return l_out


"""
Parse joint revolute
"""
def parse_joint(f_joint_xml):

    l_out  = JointData()
    #Check if type attribute is set
    if 'type' in f_joint_xml.attrib:
    
        #Set type
        l_out.type = f_joint_xml.attrib['type']

        #Check if joint has a name
        if 'name' in f_joint_xml.attrib:

            #Set name
            l_out.name = f_joint_xml.attrib['name']

            #Parse joint origin
            l_out.origin = parse_origin_element(f_joint_xml)

            #Parse joint links  
            l_links = parse_joint_links(f_joint_xml)          

            #Set parent link
            l_out.parent = l_links['parent']

            #Set child link
            l_out.child = l_links['child']

            #Parse joint axis
            l_out.axis = parse_joint_axis(f_joint_xml)

            #Parse joint calibration
            l_out.calibration = parse_joint_calibration(f_joint_xml)

            #Parse joint dynamics
            l_out.dynamics = parse_joint_dynamics(f_joint_xml)

            #Parse joint limits
            l_out.limit = parse_joint_limits(f_joint_xml, l_out['type'])

            #Parse joint mimic
            l_out.mimic = parse_joint_mimic(f_joint_xml)

            #Parse joint safety_controller
            l_out.safety_controller = pase_joint_safety_controller(f_joint_xml)

        else:
            raise Exception("Joint has no name")

    else:
        raise Exception("Joint type is not valid")


def pase_link_inertial(f_link_xml):

    l_out = LinkInertial()

    #Check if link has an inertial child
    l_inertial_xml = f_link_xml.find('inertial')
    if l_inertial_xml is not None:

        #Parse origin
        l_out.origin = parse_origin_element(f_link_xml)

        #Check if mass child exists
        l_mass_xml = l_inertial_xml.find('mass')
        if l_mass_xml is not None:
            
            #Check if value attribute is set
            if 'value' in l_mass_xml.attrib:
                #Set mass
                l_out.mass = float(l_mass_xml.attrib['value'])

            else:
                raise Exception("Link has no mass value")


        #Check if inertia child exists
        l_inertia_xml = l_inertial_xml.find('inertia')
        if l_inertia_xml is not None:
                
            #Check if ixx attribute is set
            if 'ixx' in l_inertia_xml.attrib:
                #Set ixx
                l_out.inertia.ixx = float(l_inertia_xml.attrib['ixx'])
            else:
                raise Exception("Link has no ixx inertia")

            #Check if ixy attribute is set
            if 'ixy' in l_inertia_xml.attrib:
                #Set ixy
                l_out.inertia.ixy = float(l_inertia_xml.attrib['ixy'])
            else:
                raise Exception("Link has no ixy inertia")

            #Check if ixz attribute is set
            if 'ixz' in l_inertia_xml.attrib:
                #Set ixz
                l_out.inertia.ixz = float(l_inertia_xml.attrib['ixz'])
            else:
                raise Exception("Link has no ixz inertia")

            #Check if iyy attribute is set
            if 'iyy' in l_inertia_xml.attrib:
                #Set iyy
                l_out.inertia.iyy = float(l_inertia_xml.attrib['iyy'])
            else:
                raise Exception("Link has no iyy inertia")

            #Check if iyz attribute is set
            if 'iyz' in l_inertia_xml.attrib:
                #Set iyz
                l_out.inertia.iyz = float(l_inertia_xml.attrib['iyz'])
            else:
                raise Exception("Link has no iyz inertia")

            #Check if izz attribute is set
            if 'izz' in l_inertia_xml.attrib:
                #Set izz
                l_out.inertia.izz = float(l_inertia_xml.attrib['izz'])
            else:
                raise Exception("Link has no izz inertia")

    return l_out
        
def parse_geometry(f_xml):

    l_out = Geometry()

    #Check if geometry child exists
    l_geometry_xml = f_xml.find('geometry')
    if l_geometry_xml is not None:
        
        l_geom_found = False

        #Check if child box exists
        l_box_xml = l_geometry_xml.find('box')
        if l_box_xml is not None and not l_geom_found:
            #Check if size attribute is set
            if 'size' in l_box_xml.attrib:
                #Set type
                l_out.type = 'box'

                #Set size
                l_out.data['size'] = l_box_xml.attrib['size']

                l_geom_found = True
            else:
                raise Exception("box has no size attribute")
            
            #Check if cylinder child exists
        l_cylinder_xml = l_geometry_xml.find('cylinder')
        if l_cylinder_xml is not None and not l_geom_found:
            #Check if radius attribute is set
            if 'radius' in l_cylinder_xml.attrib:
                #Set type
                l_out.type = 'cylinder'

                #Set radius
                l_out.data['radius'] = l_cylinder_xml.attrib['radius']
            else:
                raise Exception("cylinder has no radius attribute")


            #Check if length attribute is set
            if 'length' in l_cylinder_xml.attrib:
                #Set length
                l_out.data['length'] = l_cylinder_xml.attrib['length']
            else:
                raise Exception("cylinder has no length attribute")

            l_geom_found = True  

        #Check if sphere child exists
        l_sphere_xml = l_geometry_xml.find('sphere')
        if l_sphere_xml is not None and not l_geom_found:
            #Check if radius attribute is set
            if 'radius' in l_sphere_xml.attrib:
                #Set type
                l_out.type = 'sphere'

                #Set radius
                l_out.data['radius'] = l_sphere_xml.attrib['radius']

                l_geom_found = True
            else:
                raise Exception("sphere has no radius attribute")  
            
        #Check if mesh child exists
        l_mesh_xml = l_geometry_xml.find('mesh')
        if l_mesh_xml is not None and not l_geom_found:
            #Check if filename attribute is set
            if 'filename' in l_mesh_xml.attrib:
                #Set type
                l_out.type = 'mesh'

                #Check if filename attrib is set
                if 'filename' in l_mesh_xml.attrib:
                    #Set filename
                    l_out.data['filename'] = l_mesh_xml.attrib['filename']
                else:
                    raise Exception("mesh has no filename attribute")

                #Check if scale attribute is set
                if 'scale' in l_mesh_xml.attrib:
                    #Set scale
                    l_out.data['scale'] = l_mesh_xml.attrib['scale']

                l_geom_found = True

            else:
                raise Exception("mesh has no filename attribute")

        #Check if geometry was found
        if not l_geom_found:
            raise Exception("geometry child does not exist")
           

    else:
        raise Exception("geometry child does not exist")


def parse_material(f_xml):

    l_out = Material()

    #Check if material child exists
    l_material_xml = f_xml.find('material')
    if l_material_xml is not None:
            
        #Check if material has a name
        if 'name' in l_material_xml.attrib:

            #Set name
            l_out.name = l_material_xml.attrib['name']

            #Check if color child exists
            l_color_xml = l_material_xml.find('rgba')

            if l_color_xml is not None:
                
                #Check if rgba attribute is set
                if 'rgba' in l_material_xml.attrib:
                    #Get vector of float from color attrib
                    l_color = l_material_xml.attrib['rgba']
                    l_out.rgba  = [float(x) for x in l_color.split()]

                else:
                    raise Exception("Material has no rgba attribute")
                

            #Check if texture child exists
            l_texture_xml = l_material_xml.find('texture')
            if l_texture_xml is not None:
                #Check if material has a texture
                if 'filename' in l_material_xml.attrib:

                    #Set texture
                    l_out.texture_filename = l_material_xml.attrib['filename']

                else:
                    raise Exception("Material has no texture filename")

        else:
            raise Exception("Material has no name")

    return l_out

def parse_link_visual(f_link_xml):

    l_out = LinkVisual()

    #Check if visual child exists
    l_visual_xml = f_link_xml.find('visual')
    if l_visual_xml is not None:
        
        #Check if visual has a name
        if 'name' in l_visual_xml.attrib:

            #Check if name attrib is set
            if 'name' in l_visual_xml.attrib:
                #Set name
                l_out.name = l_visual_xml.attrib['name']

                #Parse visual origin
                l_out.origin = parse_origin_element(l_visual_xml)

                #Parse visual geometry
                l_out.geometry = parse_geometry(l_visual_xml)

                #Parse visual material
                l_out.material = parse_material(l_visual_xml)

        else:
            raise Exception("Visual has no name")

    return l_out


def parse_collision(f_link_xml):
    l_out = LinkCollision()

    #Check if collision child exists
    l_collision_xml = f_link_xml.find('collision')
    if l_collision_xml is not None:
            
        #Check if collision has a name
        if 'name' in l_collision_xml.attrib:

            #Check if name attrib is set
            if 'name' in l_collision_xml.attrib:
                #Set name
                l_out.name = l_collision_xml.attrib['name']


        #Check if origin child exists
        l_out.origin = parse_origin_element(l_collision_xml)

        #Parse collision geometry
        l_out.geometry = parse_geometry(l_collision_xml)

    return l_out

def parse_link(f_link_xml):

    l_out = LinkData()

    #Check if link has a name
    if 'name' in f_link_xml.attrib:
        #Set name
        l_out.name = f_link_xml.attrib['name']

        #Loop over all link children
        for link_child in f_link_xml:

            #Check if link child is inertial
            if link_child.tag == 'inertial':
                #Parse link inertial
                l_out.inertial = pase_link_inertial(link_child)

            #Check if link child is a visual
            if link_child.tag == 'visual':
                #Parse link visual
                l_out.visuals.append(parse_link_visual(link_child))

            #Check if link child is a collision
            elif link_child.tag == 'collision':
                #Parse link collision
                l_out.collisions.append(parse_collision(link_child))


    else:
        raise Exception("Link has no name")


    return l_out


def parse_transmission_joint(f_joint_xml):

    l_out : JointTransmission = {"name":"", "hardwareInterface":[]}

    #Check if joint has a name
    if 'name' in f_joint_xml.attrib:
        #Set name
        l_out['name'] = f_joint_xml.attrib['name']

        l_hw_interface_found = False

        #Loop over all joint children
        for joint_child in f_joint_xml:
                
            #Check if joint child is hardwareInterface
            if joint_child.tag == 'hardwareInterface':
                #Set hardwareInterface
                l_out['hardwareInterface'].append(joint_child.text)
                l_hw_interface_found = True
 

        #Check if hardwareInterface was found
        if not l_hw_interface_found:
            raise Exception("Joint has no hardwareInterface")
        
    else:
        raise Exception("Joint has no name")   


def parse_transmission_actuator(f_actuator_xml):
    l_out : JointActuator = {"name":"", "mechanicalReduction":0.0, "transmission":"", "hardwareInterface":[]}

    #Check if actuator has a name
    if 'name' in f_actuator_xml.attrib:
        #Set name
        l_out['name'] = f_actuator_xml.attrib['name']

        l_hw_interface_found = False
        #Loop over all actuator children
        for actuator_child in f_actuator_xml:

            #Check if child tag is mechanicalReduction
            if actuator_child.tag == 'mechanicalReduction':
                #Set mechanicalReduction
                l_out['mechanicalReduction'] = float(actuator_child.text)

            #Check if child tag is hardwareInterface
            elif actuator_child.tag == 'hardwareInterface':
                #Set hardwareInterface
                l_out['hardwareInterface'].append(actuator_child.text)
                l_hw_interface_found = True

        
        #Check if hardwareInterface was found
        if not l_hw_interface_found:
            raise Exception("Actuator has no hardwareInterface")

    else:
        raise Exception("Actuator has no name")

    return l_out

def parse_transmission(f_transmission_xml):

    l_out : Transmission = {"name":"", "type":"","joints" : [], "actuators" : []}

    #Check if transmission has a name
    if 'name' in f_transmission_xml.attrib:
        #Set name
        l_out['name'] = f_transmission_xml.attrib['name']

        l_type_foud = False
        l_actuator_found = False
        l_joint_found = False

        #Loop over all transmission children
        for transmission_child in f_transmission_xml:

          #Check if transmission child is joint
            if transmission_child.tag == 'joint':
                #Parse joint
                l_out['joints'].append(parse_transmission_joint(transmission_child))
                l_joint_found = True

            #Check if transmission child is actuator
            elif transmission_child.tag == 'actuator':
                #Parse actuator
                l_out['actuators'].append(parse_transmission_actuator(transmission_child))
                l_actuator_found = True

            #Check if transmission child is type
            elif transmission_child.tag == 'type':
                #Set type
                l_out['type'] = transmission_child.text  
                l_type_foud = True

            else:
                raise Exception("Transmission tag is not valid")
        

    else:
        raise Exception("Transmission has no name")


def parse_gazebo_sensor_camera(f_camera_xml):

    l_out = SDFCamera()

    l_out.enabled = True

    #Check if camera has a name
    if 'name' in f_camera_xml.attrib:
        #Set name
        l_out.name = f_camera_xml.attrib['name']

        #Check if camera_info_topic child exists
        l_camera_info_topic_xml = f_camera_xml.find('camera_info_topic')
        if l_camera_info_topic_xml is not None:
            #Set camera_info_topic
            l_out.camera_info_topic = l_camera_info_topic_xml.text

        #Check if horizontal_fov child exists
        l_horizontal_fov_xml = f_camera_xml.find('horizontal_fov')
        if l_horizontal_fov_xml is not None:
            #Set horizontal_fov
            l_out.horizontal_fov = float(l_horizontal_fov_xml.text)

        else:
            raise Exception("Camera has no horizontal_fov")

        #Check if image child exists
        l_image_xml = f_camera_xml.find('image')

        if l_image_xml is not None:

            #Check if width image child exists
            l_width_xml = l_image_xml.find('width')
            if l_width_xml is not None:

                #Set width
                l_out.image.width = int(l_width_xml.text)
            else:
                raise Exception("Camera has no width")
            
            #Check if height image child exists
            l_height_xml = l_image_xml.find('height')
            if l_height_xml is not None:
                    
                #Set height
                l_out.image.height = int(l_height_xml.text)

            else:
                raise Exception("Camera has no height")
            
            #Check if format image child exists
            l_format_xml = l_image_xml.find('format')
            if l_format_xml is not None:
                                
                #Set format
                l_out.image.format = l_format_xml.text    


            #Check if anti_aliasing image child exists
            l_anti_aliasing_xml = l_image_xml.find('anti_aliasing')
            if l_anti_aliasing_xml is not None:
                                    
                #Set anti_aliasing
                l_out.image.anti_aliasing = int(l_anti_aliasing_xml.text)

        else:
            raise Exception("Camera has no image")


        #Check if clip child exists
        l_clip_xml = f_camera_xml.find('clip')
        if l_clip_xml is not None:              
            
            #Check if near child of clip exists
            l_near_xml = l_clip_xml.find('near')
            if l_near_xml is not None:
                
                #Set near
                l_out.image.clip.near = float(l_near_xml.text)

            else:
                raise Exception("Camera has no near clip")
            
            #Check if far child of clip exists
            l_far_xml = l_clip_xml.find('far')
            if l_far_xml is not None:
                
                #Set far
                l_out.image.clip.far = float(l_far_xml.text)

            else:
                raise Exception("Camera has no far clip")
            
        else:
            raise Exception("Camera has no clip")


        #Check if save child exists
        l_save_xml = f_camera_xml.find('save')
        if l_save_xml is not None:

            #Check if enabled attribute is set
            if 'enabled' in l_save_xml.attrib:
                    
                #Set enabled
                l_out.save.enabled = l_save_xml.attrib['enabled']

            else:
                raise Exception("Camera has no save enabled")

            #Check if path child exists
            l_path_xml = l_save_xml.find('path')
            if l_path_xml is not None:
                    
                #Set path
                l_out.save.path = l_path_xml.text   

        #Check if depth_camera child exists
        l_depth_camera_xml = f_camera_xml.find('depth_camera')
        if l_depth_camera_xml is not None:
            
            #Check if output child exists
            l_output_xml = l_depth_camera_xml.find('output')    
            if l_output_xml is not None:
                
                #Seet output value
                l_out.depth_camera.output = l_output_xml.text

            else:
                raise Exception("Camera has no output")

            #Check if clip child exists
            l_clip_xml = l_depth_camera_xml.find('clip')
            if l_clip_xml is not None:

                #Check if near child exists
                l_near_xml = l_clip_xml.find('near')
                if l_near_xml is not None:
                    l_out.depth_camera.clip.near = float(l_near_xml.text)

                #Check if far child exists
                l_far_xml = l_clip_xml.find('far')
                if l_far_xml is not None:
                    l_out.depth_camera.clip.far = float(l_far_xml.text)
        


        #Check if noise child exists
        l_noise_xml = f_camera_xml.find('noise')
        if l_noise_xml is not None:
            l_out.noise.enabled = True

            #Check if type child exists
            l_type_xml = l_noise_xml.find('type')
            if l_type_xml is not None:
                l_out.noise.type = l_type_xml.text
            else:
                raise Exception("Camera has no noise type")
            
            #Check if mean child exists
            l_mean_xml = l_noise_xml.find('mean')
            if l_mean_xml is not None:
                l_out.noise.mean = float(l_mean_xml.text)

            #Check if stddev child exists
            l_stddev_xml = l_noise_xml.find('stddev')
            if l_stddev_xml is not None:
                l_out.noise.stddev = float(l_stddev_xml.text)

        #Check if distortion child exists
        l_distortion_xml = f_camera_xml.find('distortion')
        if l_distortion_xml is not None:
            #Check if k1 child exists
            l_k1_xml = l_distortion_xml.find('k1')
            if l_k1_xml is not None:
                l_out.distortion.k1 = float(l_k1_xml.text)

            #Check if k2 child exists
            l_k2_xml = l_distortion_xml.find('k2')
            if l_k2_xml is not None:
                l_out.distortion.k2 = float(l_k2_xml.text)

            #Check if k3 child exists
            l_k3_xml = l_distortion_xml.find('k3')
            if l_k3_xml is not None:
                l_out.distortion.k3 = float(l_k3_xml.text)
            
            #Check if p1 child exists
            l_p1_xml = l_distortion_xml.find('p1')
            if l_p1_xml is not None:
                l_out.distortion.p1 = float(l_p1_xml.text)

            #Check if p2 child exists
            l_p2_xml = l_distortion_xml.find('p2')
            if l_p2_xml is not None:
                l_out.distortion.p2 = float(l_p2_xml.text)

            #Check if center child exists
            l_center_xml = l_distortion_xml.find('center')
            if l_center_xml is not None:
                #Convert string to vector of floats
                l_center = l_center_xml.text
                l_out.distortion.center = [float(x) for x in l_center.split()]

        #end if distortion

        #Check if lens child exists
        l_lens_xml = f_camera_xml.find('lens')
        if l_lens_xml is not None:
            #Check if type child exists
            l_type_xml = l_lens_xml.find('type')
            if l_type_xml is not None:
                l_out.lens.type = l_type_xml.text
            else:
                raise Exception("Camera has no lens type")
            
            #Check if scale_to_hfov child exists
            l_scale_to_hfov_xml = l_lens_xml.find('scale_to_hfov')
            if l_scale_to_hfov_xml is not None:
                l_out.lens.scale_to_hfov = bool(l_scale_to_hfov_xml.text)

            else:
                raise Exception("Camera has no scale_to_hfov")
            
            #Check if custom_function child exists
            l_custom_function_xml = l_lens_xml.find('custom_function')
            if l_custom_function_xml is not None:
                l_out.lens.custom_function.enabled = True

                #Check if c1 child exists
                l_c1_xml = l_custom_function_xml.find('c1')
                if l_c1_xml is not None:
                    l_out.lens.custom_function.c1 = float(l_c1_xml.text)

                #Check if c2 child exists
                l_c2_xml = l_custom_function_xml.find('c2')
                if l_c2_xml is not None:
                    l_out.lens.custom_function.c2 = float(l_c2_xml.text)

                #Check if c3 child exists
                l_c3_xml = l_custom_function_xml.find('c3')
                if l_c3_xml is not None:
                    l_out.lens.custom_function.c3 = float(l_c3_xml.text)

                #Check if f child exists
                l_f_xml = l_custom_function_xml.find('f')
                if l_f_xml is not None:
                    l_out.lens.custom_function.f = float(l_f_xml.text)

                #Check if fun child exists
                l_fun_xml = l_custom_function_xml.find('fun')
                if l_fun_xml is not None:
                    l_out.lens.custom_function.fun = l_fun_xml.text
                else:
                    raise Exception("Camera has no lens custom_function fun")
                
            
            #Check if cutoff_angle child exists
            l_cutoff_angle_xml = l_lens_xml.find('cutoff_angle')
            if l_cutoff_angle_xml is not None:
                l_out.lens.cutoff_angle = float(l_cutoff_angle_xml.text)

            #Check if env_texture child exists
            l_env_texture_xml = l_lens_xml.find('env_texture')
            if l_env_texture_xml is not None:
                l_out.lens.env_texture = l_env_texture_xml.text

            #Check if intrinsics child exists
            l_intrinsics_xml = l_lens_xml.find('intrinsics')
            if l_intrinsics_xml is not None:
                #Check if fx child exists
                l_fx_xml = l_intrinsics_xml.find('fx')
                if l_fx_xml is not None:
                    l_out.lens.intrinsics.fx = float(l_fx_xml.text)
                else:
                    raise Exception("Camera has no lens intrinsics fx")
                
                #Check if fy child exists
                l_fy_xml = l_intrinsics_xml.find('fy')
                if l_fy_xml is not None:
                    l_out.lens.intrinsics.fy = float(l_fy_xml.text)

                else:
                    raise Exception("Camera has no lens intrinsics fy")
                

                #Check if cx child exists
                l_cx_xml = l_intrinsics_xml.find('cx')
                if l_cx_xml is not None:
                    l_out.lens.intrinsics.cx = float(l_cx_xml.text)

                else:
                    raise Exception("Camera has no lens intrinsics cx")
                
                #Check if cy child exists
                l_cy_xml = l_intrinsics_xml.find('cy')
                if l_cy_xml is not None:
                    l_out.lens.intrinsics.cy = float(l_cy_xml.text)

                else:
                    raise Exception("Camera has no lens intrinsics cy")
                
                #Check if s child exists
                l_s_xml = l_intrinsics_xml.find('s')
                if l_s_xml is not None:
                    l_out.lens.intrinsics.s = float(l_s_xml.text)

                else:
                    raise Exception("Camera has no lens intrinsics s")
                
            #Check if projection child exists
            l_projection_xml = l_lens_xml.find('projection')
            if l_projection_xml is not None:
                #Check if p_fx child exists
                l_p_fx_xml = l_projection_xml.find('p_fx')
                if l_p_fx_xml is not None:
                    l_out.lens.projection.p_fx = float(l_p_fx_xml.text)
                
                #Check if p_fy child exists
                l_p_fy_xml = l_projection_xml.find('p_fy')
                if l_p_fy_xml is not None:
                    l_out.lens.projection.p_fy = float(l_p_fy_xml.text)

                #Check if p_cx child exists
                l_p_cx_xml = l_projection_xml.find('p_cx')
                if l_p_cx_xml is not None:
                    l_out.lens.projection.p_cx = float(l_p_cx_xml.text)

                #Check if p_cy child exists
                l_p_cy_xml = l_projection_xml.find('p_cy')
                if l_p_cy_xml is not None:
                    l_out.lens.projection.p_cy = float(l_p_cy_xml.text)

                #Check if tx child exists
                l_tx_xml = l_projection_xml.find('tx')
                if l_tx_xml is not None:
                    l_out.lens.projection.tx = float(l_tx_xml.text)

                #Check if ty child exists
                l_ty_xml = l_projection_xml.find('ty')
                if l_ty_xml is not None:
                    l_out.lens.projection.ty = float(l_ty_xml.text)

            #end if projection

        #end if lens

        #Check if visibility_mask child exists
        l_visibility_mask_xml = f_camera_xml.find('visibility_mask')
        if l_visibility_mask_xml is not None:
            l_out.visibility_mask = int(l_visibility_mask_xml.text)

        #Check if optical_frame_id child exists
        l_optical_frame_id_xml = f_camera_xml.find('optical_frame_id')
        if l_optical_frame_id_xml is not None:
            l_out.optical_frame_id = l_optical_frame_id_xml.text

        #Check if pose child exists
        l_pose_xml = f_camera_xml.find('pose')
        if l_pose_xml is not None:
            
            #Get pose text and split it into vector of floats
            l_pose = l_pose_xml.text
            l_out.pose  = [float(x) for x in l_pose.split()]

            #Check if attribute relative_to is set
            if 'relative_to' in l_pose_xml.attrib:
                l_out.pose_relative_to = l_pose_xml.attrib['relative_to']


    #end if camera    


    else:
        raise Exception("Camera has no name")


    return l_out


def parse_gazebo(f_gazebo_xml):

    l_out = {}
    #Check if gazebo has reference attribute
    if 'reference' in f_gazebo_xml.attrib:

        l_out['reference'] = f_gazebo_xml.attrib['reference']

        #We are only interested in gazebo sensor tags
        #Check if sensor child exists
        l_sensor_xml = f_gazebo_xml.find('sensor')
        if l_sensor_xml is not None:

            #Check sensor name attribute is defined
            if 'name' in l_sensor_xml.attrib:
                l_out['name'] = l_sensor_xml.attrib['name']
            else:
                raise Exception("Sensor has no name attribute")
            
            #Check sensor type attribute is defined
            if 'type' in l_sensor_xml.attrib:
                l_out['type'] = l_sensor_xml.attrib['type']
            else:
                raise Exception("Sensor has no type attribute")
            

            #Check if child always_on is defined
            l_always_on_xml = l_sensor_xml.find('always_on')
            if l_always_on_xml is not None:
                #Set always_on
                l_out['always_on'] = True

            #Check if update_rate child is defined
            l_update_rate_xml = l_sensor_xml.find('update_rate')
            if l_update_rate_xml is not None:
                #Set update_rate
                l_out['update_rate'] = float(l_update_rate_xml.text)

            #Check if visualize child is defined
            l_visualize_xml = l_sensor_xml.find('visualize')
            if l_visualize_xml is not None:
                #Set visualize
                l_out['visualize'] = True
        
            #Check if topic child is defined
            l_topic_xml = l_sensor_xml.find('topic')
            if l_topic_xml is not None:
                #Set topic
                l_out['topic'] = l_topic_xml.text

            
            #Check if enable_metrics child is defined
            l_enable_metrics_xml = l_sensor_xml.find('enable_metrics')
            if l_enable_metrics_xml is not None:
                #Set enable_metrics
                l_out['enable_metrics'] = True


            #Check if pose child is defined
            l_pose_xml = l_sensor_xml.find('pose')
            if l_pose_xml is not None:
                #Split pose text into vector of floats
                l_pose = l_pose_xml.text
                l_out["pose"]  = [float(x) for x in l_pose.split()]


            """
            Check possible sensor types
            """
            l_out["sensor_names"] = []
            l_out["sensors"] = {}
            #Check if camera child is defined
            l_camera_xml = l_sensor_xml.find('camera')
            if l_camera_xml is not None:
                l_camera = parse_gazebo_sensor_camera(l_camera_xml)
                l_out["sensor_names"].append(l_camera.name)
                l_out["sensors"][l_camera.name] = l_camera


    else:
        l_out['reference'] = "" #Reference is the robot root link

    return l_out

def parse_urdf(f_urdf_file, f_transmission_files, f_out_file):

    # Open the URDF file to parse it as XML
    urdf_file = open(f_urdf_file, 'r')

    # Parse the URDF file as XML
    urdf_xml = ET.parse(urdf_file)

    # List of used uids
    uid_list = []

    uid_list.append(generate_random_string(13, uid_list))

    # Creat tscn file header
    str_out = "[gd_scene load_steps=4 format=3 uid='uid://" + \
        uid_list[-1] + "']\n"

    # Check if robot xml node exists
    l_xml_robot = urdf_xml.find('robot')
    if l_xml_robot is not None:

        # Add robot node as Node3D to tscn file
        str_out += "[node name='Robot' type='Node3D']\n"

        # Set container to map global material-link . TODO : Currently global materials are not used
        # because the URDF XSD does not specify how to use them



        # Write tscn file
        tscn_file = open(f_out_file, 'w')
        tscn_file.write(str_out)

       
        joint_storage = {}

        link_storage = {}

        transmission_storage = {}

        l_sensor_store = []


        #Iterate over all robot element children
        for robot_child in l_xml_robot:

            # Check if robot child is a link
            if robot_child.tag == 'link':
                    
                #Parse link
                l_link_data = parse_link(robot_child)
                link_storage[l_link_data['name']] = l_link_data

            # Check if robot child is a joint
            elif robot_child.tag == 'joint':
                #Parse joint
                l_joint_data = parse_joint(robot_child)
                joint_storage[l_joint_data['name']] = l_joint_data

            #Check if robot child is a transmission
            elif robot_child.tag == 'transmission':
                #Parse transmision
                l_transmission_data = parse_transmission(robot_child)
                transmission_storage[l_transmission_data['name']] = l_transmission_data

            #Check if robot child is gazebo
            elif robot_child.tag == 'gazebo':
                #Parse gazebo
                l_gazebo_data = parse_gazebo(robot_child)
                l_sensor_store.append(l_gazebo_data)

    else:

        # Raise error
        raise Exception("URDF file is not valid, robot node not found")
    

    return { "links": link_storage, "joints" : joint_storage, "transmissions" : transmission_storage, "sensors" : l_sensor_store}

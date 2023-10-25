from urdf_parser import parse_urdf
from bigtree import Node, tree_to_dot, dict_to_tree
import numpy as np
import pymesh
import os
import shutil



"""
godot_position.x = -ros_position.y
godot_position.y = ros_position.z
godot_position.z = ros_position.x

"""


"""

import pymesh

# Replace "input.stl" and "output.dae" with your input and output file paths.
input_file = "input.stl"
output_file = "output.dae"

# Load the .STL file
mesh = pymesh.load_mesh(input_file)

# Save it as .DAE
pymesh.save_mesh(output_file, mesh)


"""


"""
Rotation matrices to apply rpy in ros and obtain godot basis vectors 
ROS coordinate system: x forward, y left, z up (right-hand rule) . Rotation angle positive counter-clockwise

# Define the rotation matrix based on the specified axis
if axis == 'x':
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(angle_radians), -np.sin(angle_radians)],
        [0, np.sin(angle_radians), np.cos(angle_radians)]
    ])
elif axis == 'y':
    rotation_matrix = np.array([
        [np.cos(angle_radians), 0, np.sin(angle_radians)],
        [0, 1, 0],
        [-np.sin(angle_radians), 0, np.cos(angle_radians)]
    ])
elif axis == 'z':
    rotation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians), 0],
        [np.sin(angle_radians), np.cos(angle_radians), 0],
        [0, 0, 1]
    ])
else:

"""

def create_rotation_matrx(f_axis, f_angle):

    rotation_matrix = None

    if f_axis == 'x':
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(f_angle), -np.sin(f_angle)],
            [0, np.sin(f_angle), np.cos(f_angle)]
        ])
    elif f_axis == 'y':
        rotation_matrix = np.array([
            [np.cos(f_angle), 0, np.sin(f_angle)],
            [0, 1, 0],
            [-np.sin(f_angle), 0, np.cos(f_angle)]
        ])
    elif f_axis == 'z':
        rotation_matrix = np.array([
            [np.cos(f_angle), -np.sin(f_angle), 0],
            [np.sin(f_angle), np.cos(f_angle), 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Invalid axis specified. Must be 'x', 'y', or 'z'")
    
    return rotation_matrix

def urdf_to_godot_vec(f_urdf):
    return np.array([f_urdf.x, f_urdf.z, f_urdf.y])


def stl_to_dae(f_stl, f_out_dae):

    # Load the .STL file
    mesh = pymesh.load_mesh(f_stl)

    # Save it as .DAE
    pymesh.save_mesh(f_out_dae, mesh)



def urdf_pos_to_tscn_transform(f_urdf_pos):

    # Create final rotation matrix as the 
    # product of roll pitch and yaw matrices made from pose values
    l_final_rotation_matrix = np.matmul( np.matmul( create_rotation_matrx('x', f_urdf_pos.roll), create_rotation_matrx('y', f_urdf_pos.pitch) ), create_rotation_matrx('z', f_urdf_pos.yaw))

    #By the conversion from ros to godot, the y and z axis are swapped
    l_final_rotation_matrix[:, [1, 2]] = l_final_rotation_matrix[:, [2, 1]]

    #Create origin vector
    l_origin_vector = urdf_to_godot_vec(np.array[f_urdf_pos.x, f_urdf_pos.y, f_urdf_pos.z])

    return f'''transform = Transform3D({l_final_rotation_matrix[0,0]}, {l_final_rotation_matrix[1,0]}, {l_final_rotation_matrix[2,0]} , {l_final_rotation_matrix[0,1]} ,{l_final_rotation_matrix[1,1]}, {l_final_rotation_matrix[2,1]}, {l_final_rotation_matrix[2,0]} , {l_final_rotation_matrix[2,1]}, {l_final_rotation_matrix[2,2]}, {l_origin_vector[0]}, {l_origin_vector[1]}, {l_origin_vector[2]})'''          

def get_filename_and_ext_from_path(f_path):
    return os.path.splitext(os.path.basename(f_path))



def genrate_link_visual_nodes(f_link_visuals, f_tscn_body_str, f_sub_resources_str, f_extern_resources_str, f_global_id_ctr,  f_out_dir, f_parent_str):

    for  link_data in f_link_visuals:
            
            #Create Nodes for the link visuals
            l_visual_counter = 0
            for visual in link_data.visuals:


                #Check visual geometry type
                if visual.geometry.type == "mesh":
                    
                    #Get mesh file name
                    l_mesh_file_name = visual.geometry.data['filename']

                    l_file_name, l_ext = get_filename_and_ext_from_path(l_mesh_file_name)

                    #Check if the mesh file is an stl file
                    if l_ext == ".stl":
                        #Convert stl to dae
                        stl_to_dae(l_mesh_file_name, f_out_dir + "/" + l_file_name + ".dae")

                    elif l_ext == ".dae":
                        #Copy dae file to output directory
                        shutil.copy(l_mesh_file_name, f_out_dir + "/" + os.path.basename(l_mesh_file_name))

                    else:
                        raise ValueError("Invalid mesh file extension. Must be '.stl' or '.dae'")

                    l_mesh_proj_relative =  "res://" + os.path.basename(l_mesh_file_name)

                    l_geometry_id = f'{visual.name}_visual_{f_global_id_ctr[0]}_mesh_shape'''

                    f_extern_resources_str += f'''\n[ext_resource type="PackedScene" id="{l_geometry_id} uid="uid://{l_geometry_id}" path="{l_mesh_proj_relative}"]'''

                    l_node_name = "node_" + l_geometry_id

                    f_tscn_body_str += f'''\n[node name="{l_node_name} parent="{f_parent_str}" instance=ExtResource("{l_geometry_id}")]'''
                                    

                elif visual.geometry.type == "box":

                    l_geometry_id = f'''{visual.name}_visual_{f_global_id_ctr[0]}_box_shape'''
                    l_node_name = f'''node_{l_geometry_id}'''

                    f_sub_resources_str  += f'''\n[sub_resource type="BoxShape3D" id="{f_global_id_ctr[0]}"]  
                    '\nsize= Vector3({visual.geometry.data['size'][0]}, {visual.geometry.data['size'][1]}, {visual.geometry.data['size'][2]})'''

                    #Create visual node string for the resource
                    f_tscn_body_str = f'''\n[node name="{l_node_name} type="MeshInstance3D" parent="{f_parent_str}"]
                     shape= SubResource("{l_geometry_id}")'''
                    

                elif visual.geometry.type == "cylinder":

                    l_geometry_id = f'''{visual.name}_visual_{f_global_id_ctr[0]}_cylinder_shape'''
                    l_node_name = f'''node_{l_geometry_id}'''

                    f_sub_resources_str += f'''\n[sub_resource type="CylinderShape3D" id="{l_geometry_id}"]
                    radius={visual.geometry.data['radius']})
                    height={visual.geometry.data['length']})'''
                    
                    #Create visual node string for the resource
                    f_tscn_body_str = f'''\n[node name="{l_node_name} type="MeshInstance3D parent={f_parent_str})]
                    nmesh= SubResource("{l_geometry_id}")'''


                elif visual.geometry.type == "sphere":

                    l_geometry_id = f'''{visual.name}_visual_{f_global_id_ctr[0]}_sphere_shape'''
                    l_node_name = f'''node_{l_geometry_id}'''

                    f_sub_resources_str += f'''\n[sub_resource type="SphereShape3D" id="{l_geometry_id}"]
                    radius={visual.geometry.data['radius']}
                    '''

                    #Create visual node string for the resource
                    f_tscn_body_str += f'''\n[node name="{l_node_name} type="MeshInstance3D" parent="{f_parent_str}"]
                    mesh= SubResource("{l_geometry_id}")'''


                else:
                    raise ValueError("Invalid geometry type specified. Must be 'mesh', 'box', 'cylinder' or 'sphere'")


                #Add visual origin 
                f_tscn_body_str += f'{urdf_pos_to_tscn_transform(visual.origin)}'

                #Increment global id counter
                f_global_id_ctr[0] +=1



def create_godot_rigid_body_str(f_name, f_parent, f_transform, f_mass_str = "1.0", f_mass_center_mode = 1, f_mass_center_vec = "0.0, 0.0, 0.0"):

    l_ret = f'''[node name="{f_name}" type="RigidBody3D" parent="{f_parent}"]
transform = Transform3D({f_transform}) 
mass = {f_mass_str}
'''

    if f_mass_center_mode == 1:
        l_ret = f'''{l_ret} 
        center_of_mass_mode = 1
    center_of_mass = Vector3({f_mass_center_vec})'''

    return l_ret



def generate_collision_nodes(f_urdf_python_col, f_parent_str, f_global_id_ctr, f_out_dir, f_sub_resources_str, f_extern_resources_str, f_tscn_body_str, f_obj_parent_tree):

    for col_obj in f_urdf_python_col:

        #Create collision shape object
        f_tscn_body_str += f'''[node name="CollisionShape3D" type="CollisionShape3D" parent="{f_parent_str}"]
        {urdf_pos_to_tscn_transform(col_obj.origin)}
        '''

        l_geom = col_obj.geometry


        #Create sub resource shape 

        if l_geom.type == "mesh":
        
            #Get mesh and generate convex polygon shape from it
            l_mesh_file_name = l_geom.data['filename']

            l_file_name, l_ext = get_filename_and_ext_from_path(l_mesh_file_name)

            #Check if the mesh file is an stl file
            if l_ext == ".stl":
                #Convert stl to dae
                stl_to_dae(l_mesh_file_name, f_out_dir + "/" + l_file_name + ".dae")

            elif l_ext == ".dae":
                #Valid format, nothing to do
                pass

            else:
                raise ValueError("Invalid mesh file extension. Must be '.stl' or '.dae'")


            l_mesh_proj_relative =  "res://" + os.path.basename(l_mesh_file_name)

            l_geometry_id = f'{l_geom.name}_visual_{f_global_id_ctr[0]}_mesh_shape'''

            f_extern_resources_str += f'''\n[ext_resource type="PackedScene" id="{l_geometry_id} uid="uid://{l_geometry_id}" path="{l_mesh_proj_relative}"]'''

            l_node_name = "node_" + l_geometry_id

            f_tscn_body_str += f'''\n[node name="{l_node_name} parent="{f_parent_str}" instance=ExtResource("{l_geometry_id}")]'''
                        

        elif l_geom.type == "box":

            l_geometry_id = f'''{l_geom.name}_visual_{f_global_id_ctr[0]}_box_shape'''
            l_node_name = f'''node_{l_geometry_id}'''

            f_sub_resources_str  += f'''\n[sub_resource type="BoxShape3D" id="{f_global_id_ctr[0]}"]  
            '\nsize= Vector3({l_geom.data['size'][0]}, {l_geom.data['size'][1]}, {l_geom.data['size'][2]})'''
            

        elif l_geom.type == "cylinder":

            l_geometry_id = f'''{l_geom.name}_visual_{f_global_id_ctr[0]}_cylinder_shape'''
            l_node_name = f'''node_{l_geometry_id}'''

            f_sub_resources_str += f'''\n[sub_resource type="CylinderShape3D" id="{l_geometry_id}"]
            radius={l_geom.data['radius']})
            height={l_geom.data['length']})'''
            

        elif l_geom.type == "sphere":

            l_geometry_id = f'''{l_geom.name}_visual_{f_global_id_ctr[0]}_sphere_shape'''
            l_node_name = f'''node_{l_geometry_id}'''

            f_sub_resources_str += f'''\n[sub_resource type="SphereShape3D" id="{l_geometry_id}"]
            radius={l_geom.data['radius']}
            '''

        else:
            raise ValueError("Invalid geometry type specified. Must be 'mesh', 'box', 'cylinder' or 'sphere'")


            


        

def generate_link_nodes(f_urdf_python_links, f_global_id_ctr, f_out_dir, f_sub_resources_str, f_extern_resources_str, f_tscn_body_str, f_obj_parent_tree, f_mass_center_mode):

    """
    Each link will be represented by a node3D.
    The node3D has will have a Godot rigid body, a set of 3D meshes,
    a set of collision objects and optionally a set of associated joints
    """
    for key, link_data in f_urdf_python_links.items():


        #Create node3D
        f_tscn_body_str += f'[node name="{link_data.name}" type="Node3D" parent="{f_obj_parent_tree[link_data.name]}"]'


        """
        Create link rigid body
        Note that the Godot rigid body allows to set the center of mass a vector relative to the rigid body object. 
        However URDF defines directly the position and orientation of the center of mass system. So to parse from one format 
        to the othe the solution adopted is to set the transform3D of the Godot rigid body as the URDF inertial system with a 
        vector with center of mass of (0, 0, 0)
        """
        f_tscn_body_str += create_godot_rigid_body_str(f'{link_data.name}_rigid_body', f'{f_obj_parent_tree[link_data.name]}/{link_data.name}', urdf_pos_to_tscn_transform(link_data.inertial.pose),link_data.inertial.mass)

        #Create visual nodes
        genrate_link_visual_nodes(link_data.visuals, f_tscn_body_str, f_sub_resources_str, f_extern_resources_str, f_global_id_ctr, f_out_dir, f'{f_obj_parent_tree[link_data.name]}/{link_data.name}')

        #Create collision nodes



def gen_joints(f_joints, f_out_dir, f_obj_parent_tree):

    for key, joint_data in f_joints.items():

        #Create node based on the joint type
        if joint_data.type == "revolute": 

            """
            Godot hinge joint only rotates around X so if the revolution axis
            
            """


        elif joint_data.type == "continuous": 

        elif joint_data.type == "prismatic": 

        elif joint_data.type == "fixed": 

        elif joint_data.type == "floating": 
        

        elif joint_data.type == "planar": 




def search_link_child_by_name(f_joints, f_name):
    
        ret_joint = None
    
        for joint in f_joints:
    
            if joint.name == f_name:
                ret_joint = joint
                break
    
        return ret_joint


"""
Function that generates the objec parenting tree to with data
needed to put in the tscn object parent properties
"""
def generate_obj_tree(f_joints, f_root_name):

    #Return object tree
    ret_obj_tree = {}


    for joint in f_joints:

        link_path = ""
        parent = joint.parent

        #Get the full path of the element until the root node
        while (parent := search_link_child_by_name(f_joints, parent) ) != None:

            link_path = parent + "/" + link_path
            

        #Add child of this joint to the object tree if needed
        if joint.child not in ret_obj_tree:
            ret_obj_tree[joint.child] = link_path

        #Add joint the the object tree if needed
        if joint.name not in ret_obj_tree:
            ret_obj_tree[joint.name] = link_path

        #Split link_path add add to the object tree the rest of elements if needed
        link_path_split = link_path.split("/")

        root_link = link_path_split.pop(0)

        #Add root link to parent tree with its name
        if root_link not in ret_obj_tree:
            ret_obj_tree[root_link] = root_link

        #Add the rest of link paths to 
        accum_path = root_link

        for i in link_path_split:
            if i not in ret_obj_tree:
                ret_obj_tree[i] = accum_path

            accum_path = accum_path + "/" + i

    return ret_obj_tree



def urdf_python_to_tscn(f_urdf_python, f_outDir, f_tscn_name):

    l_link_nodes = {}

    l_sub_resources_str = ""
    l_extern_resources_str = ""

    #Generate parenting tree for link and joint objects
    l_obj_tree = generate_obj_tree(f_urdf_python["joints"])

    #Generate link nodes
    l_link_nodes = generate_link_nodes(f_urdf_python["links"], f_outDir, l_sub_resources_str, l_extern_resources_str)



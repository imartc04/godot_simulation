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

    return "transform = Transform3D(" + str(l_final_rotation_matrix[0,0]) + ", " + str(l_final_rotation_matrix[1,0]) + ", " + str(l_final_rotation_matrix[2,0]) + ", " + str(l_final_rotation_matrix[0,1]) + ", " + str(l_final_rotation_matrix[1,1]) + ", " + str(l_final_rotation_matrix[2,1]) + str(l_final_rotation_matrix[2,0]) + ", " + str(l_final_rotation_matrix[2,1]) + ", " + str(l_final_rotation_matrix[2,2]) + str(l_origin_vector[0]) + ", " + str(l_origin_vector[1]) + ", " +str(l_origin_vector[2]) + ")"          

def get_filename_and_ext_from_path(f_path):
    return os.path.splitext(os.path.basename(f_path))



def genrate_link_visual_nodes(f_link_visuals, f_link_node, f_sub_resources, f_extern_resources, f_out_dir):

    for  link_data in f_link_visuals:
            
            #Create Nodes for the link visuals
            l_visual_counter = 0
            for visual in link_data.visuals:
                l_visual_node = Node("visual", data=visual, parent=l_link_node)
                

                #Generate tscn string for the visual
                l_tscn_header = '[node name=' + visual.name + '"parent="' + key + '"'


                l_tscn_props = urdf_pos_to_tscn_transform(visual.origin)

                l_geometry_id = ""

                l_visual_node_str = ""

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

                    l_geometry_id = l_visual_node.name + '_visual_' + str(l_visual_counter) + '_mesh_shape'

                    l_external_resources_str += '\n[sub_resource type="PackedScene" id="' + l_geometry_id  + '"'  + ' uid="uid://' + l_geometry_id + '"' + 'path="' + l_mesh_proj_relative + '" ]'


                    l_node_name = "node_" + l_geometry_id

                    l_visual_node_str = '\n[node name="' + l_node_name + '" parent="' + link_data.name + '"' + 'instance=ExtResource("' + l_geometry_id + '")]'
                    

                    
                elif visual.geometry.type == "box":

                    l_geometry_id = l_visual_node.name + '_visual_' + str(l_visual_counter) + '_box_shape'
                
                    l_sub_resources_str += '\n[sub_resource type="BoxShape3D" id="' + l_geometry_id  + '"]'  

                    l_sub_resources_str += '\nsize= Vector3(' + str(visual.geometry.data['size'][0]) + ', ' + str(visual.geometry.data['size'][1]) + ', ' + str(visual.geometry.data['size'][2]) + ')'

                    #Create visual node string for the resource
                    l_visual_node_str = '\n[node name="' + l_node_name + ' type="MeshInstance3D"' '" parent="' + link_data.name +'")]'
                    l_visual_node_str += '\nmesh= SubResource("' + l_geometry_id + '")'

                elif visual.geometry.type == "cylinder":

                    l_geometry_id = l_visual_node.name + '_visual_' + str(l_visual_counter) + '_cylinder_shape'

                    l_sub_resources_str += '\n[sub_resource type="CylinderShape3D" id="' + l_geometry_id  + '"]'

                    l_sub_resources_str += '\nradius= ' + str(visual.geometry.data['radius'])
                    l_sub_resources_str += '\nheight= ' + str(visual.geometry.data['length'])


                    #Create visual node string for the resource
                    l_visual_node_str = '\n[node name="' + l_node_name + ' type="MeshInstance3D"' '" parent="' + link_data.name +'")]'
                    l_visual_node_str += '\nmesh= SubResource("' + l_geometry_id + '")'


                elif visual.geometry.type == "sphere":

                    l_geometry_id = l_visual_node.name + '_visual_' + str(l_visual_counter) + '_sphere_shape'

                    l_sub_resources_str += '\n[sub_resource type="SphereShape3D" id="' + l_geometry_id  + '"]'

                    l_sub_resources_str += '\nradius= ' + str(visual.geometry.data['radius'])

                    #Create visual node string for the resource
                    l_visual_node_str = '\n[node name="' + l_node_name + ' type="MeshInstance3D"' '" parent="' + link_data.name +'")]'
                    l_visual_node_str += '\nmesh= SubResource("' + l_geometry_id + '")'

                else:
                    raise ValueError("Invalid geometry type specified. Must be 'mesh', 'box', 'cylinder' or 'sphere'")


                #Create MeshInstance node string for the resource
                l_visual_node = Node(l_node_name, node_str=l_visual_node_str , parent=l_link_node)


            #Add the link node to the dictionary
            l_link_nodes[key] = l_link_node


def generate_link_nodes(f_urdf_python_links, f_out_dir, f_sub_resources, f_extern_resources):

    l_link_nodes = {}

    l_sub_resources = []

    l_external_resources = []

    #Create link nodes and link child data
    for key, link_data in f_urdf_python_links.items():

        #Create link node
        l_link_node = Node(key)
        l_link_nodes[key] = l_link_node

        genrate_link_visual_nodes(link_data.visuals, l_link_node, l_sub_resources, l_external_resources, f_out_dir)

    return l_link_nodes




def urdf_python_to_tscn(f_urdf_python, f_outDir, f_tscn_name):

    l_link_nodes = {}

    l_sub_resources = []
    l_extern_resources = []

    #Generate link nodes
    l_link_nodes = generate_link_nodes(f_urdf_python.links, f_outDir, l_sub_resources, l_extern_resources)



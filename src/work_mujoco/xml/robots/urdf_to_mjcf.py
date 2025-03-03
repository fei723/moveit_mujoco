#!/usr/bin/env python3

import sys
import os
from io import StringIO
from os import listdir, path
from typing import Optional, Dict

from lxml import etree as ET
from pyassimp.helper import additional_dirs
from xmlformatter import Formatter
import yaml
import logging
import mujoco
import xacro
from ament_index_python.packages import get_package_share_directory

logging.getLogger().setLevel(logging.INFO)

additional_disable_collosions = [
    ("L_palm", "L_thumb_proximal"),
    ("R_palm", "R_thumb_proximal"),
]

kp_kv_mapping = {
    'L_ARM1_SHOULDER_P': ['8000', '100'],
    'L_ARM2_SHOULDER_R': ['8000', '0'],
    'L_ARM3_SHOULDER_Y': ['6000', '0'],
    'L_ARM4_ELBOW_P': ['8000', '0'],
    'L_ARM5_ELBOW_Y': ['3000', '0'],
    'L_ARM6_WRIST_P': ['3000', '0'],
    'L_ARM7_WRIST_R': ['3000', '0'],
    'R_ARM1_SHOULDER_P': ['8000', '200'],
    'R_ARM2_SHOULDER_R': ['8000', '0'],
    'R_ARM3_SHOULDER_Y': ['6000', '0'],
    'R_ARM4_ELBOW_P': ['8000', '0'],
    'R_ARM5_ELBOW_Y': ['3000', '0'],
    'R_ARM6_WRIST_P': ['3000', '0'],
    'R_ARM7_WRIST_R': ['3000', '0'],
    'ANKLE_P': ['80000', '100'],
    'KNEE_P': ['80000', '100'],
    'WAIST_P': ['80000', '100'],
    'WAIST_Y': ['30000', '100'],
    'HEAD1_Y': ['3000', '0'],
    'HEAD2_P': ['3000', '0'],
    'L_pinky_finger_proximal_joint': ['10', '0'],
    'L_pinky_finger_distal_joint': ['10', '0'],
    'L_ring_finger_proximal_joint': ['10', '0'],
    'L_ring_finger_distal_joint': ['10', '0'],
    'L_middle_finger_proximal_joint': ['10', '0'],
    'L_middle_finger_distal_joint': ['10', '0'],
    'L_index_finger_proximal_joint': ['10', '0'],
    'L_index_finger_distal_joint': ['10', '0'],
    'L_thumb_opp_joint': ['30', '0'],
    'L_thumb_proximal_joint': ['10', '0'],
    'L_thumb_middle_joint': ['10', '0'],
    'L_thumb_distal_joint': ['10', '0'],
    'R_pinky_finger_proximal_joint': ['10', '0'],
    'R_pinky_finger_distal_joint': ['10', '0'],
    'R_ring_finger_proximal_joint': ['10', '0'],
    'R_ring_finger_distal_joint': ['10', '0'],
    'R_middle_finger_proximal_joint': ['10', '0'],
    'R_middle_finger_distal_joint': ['10', '0'],
    'R_index_finger_proximal_joint': ['10', '0'],
    'R_index_finger_distal_joint': ['10', '0'],
    'R_thumb_opp_joint': ['10', '0'],
    'R_thumb_proximal_joint': ['10', '0'],
    'R_thumb_middle_joint': ['10', '0'],
    'R_thumb_distal_joint': ['10', '0']
}


def main():
    srdf_file = "m92.srdf"
    urdf_file = "m92_robot_fixed.urdf"
    output_gazebo_file = "m92_robot_fixed_gazebo.urdf"
    output_mjcf_file = "m92uw.xml"

    cur_dir = path.dirname(path.realpath(__file__))

    srdf_path = path.join(cur_dir, srdf_file)
    urdf_path = path.join(cur_dir, urdf_file)
    output_path = path.join(cur_dir, output_mjcf_file)
    output_gazebo_path = path.join(cur_dir, output_gazebo_file)

    with open(srdf_path, "r") as f:
        srdf_tree = ET.parse(f)

    with open(urdf_path, "r") as f:
        urdf_tree = ET.parse(f)

    urdf_root = urdf_tree.getroot()
    mujoco_element = ET.SubElement(urdf_root, "mujoco")
    compiler_attr = {
        'angle': "degree",
        'balanceinertia': "true",
        'convexhull': "false",
        'discardvisual': "true",
        'eulerseq': "xyz",
        'fusestatic': "false",
        'meshdir': "./meshes",
        'strippath': "true"
    }
    compiler_element = ET.SubElement(mujoco_element, "compiler", compiler_attr)

    option_attr = {
        'collision': "all",
        'timestep': "0.001"
    }
    option_element = ET.SubElement(mujoco_element, "option", option_attr)

    ros2_control_tag_attr = {
        'name': 'body_system',
        'type': 'system'
    }
    ros2_control_element = ET.SubElement(urdf_root, "ros2_control", ros2_control_tag_attr)
    hardware_element = ET.SubElement(ros2_control_element, "hardware")
    plugin_element = ET.SubElement(hardware_element, "plugin")
    plugin_element.text = "mujoco_ros2_control/MujocoSystem"
    for joint in urdf_root.xpath(f"//joint"):
        mimic_tags = joint.xpath('.//mimic')
        mimic_from_joint_name = None
        if mimic_tags:
            mimic_tag = mimic_tags[0]
            mimic_from_joint_name = mimic_tag.get('joint')
            mimic_multiplier = mimic_tag.get('multiplier')
        if joint.get('type') != 'fixed':
            joint_name = joint.get('name')
            joint_element = ET.SubElement(ros2_control_element, "joint", {'name': joint_name})
            param_element = ET.SubElement(joint_element, "param", {'name': "initial_position"})
            param_element.text = '0.0'

            if mimic_from_joint_name:
                mimic_element = ET.SubElement(joint_element, "param", {'name': "mimic"})
                mimic_element.text = mimic_from_joint_name
                multiplier_element = ET.SubElement(joint_element, "param", {'name': "multiplier"})
                multiplier_element.text = mimic_multiplier


            command_interface_element = ET.SubElement(joint_element, "command_interface", {'name': 'position'})
            state_interface0_element = ET.SubElement(joint_element, "state_interface", {'name': 'position'})
            state_interface0_para_element = ET.SubElement(state_interface0_element, "param", {'name': 'initial_value'})
            state_interface0_para_element.text = '0.0'
            state_interface1_element = ET.SubElement(joint_element, "state_interface", {'name': 'velocity'})
            state_interface2_element = ET.SubElement(joint_element, "state_interface", {'name': 'effort'})

            transmission_element = ET.SubElement(
                ros2_control_element, "transmission",
                {'name': f"{joint_name}_transmission"}
            )
            transmission_plugin_element = ET.SubElement(transmission_element, "plugin")
            transmission_plugin_element.text = "transmission_interface/SimpleTransmission"

            transmission_joint_element = ET.SubElement(
                transmission_element, "joint",
                {"name": joint_name, "role": "joint1"}
            )
            transmission_joint_mechanical_reduction_element = ET.SubElement(transmission_joint_element,
                                                                            "mechanical_reduction")
            transmission_joint_mechanical_reduction_element.text = '1.0'
            transmission_joint_offset_element = ET.SubElement(transmission_joint_element, "offset")
            transmission_joint_offset_element.text = '0'

            transmission_actuator_element = ET.SubElement(
                transmission_element, "actuator",
                {"name": f"{joint_name}_actuator", "role": "actuator1"}
            )
            transmission_actuator_mechanical_reduction_element = ET.SubElement(transmission_actuator_element, "mechanical_reduction")
            transmission_actuator_mechanical_reduction_element.text = "1.0"
            transmission_actuator_offset_element = ET.SubElement(transmission_actuator_element, "offset")
            transmission_actuator_offset_element.text = "0"
    # <axis xyz="0.0 1.0 0.0"/>
    for axis_ele in urdf_root.xpath(f"//axis"):
        axis_xyz = axis_ele.get('xyz')
        axis_ele.set('xyz', ' '.join([str(int(float(p))) for p in axis_xyz.split(' ')]))

    urdf_tree_string = ET.tostring(urdf_tree).decode('utf-8')

    # Create a new XML with one model
    mujoco_tree = ET.Element('mujoco', model='m92uw')
    contact_tree = ET.Element('contact')

    srdf_root = srdf_tree.getroot()

    spec = mujoco.MjSpec.from_string(urdf_tree_string)
    spec.compile()

    for mimic_joint in urdf_root.xpath('.//mimic'):
        mimic_parent_joint_name = mimic_joint.get('joint')
        mimic_parent_joint_range = None
        for mujoco_joint in spec.joints:
            if mujoco_joint.name == mimic_parent_joint_name:
                mimic_parent_joint_range = mujoco_joint.range
        mimic_joint_multiplier = float(mimic_joint.get('multiplier'))
        mimic_joint_offset = float(mimic_joint.get('offset'))
        mimic_joint_range = mimic_parent_joint_range * mimic_joint_multiplier + mimic_joint_offset

        mimic_joint_parent = mimic_joint.getparent()
        mimic_joint_name = mimic_joint_parent.get('name')
        for mujoco_joint in spec.joints:
            if mujoco_joint.name == mimic_joint_name:
                mujoco_joint.range = mimic_joint_range

    for disable_collisions in srdf_root.findall('disable_collisions'):
        link1 = disable_collisions.get('link1')
        link2 = disable_collisions.get('link2')
        spec.add_exclude(bodyname1=link1, bodyname2=link2)

    # <exclude body1="L_palm" body2="L_thumb_proximal"/>
    for body1, body2 in additional_disable_collosions:
        spec.add_exclude(bodyname1=body1, bodyname2=body2)

    spec.compile()

    mjcf_string = spec.to_xml()
    mjcf = ET.fromstring(mjcf_string)
    actuator = ET.SubElement(mjcf, 'actuator')
    sensor = ET.SubElement(mjcf, 'sensor')

    for mujoco_joint in spec.joints:
        joint_name = mujoco_joint.name
        joint_type = mujoco_joint.type
        if joint_type == mujoco.mjtJoint.mjJNT_HINGE:
            motor_name = joint_name + '_actuator'
            if joint_name not in kp_kv_mapping:
                kp_kv_mapping[joint_name] = ['1', '0']
            # position_attr = {
            #     'name': motor_name,
            #     'ctrllimited': 'true',
            #     'ctrlrange': f'{mujoco_joint.range[0]} {mujoco_joint.range[1]}',
            #     'forcelimited': 'true',
            #     'forcerange': f'{mujoco_joint.actfrcrange[0]} {mujoco_joint.actfrcrange[1]}',
            #     'gear': '1',
            #     'joint': joint_name,
            #     'kp': kp_kv_mapping[joint_name][0],
            #     'kv': kp_kv_mapping[joint_name][1]
            # }
            # ET.SubElement(actuator, 'position', position_attr)
            general_attr = {
                'name': motor_name,
                'joint': joint_name,
                'ctrlrange': f'{mujoco_joint.range[0]} {mujoco_joint.range[1]}',
            }
            ET.SubElement(actuator, 'general', general_attr)

            actuatorfrc_attr = {
                'name': f'{joint_name}_actuatorfrc',
                'actuator': motor_name,
            }
            ET.SubElement(sensor, 'actuatorfrc', actuatorfrc_attr)

            jointpos_attr = {
                'name': f'{joint_name}_jointpos',
                'joint': joint_name,
            }
            ET.SubElement(sensor, 'jointpos', jointpos_attr)

            jointvel_attr = {
                'name': f'{joint_name}_jointvel',
                'joint': joint_name,
            }
            ET.SubElement(sensor, 'jointvel', jointvel_attr)

    mujoco_tree.append(contact_tree)

    for base_body_tag in mjcf.xpath("//body[@name='base']"):
        base_free_joint_attr = {
            'name': 'base_free_joint',
            'type': 'free',
        }
        ET.SubElement(base_body_tag, 'joint', base_free_joint_attr)

    for contains_rgba_tag in mjcf.xpath("//*[@rgba]"):
        rgba_value_list = contains_rgba_tag.get('rgba').split(' ')
        rgba_value_list[-1] = '1'
        new_rgba_value = ' '.join(rgba_value_list)
        contains_rgba_tag.set('rgba', new_rgba_value)

    for distal_body_tag in mjcf.xpath("//body[contains(@name, 'distal')]"):
        distal_body_name = distal_body_tag.get("name")
        if "thumb" in distal_body_name:
            pos_z = 0.02
        else:
            pos_z = 0.04
        site_name = f'{distal_body_name}_site'
        site_attr = {
            'name': site_name,
            'pos': f'0 0 {pos_z}',
            'size': f'0.01',
            'rgba': f'0 1 0 0.2',
            'type': f'sphere',
        }
        ET.SubElement(distal_body_tag, "site", site_attr)

        # <force name="L_pinky_force" site="L_pinky_f" />
        force_attr = {
            'name': f'{distal_body_name}_force_sensor',
            'site': site_name,
        }
        ET.SubElement(sensor, 'force', force_attr)

    for geom_tag in mjcf.xpath("//geom"):
        body_name = geom_tag.getparent().get('name')
        geom_tag.set('name', body_name)

    mjcf_string = ET.tostring(mjcf).decode('utf-8')
    formatter = Formatter(
        indent="2",
        indent_char=" ",
        encoding_output="UTF-8",
        selfclose=True,
        blanks=True,
    )
    formatted_mjcf = formatter.format_string(mjcf_string).decode("utf-8")
    formatted_urdf_tree_string = formatter.format_string(urdf_tree_string).decode("utf-8")

    with open(output_path, "wt") as f:
        f.write(formatted_mjcf)

    with open(output_gazebo_path, "wt") as f:
        f.write(formatted_urdf_tree_string)
    print('Results written into "%s"' % output_path)
    print('Results written into "%s"' % output_gazebo_path)
    print(f"{kp_kv_mapping}")


if __name__ == "__main__":
    main()

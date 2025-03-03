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

PACKAGE_NAME = 'human_description'

def read_joint():
    # Get URDF via xacro
    urdf_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "urdf",
        "M92UW",
        "M92UW.urdf"
    )
    with open(urdf_path, "r") as f:
        urdf_tree = ET.parse(f)

    urdf_root = urdf_tree.getroot()
    for joint in urdf_root.xpath(f"//joint"):
        if joint.get('type') == 'revolute' and not joint.xpath(f".//mimic"):
            print(joint.get('name'))
def read_actuator():
    # Get URDF via xacro
    mjcf_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "mjcf",
        "M92UW",
        "M92UW.mujoco.xml"
    )
    with open(mjcf_path, "r") as f:
        mjcf_tree = ET.parse(f)

    mjcf_root = mjcf_tree.getroot()
    for joint in mjcf_root.xpath(f"//general"):
        print(joint.get('name'))


if __name__ == "__main__":
    read_joint()

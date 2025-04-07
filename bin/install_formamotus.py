#!/usr/bin/env python3

import os
import subprocess

import bpy

base_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..')
zip_path = os.path.join(base_dir, "formamotus.zip")
subprocess.run(["zip", "-r", zip_path, "formamotus"], check=True)
bpy.ops.preferences.addon_install(filepath=zip_path)
bpy.ops.preferences.addon_enable(module="formamotus")
bpy.ops.wm.save_userpref()

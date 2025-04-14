# -----------------------------------------------------------------------------
# MIT License
#
# Copyright (c) 2022 Hoang Giang Nguyen - Institute for Artificial Intelligence,
# University Bremen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# This file includes portions derived from:
# https://github.com/HoangGiang93/urdf_importer/blob/master/urdf_importer_addon/urdf_importer/robot_builder.py
# -----------------------------------------------------------------------------

import os
from shutil import copy
import tempfile
from typing import Dict
from typing import List
from xml.etree import ElementTree


def fix_up_axis_and_get_materials(file_path: str, preserve_original_texture_name: bool = False):
    """
    Modify the up_axis to Z_UP and copy texture images to a temporary directory.

    This function parses a URDF/DAE file, sets the up_axis attribute to "Z_UP",
    and copies texture images referenced in the XML file to a system temporary
    directory. The naming of the texture files is determined by the
    'preserve_original_texture_name' parameter. If any modifications are
    made, the updated XML is written to a new temporary file.

    Parameters
    ----------
    file_path : str
        The path to the input URDF/DAE file.
    preserve_original_texture_name : bool, optional
        If True, the original texture name (prefixed with 'T_') is preserved without hashing.
        Otherwise, a hash of the file path is appended to ensure uniqueness, by default False.

    Returns
    -------
    tuple
        A tuple containing:

        - tmp_file_path (str): The path of the updated file (or the original file path if no changes occurred).
        - mat_sampler2D_dict (Dict[str, Dict[str, str]]): A dictionary mapping material names to another
          dictionary that maps effect parameter names to their corresponding texture paths.
    """
    tree = ElementTree.parse(file_path)
    root = tree.getroot()
    dir_path = os.path.dirname(file_path)
    mat_sampler2D_dict: Dict[str, Dict[str, str]] = {}

    # Initialize dictionaries for material and texture mappings.
    mat_dict: Dict[str, str] = {}
    effect_dict: Dict[str, List[str]] = {}
    sampler2D_dict: Dict[str, str] = {}
    surface_dict: Dict[str, str] = {}
    image_dict: Dict[str, str] = {}

    # Create a temporary texture directory if not already existing.
    tmp_texture_path = os.path.join(tempfile.gettempdir(), "texture")
    if not os.path.exists(tmp_texture_path):
        os.makedirs(tmp_texture_path)

    modified = False  # Flag to check if any modifications were made.

    # Iterate through each element in the XML tree.
    for element in root:
        # Update up_axis value to "Z_UP" in asset element.
        if "asset" in element.tag:
            for child in element:
                if "up_axis" in child.tag:
                    child.text = "Z_UP"
                    modified = True

        # Process material definitions in library_materials.
        elif "library_materials" in element.tag:
            for material in element.findall("material"):
                mat_name = material.get("name")
                instance_effect = material.find("instance_effect")
                if instance_effect is not None:
                    effect_id = instance_effect.get("url", "").lstrip("#")
                    mat_dict[mat_name] = effect_id

        # Process effect definitions in library_effects.
        elif "library_effects" in element.tag:
            for effect in element.findall("effect"):
                effect_id = effect.get("id")
                effect_dict[effect_id] = []
                profile_common = effect.find("profile_COMMON")
                if profile_common is not None:
                    for newparam in profile_common.findall("newparam"):
                        param_name = newparam.get("sid")
                        for child in newparam:
                            # Process the 'surface' tag to retrieve the initial texture.
                            if "surface" in child.tag:
                                init_from = child.find("init_from")
                                if init_from is not None:
                                    surface_dict[param_name] = init_from.text
                            # Process the 'sampler2D' tag to retrieve the texture source.
                            elif "sampler2D" in child.tag:
                                source = child.find("source")
                                if source is not None:
                                    effect_dict[effect_id].append(param_name)
                                    sampler2D_dict[param_name] = source.text

        # Process texture image paths in library_images.
        elif "library_images" in element.tag:
            for image in element.findall("image"):
                image_name = image.get("name")
                init_from = image.find("init_from")
                if init_from is not None:
                    modified = True
                    # Construct the original texture file path.
                    orig_texture_path = os.path.join(dir_path, init_from.text)
                    # Extract the base name and extension.
                    base_name, file_ext = os.path.splitext(os.path.basename(init_from.text))
                    if preserve_original_texture_name:
                        new_file_name = f"T_{base_name}{file_ext}"
                    else:
                        file_hash = str(abs(hash(os.path.dirname(file_path))) % (10**3))
                        new_file_name = f"T_{base_name}_{file_hash}{file_ext}"
                    dst_texture_path = os.path.join(tmp_texture_path, new_file_name)
                    copy(orig_texture_path, dst_texture_path)
                    init_from.text = dst_texture_path
                    image_dict[image_name] = dst_texture_path

    # Build the final material-to-texture mapping.
    for mat_name, effect_id in mat_dict.items():
        mat_sampler2D_dict[mat_name] = {}
        for effect_param in effect_dict.get(effect_id, []):
            sampler2D_name = sampler2D_dict.get(effect_param)
            image_name = surface_dict.get(sampler2D_name)
            image_path = image_dict.get(image_name)
            mat_sampler2D_dict[mat_name][effect_param] = image_path

    # If modifications were made, write the XML to a new temporary file.
    if modified:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".dae", mode="w", encoding="utf-8") as tmp_file:
            tree.write(tmp_file, encoding="unicode")
            tmp_file_path = tmp_file.name
    else:
        tmp_file_path = file_path

    return tmp_file_path, mat_sampler2D_dict

# -----------------------------------------------------------------------------
#
# -*- coding: utf-8 -*-
#
# phlox-libdc1394/phlox1394/_core.py
#
# Copyright (C) 2016, by Matthias Yang Chen <matthias_cy@outlook.com>
# All rights reserved.
#
# phlox-libdc1394 is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# phlox-libdc1394 is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with phlox-libdc1394. If not,
# see <http://www.gnu.org/licenses/>.
# -----------------------------------------------------------------------------
"""
Core functions of libdc1394.
"""

from __future__ import division, print_function, unicode_literals

from ctypes import *
from ctypes.util import find_library
import sys

try:
    _dll = cdll.LoadLibrary(find_library('dc1394'))
except Exception as e:
    raise RuntimeError("Fatal: libdc1394 could not be found or open: %s" % e)






# A list of de-mosaicing techniques for Bayer-patterns.
bayer_methods = {
    'BAYER_METHOD_NEAREST': 0,
    'BAYER_METHOD_SIMPLE': 1,
    'BAYER_METHOD_BILINEAR': 2,
    'BAYER_METHOD_HQLINEAR': 3,
    'BAYER_METHOD_DOWNSAMPLE': 4,
    'BAYER_METHOD_EDGESENSE': 5,
    'BAYER_METHOD_VNG': 6,
    'BAYER_METHOD_AHD': 7,
}

bayer_method_t = c_int

BAYER_METHOD_MIN = bayer_methods['BAYER_METHOD_NEAREST']
BAYER_METHOD_MAX = bayer_methods['BAYER_METHOD_AHD']
BAYER_METHOD_NUM = BAYER_METHOD_MAX - BAYER_METHOD_MIN + 1

# A list of known stereo-in-normal-video modes used by manufacturers like
# Point Grey Research and Videre Design.
stereo_methods = {
    'STEREO_METHOD_INTERLACED': 0,
    'STEREO_METHOD_FIELD': 1,
}

stereo_method_t = c_int

STEREO_METHOD_MIN = stereo_methods['STEREO_METHOD_INTERLACED']
STEREO_METHOD_MAX = stereo_methods['STEREO_METHOD_FIELD']
STEREO_METHOD_NUM = STEREO_METHOD_MAX - STEREO_METHOD_MIN + 1

# Error codes.
errors = {
    'SUCCESS': 0,
    'FAILURE': -1,
    'NOT_A_CAMERA': -2,
    'FUNCTION_NOT_SUPPORTED': -3,
    'CAMERA_NOT_INITIALIZED': -4,
    'MEMORY_ALLOCATION_FAILURE': -5,
    'TAGGED_REGISTER_NOT_FOUND': -6,
    'NO_ISO_CHANNEL': -7,
    'NO_BANDWIDTH': -8,
    'IOCTL_FAILURE': -9,
    'CAPTURE_IS_NOT_SET': -10,
    'CAPTURE_IS_RUNNING': -11,
    'RAW1394_FAILURE': -12,
    'FORMAT7_ERROR_FLAG_1': -13,
    'FORMAT7_ERROR_FLAG_2': -14,
    'INVALID_ARGUMENT_VALUE': -15,
    'REQ_VALUE_OUTSIDE_RANGE': -16,
    'INVALID_FEATURE': -17,
    'INVALID_VIDEO_FORMAT': -18,
    'INVALID_VIDEO_MODE': -19,
    'INVALID_FRAMERATE': -20,
    'INVALID_TRIGGER_MODE': -21,
    'INVALID_TRIGGER_SOURCE': -22,
    'INVALID_ISO_SPEED': -23,
    'INVALID_IIDC_VERSION': -24,
    'INVALID_COLOR_CODING': -25,
    'INVALID_COLOR_FILTER': -26,
    'INVALID_CAPTURE_POLICY': -27,
    'INVALID_ERROR_COD': -28,
    'INVALID_BAYER_METHOD': -29,
    'INVALID_VIDEO1394_DEVICE': -30,
    'INVALID_OPERATION_MODE': -31,
    'INVALID_TRIGGER_POLARITY': -32,
    'INVALID_FEATURE_MODE': -33,
    'INVALID_LOG_TYPE': -34,
    'INVALID_BYTE_ORDER': -35,
    'INVALID_STEREO_METHOD': -36,
    'BASLER_NO_MORE_SFF_CHUNKS': -37,
    'BASLER_CORRUPTED_SFF_CHUNK': -38,
    'BASLER_UNKNOWN_SFF_CHUNK': -39,
}

error_t = c_int

ERROR_MIN = errors['BASLER_UNKNOWN_SFF_CHUNK']
ERROR_MAX = errors['SUCCESS']
ERROR_NUM = ERROR_MAX - ERROR_MIN + 1

# Types of logging messages
logs = {
    'LOG_ERROR': 768,
    'LOG_WARNING': 769,
    'LOG_DEBUG': 770,
}

log_t = c_int

LOG_MIN = logs['LOG_ERROR']
LOG_MAX = logs['LOG_DEBUG']
LOG_NUM = LOG_MAX - LOG_MIN + 1

# Enumeration of iso data speeds.
# Most (if not all) cameras are compatible with 400Mbps speed. Only older
# cameras (pre-1999) may still only work at sub-400.
iso_speeds = {
    'ISO_SPEED_100': 0,
    'ISO_SPEED_200': 1,
    'ISO_SPEED_400': 2,
    'ISO_SPEED_800': 3,
    'ISO_SPEED_1600': 4,
    'ISO_SPEED_3200': 5,
}

iso_speed_t = c_int

ISO_SPEED_MIN = iso_speeds['ISO_SPEED_100']
ISO_SPEED_MAX = iso_speeds['ISO_SPEED_3200']
ISO_SPEED_NUM = ISO_SPEED_MAX - ISO_SPEED_MIN + 1

# Enumeration of video framerates.
framerates = {
    'FRAMERATE_1_875': 32,
    'FRAMERATE_3_75': 33,
    'FRAMERATE_7_5': 34,
    'FRAMERATE_15': 35,
    'FRAMERATE_30': 36,
    'FRAMERATE_60': 37,
    'FRAMERATE_120': 38,
    'FRAMERATE_240': 39,
}

framerate_t = c_int

FRAMERATE_MIN = framerates['FRAMERATE_1_875']
FRAMERATE_MAX = framerates['FRAMERATE_240']
FRAMERATE_NUM = FRAMERATE_MAX - FRAMERATE_MIN + 1

# Operation modes.
operation_modes = {
    'OPERATION_MODE_LEGACY': 480,
    'OPERATION_MODE_1394B': 481,
}

operation_mode_t = c_int

OPERATION_MODE_MIN = operation_modes['OPERATION_MODE_LEGACY']
OPERATION_MODE_MAX = operation_modes['OPERATION_MODE_1394B']
OPERATION_MODE_NUM = OPERATION_MODE_MAX - OPERATION_MODE_MIN + 1

# Capture flags.
capture_flags = {
    'CAPTURE_FLAGS_CHANNEL_ALLOC': 0x00000001,
    'CAPTURE_FLAGS_BANDWIDTH_ALLOC': 0x00000002,
    'CAPTURE_FLAGS_DEFAULT': 0x00000004,
    'CAPTURE_FLAGS_AUTO_ISO': 0x00000008,
}


# ------------------------------- structures ----------------------------------
# A struct containing a list of color codings.
class color_codings_t(Structure):
    _fields_ = [
        ('num', c_uint32),
        ('codings', (ColorCodings)*COLOR_CODING_NUM),
    ]


# A struct containing a list of video modes.
class video_modes_t(Structure):
    _fields_ = [
        ('num', c_uint32),
        ('modes', (VideoModes)*VIDEO_MODE_NUM),
    ]


bool_t = c_int
switch_t = c_int


# Camera structure.
class camera_t(Structure):
    _fields_ = [
        ('guid', c_uint64),
        ('uint', c_int),
        ('unit_spec_ID', c_uint32),
        ('unit_sw_version', c_uint32),
        ('unit_sub_sw_version', c_uint32),
        ('command_registers_base', c_uint32),
        ('unit_directory', c_uint32)
        ('unit_dependent_directory', c_uint32),
        ('advanced_features_csr', c_uint64),
        ('PIO_control_csr', c_uint64),
        ('SIO_control_csr', c_uint64),
        ('strobe_control_csr', c_uint64),
        ('fromat7_csr', (c_uint64)*VIDEO_MODE_FORMAT7_NUM),
        ('iidc_version', iidc_version_t)
    ]

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

from ctypes import cdll, c_void_p, c_int, c_uint32, c_uint64, c_float
from ctypes.util import find_library
from ctypes import POINTER as PTR

from phlox1394._camera import camera_t, camera_list_t
# from phlox1394._capture import *
# from phlox1394._convesions import *
from phlox1394._control import feature_t, featureset_t, feature_info_t, trigger_source_t
from phlox1394._control import feature_modes_t, feature_mode_t, trigger_polarity_t, trigger_sources_t
# from phlox1394._control import *
# from phlox1394._format7 import *
from phlox1394._log import err_val, error_t
from phlox1394._types import bool_t, switch_t
# from phlox1394._video import *
# import sys

# REMINDER:
#  By default the ctypes API does not know or care about how a dll function
#  should be called. This may lead a lot of crashes and memory corruption.
#  I think it is safer to declare all functions here, independent of usage,
#  so at least the API can handle the ingoing/returning parameters properly.

try:
    _dll = cdll.LoadLibrary(find_library('dc1394'))
except Exception as e:
    raise RuntimeError("Fatal: libdc1394 could not be found or open: %s" % e)


# ---------------------------- python functions -------------------------------
# Global Error checking functions
def _errcheck(rtype, func, arg):
    """
    This function checks for the errortypes declared by the error_t.
    Use it for functions with restype = error_t to receive correct error
    messages from the library.
    """
    if rtype != 0:
        raise RuntimeError("Error in dc1394 function call: %s" %
                           err_val(rtype))


# ------------------------ Startup functions: camera.h ------------------------
#   Creates a new context in which cameras can be searched and used. This
#   should be called before using any other libdc1394 funcions.
_dll.dc1394_new.argtypes = None
_dll.dc1394_new.restype = c_void_p

#   Liberates a context. Last function to use in your program. After this, no
#   libdc1394 function can be used.
_dll.dc1394_free.argtypes = [c_void_p]
_dll.dc1394_free.restype = None

# Bus level functions:
#   Sets and gets the broadcast flag of a camera. If the broadcast flag is set,
#   all devices on the bus will execute the command. Useful to sync ISO start
#   commands or setting a bunch of cameras at the same time. Broadcast only
#   works with identical devices (brand/model). If the devices are not
#   identical your mileage may vary. Some cameras may not answer broadcast
#   commands at all. Also, this only works with cameras on the SAME bus
#   (IOW, the same port).
_dll.dc1394_camera_set_broadcast.argtypes = [PTR(camera_t), bool_t]
_dll.dc1394_camera_set_broadcast.restype = error_t
_dll.dc1394_camera_set_broadcast.errcheck = _errcheck

_dll.dc1394_camera_get_broadcast.argtypes = [PTR(camera_t), PTR(bool_t)]
_dll.dc1394_camera_get_broadcast.restype = error_t
_dll.dc1394_camera_get_broadcast.errcheck = _errcheck

# Resets the IEEE1394 bus which camera is attached to. Calling this function
# is "rude" to other devices because it causes them to re-enumerate on the bus
# may cause a temporary disruption in their current activities. Thus, use it
# sparingly. Its primary use is if a program shuts down uncleanly and needs
# to free leftover ISO channels or bandwidth. A bus reset will free those
# things as a side effect.
_dll.dc1394_reset_bus.argtypes = [PTR(camera_t)]
_dll.dc1394_reset_bus.restype = error_t
_dll.dc1394_reset_bus.errcheck = _errcheck

_dll.dc1394_read_cycle_timer.argtypes = [PTR(camera_t), PTR(c_uint32),
                                         PTR(c_uint64)]
_dll.dc1394_read_cycle_timer.restype = error_t
_dll.dc1394_read_cycle_timer.errcheck = _errcheck

# Gets the IEEE 1394 node ID of the camera.
_dll.dc1394_camera_get_node.argtypes = [PTR(camera_t), PTR(c_uint32),
                                        PTR(c_uint32)]
_dll.dc1394_camera_get_node.restype = error_t
_dll.dc1394_camera_get_node.errcheck = _errcheck


# ----------------------------- Camera functions ------------------------------
# Returns the list of cameras available on the computer. If present, multiple
# cards will be probed.
_dll.dc1394_camera_enumerate.argtypes = [c_void_p, PTR(PTR(camera_list_t))]
_dll.dc1394_camera_enumerate.restype = error_t
_dll.dc1394_camera_enumerate.errcheck = _errcheck

# Frees the memory allocated in dc1394_enumerate_cameras for the camera list
_dll.dc1394_camera_free_list.argtypes = [PTR(camera_list_t)]
_dll.dc1394_camera_free_list.restype = None

# Create a new camera on a GUID (Global Unique IDentifier)
_dll.dc1394_camera_new.argtypes = [c_void_p, c_uint64]
_dll.dc1394_camera_new.restype = PTR(camera_t)

# Create a new camera based on a GUID and a unit number(for multi-unit cameras)
_dll.dc1394_camera_new_unit.argtypes = [c_void_p, c_uint64, c_int]
_dll.dc1394_camera_new_unit.restype = PTR(camera_t)

# Free a camera structure
_dll.dc1394_camera_free.argtypes = [PTR(camera_t)]
_dll.dc1394_camera_free.restype = None

# Print various camera information, ushc as GUID, vendor, model, supported IIDC
# specs, etc...
# dc1394error_t dc1394_camera_print_info(dc1394camera_t *camera, FILE *fd);
_dll.dc1394_camera_print_info.argtypes = [PTR(camera_t), c_void_p]
_dll.dc1394_camera_print_info.restype = error_t
_dll.dc1394_camera_print_info.errcheck = _errcheck

# ------------------------ Feature control: control.h  ------------------------
# Collects the available features fro the camera described by node and stores
# in features.
_dll.dc1394_feature_get_all.argtypes = [PTR(camera_t), PTR(featureset_t)]
_dll.dc1394_feature_get_all.restype = error_t
_dll.dc1394_feature_get_all.errcheck = _errcheck

# Stores the bounds and options associated with the feature described by
# feature->feature_id
_dll.dc1394_feature_get.argtypes = [PTR(camera_t), PTR(feature_info_t)]
_dll.dc1394_feature_get.restype = error_t
_dll.dc1394_feature_get.errcheck = _errcheck

# Displays the bounds and options of the given feature
# dc1394error_t dc1394_feature_print(dc1394feature_info_t *feature, FILE *fd);
_dll.dc1394_feature_print.argtypes = [PTR(feature_info_t), c_void_p]
_dll.dc1394_feature_print.restype = error_t
_dll.dc1394_feature_print.errcheck = _errcheck

# Displays the bounds and options of every feature supported by the camera
_dll.dc1394_feature_print_all.argtypes = [PTR(featureset_t), c_void_p]
_dll.dc1394_feature_print_all.restype = error_t
_dll.dc1394_feature_print_all.errcheck = _errcheck

# White balance: get/set
_dll.dc1394_feature_whitebalance_get_value.argtypes = [PTR(camera_t),
                                                       PTR(c_uint32),
                                                       PTR(c_uint32)]
_dll.dc1394_feature_whitebalance_get_value.restype = error_t
_dll.dc1394_feature_whitebalance_get_value.errcheck = _errcheck

_dll.dc1394_feature_whitebalance_set_value.argtypes = [PTR(camera_t),
                                                       c_uint32,
                                                       c_uint32]
_dll.dc1394_feature_whitebalance_set_value.restype = error_t
_dll.dc1394_feature_whitebalance_set_value.errcheck = _errcheck

# Temperature: get/set
_dll.dc1394_feature_temperature_get_value.argtypes = [PTR(camera_t),
                                                      PTR(c_uint32),
                                                      PTR(c_uint32)]
_dll.dc1394_feature_temperature_get_value.restype = error_t
_dll.dc1394_feature_temperature_get_value.errcheck = _errcheck

_dll.dc1394_feature_temperature_set_value.argtypes = [PTR(camera_t), c_uint32]
_dll.dc1394_feature_temperature_set_value.restype = error_t
_dll.dc1394_feature_temperature_set_value.errcheck = _errcheck

# White shading: get/set
_dll.dc1394_feature_whiteshading_get_value.argtypes = [PTR(camera_t),
                                                       PTR(c_uint32),
                                                       PTR(c_uint32),
                                                       PTR(c_uint32)]
_dll.dc1394_feature_whiteshading_get_value.restype = error_t
_dll.dc1394_feature_whiteshading_get_value.errcheck = _errcheck

_dll.dc1394_feature_whiteshading_set_value.argtypes = [PTR(camera_t),
                                                       c_uint32,
                                                       c_uint32, c_uint32]
_dll.dc1394_feature_whiteshading_set_value.restype = error_t
_dll.dc1394_feature_whiteshading_set_value.errcheck = _errcheck

# Feature value: get/set
_dll.dc1394_feature_get_value.argtypes = [PTR(camera_t), PTR(feature_t),
                                          PTR(c_uint32)]
_dll.dc1394_feature_get_value.restype = error_t
_dll.dc1394_feature_get_value.errcheck = _errcheck

_dll.dc1394_feature_set_value.argtypes = [PTR(camera_t), feature_t, c_uint32]
_dll.dc1394_feature_set_value.restype = error_t
_dll.dc1394_feature_set_value.errcheck = _errcheck

# Tells whether a feature is present or not
_dll.dc1394_feature_is_present.argtypes = [PTR(camera_t), feature_t,
                                           PTR(bool_t)]
_dll.dc1394_feature_is_present.restype = error_t
_dll.dc1394_feature_is_present.errcheck = _errcheck

# Tells whether a feature is readable or not
_dll.dc1394_feature_is_readable.argtypes = [PTR(camera_t), feature_t,
                                            PTR(bool_t)]
_dll.dc1394_feature_is_readable.restype = error_t
_dll.dc1394_feature_is_readable.errcheck = _errcheck

# Gets the boundaries of a feature
_dll.dc1394_feature_get_boundaries.argtypes = [PTR(camera_t), feature_t,
                                               PTR(c_uint32), PTR(c_uint32)]
_dll.dc1394_feature_get_boundaries.restype = error_t
_dll.dc1394_feature_get_boundaries.errcheck = _errcheck

# Tells whether a feature is switcheable or not (ON/OFF)
_dll.dc1394_feature_is_switchable.argtypes = [PTR(camera_t), feature_t,
                                              PTR(bool_t)]
_dll.dc1394_feature_is_switchable.restype = error_t
_dll.dc1394_feature_is_switchable.errcheck = _errcheck

# Power status of a feature (ON/OFF): get/set
_dll.dc1394_feature_get_power.argtypes = [PTR(camera_t), feature_t,
                                          PTR(switch_t)]
_dll.dc1394_feature_get_power.restype = error_t
_dll.dc1394_feature_get_power.errcheck = _errcheck

_dll.dc1394_feature_set_power.argtypes = [PTR(camera_t), feature_t,
                                          switch_t]
_dll.dc1394_feature_set_power.restype = error_t
_dll.dc1394_feature_set_power.errcheck = _errcheck

# Gets the list of control modes for a feature (manual, auto, etc...)
_dll.dc1394_feature_get_modes.argtypes = (PTR(camera_t), feature_t,
                                          PTR(feature_modes_t))
_dll.dc1394_feature_get_modes.restype = error_t
_dll.dc1394_feature_get_modes.errcheck = _errcheck

# Current control modes for a feature: get/set
_dll.dc1394_feature_get_mode.argtypes = [PTR(camera_t), feature_t,
                                         PTR(feature_mode_t)]
_dll.dc1394_feature_get_mode.restype = error_t
_dll.dc1394_feature_get_mode.errcheck = _errcheck

_dll.dc1394_feature_set_mode.argtypes = [PTR(camera_t), feature_t,
                                         feature_mode_t]
_dll.dc1394_feature_set_mode.restype = error_t
_dll.dc1394_feature_set_mode.errcheck = _errcheck

# Tells whether a feature can be controlled in absolute mode
_dll.dc1394_feature_has_absolute_control.argtypes = [PTR(camera_t),
                                                     feature_t, PTR(bool_t)]
_dll.dc1394_feature_has_absolute_control.restype = error_t
_dll.dc1394_feature_has_absolute_control.errcheck = _errcheck

# Gets the absolute boundaries of a feature
_dll.dc1394_feature_get_absolute_boundaries.argtypes = [PTR(camera_t),
                                                        feature_t,
                                                        PTR(c_float),
                                                        PTR(c_float)]
_dll.dc1394_feature_get_absolute_boundariesrestype = error_t
_dll.dc1394_feature_get_absolute_boundaries.errcheck = _errcheck

# Absolute value of a feature: get/set
_dll.dc1394_feature_get_absolute_value.argtypes = [PTR(camera_t), feature_t,
                                                   PTR(c_float)]
_dll.dc1394_feature_get_absolute_value.restype = error_t
_dll.dc1394_feature_get_absolute_value.errcheck = _errcheck

_dll.dc1394_feature_set_absolute_value.argtypes = [PTR(camera_t), feature_t,
                                                   c_float]
_dll.dc1394_feature_set_absolute_value.restype = error_t
_dll.dc1394_feature_set_absolute_value.errcheck = _errcheck

# The status of absolute control of a feature(ON/OFF): get/set
_dll.dc1394_feature_get_absolute_control.argtypes = [PTR(camera_t),
                                                     feature_t, PTR(switch_t)]
_dll.dc1394_feature_get_absolute_control.restype = error_t
_dll.dc1394_feature_get_absolute_control.errcheck = _errcheck

_dll.dc1394_feature_set_absolute_control.argtypes = [PTR(camera_t),
                                                     feature_t, switch_t]
_dll.dc1394_feature_set_absolute_control.restype = error_t
_dll.dc1394_feature_set_absolute_control.errcheck = _errcheck

# ----------------------------------- Trigger ---------------------------------
# The polarity of the external trigger: get/set
_dll.dc1394_external_trigger_get_polarity.argtypes = [PTR(camera_t),
                                                      PTR(trigger_polarity_t)]
_dll.dc1394_external_trigger_get_polarity.restype = error_t
_dll.dc1394_external_trigger_get_polarity.errcheck = _errcheck

_dll.dc1394_external_trigger_set_polarity.argtypes = [PTR(camera_t),
                                                      trigger_polarity_t]
_dll.dc1394_external_trigger_set_polarity.restype = error_t
_dll.dc1394_external_trigger_set_polarity.errcheck = _errcheck

# Tells whether the external trigger can change its polarity or not
_dll.dc1394_external_trigger_has_polarity.argtypes = [PTR(camera_t), PTR(bool_t)]
_dll.dc1394_external_trigger_has_polarity.restype = error_t
_dll.dc1394_external_trigger_has_polarity.errcheck = _errcheck

# Switch between internal and external trigger
_dll.dc1394_external_trigger_set_power.argtypes = [PTR(camera_t), switch_t]
_dll.dc1394_external_trigger_set_power.restype = error_t
_dll.dc1394_external_trigger_set_power.errcheck = _errcheck

# Gets the status of the external trigger
_dll.dc1394_external_trigger_get_power.argtypes = [PTR(camera_t), PTR(switch_t)]
_dll.dc1394_external_trigger_get_power.restype = error_t
_dll.dc1394_external_trigger_get_power.errcheck = _errcheck

# External trigger mode: get/set
_dll.dc1394_external_trigger_get_mode.argtypes = [PTR(camera_t), PTR(trigger_mode_t)]
_dll.dc1394_external_trigger_get_mode.restype = error_t
_dll.dc1394_external_trigger_get_mode.errcheck = _errcheck

_dll.dc1394_external_trigger_set_mode.argtypes = [PTR(camera_t), trigger_mode_t]
_dll.dc1394_external_trigger_set_mode.restype = error_t
_dll.dc1394_external_trigger_set_mode.errcheck = _errcheck

# External trigger source: get/set
_dll.dc1394_external_trigger_get_source.argtypes = [PTR(camera_t), PTR(trigger_source_t)]
_dll.dc1394_external_trigger_get_source.restype = error_t
_dll.dc1394_external_trigger_get_source.errcheck = _errcheck

_dll.dc1394_external_trigger_set_source.argtypes = [PTR(camera_t), trigger_source_t]
_dll.dc1394_external_trigger_set_source.restype = error_t
_dll.dc1394_external_trigger_set_source.errcheck = _errcheck

# Gets the list of available external trigger source
_dll.dc1394_external_trigger_get_supported_sources.argtypes = [PTR(camera_t),
                                                               PTR(trigger_sources_t)]
_dll.dc1394_external_trigger_get_supported_sources.restype = error_t
_dll.dc1394_external_trigger_get_supported_sources.errcheck = _errcheck

# Turn software trigger on or off
_dll.dc1394_software_trigger_set_power.argtypes = [PTR(camera_t), switch_t]
_dll.dc1394_software_trigger_set_power.restype = error_t
_dll.dc1394_software_trigger_set_power.errcheck = _errcheck

# Gets the state of software trigger
_dll.dc1394_software_trigger_get_power.argtypes = [PTR(camera_t), PTR(switch_t)]
_dll.dc1394_software_trigger_get_power.restype = error_t
_dll.dc1394_software_trigger_get_power.errcheck = _errcheck


# ------------------------ PIO, SIO and Strobe Functions ----------------------
# Sends a quadlet on the PIO (output)
_dll.dc1394_pio_set.argtypes = [PTR(camera_t), c_uint32]
_dll.dc1394_pio_set.restype = error_t
_dll.dc1394_pio_set.errcheck = _errcheck

# Gets the current quadlet at the PIO (input)
_dll.dc1394_pio_get.argtypes = [PTR(camera_t), PTR(c_uint32)]
_dll.dc1394_pio_get.restype = error_t
_dll.dc1394_pio_get.errcheck = _errcheck

# ------ other functionalities ------
# reset a camera to factory default settings
_dll.dc1394_camera_reset.argtypes = [PTR(camera_t)]
_dll.dc1394_camera_reset.restype = error_t
_dll.dc1394_camera_reset.errcheck = _errcheck

# Turn a camera on or off
_dll.dc1394_camera_set_power.argtypes = [PTR(camera_t), switch_t]
_dll.dc1394_camera_set_power.restype = error_t
_dll.dc1394_camera_set_power.errcheck = _errcheck

# Download a camera setup from the memory
_dll.dc1394_memory_busy.argtypes = [PTR(camera_t), PTR(bool_t)]
_dll.dc1394_memory_busy.restype = error_t
_dll.dc1394_memory_busy.errcheck = _errcheck

# Uploads a camera setup in the memory
# Note that this operation can only be performed a certain number of
# times for a given camera, as it requires reprogramming of an EEPROM.
_dll.dc1394_memory_save.argtypes = [PTR(camera_t), c_uint32]
_dll.dc1394_memory_save.restype = error_t
_dll.dc1394_memory_save.errcheck = _errcheck

# Tells whether the writing of the camera setup in memory is finished or not
_dll.dc1394_memory_load.argtypes = [PTR(camera_t), c_uint32]
_dll.dc1394_memory_load.restype = error_t
_dll.dc1394_memory_load.errcheck = _errcheck

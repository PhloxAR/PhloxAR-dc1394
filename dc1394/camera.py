# -*- coding: utf-8 -*-

from __future__ import division, print_function
from __future__ import absolute_import, unicode_literals
from ctypes import pointer, byref, POINTER, c_char
from numpy import fromstring, ndarray
from threading import Thread, Lock
from Queue import Queue, Full

from .core import *
from .mode import mode_map, create_mode


class DC1394Error(Exception):
    """
    Base class for exceptions.
    """
    pass


class CameraError(DC1394Error, RuntimeError):
    pass


class DC1394Library(object):
    """
    This wraps the dc1394 library object which is a nuisance to have around.
    This is bad design on behave of DC1394. Oh well... This object must stay
    valid until all cameras are closed. But then use it well: it not only
    opens the library, collects a reference to the library and the camera list.
    """
    def __init__(self):
        self._dll = dll
        self._handle = dll.dc1394_new()

    def __del__(self):
        self.close()

    @property
    def handle(self):
        """
        The handle to the library context.
        """
        return self._handle

    def close(self):
        """
        Close the library permanently. All camera handles created with it
        are invalidated by this.
        """
        if self._handle is not None:
            self._dll.dc1394_free(self._handle)

        self._handle = None

    def enumerate_cameras(self):
        """
        Enumerate the cameras currently attached to the bus.
        :return: a list of dictionaries with the following keys per camera:
                  - unit
                  - guid
                  - vendor
                  - model
        """
        l = POINTER(camera_list_t)()
        self._dll.dc1394_camera_enumerate(self.handle, byref(l))

        # cams -> clist renamed for using cam as a camera
        clist = []
        for i in range(l.contents.num):
            ids = l.contents.ids[i]
            # we can be nice to the users, providing some more
            # than mere GUID and unitIDs
            # also, if this fails, we have a problem.
            cam = self._dll.dc1394_camera_new(self.handle, ids.guid)

            # it seems not all cameras have these fields:
            vendor = cam.contents.vendor if cam.contents.vendor else 'unknown'
            model = cam.contents.model if cam.contents.model else 'unknown'

            clist.append({
                'uint': ids.unit,
                'guid': ids.guid,
                'vendor': vendor,
                'model': model
            })

            self._dll.dc1394_camera_free(cam)

        self._dll.dc1394_camera_free_list(l)

        return clist


class DC1394Image(ndarray):
    """
    This class is the image returned by the camera. It is basically a
    numpy array with some additional information (like timestamps).
    It is not based on the video_frame structure of the dc1394, but
    rather augments the information from numpy through information
    of the acquisition of this image.
    """
    # ROI position
    _position = None
    # Size of a data packet in bytes
    _packet_size = None
    # Number of packets per frame
    _packets_per_frame = None
    # IEEE bus time when the picture was acquired
    _timestamp = None
    # Number of frames in the ring buffer that are yet to be accessed by user
    _frames_behind = None
    # frame position in the ring buffer
    _id = None

    @property
    def position(self):
        """
        ROI position (offset)
        """
        return self._position

    @property
    def packet_size(self):
        """
        The size of a data packet in bytes.
        """
        return self._packet_size

    @property
    def packets_per_frame(self):
        """
        Number of packets per frame.
        """
        return self._packets_per_frame

    @property
    def timestamp(self):
        """
        The IEEE Bus time when the picture was acquired (microseconds)
        """
        return self._timestamp

    @property
    def frames_behind(self):
        """
        The number of frames in the ring buffer that are yet to be accessed
        by the user
        """
        return self._frames_behind

    @property
    def id(self):
        """
        The frame position in the ring buffer.
        """
        return self._id


class _CamAcquisitionThread(Thread):
    """
    This class is created and launched whenever a camera is start.
    It continuously acquires the pictures from the camera and sets
    a condition to inform other threads of the arrival of a new
    picture.
    """
    _cam = None
    _should_abort = None
    _last_frame = None
    _condition = None
    # a Lock object from the threading module
    _abort_lock = None

    def __init__(self, cam, condition):
        super(_CamAcquisitionThread, self).__init__()
        self._cam = cam
        self._should_abort = False
        self._last_frame = None
        self._condition = condition
        self._abort_lock = Lock()
        self.start()

    def abort(self):
        self._abort_lock.acquire()
        self._should_abort = True
        self._abort_lock.release()

    def run(self):
        """
        Core function which contains the acquisition loop
        """
        while True:
            self._abort_lock.acquire()
            sa = self._should_abort
            self._abort_lock.release()

            if sa:
                break

            if self._last_frame:
                self._cam._dll.dc1394_capture_enqueue(self._cam._cam,
                                                      self._last_frame)

            frame = POINTER(video_frame_t)()
            self._cam._dll.dc1394_capture_dequeue(
                    self._cam._cam,
                    capture_policies['CAPTURE_POLICY_WAIT'], byref(frame))

            dtype = c_char * frame.contents.image_bytes
            buf = dtype.from_address(frame.contents.image)

            self._last_frame = frame
            self._condition.acquire()

            # generate an Image class from the buffer
            img = fromstring(buf, dtype=self._cam.mode.dtype).reshape(
                self._cam.mode.shape
            ).view(DC1394Image)

            img._position = frame.contents.position
            img._packet_size = frame.contents.packet_size,
            img._packets_per_frame = frame.contents.packets_per_frame
            img._timestamp = frame.contents.timestamp
            img._frames_behind = frame.contents.frames_behind
            img._id = frame.contents.id
            self._cam._current_img = img

            # is the camera streaming to a queue?
            if self._cam._queue:
                # will throw an exception if you're to slow while processing
                self._cam._queue.put_nowait(img)

            self._condition.notifyAll()
            self._condition.release()

        # return the last frame
        if self._last_frame:
            self._cam._dll.dc1394_capture_enqueue(self._cam._cam,
                                                  self._last_frame)
        self._last_frame = None

# -*- coding: utf-8 -*-
# Part of the PsychoPy library
# Copyright (C) 2012-2020 iSolver Software Solutions (C) 2021 Open Science Tools Ltd.
# Distributed under the terms of the GNU General Public License (GPL).
from psychopy.iohub.devices.eyetracker.eye_events import *
from psychopy.iohub.devices import Computer, Device, ioDeviceError
from psychopy.iohub.devices.eyetracker import EyeTrackerDevice
from psychopy.iohub.constants import EventConstants, DeviceConstants, EyeTrackerConstants
from psychopy.iohub.errors import print2err, printExceptionDetailsToStdErr

from .calibration import EyeLogicCalibrationProcedure

from eyelogic.ELApi import *

from screeninfo import get_monitors, Enumerator

import numpy as np
import copy
import inspect
import traceback
import typing as T
import time

LOG_LEVEL = 0

def log_local(level: int = 0, message: str | None = None):
    if level > LOG_LEVEL:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    if message is None:
        print2err('{:s} : {:20s} : {:5d}'.format(info.filename[-20:], info.function, int(info.lineno)))
    else:
        print2err('{:s} : {:20s} : {:5d} : {:s}'.format(info.filename[-20:], info.function, int(info.lineno), message))


class EyeTracker(EyeTrackerDevice):
    plugin = "psychopy-eyetracker-eyelogic"
    # specify what libraries it has code for - PsychoPy and/or PsychoJS
    targets = ["PsychoPy"]
    """The EyeTrackerDevice class is the main class for the ioHub Common Eye
    Tracker interface.

    The Common Eye Tracker Interface--a set of common functions and methods
    such that the same experiment script and data analyses can be shared,
    used, and compared regardless of the actual eye tracker used--works by
    extending the EyeTrackerDevice class to configure device monitoring and
    data access to individual eye tracker manufacturers and models.

    Not every EyeTrackerDevice subclass will support all of the umbrella functionality
    within the Common Eye Tracker Interface, but a core set of critical functions are
    supported by all eye tracker models to date. Any Common Eye Tracker Interface
    method not supported by the configured Eye Tracker hardware returns a constant
    (EyeTrackerConstants.FUNCTIONALITY_NOT_SUPPORTED).

    Methods in the EyeTrackerDevice class are broken down into several categories:

    #. Initializing the Eye Tracker / Setting the Device State.
    #. Defining the Graphics Layer for Calibration / System Setup.
    #. Starting and Stopping of Data Recording.
    #. Sending Messages or Codes to Synchronize the ioHub with the Eye Tracker.
    #. Accessing Eye Tracker Data During Recording.
    #. Accessing the Eye Tracker native time base.
    #. Synchronizing the ioHub time base with the Eye Tracker time base

    .. note::

        Only **one** instance of EyeTracker can be created within an experiment.
        Attempting to create > 1 instance will raise an exception.

    """

    # Used to hold the EyeTracker subclass instance to ensure only one instance of
    # a given eye tracker type is created. This is a current ioHub limitation, not the limitation of
    # all eye tracking hardware.
    _INSTANCE = None

    #: The multiplier needed to convert a device's native time base to sec.msec-usec times.
    DEVICE_TIMEBASE_TO_SEC = 1

    # Used by pyEyeTrackerDevice implementations to store relationships between an eye
    # trackers command names supported for EyeTrackerDevice sendCommand method and
    # a private python function to call for that command. This allows an implementation
    # of the interface to expose functions that are not in the core EyeTrackerDevice spec
    # without have to use the EXT extension class.
    _COMMAND_TO_FUNCTION = {}

    DEVICE_TYPE_ID = DeviceConstants.EYETRACKER
    DEVICE_TYPE_STRING = 'EYETRACKER'

    EVENT_CLASS_NAMES = [
        'MonocularEyeSampleEvent',
        'BinocularEyeSampleEvent',
        'FixationStartEvent',
        'FixationEndEvent',
        'SaccadeStartEvent',
        'SaccadeEndEvent',
        'BlinkStartEvent',
        'BlinkEndEvent']

    __slots__ = [
        '_elapi',
        '_event_callback',
        '_sample_callback',
        '_is_recording',
        '_tracker_below',
        '_timestamp_anchor',
        '_tracker_infront',
        '_latest_sample',
        '_latest_gaze_position',
        '_runtime_settings']

    _event_callback: EventCallback | None
    _sample_callback: GazeSampleCallback | None
    _is_recording: bool
    _tracker_below: float | None
    _tracker_infront: float | None
    _timestamp_anchor: T.Tuple[float, float] | None

    def __init__(self, *args, **kwargs):
        log_local(0)
        EyeTrackerDevice.__init__(self, *args, **kwargs)
        log_local(0)
        self._event_callback = None
        self._sample_callback = None
        self._timestamp_anchor = None

        self._elapi = ELApi('psychopy client')
        log_local(1)
        self._elapi.registerEventCallback(self.getEventCallback())
        self._elapi.registerGazeSampleCallback(self.getSampleCallback())
        log_local(1)
        self._elapi.connect()
        log_local(1)
        assert self._elapi is not None
        log_local(0)
        self._is_recording = False
        self._tracker_below = None
        self._tracker_infront = None

    def getSampleCallback(self):
        log_local(1)
        @GazeSampleCallback
        def sampleCallback(sample: POINTER(ELGazeSample)):
            log_local(4)
            if sample.contents.index % 60 == 0:
                if (sample.contents.porRawX == ELInvalidValue):
                    porStr = "INVALID"
                else:
                    porStr = "{}, {}".format(sample.contents.porRawX,
                                             sample.contents.porRawY)
                log_local(3, message='GazeSample: time={}, index={}, POR = {}'.format(sample.contents.timestampMicroSec,
                                                                       sample.contents.index, porStr))

            if sample.contents.porRawX == ELInvalidValue:
                log_local(4)
                return
            log_local(4)
            self._handleNativeEvent(sample)

        if self._sample_callback is None:
            log_local(1)
            self._sample_callback = GazeSampleCallback(sampleCallback)

        log_local(1)
        return self._sample_callback

    def getEventCallback(self):

        @EventCallback
        def eventCallback(event: ELEvent):
            if (ELEvent(event) == ELEvent.DEVICE_CONNECTED):
                log_local()
                pass
            elif (ELEvent(event) == ELEvent.CONNECTION_CLOSED):
                log_local()
                self.setRecordingState(False)
                self.setConnectionState(False)
            elif (ELEvent(event) == ELEvent.SCREEN_CHANGED) or (ELEvent(event) == ELEvent.DEVICE_DISCONNECTED):
                log_local()
                # SCREEN_CHANGED should only be possible while setRecordingState is False anyway
                self.setRecordingState(False)
            elif (ELEvent(event) == ELEvent.TRACKING_STOPPED):
                log_local()
                self.setRecordingState(False)
            log_local()


        if self._event_callback is None:
            self._event_callback = EventCallback(eventCallback)

        return self._event_callback

    def enableEventReporting(self, enabled=True):
        """enableEventReporting is functionally identical to the eye tracker
        device specific enableEventReporting method."""
        try:
            log_local(0)
            enabled = EyeTrackerDevice.enableEventReporting(self, enabled)
            log_local(0)
            self.setRecordingState(enabled)
            return enabled
        except Exception as e:
            log_local(0, 'Error during enableEventReporting')
            printExceptionDetailsToStdErr()
        return EyeTrackerConstants.EYETRACKER_ERROR

    def _handleNativeEvent(self, *args, **kwargs):
        """The _handleEvent method can be used by the native device interface
                (implemented by the ioHub Device class) to register new native device
                events by calling this method of the ioHub Device class.

                When a native device interface uses the _handleNativeEvent method it is
                employing an event callback approach to notify the ioHub Process when new
                native device events are available. This is in contrast to devices that use
                a polling method to check for new native device events, which would implement
                the _poll() method instead of this method.

                Generally speaking this method is called by the native device interface
                once for each new event that is available for the ioHub Process. However,
                with good cause, there is no reason why a single call to this
                method could not handle multiple new native device events.

                .. note::
                    If using _handleNativeEvent, be sure to remove the device_timer
                    property from the devices configuration section of the iohub_config.yaml.

                Any arguments or kwargs passed to this method are determined by the ioHub
                Device implementation and should contain all the information needed to create
                an ioHub Device Event.

                Since any callbacks should take as little time to process as possible,
                a two stage approach is used to turn a native device event into an ioHub
                Device event representation:
                    #. This method is called by the native device interface as a callback, providing the necessary information to be able to create an ioHub event. As little processing should be done in this method as possible.
                    #. The data passed to this method, along with the time the callback was called, are passed as a tuple to the Device classes _addNativeEventToBuffer method.
                    #. During the ioHub Servers event processing routine, any new native events that have been added to the ioHub Server using the _addNativeEventToBuffer method are passed individually to the _getIOHubEventObject method, which must also be implemented by the given Device subclass.
                    #. The _getIOHubEventObject method is responsible for the actual conversion of the native event representation to the required ioHub Event representation for the accociated event type.

                Args:
                    args(tuple): tuple of non keyword arguments passed to the callback.

                Kwargs:
                    kwargs(dict): dict of keyword arguments passed to the callback.

                Returns:
                    None

                """

        log_local(4)

        sample = copy.deepcopy(args[0].contents)
        local_time = Computer.getTime()
        sample_time = self._sampleTimeToDeviceTime(sample.timestampMicroSec)
        delay = local_time - sample_time
        event_time = sample_time
        self._addNativeEventToBuffer((local_time, sample_time, event_time, delay, sample))
        log_local(4)

    def _getIOHubEventObject(self, native_event_data):
        """The _getIOHubEventObject method is called by the ioHub Process to
        convert new native device event objects that have been received to the
        appropriate ioHub Event type representation.

        If the ioHub Device has been implemented to use the _poll() method of checking for
        new events, then this method simply should return what it is passed, and is the
        default implementation for the method.

        If the ioHub Device has been implemented to use the event callback method
        to register new native device events with the ioHub Process, then this method should be
        overwritten by the Device subclass to convert the native event data into
        an appropriate ioHub Event representation. See the implementation of the
        Keyboard or Mouse device classes for an example of such an implementation.

        Args:
            native_event_data: object or tuple of (callback_time, native_event_object)

        Returns:
            tuple: The appropriate ioHub Event type in list form.

        """
        log_local(4)
        logged_time, sample_time, event_time, delay, gaze_sample = native_event_data

        def replaceInvalid( value ):
            return np.nan if value == ELInvalidValue else value

        sample_por_left_X, sample_por_left_Y = self._eyeTrackerToDisplayCoords([
            replaceInvalid(gaze_sample.porLeftX), replaceInvalid(gaze_sample.porLeftY)])
        sample_eye_left_X = replaceInvalid(gaze_sample.eyePositionLeftX)
        sample_eye_left_Y = replaceInvalid(gaze_sample.eyePositionLeftY)
        sample_eye_left_Z = replaceInvalid(gaze_sample.eyePositionLeftZ)
        sample_radius_left = replaceInvalid(gaze_sample.pupilRadiusLeft)
        sample_por_right_X, sample_por_right_Y = self._eyeTrackerToDisplayCoords([replaceInvalid(gaze_sample.porRightX), replaceInvalid(gaze_sample.porRightY)])
        sample_eye_right_X = replaceInvalid(gaze_sample.eyePositionRightX)
        sample_eye_right_Y = replaceInvalid(gaze_sample.eyePositionRightY)
        sample_eye_right_Z = replaceInvalid(gaze_sample.eyePositionRightZ)
        sample_radius_right = replaceInvalid(gaze_sample.pupilRadiusRight)
        # I do not see where to fit the below parameters into BinocularEyeSampleEvent
        sample_index = gaze_sample.index
        sample_por_raw_X, sample_por_raw_Y = self._eyeTrackerToDisplayCoords([gaze_sample.porRawX, gaze_sample.porRawY])
        sample_por_filtered_X, sample_por_filtered_Y = self._eyeTrackerToDisplayCoords([gaze_sample.porFilteredX, gaze_sample.porFilteredY])

        status = 0
        if sample_por_left_X == ELInvalidValue:
            status += 20
        if sample_por_right_X == ELInvalidValue:
            status += 2

        confidence_interval = -1.

        eventTypeBinoc = EventConstants.BINOCULAR_EYE_SAMPLE
        self._latest_sample = [
        # DeviceEvent
        0, # experiment_id
        0, # session_id
        0, # device_id
        Device._getNextEventID(), # event_id
        eventTypeBinoc, # type
        sample_time, # device_time
        logged_time, # logged_time
        event_time, # event time
        confidence_interval, # confidence_interval
        delay, # delay
        0, # filter_id
        # BinocularEyeSampleEvent
        EyeTrackerConstants.UNDEFINED, # left_gaze_x, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_gaze_y, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_gaze_z, 'f4'
        sample_eye_left_X, # left_eye_cam_x, 'f4'
        sample_eye_left_Y, # left_eye_cam_y, 'f4'
        sample_eye_left_Z, # left_eye_cam_z, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_angle_x, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_angle_y, 'f4'
        sample_por_left_X, # left_raw_x, 'f4'
        sample_por_left_Y, # left_raw_y, 'f4'
        sample_radius_left, # left_pupil_measure1, 'f4'
        EyeTrackerConstants.PUPIL_DIAMETER_MM, # left_pupil_measure1_type, 'u1'
        EyeTrackerConstants.UNDEFINED, # left_pupil_measure2, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_pupil_measure2_type, 'u1'
        EyeTrackerConstants.UNDEFINED, # left_ppd_x, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_ppd_y, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_velocity_x, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_velocity_y, 'f4'
        EyeTrackerConstants.UNDEFINED, # left_velocity_xy, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_gaze_x, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_gaze_y, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_gaze_z, 'f4'
        sample_eye_right_X ,# right_eye_cam_x, 'f4'
        sample_eye_right_Y ,# right_eye_cam_y, 'f4'
        sample_eye_right_Z ,# right_eye_cam_z, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_angle_x, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_angle_y, 'f4'
        sample_por_right_X ,# right_raw_x, 'f4'
        sample_por_right_Y ,# right_raw_y, 'f4'
        sample_radius_right, # right_pupil_measure1, 'f4'
        EyeTrackerConstants.PUPIL_DIAMETER_MM, # right_pupil_measure1_type, 'u1'
        EyeTrackerConstants.UNDEFINED, # right_pupil_measure2, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_pupil_measure2_type, 'u1'
        EyeTrackerConstants.UNDEFINED, # right_ppd_x, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_ppd_y, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_velocity_x, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_velocity_y, 'f4'
        EyeTrackerConstants.UNDEFINED, # right_velocity_xy, 'f4'
        status # status, 'u1
        ]

        if status != 22:
            self._latest_gaze_position = [sample_por_raw_X, sample_por_raw_Y]

        if gaze_sample.index % 60 == 0:
            log_local(3, message=str(self._latest_sample))

        log_local(4)
        return self._latest_sample

    def _sampleTimeToDeviceTime(self, sample_time):
        """ converts sample timestamps [us] to [s] in device time equivalent time base

        :param sample_time: sample timestamp in microseconds since epoch
        :return: time in seconds in Computer.getTime's time base
        """
        if self._timestamp_anchor is None:
            self._timestamp_anchor = (Computer.getTime(), int(time.time_ns() / 1000))
            log_local(1, 'timestamp anchor ([s], [us]): {:}, {:}'.format(self._timestamp_anchor[0], self._timestamp_anchor[1]))

        elapsed_time_us = sample_time - self._timestamp_anchor[1]
        return self._timestamp_anchor[0] + elapsed_time_us * self.DEVICE_TIMEBASE_TO_SEC

    def trackerTime(self):
        """trackerTime returns the current time reported by the eye tracker
        device. The time base is implementation dependent.


        Args:
            None

        Return:
            float: The eye tracker hardware's reported current time.

        """
        log_local()
        return Computer.getTime()

    def trackerSec(self):
        """
        trackerSec takes the time received by the EyeTracker.trackerTime() method
        and returns the time in sec.usec-msec format.

        Args:
            None

        Return:
            float: The eye tracker hardware's reported current time in sec.msec-usec format.
        """
        log_local()
        return Computer.getTime()

    def setConnectionState(self, enable):
        """setConnectionState either connects ( setConnectionState(True) ) or
        disables ( setConnectionState(False) ) active communication between the
        ioHub and the Eye Tracker.

        .. note::
            A connection to the Eye Tracker is automatically established
            when the ioHub Process is initialized (based on the device settings
            in the iohub_config.yaml), so there is no need to
            explicitly call this method in the experiment script.

        .. note::
            Connecting an Eye Tracker to the ioHub does **not** necessarily collect and send
            eye sample data to the ioHub Process. To start actual data collection,
            use the Eye Tracker method setRecordingState(bool) or the ioHub Device method (device type
            independent) enableEventRecording(bool).

        Args:
            enable (bool): True = enable the connection, False = disable the connection.

        Return:
            bool: indicates the current connection state to the eye tracking hardware.

        """
        log_local()
        if enable:
            log_local(1)
            self._elapi.connect()
            log_local(1)
        else:
            log_local(1)
            self.setRecordingState(False)
            self._elapi.disconnect()
            log_local(1)
        log_local()
        return self._elapi.isConnected()

    def isConnected(self):
        """isConnected returns whether the ioHub EyeTracker Device is connected
        to the eye tracker hardware or not. An eye tracker must be connected to
        the ioHub for any of the Common Eye Tracker Interface functionality to
        work.

        Args:
            None

        Return:
            bool:  True = the eye tracking hardware is connected. False otherwise.

        """
        log_local()
        return self._elapi.isConnected()

    def sendCommand(self, key, value=None):
        """
        The sendCommand method allows arbitrary *commands* or *requests* to be
        issued to the eye tracker device. Valid values for the arguments of this
        method are completely implementation-specific, so please refer to the
        eye tracker implementation page for the eye tracker being used for a list of
        valid key and value combinations (if any).

        In general, eye tracker implementations should **not** need to support
        this method unless there is critical eye tracker functionality that is
        not accessible using the other methods in the EyeTrackerDevice class.

        Args:
            key (str): the command or function name that should be run.
            value (object): the (optional) value associated with the key.

        Return:
            object: the result of the command call
            int: EyeTrackerConstants.EYETRACKER_OK
            int: EyeTrackerConstants.EYETRACKER_ERROR
            int: EyeTrackerConstants.EYETRACKER_INTERFACE_METHOD_NOT_SUPPORTED
        """
        log_local()
        return EyeTrackerConstants.FUNCTIONALITY_NOT_SUPPORTED

    def sendMessage(self, message_contents, time_offset=None):
        """The sendMessage method sends a text message to the eye tracker.

        Messages are generally used to send information you want
        saved with the native eye data file and are often used to
        synchronize stimulus changes in the experiment with the eye
        data stream being saved to the native eye tracker data file (if any).

        This means that the sendMessage implementation needs to
        perform in real-time, with a delay of <1 msec from when a message is
        sent to when it is time stamped by the eye tracker, for it to be
        accurate in this regard.

        If this standard can not be met, the expected delay and message
        timing precision (variability) should be provided in the eye tracker's
        implementation notes.

        .. note::
            If using the ioDataStore to save the eye tracker data, the use of
            this method is quite optional, as Experiment Device Message Events
            will likely be preferred. ioHub Message Events are stored in the ioDataStore,
            alongside all other device data collected via the ioHub, and not
            in the native eye tracker data.

        Args:
           message_contents (str):
               If message_contents is a string, check with the implementations documentation if there are any string length limits.

        Kwargs:
           time_offset (float): sec.msec_usec time offset that the time stamp of
                              the message should be offset in the eye tracker data file.
                              time_offset can be used so that a message can be sent
                              for a display change **BEFORE** or **AFTER** the actual
                              flip occurred, using the following formula:

                              time_offset = sendMessage_call_time - event_time_message_represent

                              Both times should be based on the iohub.devices.Computer.getTime() time base.

                              If time_offset is not supported by the eye tracker implementation being used, a warning message will be printed to stdout.

        Return:
            (int): EyeTrackerConstants.EYETRACKER_OK, EyeTrackerConstants.EYETRACKER_ERROR, or EyeTrackerConstants.EYETRACKER_INTERFACE_METHOD_NOT_SUPPORTED

        """
        log_local()
        return EyeTrackerConstants.FUNCTIONALITY_NOT_SUPPORTED

    def runSetupProcedure(self, calibration_args={}):
        """
        The runSetupProcedure method starts the eye tracker calibration
        routine. If calibration_args are provided, they should be used to
        update calibration related settings prior to starting the calibration.

        The details of this method are implementation-specific.

        .. note::
            This is a blocking call for the PsychoPy Process
            and will not return to the experiment script until the necessary steps
            have been completed so that the eye tracker is ready to start collecting
            eye sample data when the method returns.

        Args:
            None
        """

        log_local()
        turn_off_after = False
        if not self._is_recording:
            log_local()
            turn_off_after = True
            if not self.setRecordingState(recording=True):
                log_local()
                return False
            log_local()
        else:
            # the expectation is that runtime settings cannot change during an experiment
            assert self._tracker_infront == self._runtime_settings['tracker_infront']
            assert self._tracker_below == self._runtime_settings['tracker_below']
        if not self._is_recording:
            return False

        genv = EyeLogicCalibrationProcedure(self, calibration_args)
        genv.runCalibration()

        result = genv.calibrationResult
        return { 'success': (result is not None and result == ELApi.ReturnCalibrate.SUCCESS) }

    def setRecordingState(self, recording) -> bool:
        """The setRecordingState method is used to start or stop the recording
        and transmission of eye data from the eye tracking device to the ioHub
        Process.

        Args:
            recording (bool): if True, the eye tracker will start recordng data.; false = stop recording data.

        Return:
            bool: the current recording state of the eye tracking device

        """

        def printPsychopyDisplayInfo(log_level: int):
            screen_infos = self._display_device._getComputerDisplayRuntimeInfoList()
            log_local( log_level, message='')
            log_local( log_level, message='psychopy display info')
            log_local( log_level, message=str(self._display_device))
            # if len(screen_infos):
            #     log_local( log_level, message=str(screen_infos[0]))
            log_local(log_level, message=str(self._display_device))
            log_local( log_level, message='active display (index, psychopy name): {:}, {:}'.format(self._display_device.getIndex(),
                                                                           self._display_device.getPsychopyMonitorName()))

            for info in screen_infos:
                log_local( log_level, message='')
                for k, v in info.items():
                    log_local( log_level, message='{:}: {:}'.format(k, v))
            log_local( log_level, message='')

        def printEyelogicDisplayInfo(log_level: int):
            el_screens = self._elapi.getAvailableScreens()
            log_local( log_level, message='')
            log_local( log_level, message='eyelogic display info')
            log_local(log_level, message='displays: {}'.format(len(el_screens)))
            for el_screen in el_screens:
                log_local( log_level, message='')
                log_local( log_level, message='{:8s}: {:}'.format('id', str(el_screen.id)))
                log_local( log_level, message='{:8s}: {:}'.format('name', str(el_screen.name)))
                log_local( log_level, message='{:8s}: {:}'.format('local', str(el_screen.localMachine)))
                log_local( log_level, message='{:8s}: {:}'.format('res X', str(el_screen.resolutionX)))
                log_local( log_level, message='{:8s}: {:}'.format('res Y', str(el_screen.resolutionY)))
                log_local( log_level, message='{:8s}: {:}'.format('size X', str(el_screen.physicalSizeX_mm)))
                log_local( log_level, message='{:8s}: {:}'.format('size Y', str(el_screen.physicalSizeY_mm)))
            log_local( log_level, message='')

        def printScreenInfo(log_level: int):
            log_local(log_level, message='screeninfo display info')
            try:
                # printMonitors()
                screen_infos = get_monitors()
                for info in screen_infos:
                    log_local(log_level, message=str(info))
                log_local(log_level, message='')
            except Exception as e:
                traceback.print_exc()

        def printWindowsInfo(log_level: int):
            try:
                import win32api
                import win32con
                import ctypes
                from ctypes import wintypes
            except (ModuleNotFoundError, ImportError, NameError):
                logging.error(
                    "Error importing windows api modules. EyeLogic devices can currently only be used on Windows machines.")

            class DISPLAY_DEVICEW(ctypes.Structure):
                _fields_ = [
                    ('cb', wintypes.DWORD),
                    ('DeviceName', wintypes.WCHAR * 32),
                    ('DeviceString', wintypes.WCHAR * 128),
                    ('StateFlags', wintypes.INT),
                    ('DeviceID', wintypes.WCHAR * 128),
                    ('DeviceKey', wintypes.WCHAR * 128)
                ]

            EnumDisplayDevices = ctypes.windll.user32.EnumDisplayDevicesW
            EnumDisplayDevices.restype = ctypes.c_bool

            displays = []
            i = 0
            while True:
                INFO = DISPLAY_DEVICEW()
                INFO.cb = ctypes.sizeof(INFO)
                Monitor_INFO = DISPLAY_DEVICEW()
                Monitor_INFO.cb = ctypes.sizeof(Monitor_INFO)
                if not EnumDisplayDevices(None, i, ctypes.byref(INFO), 0):
                    break
                displays.append(INFO)
                i += 1

            for i, x in enumerate(displays):
                log_local(log_level, 'Index:        \t{:}'.format(i))
                log_local(log_level, 'DeviceName:   \t{:}'.format(x.DeviceName))
                log_local(log_level, "DeviceString: \t{:}".format(x.DeviceString))
                log_local(log_level, "DeviceID:     \t{:}".format(x.DeviceID))
                log_local(log_level, "DeviceKey:    \t{:}".format(x.DeviceKey))
                log_local(log_level, "StateFlags:   \t{:}".format(x.StateFlags))
                log_local(log_level, "\tAttached:   \t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_ATTACHED_TO_DESKTOP > 0))
                log_local(log_level, "\tMDriver:    \t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_MULTI_DRIVER > 0))
                log_local(log_level, "\tPrimary:    \t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_PRIMARY_DEVICE > 0))
                log_local(log_level, "\tMirroring:  \t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_MIRRORING_DRIVER > 0))
                log_local(log_level, "\tVGA Comp:   \t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_VGA_COMPATIBLE > 0))
                log_local(log_level, "\tRemovable:  \t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_REMOVABLE > 0))
                log_local(log_level, "\tModespruned:\t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_MODESPRUNED > 0))
                log_local(log_level, "\tRemote:     \t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_REMOTE > 0))
                log_local(log_level, "\tDisconnect: \t{:}".format(x.StateFlags & win32con.DISPLAY_DEVICE_DISCONNECT > 0))
                print()

        def mapIndex2WindowsName():
            try:
                import win32api
                import win32con
                import ctypes
                from ctypes import wintypes
            except (ModuleNotFoundError, ImportError, NameError):
                logging.error(
                    "Error importing windows api modules. EyeLogic devices can currently only be used on Windows machines.")


            class DISPLAY_DEVICEW(ctypes.Structure):
                _fields_ = [
                    ('cb', wintypes.DWORD),
                    ('DeviceName', wintypes.WCHAR * 32),
                    ('DeviceString', wintypes.WCHAR * 128),
                    ('StateFlags', wintypes.INT),
                    ('DeviceID', wintypes.WCHAR * 128),
                    ('DeviceKey', wintypes.WCHAR * 128)
                ]

            EnumDisplayDevices = ctypes.windll.user32.EnumDisplayDevicesW
            EnumDisplayDevices.restype = ctypes.c_bool

            displays = []
            i = 0
            while True:
                INFO = DISPLAY_DEVICEW()
                INFO.cb = ctypes.sizeof(INFO)
                Monitor_INFO = DISPLAY_DEVICEW()
                Monitor_INFO.cb = ctypes.sizeof(Monitor_INFO)
                if not EnumDisplayDevices(None, i, ctypes.byref(INFO), 0):
                    break
                state_flags = INFO.StateFlags
                if state_flags & win32con.DISPLAY_DEVICE_ATTACHED_TO_DESKTOP:
                    displays.append((state_flags & win32con.DISPLAY_DEVICE_PRIMARY_DEVICE > 0, i, INFO))
                i += 1
            return displays

        log_local(message='recording = ' + str(recording))
        if not self.isConnected() and not self.setConnectionState(True):
            log_local(message='failure: not connected')
            return False
        if recording == self.isRecordingEnabled():
            log_local(message='recording state already {}'.format(str(recording)))
            return True

        if recording:
            printPsychopyDisplayInfo(2)
            printEyelogicDisplayInfo(2)
            printScreenInfo(2)
            printWindowsInfo(2)
            displaysMapping = mapIndex2WindowsName()

            infront = self._runtime_settings['tracker_infront']
            below   = self._runtime_settings['tracker_below']
            display_index = self._display_device.getIndex()
            for map in displaysMapping:
                if map[1] == display_index:
                    display_windows_name = map[2].DeviceName
                    break
            returnSetScreen = self._elapi.setActiveScreen(display_windows_name, ELApi.DeviceGeometry(below, infront))
            log_local(message='returnSetScreen = ' + str(returnSetScreen))
            if returnSetScreen != ELApi.ReturnSetActiveScreen.SUCCESS:
                return False


            # returnStart = self._elapi.requestTracking(0)
            requested_rate = self._runtime_settings['sampling_rate']
            sampling_mode = None
            if requested_rate == 'default':
                sampling_mode = 0
            else:
                requested_rate = int(requested_rate)
                log_local(2, message='framerate requested: {:}'.format(requested_rate))
                for i, available_rate in enumerate(self._elapi.getDeviceConfig().frameRates):
                    log_local(2, message='framerate available: {:}'.format(available_rate))
                    if requested_rate == available_rate:
                        log_local(1)
                        sampling_mode = i
                        break
            if sampling_mode is None:
                log_local(1, 'failure: sampling rate not available')
                return False

            log_local(message='sampling rate = ' + str(sampling_mode) + ': ' + str(requested_rate))
            returnStart = self._elapi.requestTracking(sampling_mode)
            log_local(message='returnStart = ' + str(returnStart))
            if returnStart != ELApi.ReturnStart.SUCCESS:
                return False
            log_local()
            if EyeTrackerDevice.enableEventReporting(self, True):
                self._is_recording = True
                self._tracker_below = below
                self._tracker_infront = infront
            else:
                return False
        else:
            log_local()
            EyeTrackerDevice.enableEventReporting(self, False)
            self._elapi.unrequestTracking()
            self._is_recording = False
            self._tracker_below = None
            self._tracker_infront = None
            log_local()

        self._latest_sample = None
        self._latest_gaze_position = None
        log_local()
        return self._is_recording

    def isRecordingEnabled(self) -> bool:
        """The isRecordingEnabled method indicates if the eye tracker device is
        currently recording data.

        Args:
           None

        Return:
            bool: True == the device is recording data; False == Recording is not occurring

        """
        log_local()
        return self._is_recording

    def getLastSample(self):
        """The getLastSample method returns the most recent eye sample received
        from the Eye Tracker. The Eye Tracker must be in a recording state for
        a sample event to be returned, otherwise None is returned.

        Args:
            None

        Returns:
            int: If this method is not supported by the eye tracker interface, EyeTrackerConstants.FUNCTIONALITY_NOT_SUPPORTED is returned.

            None: If the eye tracker is not currently recording data.

            EyeSample: If the eye tracker is recording in a monocular tracking mode, the latest sample event of this event type is returned.

            BinocularEyeSample:  If the eye tracker is recording in a binocular tracking mode, the latest sample event of this event type is returned.

        """
        log_local()
        return self._latest_sample

    def getLastGazePosition(self):
        """The getLastGazePosition method returns the most recent eye gaze
        position received from the Eye Tracker. This is the position on the
        calibrated 2D surface that the eye tracker is reporting as the current
        eye position. The units are in the units in use by the ioHub Display
        device.

        If binocular recording is being performed, the average position of both
        eyes is returned.

        If no samples have been received from the eye tracker, or the
        eye tracker is not currently recording data, None is returned.

        Args:
            None

        Returns:
            int: If this method is not supported by the eye tracker interface, EyeTrackerConstants.EYETRACKER_INTERFACE_METHOD_NOT_SUPPORTED is returned.

            None: If the eye tracker is not currently recording data or no eye samples have been received.

            tuple: Latest (gaze_x,gaze_y) position of the eye(s)
        """
        log_local()
        return self._latest_gaze_position

    def getPosition(self):
        """
        See getLastGazePosition().
        """
        log_local()
        return self.getLastGazePosition()

    def getPos(self):
        """
        See getLastGazePosition().
        """
        log_local()
        return self.getLastGazePosition()

    def _eyeTrackerToDisplayCoords(self, eyetracker_point):
        """The _eyeTrackerToDisplayCoords method is required for implementation
        of the Common Eye Tracker Interface in order to convert the native Eye
        Tracker coordinate space to the ioHub.devices.Display coordinate space
        being used in the PsychoPy experiment. Any screen based coordinates
        that exist in the data provided to the ioHub by the device
        implementation must use this method to convert the x,y eye tracker
        point to the correct coordinate space.

        Default implementation is to call the Display device method:

            self._display_device._pixel2DisplayCoord(gaze_x,gaze_y,self._display_device.getIndex())

        where gaze_x,gaze_y = eyetracker_point, which is assumed to be in screen pixel
        coordinates, with a top-left origin. If the eye tracker provides the eye position
        data in a coordinate space other than screen pixel position with top-left origin,
        the eye tracker position should first be converted to this coordinate space before
        passing the position data px,py to the _pixel2DisplayCoord method.

        self._display_device.getIndex() provides the index of the display for multi display setups.
        0 is the default index, and valid values are 0 - N-1, where N is the number
        of connected, active, displays on the computer being used.

        Args:
            eyetracker_point (object): eye tracker implementation specific data type representing an x, y position on the calibrated 2D plane (typically a computer display screen).

        Returns:
            (x,y): The x,y eye position on the calibrated surface in the current ioHub.devices.Display coordinate type and space.

        """
        log_local(4)
        gaze_x = eyetracker_point[0]
        gaze_y = eyetracker_point[1]
        screen_index = self._elapi.getActiveScreen().id
        log_local(4)
        return self._display_device._pixel2DisplayCoord(gaze_x, gaze_y, screen_index)

    def _displayToEyeTrackerCoords(self, display_x, display_y):
        """The _displayToEyeTrackerCoords method must be used by an eye
        trackers implementation of the Common Eye Tracker Interface to convert
        any gaze positions provided by the ioHub to the appropriate x,y gaze
        position coordinate space for the eye tracking device in use.

        This method is simply the inverse operation performed by the _eyeTrackerToDisplayCoords
        method.

        Default implementation is to just return the result of self._display_device.display2PixelCoord(...).

        Args:
            display_x (float): The horizontal eye position on the calibrated 2D surface in ioHub.devices.Display coordinate space.
            display_y (float): The vertical eye position on the calibrated 2D surface in ioHub.devices.Display coordinate space.

        Returns:
            (object): eye tracker implementation specific data type representing an x, y position on the calibrated 2D plane (typically a computer display screen).

        """
        log_local()
        screen_index = self._elapi.getActiveScreen().id
        pixel_x, pixel_y = self._display_device.display2PIxelCoord(display_x, display_y, screen_index)
        log_local()
        return pixel_x, pixel_y

    def __del__(self):
        """Do any final cleanup of the eye tracker before the object is
        destroyed."""
        log_local()
        self._elapi.disconnect()
        self.__class__._INSTANCE = None
        log_local()
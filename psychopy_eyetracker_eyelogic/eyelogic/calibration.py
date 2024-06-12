# -*- coding: utf-8 -*-
# Part of the PsychoPy library
# Copyright (C) 2012-2020 iSolver Software Solutions (C) 2021 Open Science Tools Ltd.
# Distributed under the terms of the GNU General Public License (GPL).


from psychopy.iohub.devices import DeviceEvent
from psychopy.iohub.constants import EventConstants as EC
from psychopy.iohub.devices.eyetracker.calibration import BaseCalibrationProcedure

import threading
import gevent

class EyeLogicCalibrationProcedure(BaseCalibrationProcedure):

    calibrationThread: None | threading.Thread
    def __init__(self, eyetrackerInterface, calibration_args):
        BaseCalibrationProcedure.__init__(self, eyetrackerInterface, calibration_args, allow_escape_in_progress=True)
        self.calibrationCond = threading.Condition()
        self.calibrationThread = None
        self.calibrationResult = None

    def _handleEvent(self, event):
        event_type_index = DeviceEvent.EVENT_TYPE_ID_INDEX
        if self.calibrationThread is not None:
            if event[event_type_index] == EC.KEYBOARD_RELEASE:
                ek = event[self._keyboard_key_index]
                if isinstance(ek, bytes):
                    ek = ek.decode('utf-8')
                if ek == 'escape':
                    self._eyetracker._elapi.setRecordingState(False)
                    self._eyetracker._elapi.setRecordingState(True)
                    return
        BaseCalibrationProcedure._handleEvent(self, event)

    def runCalibration(self):
        """Run calibration sequence
        """

        if self.showIntroScreen() is False:
            # User pressed escape  to exit calibration
            return False
        self.window.fullscr = False
        # self.window.winHandle.set_visible(False)
        self.window.callOnFlip(self.startCalibrationHook)
        self.clearCalibrationWindow()

        self.finishCalibrationHook()

        success = self.calibrationResult == ELApi.ReturnCalibrate.SUCCESS
        if success:
            self.showFinishedScreen()
            self.clearCalibrationWindow()
            self.clearAllEventBuffers()
        self.window.close()

        return success

    def eyelogicCalibrate(self, mode):
        self.calibrationResult = self._eyetracker._elapi.calibrate(mode)

    def startCalibrationHook(self):
        if self.calibrationThread is not None:
           return
        type = self.getCalibSetting('type')
        calibPoints = None
        if 'ONE' in type:
           calibPoints = 1
        elif 'TWO' in type:
            calibPoints = 2
        elif 'FIVE' in type:
            calibPoints = 5
        elif 'NINE' in type:
            calibPoints = 9
        mode = None
        if calibPoints is not None:
            for i, npts in enumerate(self._eyetracker._elapi.getDeviceConfig().calibrationMethods):
                if calibPoints == npts:
                    mode = i
                    break
        if mode is None:
            return
        self.calibrationThread = threading.Thread(target=self.eyelogicCalibrate, args=(mode,))
        self.calibrationThread.start()

    def finishCalibrationHook(self):
        # self.window.winHandle.activate()
        self.calibrationThread.join()
        self.calibrationThread = None
        self.window.fullscr = True
        # self.window.winHandle.set_visible(True)





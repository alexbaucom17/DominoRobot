"""
Python wrapper for Marvelmind C API
"""

import ctypes
import logging
from collections import defaultdict
from Utils import NonBlockingTimer


class MarvelmindWrapper:

    def __init__(self, cfg):
        self.lib = ctypes.windll.LoadLibrary(cfg.mm_api_path)
        self.devices = defaultdict(dict)
        self.expected_devices = []
        self.wake_timer = None
        for name,beacons in cfg.device_map.items():
            for b in beacons:
                self.expected_devices.append(b)
        self._open_serial_port()

    def __del__(self):
        self._close_serial_port()

    def _open_serial_port(self):
        """
        Establish communication with marvelmind router. Required for all other functions.
        """

        logging.info("Opening communication with MarvelMind")
        fn = self.lib.mm_open_port
        fn.restype = ctypes.c_bool
        fn.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        status = fn(None)

    def _close_serial_port(self):
        """
        Close communication with marvelmind router.
        """

        fn = self.lib.mm_close_port
        fn.restype = ctypes.c_bool
        fn.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        status = fn(None)

    def check_all_devices_status(self):
        """
        Checks to see if all devices are awake and ready
        """

        logging.info("Getting device list")
        fn = self.lib.mm_get_devices_list
        fn.restype = ctypes.c_bool
        fn.argtypes = [ctypes.POINTER(ctypes.c_uint8)]

        buff_size = 255
        dataptr = (ctypes.c_uint8*buff_size)()
        dataptr = ctypes.cast(dataptr, ctypes.POINTER(ctypes.c_uint8))
        status = fn(dataptr)

        num_devices = dataptr[0]

        logging.info("Num devices found: " + str(num_devices))
        idx = 1
        device_offset = 9
        addr_offset = 0
        sleep_offset = 2
        for i in range(num_devices):
            addr = dataptr[idx + addr_offset]
            sleep = dataptr[idx + sleep_offset]

            self.devices[addr] = {'sleep': bool(sleep)}

            idx += device_offset

        ready = True
        if self.wake_timer:
            if not self.wake_timer.check():
                ready = False
        else:
            for d in self.expected_devices:
                if d not in self.devices.keys():
                    ready = False
                    logging.warn("Could not find expected device {}".format(d))
                    continue

                if self.devices[d]['sleep'] is False:
                    ready = False
                    logging.warn("Device {} is not awake".format(d))
                    continue


        return ready

    def wake_all_devices(self):
        for d in self.expected_devices:
            self.wake_device(d)

    def wake_all_devices_only_if_needed(self):
        ok = self.check_all_devices_status()
        if not ok:
            logging.info("Triggering marvelmind device wakeup. Will wait for a moment to let them warmup")
            self.wake_timer = NonBlockingTimer(30)
            self.wake_all_devices()

    def sleep_all_devices(self):
        for d in self.expected_devices:
            self.sleep_device(d)

    def wake_device(self, address):
        """
        Wake up the device with the given address
        """

        logging.info("Waking device: {}".format(address))
        fn = self.lib.mm_wake_device
        fn.restype = ctypes.c_bool
        fn.argtypes = [ctypes.c_uint8]

        status = fn(address)
        if not status:
            logging.info("Warning: unable to wake device {}".format(address))


    def sleep_device(self, address):
        """
        Sleep the device with the given address
        """

        logging.info("Sleeping device: {}".format(address))
        fn = self.lib.mm_send_to_sleep_device
        fn.restype = ctypes.c_bool
        fn.argtypes = [ctypes.c_uint8]

        status = fn(address)
        if not status:
            logging.info("Warning: unable to sleep device {}".format(address))

    def get_metrics(self):
        return self.devices


class MockMarvelmindWrapper:
    def __init__(self, cfg):    
        pass
    def check_all_devices_status(self):
        return True
    def wake_all_devices_only_if_needed(self):
        pass
    def wake_all_devices(self):
        pass
    def sleep_all_devices(self):
        pass
    def wake_device(self, address):
        pass
    def sleep_device(self, address):
        pass
    def get_metrics(self):
        return {}

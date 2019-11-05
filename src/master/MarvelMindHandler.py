"""
Python wrapper for Marvelmind C API
"""

import ctypes

API_PATH = "marvelmind_SW_2019_08_25\\API\\api_windows_64bit\\dashapi.dll"

def uint8_to_int32(uint8_arr):
    out_val = ctypes.c_int32()
    out_val = uint8_arr[0]
    out_val = (out_val << 8) + uint8_arr[1]
    out_val = (out_val << 8) + uint8_arr[2]
    out_val = (out_val << 8) + uint8_arr[3]
    return out_val

class MarvelMindDevice:
    """
    Simple class to hold device data
    """
    def __init__(self, address, sleep_status):
        self.address = address
        self.sleep_status = sleep_status

    def __str__(self):
        return "Device: {}, Sleep: {}".format(self.address, self.sleep_status)

class MarvelMindWrapper:

    def __init__(self, root_path):
        self.lib = ctypes.windll.LoadLibrary(root_path + API_PATH)

    def __enter__(self):
        self._open_serial_port()
        return self

    def __exit__(self, type, value, traceback):
        self._close_serial_port()
        if type is not None:
            return False
        else:
            return True

    def _open_serial_port(self):
        """
        Establish communication with marvelmind router. Required for all other functions.
        """

        print("Opening communication with MarvelMind")
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

    def get_all_devices(self):
        """
        Get list of all devices connected to the modem (including sleeping ones).
        Returns list of MarvelMindDevice objects
        """

        print("Getting device list")
        fn = self.lib.mm_get_devices_list
        fn.restype = ctypes.c_bool
        fn.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
    
        buff_size = 20 # TODO: Verify this
        dataptr = (ctypes.c_uint8*buff_size)()
        dataptr = ctypes.cast(dataptr, ctypes.POINTER(ctypes.c_uint8))
        status = fn(dataptr)

        num_devices = dataptr[0]

        print("Num devices: " + str(num_devices))
        idx = 1
        device_offset = 9
        addr_offset = 0
        sleep_offset = 2
        device_list = []
        for i in range(num_devices):
            addr = dataptr[idx + addr_offset]
            sleep = dataptr[idx + sleep_offset]
            idx += device_offset
            device = MarvelMindDevice(addr, sleep)
            print(device)
            device_list.append(device)
    
        return device_list

    def wake_device(self, device):
        """
        Wake up the device if needed
        Device should be a MarvelMindDevice object
        """

        if device.sleep_status:
            print("Waking device: {}".format(device.address))
            fn = self.lib.mm_wake_device
            fn.restype = ctypes.c_bool
            fn.argtypes = [ctypes.c_uint8]

            status = fn(device.address)
            if not status:
                print("Warning: unable to wake device {}".format(device.address))
            else:
                device.sleep_status = 0
        else:
            print("Not waking device {}, already awake".format(device.address))


    def sleep_device(self, device):
        """
        Sleep the device if needed
        Device should be a MarvelMindDevice object
        """

        if not device.sleep_status:
            print("Sleeping device: {}".format(device.address))
            fn = self.lib.mm_send_to_sleep_device
            fn.restype = ctypes.c_bool
            fn.argtypes = [ctypes.c_uint8]

            status = fn(device.address)
            if not status:
                print("Warning: unable to sleep device {}".format(device.address))
            else:
                device.sleep_status = 1
        else:
            print("Not sleeping device {}, already asleep".format(device.address))



    def get_latest_location_data(self):
        """
        Gets the latest positioning data available for one of the beacons
        Returns position data
        """

        print("Getting latest position data")
        fn = self.lib.mm_get_last_locations
        fn.restype = ctypes.c_bool
        fn.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
    
        buff_size = 255 # TODO: Verify this
        dataptr = (ctypes.c_uint8*buff_size)()
        dataptr = ctypes.cast(dataptr, ctypes.POINTER(ctypes.c_uint8))
        status = fn(dataptr)

        num_points = 6 # Always returns last 6 points
        point_size = 18
        new_data_flag_offset = num_points * point_size

        new_data_ready = dataptr[new_data_flag_offset]

        print("New data: " + str(new_data_ready))

        if new_data_ready:
            idx = 0
            dev_addr_offset = 0
            x_point_offset = 2
            y_point_offset = 6
            z_point_offset = 10
            for i in range(num_points):
                tmp_uint8_arr_type = ctypes.c_uint8 * 4
                tmp_uint8_arr = tmp_uint8_arr_type(dataptr[idx + x_point_offset + 3],
                                                   dataptr[idx + x_point_offset + 2],
                                                   dataptr[idx + x_point_offset + 1],
                                                   dataptr[idx + x_point_offset + 0])
                x_point = uint8_to_int32(tmp_uint8_arr)
                addr = dataptr[idx + dev_addr_offset]
                print("New point")
                print(addr)
                print(tmp_uint8_arr[3])
                print(tmp_uint8_arr[2])
                print(tmp_uint8_arr[1])
                print(tmp_uint8_arr[0])
                print(x_point)
                idx += point_size


class RobotPositionHandler():

    def __init__(self):

        # Robot position queues
        self.prev_positions = []

        # TODO: Actually use config for root path and robot/mm device map
        ROOT_PATH = "C:\\Users\\alexb\\Documents\\Github\\DominoRobot\\"
        self._mm = MarvelMindWrapper(ROOT_PATH)
        self._mm._open_serial_port()

        # Wake static beacons
        static_beacons = DEVICE_MAP["static"]
        for beacon in static_beacons:
            self._mm.wake_device(beacon)

    def close(self):
        # Sleep static beacons
        static_beacons = DEVICE_MAP["static"]
        for beacon in static_beacons:
            self._mm.sleep_device(beacon)

        self._mm._close_serial_port()

    def wake_robot(self, robot_number):
        # Wake up all beacons on the given robot
        robot_beacons = DEVICE_MAP[str(robot_number)]
        for beacon in robot_beacons:
            self._mm.wake_device(beacon)

    def sleep_robot(self, robot_number):
        # Sleep all beacons on the given robot
        robot_beacons = DEVICE_MAP[str(robot_number)]
        for beacon in robot_beacons:
            self._mm.sleep_device(beacon)

    def loop(self):
        # Main loop that gets data from the marvelmind system and makes sense of it

        for i in range(5):
            self._mm.get_latest_location_data()

    def get_all_devices(self):
        a = self._mm.get_all_devices()

        


# Maps robot (or static) to sets of marvel mind beacons
DEVICE_MAP = {
    "static": (MarvelMindDevice(20,1), MarvelMindDevice(21,1)),
    "1": (MarvelMindDevice(10,1), MarvelMindDevice(11,1))
}
    


if __name__ == '__main__':
    
    r = RobotPositionHandler()
    r.wake_robot(1)
    r.loop()
    #r.get_all_devices()
    r.sleep_robot(1)
    r.close()
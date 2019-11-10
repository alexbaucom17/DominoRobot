"""
Python wrapper for Marvelmind C API
"""

import ctypes
import math
import time

class MarvelMindPoint:
    """
    Simple class to hold a point structure
    """
    def __init__(self):
        self.address = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.t = 0

    def __str__(self):
        return "Address: {}, Location: [{},{},{}], Time: {}".format(
            self.address, self.x, self.y, self.z, self.t)

    def __eq__(self, other):
        # Since Marvelmind doesn't provide timing info, I'm adding the times 
        # manually after the fact, so don't use them in comparison
        if self.address == other.address and self.x == other.x and \
           self.y == other.y and self.z == other.z:
            return True
        else:
            return False

class MarvelMindDevice:
    """
    Simple class to hold device data
    """
    def __init__(self, address, sleep_status):
        self.address = address
        self.sleep_status = sleep_status

    def __str__(self):
        return "Device: {}, Sleep: {}".format(self.address, self.sleep_status)

def uint8_to_int32(uint8_arr):
    """
    Converts array of 4 uint8 values to an int32
    """
    out_val = ctypes.c_int32()
    out_val = uint8_arr[0]
    out_val = (out_val << 8) + uint8_arr[1]
    out_val = (out_val << 8) + uint8_arr[2]
    out_val = (out_val << 8) + uint8_arr[3]
    return out_val

def extract_point_data(ptr, start_idx):
    """
    Extracts point from marvelmind location data given a pointer and the starting index
    """
    # Data offsets from documentation
    dev_addr_offset = 0
    point_offsets = [2,6,10] # for x, y, z

    # Initialize point
    point = MarvelMindPoint()
    point.address = ptr[start_idx + dev_addr_offset]
    point.t = time.time()

    # Extract each point
    for i in range(3):
        offset = point_offsets[i]

        # Each point is transmitted as an int32 so we need to convert from the uint8s we currently have
        tmp_uint8_arr_type = ctypes.c_uint8 * 4
        tmp_uint8_arr = tmp_uint8_arr_type( ptr[start_idx + offset + 3],
                                            ptr[start_idx + offset + 2],
                                            ptr[start_idx + offset + 1],
                                            ptr[start_idx + offset + 0])
        new_pt = uint8_to_int32(tmp_uint8_arr)
        if i == 0:
            point.x = new_pt
        elif i == 1:
            point.y = new_pt
        else:
            point.z = new_pt

    return point


API_PATH = "marvelmind_SW_2019_08_25\\API\\api_windows_64bit\\dashapi.dll"

# Maps robot (or static) to sets of marvel mind beacons
# Defining first device in list as the one on the left side of the robot
DEVICE_MAP = {
    "static": (MarvelMindDevice(1,1), MarvelMindDevice(2,1)),
    "1": (MarvelMindDevice(5,1), MarvelMindDevice(6,1))
}


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
        points = []

        if new_data_ready:
            idx = 0
            for i in range(num_points):
                points.append(extract_point_data(dataptr, idx))
                idx += point_size

        return points

class RobotPositionHandler():

    def __init__(self):

        # TODO: Actually use config for root path and robot/mm device map
        ROOT_PATH = "C:\\Users\\alexb\\Documents\\Github\\DominoRobot\\"
        self._mm = MarvelMindWrapper(ROOT_PATH)
        self._mm._open_serial_port()

        # Wake static beacons
        static_beacons = DEVICE_MAP["static"]
        for beacon in static_beacons:
            self._mm.wake_device(beacon)

        # Initialize position queues
        self.max_device_queue_size = 15
        self.device_position_queues = {}
        for beacon_pairs in DEVICE_MAP.values():
            for beacon in beacon_pairs:
                self.device_position_queues[beacon.address] = []

        self.max_robot_queue_size = 5
        self.robot_position_queues = {}
        for robot_name in DEVICE_MAP:
            if robot_name == 'static':
                continue
            self.robot_position_queues[robot_name] = []


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

    def get_position(self, robot_number):
        # Get current position of a specific robot
        try:
            return self.robot_position_queues[str(robot_number)][0]
        except IndexError:
            return []

    def _get_device_position(self, device_addr):
        # Returns device position
        try:
            return self.device_position_queues[device_addr][0]
        except IndexError:
            return []

    def service_queues(self):
        # Gets data from the marvelmind system and makes sense of it. Needs to be called
        # regularly to make sure all data is collected
        
        new_data = self._mm.get_latest_location_data()
        self._update_device_positions(new_data)
        self._update_robot_positions()

    def _update_device_positions(self, new_position_data):
        # Distribute position data to queues and do any post processing as needed
        if new_position_data:
            for point in new_position_data:
                addr = point.address
                cur_queue = self.device_position_queues[addr]

                # Marvelmind doesn't send any timing information about the measurments and it will
                # gladly send repeat data, so this is just a simple way to only update the position
                # if it has changed. Thankfully, the devices are quite sensitive and even sitting very
                # will trigger very small fluctuations in the position. So this should continue
                # to update the queues pretty regularly. If it isn't regular or accurate enough, I may need
                # to look into other ways to handle this
                if not cur_queue or point != cur_queue[0]:
                    #print(str(point))
                    cur_queue.insert(0,point) # Add new element to front of queue
                    if len(cur_queue) > self.max_device_queue_size:
                        cur_queue.pop() # remove element from end of list
        else:
            print("No new data")


    def _update_robot_positions(self):
        # Handle the math of transforming device positions into robot positions and angles

        for robot_name in DEVICE_MAP:
            if robot_name == 'static':
                continue

            # Get position data from the robot beacons
            beacons = DEVICE_MAP[robot_name]
            p0 = self._get_device_position(beacons[0].address)
            p1 = self._get_device_position(beacons[1].address)

            # Make sure that array are not empty
            if not p0 or not p1:
                continue

            # Compute position as the average
            x = (p0.x + p1.x)/2.0 / 1000.0
            y = (p0.y + p1.y)/2.0 / 1000.0

            # Angle is defined as the angle of the vector from p0 (left of robot)
            # to p1 (right of robot) + 90 degrees so angle vector points towards front of robot
            xdiff = p1.x - p0.x
            ydiff = p1.y - p0.y
            angle = math.atan2(ydiff, xdiff) + math.pi/2.0

            # Timing is a bit tricky. This assumes that the devices are both getting updated
            # regularly enough to consider them updated simulatneously relative to motion
            # TODO: Figure out how much error this introduces and if it is a problem
            max_time_delta = 0.3 #seconds
            # if abs(p0.t - p1.t) > max_time_delta:
            #     print("WARNING: Max time delta exceeded between {} and {}".format(str(p0), str(p1))) 
            t = (p0.t + p1.t)/2.0 # Just take average time

            # Similar note to above - only updating robot position if it has changed
            cur_queue = self.robot_position_queues[robot_name]
            prev_point = []
            if cur_queue:
                prev_point = cur_queue[0]
            if not prev_point or x != prev_point[0] or y != prev_point[1] or angle != prev_point[2]:
                print("Robot {}: {},{},{}, {}".format(robot_name, x, y, angle, t))

                # Insert the new position, angle, and time into the queue
                cur_queue.insert(0,(x,y,angle, t)) # Add new element to front of queue
                if len(cur_queue) > self.max_robot_queue_size:
                    cur_queue.pop # remove element from end of list


if __name__ == '__main__':
    
    r = RobotPositionHandler()
    r.wake_robot(1)

    # Wait for devices to fully wake up and give good data
    sleep_time = 30
    print('Waiting {} seconds for beacons to fully wake up'.format(sleep_time))
    time.sleep(sleep_time)

    t_start = time.time()
    t_total = 15
    while time.time() - t_start < t_total:
        r.service_queues()
        time.sleep(0.05)
    
    r.sleep_robot(1)
    r.close()
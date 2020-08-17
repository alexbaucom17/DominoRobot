# Testing out trajectory generation where I have a gui

import math
import numpy as np
from matplotlib import pyplot as plt

# Parameters
MAX_VEL = 3.0  # m/s
MAX_ACC = 2.0    # m/s^2

# Find min time and position change to reach full speed
T_ACC = MAX_VEL / MAX_ACC
print("T_ACC: {}".format(T_ACC))
P_ACC = 0.5 * MAX_ACC * T_ACC * T_ACC
print("P_ACC: {}".format(P_ACC))


def sign(n):
    if n > 0:
        return 1
    elif n < 0:
        return -1
    else:
        return 0


def gen_traj2(p1, p2, dt):

    position_change = abs(p2 - p1)

    # Determine if position change is less than required for steady state
    if position_change < 2*P_ACC:
        return gen_triangle_traj(p1, p2, dt)
    else:
        return gen_trap_traj(p1, p2, dt)

def gen_triangle_traj(p1, p2, dt):
    turn_pt = (p2 - p1)/2
    dir = sign(p2 - p1)
    traj = {'time': [0], 'pos': [p1], 'vel': [0], 'acc': [dir * MAX_ACC]}
    gen_const_acc_until_pos(traj, dt, dir*MAX_ACC, turn_pt, dir)
    gen_const_acc_until_pos(traj, dt, -1*dir*MAX_ACC, p2, dir)
    return traj

def gen_trap_traj(p1, p2, dt):
    dir = sign(p2 - p1)
    max_vel_pt = dir*P_ACC
    slow_down_pt = p2 - dir * P_ACC
    traj = {'time': [0], 'pos': [p1], 'vel': [0], 'acc': [dir * MAX_ACC]}
    gen_const_acc_until_pos(traj, dt, dir*MAX_ACC, max_vel_pt, dir)
    gen_const_acc_until_pos(traj, dt, 0, slow_down_pt, dir)
    gen_const_acc_until_pos(traj, dt, -1*dir*MAX_ACC, p2, dir)
    return traj


def gen_const_acc_until_pos(traj, dt, acc, stop_pos, stop_dir):
    """
    Generate constant acceleration trajectory
    """
    n_iter = 0
    while True:
        new_vel = traj['vel'][-1] + acc * dt
        new_pos = traj['pos'][-1] + traj['vel'][-1] * dt
        new_time = traj['time'][-1] + dt

        if stop_dir > 0 and new_vel < 0:
            break
        if stop_dir < 0 and new_vel > 0:
            break

        if stop_dir > 0 and new_pos >= stop_pos:
            break
        elif stop_pos < 0 and new_pos <= stop_pos:
            break
        elif n_iter > 100000:
            raise StopIteration("Too many iterations")

        traj['time'].append(new_time)
        traj['pos'].append(new_pos)
        traj['vel'].append(new_vel)
        traj['acc'].append(acc)
        n_iter += 1

    return traj


def plot_traj(pvt_data):
    """ 
    Makes a plot of position, velocity, and acceleration versus time
    """
    ax = plt.axes()
    ax.plot(pvt_data["time"], pvt_data["pos"],'r',label='Position')
    ax.plot(pvt_data["time"], pvt_data["vel"],'g',label='Velocity')
    ax.plot(pvt_data["time"], pvt_data["acc"],'b',label='Acceleration')
    ax.legend()
    plt.show()



if __name__ == '__main__':
    p1 = 0
    p2 = 20
    dt = 0.05
    pvt = gen_traj2(p1, p2, dt)
    plot_traj(pvt)
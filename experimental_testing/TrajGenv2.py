import math
import numpy as np
from matplotlib import pyplot as plt

p_target = 2.0  # m
v_max = 0.5  #m/s
a_max = 1.0  #m/s^2
j_max = 5.0 #m/s^3
alpha = 0.8 # velocity decay
beta = 0.8 # acceleation decay
loop_limit = 10 # max loops for convergence

plot_timestep = 0.01


def generate(target_position):

    v_lim = v_max
    a_lim = a_max
    j_lim = j_max

    output = {}
    output['done'] = False

    for i in range(loop_limit):
        print("Starting loop {}".format(i))
        output = generate_once(target_position, v_lim, a_lim, j_lim)

        if output['done']:
            break
        else:
            v_lim = output['v_lim']
            a_lim = output['a_lim']
            j_lim = output['j_lim']
            

    if output['done']:
        print("Trajectory found")
        return output
    else:
        print("Trajectory not found")
        return {}

    


def generate_once(p, v_lim, a_lim, j_lim):

    output = {}
    output['done'] = False
    output['v_lim'] = v_lim
    output['a_lim'] = a_lim
    output['j_lim'] = j_lim
    output['t'] = []

    # Constant jerk region
    dt_j = a_lim / j_lim
    print("dt_j = {}".format(dt_j))
    dv_j = 0.5 * j_lim * dt_j ** 2
    dp_j = 1/6.0 * j_lim * dt_j ** 3

    # Constant accel region
    dt_a = (v_lim - 2 * dv_j) / a_lim
    print("dt_a = {}".format(dt_a))
    if dt_a <= 0:
        output['a_lim'] = a_lim * beta
        print("New a_lim = {}".format(output['a_lim'] ))
        return output
    dp_a = dv_j * dt_a + 0.5 * a_lim * dt_a ** 2

    # Constant vel region
    dt_v = (p - 4 * dp_j - 2 * dp_a) / v_lim
    print("dt_v = {}".format(dt_v))
    if dt_v <= 0:
        output['v_lim'] = alpha * v_lim
        print("New v_lim = {}".format(output['v_lim'] ))
        return output
    dp_v = v_lim * dt_v

    # If we get here, the genertation should be correct, so compute time regions and return
    output['done'] = True
    t = [0,0,0,0,0,0,0,0]
    t[0] = 0
    t[1] = t[0] + dt_j   
    t[2] = t[1] + dt_a
    t[3] = t[2] + dt_j                
    t[4] = t[3] + dt_v
    t[5] = t[4] + dt_j
    t[6] = t[5] + dt_a
    t[7] = t[6] + dt_j
    output['t'] = t

    print("Times: {}".format(t))
    
    return output

def generate_inverse(p, dt_j, dt_a, dt_v):
    output = {}
    output['done'] = True
    t = [0,0,0,0,0,0,0,0]
    t[0] = 0
    t[1] = t[0] + dt_j   
    t[2] = t[1] + dt_a
    t[3] = t[2] + dt_j                
    t[4] = t[3] + dt_v
    t[5] = t[4] + dt_j
    t[6] = t[5] + dt_a
    t[7] = t[6] + dt_j
    output['t'] = t

    # Solve system of equations to get kinematic limits
    A = np.array([
        [dt_j,                                          -1,         0    ],
        [dt_j ** 2,                                     dt_a,      -1    ],
        [2/3.0 * dt_j ** 3 + 0.5 * (dt_j ** 2) * dt_a,  dt_a ** 2,  dt_v ]])
    b = np.array([0, 0, p])
    lims = np.linalg.solve(A, b)

    output['v_lim'] = lims[2]
    output['a_lim'] = lims[1]
    output['j_lim'] = lims[0]

def generate_profile_from_params(output, timestep):

    T = output['t']
    t_vals = np.arange(0, T[7], timestep)
    p = [0]
    v = [0]
    a = [0]
    j = [0]

    j_lim = output['j_lim']
    a_lim = output['a_lim']
    v_lim = output['v_lim']

    for t in t_vals[1:]:
        if t >= T[0] and t < T[1]:
            j.append(j_lim)        
            a.append(a[-1] + j[-1] * timestep)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
        elif t >= T[1] and t < T[2]:
            j.append(0)
            a.append(a_lim)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
        elif t >= T[2] and t < T[3]:
            j.append(-j_lim)
            a.append(a[-1] + j[-1] * timestep)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
        elif t >= T[3] and t < T[4]:
            j.append(0)
            a.append(0)
            v.append(v_lim)
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
        elif t >= T[4] and t < T[5]:
            j.append(-j_lim)
            a.append(a[-1] + j[-1] * timestep)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
        elif t >= T[5] and t < T[6]:
            j.append(0)
            a.append(-a_lim)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
        elif t >= T[6] and t < T[7]:
            j.append(j_lim)
            a.append(a[-1] + j[-1] * timestep)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)


   
    return (t_vals, p, v, a, j)


def plot_data(t,p,v,a,j):
    ax = plt.axes()
    ax.plot(t, p,'r',label='Position')
    ax.plot(t, v,'g',label='Velocity')
    ax.plot(t, a,'b',label='Acceleration')
    ax.plot(t, j,'k',label='Jerk')
    ax.legend()
    plt.show()


if __name__ == '__main__':

    output = generate(p_target)
    if output:
        data = generate_profile_from_params(output, plot_timestep)
        plot_data(*data)

        # Test inverse
        dt_j = output['t'][1]
        dt_a = output['t'][2] - output['t'][1]
        dt_v = output['t'][4] - output['t'][3]
        output2 = generate_inverse(target_position, dt_j, dt_a, dt_v)

        eps = 0.001
        all_valid = True

        if abs(output2['v_lim'] - output['v_lim']) > eps:
            print("v_lim not close")
            all_valid = False
        if abs(output2['a_lim'] - output['a_lim']) > eps:
            print("a_lim not close")
            all_valid = False
        if abs(output2['j_lim'] - output['j_lim']) > eps:
            print("j_lim not close")
            all_valid = False

        if all_valid:
            print("All inverse values valid")

        

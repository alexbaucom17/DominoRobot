import math
import numpy as np
from matplotlib import pyplot as plt

p_target = 20.0 # m
v_max = 1.0  #m/s
a_max = 2.0  #m/s^2
j_max = 8.0 #m/s^3
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

    # Constant jerk regions
    dt_j = a_lim / j_lim
    print("dt_j = {}".format(dt_j))
    dv_j = 0.5 * j_lim * dt_j ** 2  # Positive jerk region
    dp_j1 = 1/6.0 * j_lim * dt_j ** 3 # Positive jerk region
    dp_j2 = (v_lim - dv_j) * dt_j + 0.5 * a_lim * dt_j ** 2 - 1/6 * j_lim * dt_j ** 3  # Negative jerk region
    print("dv_j: {}, dp_j1: {}, dp_j2: {}".format(dv_j, dp_j1, dp_j2))

    # Constant accel region
    dt_a = (v_lim - 2*dv_j ) / a_lim
    print("dt_a = {}".format(dt_a))
    if dt_a <= 0:
        output['a_lim'] = a_lim * beta
        print("New a_lim = {}".format(output['a_lim'] ))
        return output
    dp_a = dv_j * dt_a + 0.5 * a_lim * dt_a ** 2
    print("dp_a: {}".format(dp_a))

    # Constant vel region
    dt_v = (p - 2 * dp_j1 - 2 * dp_j2 - 2 * dp_a) / v_lim
    print("dt_v = {}".format(dt_v))
    if dt_v <= 0:
        output['v_lim'] = alpha * v_lim
        print("New v_lim = {}".format(output['v_lim'] ))
        return output
    dp_v = v_lim * dt_v
    print("dp_v: {}".format(dp_v))

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
        [(dt_j ** 2) * (dt_a - dt_j),     dt_j ** 2 + dt_a ** 2,     dt_v + 2* dt_j ]])
    b = np.array([0, 0, p])
    lims = np.linalg.solve(A, b)

    output['v_lim'] = lims[2]
    output['a_lim'] = lims[1]
    output['j_lim'] = lims[0]

    return output

def generate_profile_from_params(output, timestep):

    j_lim = output['j_lim']
    a_lim = output['a_lim']
    v_lim = output['v_lim']

    T = output['t']
    t_vals = np.arange(0, T[7], timestep)
    p = [0]
    v = [0]
    a = [0]
    j = [j_lim]

    for t in t_vals[1:]:
        if t >= T[0] and t < T[1]:
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            a.append(a[-1] + j[-1] * timestep)
            j.append(j_lim)        
        elif t >= T[1] and t < T[2]:
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            a.append(a_lim)
            j.append(0)
        elif t >= T[2] and t < T[3]:
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            a.append(a[-1] + j[-1] * timestep)
            j.append(-j_lim)
        elif t >= T[3] and t < T[4]:
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
            v.append(v_lim)
            a.append(0)
            j.append(0)
        elif t >= T[4] and t < T[5]:
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            a.append(a[-1] + j[-1] * timestep)
            j.append(-j_lim)
        elif t >= T[5] and t < T[6]:
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            a.append(-a_lim)
            j.append(0)
        elif t >= T[6] and t < T[7]:
            p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)
            v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
            a.append(a[-1] + j[-1] * timestep)
            j.append(j_lim)
   
    return (t_vals, p, v, a, j)


def plot_data(t,p,v,a,j, output):
    ax = plt.axes()
    ax.plot(t, p,'r',label='Position', linewidth=4)
    ax.plot(t, v,'g',label='Velocity', linewidth=4)
    ax.plot(t, a,'b',label='Acceleration', linewidth=4)
    ax.plot(t, j,'k',label='Jerk', linewidth=4)
    ax.legend()
    ax.set_xlabel('Time (s)')
    ax.set_ylabel("Value (m, m/s, m/s^2, m/s^3)")
    ax.set_title("Trajectory Generation\nDist: {} m, Vel limit: {} m/s, Acc limit: {} m/s^2, Jerk limit: {} m/s^3".format(p_target, v_max, a_max, j_max))

    # Plot vertical lines
    # ax.set_ylabel("Velocity (m/s)")
    # ax.set_ylim((-0.5, 2))
    # t_vert = output['t']
    # y_vert = np.linspace(-10.0, 10.0, 100)
    # for new_t in t_vert:
    #     ax.plot([new_t]*100, y_vert, 'k--', linewidth=3)
    # ax.grid(True)
    # ax.text(t_vert[0] + (t_vert[1] - t_vert[0])/6.0, -0.2, "Constant Jerk", fontsize='x-large')
    # ax.text(t_vert[1] + (t_vert[2] - t_vert[1])/6.0, -0.1, "Constant Accel", fontsize='x-large')
    # ax.text(t_vert[2] + (t_vert[3] - t_vert[2])/6.0, -0.2, "Constant Jerk", fontsize='x-large')
    # ax.text(t_vert[3] + (t_vert[4] - t_vert[3])/3.0, 0.0, "Constant Velocity", fontsize='x-large')
    # ax.text(t_vert[4] + (t_vert[5] - t_vert[4])/6.0, -0.2, "Constant Jerk", fontsize='x-large')
    # ax.text(t_vert[5] + (t_vert[6] - t_vert[5])/6.0, -0.1, "Constant Accel", fontsize='x-large')
    # ax.text(t_vert[6] + (t_vert[7] - t_vert[6])/6.0, -0.2, "Constant Jerk", fontsize='x-large')

    plt.show()


def plot_trapazoidal_profile():

    n_divisions = 91
    a_steps = int(n_divisions/3)
    t = np.linspace(0, 5, n_divisions)
    a = [1]*a_steps + [0]*a_steps + [-1]*a_steps + [0]
    v = [0]
    p = [0]
    for i in range(1, n_divisions):
        dt = t[i] - t[i-1]
        new_v = v[i-1] + a[i-1] * dt
        new_p = p[i-1] + v[i-1] * dt + 0.5 * a[i-1] * dt * dt
        v.append(new_v)
        p.append(new_p)

    ax = plt.axes()
    # ax.plot(t, p,'r',label='Position')
    ax.plot(t, v,'g',label='Velocity',linewidth=5)
    # ax.plot(t, a,'b',label='Acceleration')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel("Velocity (m/s)")
    ax.legend()

    # Plot vertical lines
    t_vert = [t[0], t[a_steps], t[2*a_steps], t[3*a_steps]]
    y_vert = np.linspace(-10.0, 10.0, 100)
    for new_t in t_vert:
        ax.plot([new_t]*100, y_vert, 'k--', linewidth=3)
    ax.set_ylim((-0.5, 3))
    ax.grid(True)
    ax.text(t_vert[0] + (t_vert[1] - t_vert[0])/3.0, -0.2, "Constant Accel", fontsize='x-large')
    ax.text(t_vert[1] + (t_vert[2] - t_vert[1])/3.0, -0.1, "Constant Velocity", fontsize='x-large')
    ax.text(t_vert[2] + (t_vert[3] - t_vert[2])/3.0, -0.2, "Constant Accel", fontsize='x-large')



    plt.show()

    

if __name__ == '__main__':

    output = generate(p_target)
    if output:

        # Test inverse
        dt_j = output['t'][1]
        dt_a = output['t'][2] - output['t'][1]
        dt_v = output['t'][4] - output['t'][3]
        output2 = generate_inverse(p_target, dt_j, dt_a, dt_v)

        eps = 0.01
        all_valid = True
        if output2:
            if abs(output2['v_lim'] - output['v_lim']) / output['v_lim'] > eps:
                print("v_lim not close. Expected: {}, Actual: {}, Percent diff: {}".format(output['v_lim'], output2['v_lim'], abs(output2['v_lim'] - output['v_lim']) / output['v_lim'] ))
                all_valid = False
            if abs(output2['a_lim'] - output['a_lim']) / output['a_lim'] > eps:
                print("a_lim not close. Expected: {}, Actual: {}, Percent diff: {}".format(output['a_lim'], output2['a_lim'], abs(output2['a_lim'] - output['a_lim']) / output['a_lim'] ))
                all_valid = False
            if abs(output2['j_lim'] - output['j_lim']) / output['j_lim'] > eps:
                print("j_lim not close. Expected: {}, Actual: {}, Percent diff: {}".format(output['j_lim'], output2['j_lim'], abs(output2['j_lim'] - output['j_lim']) / output['j_lim'] ))
                all_valid = False

            if all_valid:
                print("All inverse values valid")
        else:
            print("Inverse failed")

        # Generate plot
        data = generate_profile_from_params(output, plot_timestep)
        plot_data(*data, output)

    # plot_trapazoidal_profile()

        

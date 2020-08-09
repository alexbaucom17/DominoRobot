import math
from matplotlib import pyplot as plt

p_target = 10.0  # m
v_max = 1.0  #m/s
a_max = 5.0  #m/s^2
j_max = 10.0 #m/s^3
alpha = 0.8 # velocity decay
beta = 0.8 # acceleation decay
loop_limit = 10 # max loops for convergence

plot_timestep = 0.01


def generate(target_position):

    v_lim = v_max
    a_lim = a_max
    j_lim = j_max

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

    output['done'] = False
    output['v_lim'] = v_lim
    output['a_lim'] = a_lim
    output['j_lim'] = j_lim
    output['t'] = []

    # Constant jerk region
    dt_j = a_lim / j_lim
    dv_j = 0.5 * j_lim * dt_j ** 2
    dp_j = 1/6.0 * j_lim * dt_j ** 3

    # Constant accel region
    dt_a = (v_lim - 2 * dv_j) / a_lim
    if dt_a <= 0:
        output['a_lim'] = a_lim * beta
        return output
    dp_a = dv_j * dt_a + 0.5 * a_lim * dt_a ** 2

    # Constant vel region
    dt_v = (p - 4 * dp_j - 2 * dp_a) / v_lim
    if dt_v <= 0:
        output['v_lim'] = alpha * v_lim
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
    
    return output


def generate_profile_from_params(output, timestep):

    T = [x for x in range(0, output['t'][7], timestep)]
    p = [0]
    v = [0]
    a = [0]
    j = [0]

    j_lim = output['j_lim']
    a_lim = output['a_lim']
    v_lim = output['v_lim']

    for t in T:
        if t >= t[0] and t < t[1]:
            j.append(j_lim)
        elif t >= t[1] and t < t[2]:
            j.append(0)
        elif t >= t[2] and t < t[3]:
            j.append(-j_lim)
        elif t >= t[3] and t < t[4]:
            j.append(0)
        elif t >= t[4] and t < t[5]:
            j.append(-j_lim)
        elif t >= t[5] and t < t[6]:
            j.append(0)
        elif t >= t[7] and t < t[8]:
            j.append(j_lim)

        a.append(a[-1] + j[-1] * timestep)
        v.append(v[-1] + a[-1] * timestep + 0.5* j[-1] * timestep ** 2)
        p.append(p[-1] + v[-1] * timestep + 0.5 * a[-1] * timestep **2 + 1/6.0 * j[-1] * timestep ** 3)

    return (t, p, v, a, j)


def plot_data(t,p,v,a,j):
    ax = plt.axes()
    ax.plot(t, p,'r',label='Position')
    ax.plot(t, v,'g',label='Velocity')
    ax.plot(t, a,'b',label='Acceleration')
    ax.plot(t, j,'k',label='Jerk')
    ax.legend()
    plt.show()


if __name__ == '__main__':

    output = generate(target_position)
    if output:
        data = generate_profile_from_params(output, plot_timestep)
        plot_data(**data)
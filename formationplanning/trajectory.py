"""
FormationPlanning
Copyright (C) 2020 : Northumbria University
            Author : John Hartley

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
from pathlib import Path

import numpy as np
from scipy import interpolate
from scipy.optimize import NonlinearConstraint
from scipy.optimize import minimize
from scipy.optimize import BFGS
from scipy.optimize import NonlinearConstraint
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

# max velocity of particle
v_min = -10
v_max = 10
a_min = -10
a_max = 10

# sampling interval
s_i = 0.01

def preprocess(x):
    # last saved point
    y = x[0, :, :]
    x_t_s = [y]

    # filter small changes in position
    for i in range(x.shape[0] - 1):    
        d1 = np.linalg.norm(x[i+1, 0, :] - y[0, :])
        d2 = np.linalg.norm(x[i+1, 1, :] - y[1, :])
        d3 = np.linalg.norm(x[i+1, 2, :] - y[2, :])
        d4 = np.linalg.norm(x[i+1, 3, :] - y[3, :])
        m = 3
        if d1 > m or d2 > m or d3 > m or d4 > m:
            x_t_s.append(x[i+1, :, :])
            y = x[i+1, :, :]

    x_t_s.append(x[-1, :, :])
    return np.array(x_t_s)

def reparametrise(x, v_min, v_max, a_min, a_max):

    # stack points as 12d waypoints
    waypts = x.reshape((x.shape[0], x.shape[1] * x.shape[2]))

    # initial path
    path = ta.SplineInterpolator(np.linspace(0, 1, waypts.shape[0]), waypts)

    dof = waypts.shape[1]

    # kinematic constraints
    vlim = np.repeat([[v_min, v_max]], dof, axis=0)
    alim = np.repeat([[a_min, a_max]], dof, axis=0)

    pc_vel = constraint.JointVelocityConstraint(vlim)
    pc_acc = constraint.JointAccelerationConstraint(
        alim, discretization_scheme=constraint.DiscretizationType.Interpolation)

    instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper='seidel')
    jnt_traj = instance.compute_trajectory(0, 0)

    duration = jnt_traj.duration
    print("Found optimal trajectory with duration {:f} sec".format(duration))
    ts = np.linspace(0, duration, int(duration * 1 / 0.01))
    qs = jnt_traj.eval(ts)
    qds = jnt_traj.evald(ts)
    qdds = jnt_traj.evaldd(ts)

    # time, position, velocity, acceleration
    return qs, qds, qdds, ts

def map_to_time(x_i):
    l = x_i.shape[0]
    t = np.zeros((l))

    for i in range(1, l):
        # points at last iteration
        x0 = x_i[i - 1]
        # points at current iteration
        x1 = x_i[i]
        d = x0 - x1
        d2 = d * d
        ds = np.sqrt(np.sum(d2, axis=1))
        # longest arc length over the interval between any 1 position
        ds_max = np.amax(ds)
        # delta required to travel at vmax over the interval
        dt = ds_max / v_max
        t[i] = dt
    
    return np.cumsum(t)

def interpolate_trajectory(t, x_t):
    # interpolate to give value every 0.1 seconds
    interp = interpolate.interp1d(t, x_t, axis=0)
    t_n = np.arange(t[0], t[-1], s_i)
    x_t_in = interp(t_n)
    return t_n, x_t_in

def generate_hemisphere_followers(x_t):
    r2 = x_t[:, 0, :] 
    r6 = x_t[:, 1, :] 
    r7 = x_t[:, 2, :] 

    # generate the A B D basis

    c = 0.5 * (r2 + r7)
    
    a = c - r7
    a_l = np.linalg.norm(a, axis=1)
    a_l = np.stack((a_l, a_l, a_l), axis=1)
    a = np.multiply(a, 1 / a_l)
    
    b = c - r6
    b_l = np.linalg.norm(b, axis=1)
    b_l = np.stack((b_l, b_l, b_l), axis=1)
    b = np.multiply(b, 1 / b_l)
    
    d = np.cross(a, b)
    d_l = np.linalg.norm(d, axis=1)
    d_l = np.stack((d_l, d_l, d_l), axis=1)
    d = np.multiply(d, 1 / d_l)

    # radius of the formation
    r = np.linalg.norm(c - r7, axis=1)

    # now construct any points we like in this basis.

    # distance along d to the centre of the circle
    d_2 = r / 2
    d_2 = np.stack((d_2, d_2, d_2), axis=1)
    r = np.stack((r, r, r), axis=1)
    r_2 = np.sqrt(r**2 - d_2**2)

    # vector of first follower
    f1 = (d_2 * d + r_2 * a) + c
    f2 = (d_2 * d - r_2 * a) + c
    f3 = (d_2 * d + r_2 * b) + c
    f4 = (d_2 * d - r_2 * b) + c
    f5 = (r * d) + c

    followers = np.stack((f1, f2, f3, f4, f5), axis=1)
    return followers

def export_trajectory(x_t, directory):

    # save the multi trajectory data
    Path(directory).mkdir(parents=True, exist_ok=True)

    # iterate over each drones trajectories.
    for i in range(0, x_t.shape[1]):
        # save each drone trajectory by component
        for j, c in enumerate(['x', 'y', 'z']):
            fp = "{}{}_t.csv".format(c, i)
            t = x_t[:, i, j]
            t.tofile(directory / fp, sep="\n")


def test_traj_min():

    # change in position
    x1 = 1
    x2 = 2
    
    # initial velocity
    v1 = 0
    # guess
    v2 = 1

    dx = x2 - x1

    # performance constraints
    v_max = 5 # max velocity
    a_max = 5 # max acceleration

    def obj(v2):
        t = (2 * (x2 - x1)) / (v1 + v2) 
        return t

    def cons_a(x):
        # contraint on acceleration
        a = (x[0]**2 - v1**2) / 2 / dx

        return [
            np.abs(a)
        ]

    def cons_v(x):
        # contraint on velocity
        return np.abs(x)

    def callback(xk, state):
        pass

    nonlinear_constraint_1 = NonlinearConstraint(cons_a, 0, a_max, jac='2-point', hess=BFGS())
    nonlinear_constraint_2 = NonlinearConstraint(cons_v, 0, v_max, jac='2-point', hess=BFGS())

    res = minimize(
        obj, x0=v2,
        method='trust-constr',
        options={
                #'xtol': 1e-12,
                 'maxiter': 20000,
                 'disp':True,
                 'verbose': 0,
                },
        #jac=jacobian_flux,
        callback=callback,
        constraints=[
            # sides must have some minimum length
            nonlinear_constraint_1,
            nonlinear_constraint_2
        ])

    dt = res.fun[0]
    v2 = res.x[0]


    # results

    # minimum dt
    print('dt', dt)
    # predicted final velocity
    print('v2', v2)
    # acceleration over the step
    print('acceleration', (v2 - v1) / dt)
    # distance travelled
    print('dx', ((v2 + v1) / 2) * dt)

def test_traj_min_v3(x0, x1, v0):

    dx = x1 - x0

    #dx[np.where(dx==0.0)] = 1e-6
    dx_n = np.linalg.norm(dx)

    # come up with an initial guess for v1
    # have to careful as some solution are non-physical

    sign = np.sign(dx)

    # condition x0==x1 falls under x1 - x0 >= 0    
    sign[sign==0] = 1

    v1 = (v0 * -1) + sign * 1

    # performance constraints
    v_max = 5 # max velocity
    a_max = 1e10 # max acceleration

    def obj(v1):
        # objective function is the time to travel between two points
        v_av = (v0 + v1) / 2
        t = np.linalg.norm(dx) / np.linalg.norm(v_av)
        return t
        #return 2 * (x1[0] - x0[0] + x1[1] - x0[1] + x1[2] - x0[2]) / (v0[0] + v1[0] + v0[1] + v1[1] + v0[2] + v1[2])

    def cons_a(v1):
        "acceleration constraint"

        #v0 = np.array((1, 0, 0))
        #v1 = np.array((0, 1, 1))
        # where there is no position change the acceleration calculation is infinite.
        # however no change in position is also just 0 acceleration
        #a = (v1 - v0) / obj(v1)
        #print('a', a)
        a_n = (np.linalg.norm(v1)**2 - np.linalg.norm(v0)**2) / 2 / np.linalg.norm(dx)
        #a = np.nan_to_num(a, copy=True, nan=0.0, posinf=0.0, neginf=0.0)
        #return np.linalg.norm(a)
        return a_n
        #return np.linalg.norm(a)

    def cons_v(v1):
        # contraint on velocity
        #return v1
        return np.linalg.norm(v1)

    def cons_dx(v1):
        #a_n = (np.linalg.norm(v1)**2 - np.linalg.norm(v0)**2) / 2 / cons_a(v1)
        #return a_n
        return ((v0 + v1) / 2) * obj(v1)
    
    def cons_x1(v1):
        xf = x0 + ((v0 + v1) / 2) * obj(v1)
        return xf

    def callback(v1, srate):
        #print('dx min', np.linalg.norm(cons_dx(v1)), 'dx', np.linalg.norm(dx))
        pass

    # accleration performance constraint
    acceleration_cons = NonlinearConstraint(cons_a, 0, 5, jac='2-point', hess=BFGS())
    # velocity performance constraint
    velocity_cons = NonlinearConstraint(cons_v, 0, 5, jac='2-point', hess=BFGS())
    # final position constraint
    dx_cons = NonlinearConstraint(cons_dx, dx, dx, jac='3-point', hess=BFGS())

    x1_cons = NonlinearConstraint(cons_x1, x1, x1, jac='3-point', hess=BFGS())

    res = minimize(
        obj, x0=v1,
        #callback=callback,
        method='trust-constr',
        options={
                 #'gtol': 1e-2,
                 'maxiter': 20000,
                 'verbose': 0,
                },
        #jac=jacobian_flux,
        constraints=[
            # sides must have some minimum length
            acceleration_cons,
            velocity_cons,
            dx_cons,
            #x1_cons
        ])
    
    dt = res.fun
    v1 = res.x
    a = (v1 - v0) / dt

    x1_p = x0 + (v0 * dt) + 0.5*a*dt*dt
    dx_p = x1_p - x0
    dx_p_n = np.linalg.norm(dx_p) 

    print('results')
    print('dt', dt)
    print('x0', x0)
    print('x1', x1)
    print('v0', v0)
    print('v1', v1)
    print('v av', (v0 + v1) / 2)
    print('v mag', np.linalg.norm(v1))
    print('a mag', cons_a(v1))
    print('a', a)
    print('x1', x1, x1_p)
    print('dx norm', np.linalg.norm(dx), dx_p_n)

    print('\n')

    return dt, v1

def generate_follower_path(x):
    
    r2 = x[:, 0, :]
    r6 = x[:, 1, :]
    r7 = x[:, 2, :]
    r3 = x[:, 3, :]

    # generate the A B D basis
    c = 0.5 * (r2 + r7)

    a = r7 - c
    a_l = np.linalg.norm(a, axis=1)
    a_l = np.stack((a_l, a_l, a_l), axis=1)
    a = np.multiply(a, 1 / a_l)

    b = r6 - c
    b_l = np.linalg.norm(b, axis=1)
    b_l = np.stack((b_l, b_l, b_l), axis=1)
    b = np.multiply(b, 1 / b_l)

    d = np.cross(a, b)
    d_l = np.linalg.norm(d, axis=1)
    d_l = np.stack((d_l, d_l, d_l), axis=1)
    d = np.multiply(d, 1 / d_l)

    # radius of the formation
    r = np.linalg.norm(c - r7, axis=1)


    # now construct any points we like in this basis.

    # distance along d to the centre of the circle
    d_2 = r / 2
    d_2 = np.stack((d_2, d_2, d_2), axis=1)
    r = np.stack((r, r, r), axis=1)
    r_2 = np.sqrt(r**2 - d_2**2)

    # vector of first follower
    f3 = (d_2 * d + r_2 * a) + c
    f1 = (d_2 * d - r_2 * a) + c
    f2 = (d_2 * d + r_2 * b) + c
    f4 = (d_2 * d - r_2 * b) + c
    f5 = (r * d) + c

    followers = np.stack((f1, f2, f3, f4, f5), axis=1)

    return followers

def path_length(path):
    # total path lengths
    path_length = 0
    for i in range(path.shape[0] - 1):
        for j in range(4):
            p0 = path[i, j, :]
            p1 = path[i+1, j, :]
            path_length += np.linalg.norm(p1-p0)
    return path_length
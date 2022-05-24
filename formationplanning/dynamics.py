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
import numpy as np
from scipy.optimize import minimize
from scipy.optimize import BFGS
from scipy.optimize import NonlinearConstraint
from numpy import inf
from numpy import nan
import sys

def dynamics_1d(x0, dt, x_r, u_mod, plot=False):

    # proportional gain
    k = 20
    # system matrix
    A = np.array([[1, dt],[0, 1]])
    # input matrix
    B = np.array([0.5 * dt, dt])
    
    x_t = []
    v_t = []
    u_t = []

    c = 0

    for i in range(x_r.shape[0]):
        try:
            u = k * (x_r[i] - x0[0])
            c = i
        except IndexError:
            u = k * (x_r[c] - x0[0])            
        if u > u_mod:
            u = u_mod
        if u < -u_mod:
            u = -u_mod
        x0 = A.dot(x0) + B.dot(u) 
        x_t.append(x0[0])
        v_t.append(x0[1])
        u_t.append(u)
    t = np.arange(0, x_r.size) * dt

    return np.array(x_t), np.array(v_t), np.array(u_t)

def dynamics_sim(p0, x_r, dt, u_mod):
    """
    p0 - starting position
    x_r - desired trajectory
    dt - time step/interval in desired trajectory
    """

    x0 = np.array([p0[0], 0])
    x, xd, xdd = dynamics_1d(x0, dt, x_r[:, 0], u_mod) # x component
    y0 = np.array([p0[1], 0])
    y, yd, ydd = dynamics_1d(y0, dt, x_r[:, 1], u_mod) # component
    z0 = np.array([p0[2], 0])
    z, zd, zdd = dynamics_1d(z0, dt, x_r[:, 2], u_mod) # component of the first drone

    x_t = np.stack((x, y, z), axis=1)
    xd_t = np.stack((xd, yd, zd), axis=1)
    xdd_t = np.stack((xdd, ydd, zdd), axis=1)

    return x_t, xd_t, xdd_t

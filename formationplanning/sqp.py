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
import math

import numpy as np
from scipy.optimize import minimize
from scipy.optimize import BFGS
from scipy.optimize import NonlinearConstraint

target = []
x_t = []
area_t = []
phi_t = []


s = 5

n_obs = 0

def flux_through_triangle(A, B, C, P):
    """
        Evaluate the solid angle illuminating a triangle.

        The triangle is defined by defined A, B, C by a point P.
    """

    """Gets and prints the spreadsheet's header columns

    Parameters
    ----------
    A : np.array((3))
        Vertex A of the triangle

    B : np.array((3))
        Vertex B of the triangle

    C : np.array((3))
        Vertex C of the triangle

    P : np.array((3))
        Source

    Returns
    -------
    float
        The value of the solid angle.
    """
    a = P - A
    b = P - B
    c = P - C

    num = np.dot(np.cross(a, b), c)

    a_m = np.sqrt(a.dot(a))
    b_m = np.sqrt(b.dot(b))
    c_m = np.sqrt(c.dot(c))

    de = a_m * b_m * c_m + np.dot(a, b) * c_m + np.dot(a, c) * b_m + np.dot(b, c) * a_m
    phi = math.atan2(num, de) * 2

    #print('phi triangle', phi)
    return phi

def flux(x):
    """
        Evaluate the Flux through the surface x.

    Parameters
    ----------
    x : np.array((12))
        Fortran order list of coordinates for the surface

    Returns
    -------
    float
        The value of the solid angle.
    """

    # map the solution space back to the vector points.
    p = x.reshape((4, 3), order='F')

    # Order of vertices [r2, r6, r7, r3]

    # define the triangulation for the surface.
    triangle_list = [
        # r2, r6, r7
        [0, 2, 1],
        # r2, r7, r3
        [0, 3, 2]
    ]

    # total flux through triangulated surface
    phi = 0

    # evaluate the flux through each triangle.
    for i, tri in enumerate(triangle_list):
        # get triangle vertices
        r1 = p[tri[0]]
        r2 = p[tri[1]]
        r3 = p[tri[2]]

        # evaluate the flux through the triangle.
        phi_i = flux_through_triangle(r1, r2, r3, target)
        phi += phi_i

    return phi


def jacobian_flux(x):
    """
        Evaluate the jacobian of the flux through the surface x.

        Evaluating the jacobian is significantly faster than using
        an automatic differentiation provided by numpy.

        The jacobian is evaluated using a central finite difference

    Parameters
    ----------
    x : np.array((12))
        Fortran order list of coordinates for the surface

    Returns
    -------
    np.array(12)
        The flux jacobian
    """

    # jacobian
    j = np.zeros(12)

    dx = 1e-8

    for i, x_i in enumerate(x):

        # evaluate the derivate using a
        # central finite difference.

        x_i_u = x_i + dx
        x_i_l = x_i - dx

        x[i] = x_i_u
        flux_u = flux(x)

        x[i] = x_i_l
        flux_l = flux(x)

        # derivative
        j_x = (flux_u - flux_l) / 2 / dx
        j[i] = j_x

        # recover original position
        x[i] = x_i

    return j

def callback(xk, state):
    """Memorise the state at each iteration of the minimisation."""
    # save the positions
    x_t.append(xk)
    # save the current flux
    phi_t.append(flux(xk))
    area_t.append(area(xk))
    return False

def cons_f(x):
    """"Function to measure the independence of the connecting edges of the surface."""
    p = x.reshape((4, 3), order='F')

    a = p[0] - p[1]
    b = p[1] - p[2]
    c = p[2] - p[3]
    d = p[3] - p[0]

    return [
        a.dot(a),
        b.dot(b),
        c.dot(c),
        d.dot(d)
    ]

def cons_area(x):
    """"A function to measure the area of surface by four metrics."""
    p = x.reshape((4, 3), order='F')

    a = p[0] - p[1]
    b = p[1] - p[2]
    c = p[2] - p[3]
    d = p[3] - p[0]

    return [
        np.linalg.norm(np.cross(a, b)),
        np.linalg.norm(np.cross(b, c)),
        np.linalg.norm(np.cross(c, d)),
        np.linalg.norm(np.cross(d, a))
    ]

def cons_angle(x):
    """"A function to measure the angles between each connecting edge in the surface."""
    p = x.reshape((4, 3), order='F')

    a = p[0] - p[1]
    b = p[1] - p[2]
    c = p[2] - p[3]
    d = p[3] - p[0]

    return [
        np.dot(a, b),
        np.dot(b, c),
        np.dot(c, d),
        np.dot(d, a)
    ]

def cons_z(x):
    """"A function to return the z height of each particle."""
    p = x.reshape((4, 3), order='F')

    return [
        p[0][2],
        p[1][2],
        p[2][2],
        p[3][2],
    ]

def cons_side_diff(x):
    """"A function to measure the difference in the sidelengths of the connecting edges."""
    p = x.reshape((4, 3), order='F')

    a = p[0] - p[1]
    b = p[1] - p[2]
    c = p[2] - p[3]
    d = p[3] - p[0]

    return [
        np.linalg.norm(a) - np.linalg.norm(b),
        np.linalg.norm(b) - np.linalg.norm(c),
        np.linalg.norm(c) - np.linalg.norm(d),
        np.linalg.norm(d) - np.linalg.norm(a),
    ]

def area(x):
    """
        Measure the area of the surface.

    Parameters
    ----------
    x : np.array((12))
        Fortran order list of coordinates for the surface.

    Returns
    -------
    float
        The area of the surface.
    """
    p = x.reshape((4, 3), order='F')

    # surface connectivity
    triangle_list = [
        # r2, r6, r7
        [0, 2, 1],
        # r2, r7, r3
        [0, 3, 2]
    ]

    area = 0.0

    for t in triangle_list:
        a = p[t[0]]
        b = p[t[1]]
        c = p[t[2]]

        ab = a - b
        ca = c - a
        area += np.linalg.norm(np.cross(ab, ca)) / 2

    return area

def obj(x):
    """
        Weighted objective function for flux and area

    Parameters
    ----------
    x : np.array((12))
        Fortran order list of coordinates for the surface.

    Returns
    -------
    float
        scalar mapping
    """
    a = 1
    b = 0
    return a * flux(x) + b * area(x)

def cons_flux(x):
    return [flux(x)]

def minimise_flux(targets, surface, l=None):
    """
        Minimise the flux generated by a target through a connected surface.

    Parameters
    ----------
    targets : np.array((n, 3))
        array of targets or single target with Numpy single dimension.

    surface : np.array((4, 3))
        four 3d points which define a surface.

    l: float (optional)
        length of contraint for the sidelength of the open surface. For multiple targets
        this is calculated internally.

    Returns
    -------
    x_t: np.array((iterations, 4, 3))
        Iteration history of the surface.
    phi_t: np.array((iteration))
        Iteration history of the flux through the surface.
    """
    global target
    global x_t
    global phi_t

    # Reset surface, and flux history.
    x_t = []
    phi_t = []

    # "Starting value guess"
    x0 = surface.flatten(order='F')

    # find the centre of mass of the targets
    if targets.ndim > 1:
        n = targets.shape[0]
        centre = np.zeros((3))
        for t in targets:
            centre += t

        centre = centre / n

        d = []
        for t in targets:
            d.append(np.linalg.norm(t - centre))

        # max side length of formation. 10 is a padding offset
        l = (2 * np.amax(d) + 10) / np.sqrt(2)

        # new target is the centre of mass
        target = centre

    # use given target
    else:
        target = targets

    nonlinear_constraint_1 = NonlinearConstraint(cons_f, l**2, l**2, jac='2-point', hess=BFGS())

    # some other contraints to play with
    # nonlinear_constraint_3 = NonlinearConstraint(cons_area, 25, np.inf, jac='2-point', hess=BFGS())
    # nonlinear_constraint_2 = NonlinearConstraint(cons_flux, 10 * 4 * np.pi, 10 * 4 * np.pi, jac='2-point', hess=BFGS())
    # nonlinear_constraint_4 = NonlinearConstraint(cons_angle, 0, np.pi / 8, jac='2-point', hess=BFGS())
    # nonlinear_constraint_5 = NonlinearConstraint(cons_side_diff, 0, 0, jac='2-point', hess=BFGS())
    # nonlinear_constraint_6 = NonlinearConstraint(cons_z, 0, np.inf, jac='2-point', hess=BFGS())

    minimize(
        obj, x0=x0,
        method='trust-constr',
        #tol=1e-8,
        options={'maxiter': 100000,
                 'disp':True,
                 'verbose': 0,
                },
        jac=jacobian_flux,
        callback=callback,
        constraints=[
                    # sides must have some minimum length
                    nonlinear_constraint_1,

                    # other constraints to play with
                    # flux constraint
                    #nonlinear_constraint_2,
                    # side are perpendicular
                    #nonlinear_constraint_4,
                    # sides are equal
                    #nonlinear_constraint_5
        ])

    x_j = np.reshape(x_t, (len(x_t), 4, 3), order='F')

    return x_j, phi_t

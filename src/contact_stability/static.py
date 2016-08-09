#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 Stephane Caron <stephane.caron@normalesup.org>
#
# This file is part of contact_stability.
#
# contact_stability is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
#
# contact_stability is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# contact_stability. If not, see <http://www.gnu.org/licenses/>.

import bretl_projection
import cdd
import cvxopt

from numpy import array, dot, hstack, vstack, zeros


BB_SIZE = 50  # [m], bounding box size


def compute_static_area_bretl(G, F, mass, z_out):
    """
    Compute the COM static-equilibrium polygon.

    G -- grasp matrix of the contact set
    F -- friction matrix (in half-space representation) of all contact wrenches
    mass -- total mass of the robot
    z_out -- altitude of the output polygon
    """

    # Inequality constraints on [f_all, u, v]
    lp_G = zeros((F.shape[0]+4, F.shape[1]+2))
    lp_G[:-4, :-2] = F
    lp_G[-4, -2] = 1
    lp_G[-3, -2] = -1
    lp_G[-2, -1] = 1
    lp_G[-1, -1] = -1
    lp_G = cvxopt.matrix(lp_G)
    lp_h = zeros(F.shape[0]+4)
    lp_h[-4:] = array([BB_SIZE, BB_SIZE, BB_SIZE, BB_SIZE])
    lp_h = cvxopt.matrix(lp_h)

    # Equality constraints on [f_all, u, v]
    C = G[(0, 1, 2, 5), :]
    D = 1. / (-mass * 9.81) * vstack([-G[4, :], +G[3, :]])
    lp_A = zeros((C.shape[0]+2, C.shape[1]+2))
    lp_A[:-2, :-2] = C
    lp_A[-2:, :-2] = D
    lp_A[-2:, -2:] = array([[-1, 0], [0, -1]])
    lp_A = cvxopt.matrix(lp_A)
    d = -array([0, 0, mass * 9.81, 0])
    lp_b = zeros(C.shape[0]+2)
    lp_b[:-2] = d
    lp_b = cvxopt.matrix(lp_b)

    lp_q = cvxopt.matrix(zeros(F.shape[1]+2))

    lp = lp_q, lp_G, lp_h, lp_A, lp_b

    res, P = bretl_projection.ComputePolygon(lp)
    if not res:
        msg = "bretl_projection.ComputePolygon: "
        msg += "could not optimize in direction %s" % str(P)
        raise Exception(msg)
    else:
        P.sort_vertices()
        vertices_list = P.export_vertices()
        vertices = [array([v.x, v.y, z_out]) for v in vertices_list]
        return vertices, []


def compute_static_area_cdd(G, F, mass, z_out):
    """
    Compute the COM static-equilibrium polygon.

    G -- grasp matrix of the contact set
    F -- friction matrix (in half-space representation) of all contact wrenches
    mass -- total mass of the robot
    z_out -- altitude of the output polygon
    req_time -- time at which the request was scheduled
    """
    b = zeros((F.shape[0], 1))
    # the input [b, -F] to cdd.Matrix represents (b - F x >= 0)
    # see ftp://ftp.ifor.math.ethz.ch/pub/fukuda/cdd/cddlibman/node3.html
    M = cdd.Matrix(hstack([b, -F]), number_type='float')
    M.rep_type = cdd.RepType.INEQUALITY

    # Equalities:  C [GAW_1 GAW_2 ...] + d == 0
    C = G[(0, 1, 2, 5), :]
    d = -array([0, 0, mass * 9.81, 0])
    # the input [d, -C] to cdd.Matrix.extend represents (d - C x == 0)
    # see ftp://ftp.ifor.math.ethz.ch/pub/fukuda/cdd/cddlibman/node3.html
    M.extend(hstack([d.reshape((4, 1)), -C]), linear=True)

    # Convert from H- to V-representation
    # M.canonicalize()
    P = cdd.Polyhedron(M)
    V = array(P.get_generators())
    if V.shape[0] < 1:
        return [], []

    # COM position from GAW:  [pGx, pGy] = D * [GAW_1 GAW_2 ...]
    D = 1. / (-mass * 9.81) * vstack([-G[4, :], +G[3, :]])
    vertices, rays = [], []
    for i in xrange(V.shape[0]):
        if V[i, 0] != 1:
            r = dot(D, V[i, 1:])
            rays.append([r[0], r[1], z_out])
            # rospy.logwarn("We have a ray (%s) here:" % str(V[i, 0]))
            # rospy.logwarn("r = %s" % str(r))
        else:
            p = dot(D, V[i, 1:])
            vertices.append([p[0], p[1], z_out])
    return vertices, rays

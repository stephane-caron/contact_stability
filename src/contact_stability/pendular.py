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

from numpy import array, dot, eye, hstack, vstack, zeros


BB_SIZE = 50  # [m], bounding box size


def crossmat(x):
    """Cross-product matrix of a 3D vector"""
    return array([
        [0, -x[2], x[1]],
        [x[2], 0, -x[0]],
        [-x[1], x[0], 0]])


def compute_pendular_area_bretl(G, F, mass, p_in, z_out):
    """
    Compute pendular (COM or ZMP) support area for contact set.

    INPUT:

    - ``G`` -- grasp matrix of the contact set
    - ``F`` -- friction matrix (H-representation) of all contact wrenches
    - ``mass`` -- total mass
    - ``p_in`` -- reference point (COM or ZMP)
    - ``z_out`` -- altitude of the output point
    """
    z_in = p_in[2]
    n = [0, 0, 1]

    # Inequality constraints on [f_all, u, v]
    lp_G = zeros((F.shape[0]+4, F.shape[1]+2))
    lp_G[:-4, :-2] = F
    lp_G[-4, -2] = 1
    lp_G[-3, -2] = -1
    lp_G[-2, -1] = 1
    lp_G[-1, -1] = -1
    lp_G = cvxopt.matrix(lp_G)
    lp_h = zeros(F.shape[0] + 4)
    lp_h[-4:] = array([BB_SIZE, BB_SIZE, BB_SIZE, BB_SIZE])
    lp_h = cvxopt.matrix(lp_h)

    # Equality constraints on [f_all, u, v]
    B = vstack([
        hstack([z_in * eye(3), crossmat(n)]),
        hstack([zeros(3), p_in])])  # hstack([-(cross(n, p_in)), n])])
    C = 1. / (- mass * 9.81) * dot(B, G)
    D = (z_out - z_in) / (-mass * 9.81) * G[:2, :]
    lp_A = zeros((C.shape[0]+2, C.shape[1]+2))
    lp_A[:-2, :-2] = C
    lp_A[-2:, :-2] = D
    lp_A[-2:, -2:] = array([[-1, 0], [0, -1]])
    lp_A = cvxopt.matrix(lp_A)
    d = hstack([p_in, [0]])
    lp_b = zeros(C.shape[0]+2)
    lp_b[:-2] = d
    lp_b[-2:] = -p_in[:2]
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


def compute_pendular_area_cdd(G, F, mass, p_in, z_out):
    """
    Compute pendular (COM or ZMP) support area for contact set.

    INPUT:

    - ``G`` -- grasp matrix of the contact set
    - ``F`` -- friction matrix (H-representation) of all contact wrenches
    - ``mass`` -- total mass
    - ``p_in`` -- reference point (COM or ZMP)
    - ``z_out`` -- altitude of the output point
    """
    z_in = p_in[2]
    n = [0, 0, 1]

    b = zeros((F.shape[0], 1))
    # the input [b, -F] to cdd.Matrix represents (b - F x >= 0)
    # see ftp://ftp.ifor.math.ethz.ch/pub/fukuda/cdd/cddlibman/node3.html
    M = cdd.Matrix(hstack([b, -F]), number_type='float')
    M.rep_type = cdd.RepType.INEQUALITY

    B = vstack([
        hstack([z_in * eye(3), crossmat(n)]),
        hstack([zeros(3), p_in])])  # hstack([-(cross(n, p_in)), n])])
    C = 1. / (- mass * 9.81) * dot(B, G)
    d = hstack([p_in, [0]])
    # the input [d, -C] to cdd.Matrix.extend represents (d - C x == 0)
    # see ftp://ftp.ifor.math.ethz.ch/pub/fukuda/cdd/cddlibman/node3.html
    M.extend(hstack([d.reshape((4, 1)), -C]), linear=True)

    # Convert from H- to V-representation
    # M.canonicalize()
    P = cdd.Polyhedron(M)
    V = array(P.get_generators())

    # Project output wrenches to 2D set
    vertices, rays = [], []
    for i in xrange(V.shape[0]):
        f_gi = dot(G, V[i, 1:])[:3]
        if V[i, 0] == 1:  # 1 = vertex, 0 = ray
            p_out = (z_out - z_in) * f_gi / (- mass * 9.81) + p_in
            vertices.append(p_out)
        else:
            r_out = (z_out - z_in) * f_gi / (- mass * 9.81)
            rays.append(r_out)
    return vertices, rays

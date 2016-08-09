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

import numpy
import os
import sys
import threading
import time

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../pymanoid')
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../src')

from contact_stability import compute_static_area_bretl
from pymanoid import init as pymanoid_init
from pymanoid import draw_2d_cone, get_viewer, ContactSet, Contact


nb_contacts = 2
z_out = 2.  # [m]


def compute_support_area(contacts):
    global area_handle
    mass = 38  # [kg]
    G = contacts.compute_grasp_matrix([0, 0, 0])
    F = -contacts.compute_stacked_wrench_cones()
    try:
        vertices, rays = compute_static_area_bretl(G, F, mass, z_out)
        area_handle = draw_2d_cone(
            vertices, rays, numpy.array([0, 0, 1]), 'g-#')
        return vertices, rays
    except Exception as e:
        print "Exception: %s" % e
    return [], []


def execute_area_thread():
    while True:
        compute_support_area(contacts)
        time.sleep(0.01)


if __name__ == "__main__":
    pymanoid_init()
    get_viewer().SetCamera(numpy.array([
        [0.60587192, -0.36596244,  0.70639274, -2.4904027],
        [-0.79126787, -0.36933163,  0.48732874, -1.6965636],
        [0.08254916, -0.85420468, -0.51334199,  2.79584694],
        [0.,  0.,  0.,  1.]]))
    contacts = ContactSet([
        Contact(
            X=0.2,
            Y=0.1,
            pos=[(2 * i - 1) * numpy.random.random(),
                 (2 * i - 1) * numpy.random.random(),
                 0],
            rpy=0.5 * numpy.random.random(3),
            friction=0.5,
            visible=True
        )
        for i in xrange(nb_contacts)])
    thread = threading.Thread(target=execute_area_thread, args=())
    thread.daemon = True
    thread.start()
    # execute_area_thread()
    import IPython
    IPython.embed()

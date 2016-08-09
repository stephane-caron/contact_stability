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

ALGORITHM = 'bretl'

import pebble
import openravepy
import os
import numpy
import rospy
import time
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../pymanoid')

if ALGORITHM == 'bretl':
    from contact_stability import compute_static_area_bretl \
        as _compute_static_area
else:  # ALGORITHM == 'cdd'
    from contact_stability import compute_static_area_cdd \
        as _compute_static_area

from contact_stability.srv import StaticStabilityArea
from contact_stability.srv import StaticStabilityAreaResponse
from geometry_msgs.msg import Point
from pymanoid import register_env, ContactSet, Contact


cdd_timeout = 1  # [s]
last_request_time = 0.  # [s]
nb_workers = 4


def compute_static_area(G, F, mass, z_out, req_time):
    if last_request_time > req_time:
        rospy.logwarn("Dropping expired request...")
        return []
    return _compute_static_area(G, F, mass, z_out)


def convert_contact_set(req_contacts):
    contacts = ContactSet()
    for ct in req_contacts:
        p = ct.position
        quat = ct.orientation
        contact = Contact(
            X=ct.halflen, Y=ct.halfwidth,
            pose=[quat.w, quat.x, quat.y, quat.z, p.x, p.y, p.z],
            friction=ct.friction)
        contacts.append(contact)
    return contacts


def compute_support_area(req):
    global last_request_time
    global pool
    start_time = time.time()
    last_request_time = start_time
    contacts = convert_contact_set(req.contacts)
    area = StaticStabilityAreaResponse(vertices=[])
    last_request_time = start_time
    G = contacts.compute_grasp_matrix(numpy.zeros(3))
    F = -contacts.compute_stacked_wrench_cones()
    task = pool.schedule(
        compute_static_area,
        args=(G, F, req.mass, req.z_out, start_time),
        timeout=cdd_timeout)
    try:
        vertices, rays = task.get()
        area.vertices = [Point(v[0], v[1], v[2]) for v in vertices]
        area.rays = [Point(r[0], r[1], r[2]) for r in rays]
        return area
    except RuntimeError as e:
        rospy.logwarn("cdd: %s" % e)
    except pebble.TimeoutError:
        rospy.logwarn("cdd took more than %d seconds (%.2f s)" %
                      (cdd_timeout, time.time() - start_time))
    return area


if __name__ == "__main__":
    pool = pebble.process.Pool(nb_workers)
    rave_env = openravepy.Environment()
    register_env(rave_env)
    rospy.init_node('static_stability',  # log_level=rospy.DEBUG,
                    disable_signals=True)  # see note in other service
    s = rospy.Service(
        '/contact_stability/static/compute_support_area',
        StaticStabilityArea, compute_support_area)
    rospy.spin()
    rospy.signal_shutdown("[static_server.py] shutdown")

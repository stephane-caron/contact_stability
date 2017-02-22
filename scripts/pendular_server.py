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
import openravepy
import os
import pebble
import rospy
import time
import sys

from contact_stability.srv import PendularArea, PendularAreaResponse
from geometry_msgs.msg import Point
from numpy import array

# Settings
ALGORITHM = 'bretl'
NB_WORKERS = 4

try:
    from pymanoid import register_env, Contact, ContactSet
except ImportError:
    script_path = os.path.dirname(os.path.realpath(__file__))
    sys.path.append(script_path + '/../pymanoid')
    from pymanoid import register_env, Contact, ContactSet

if ALGORITHM == 'bretl':
    from contact_stability import compute_pendular_area_bretl \
        as _compute_pendular_area
else:  # ALGORITHM == 'cdd'
    from contact_stability import compute_pendular_area_cdd \
        as _compute_pendular_area


# Global variables
last_request_time = 0.  # [s]
timeout = 1  # [s]


def compute_pendular_area(G, F, mass, p_in, z_out, req_time):
    if last_request_time > req_time:
        rospy.logwarn("Dropping expired request...")
        return [], []
    return _compute_pendular_area(G, F, mass, p_in, z_out)


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
    area = PendularAreaResponse(vertices=[], rays=[])
    try:
        start_time = time.time()
        last_request_time = start_time
        contacts = convert_contact_set(req.contacts)
        p_in = array([req.p_in.x, req.p_in.y, req.p_in.z])
        G = contacts.compute_grasp_matrix(numpy.zeros(3))
        F = -contacts.compute_stacked_wrench_cones()
        task = pool.schedule(
            compute_pendular_area,
            args=(G, F, req.mass, p_in, req.z_out, start_time),
            timeout=timeout)
        vertices, rays = task.get()
        area.vertices = [Point(v[0], v[1], v[2]) for v in vertices]
        area.rays = [Point(r[0], r[1], r[2]) for r in rays]
        return area
    except RuntimeError as e:
        rospy.logwarn("cdd: %s" % e)
    except pebble.TimeoutError:
        dur = timeout, time.time() - start_time
        rospy.logwarn("cdd took more than %d seconds (%.2f s)" % dur)
        return area
    except Exception as e:
        rospy.logwarn("Exception: %s" % e)
    return area


if __name__ == "__main__":
    pool = pebble.process.Pool(workers=NB_WORKERS, )
    rave_env = openravepy.Environment()
    register_env(rave_env)
    rospy.init_node('pendular_stability',
                    # log_level=rospy.DEBUG,
                    disable_signals=True)
    #
    # It is important to disable ROS signals when using pebble. The latter calls
    # signal.signal (e.g. at pebble.process.pool:291) which seems to be somehow
    # conflicting with ROS. Without disable_signals, in some situations, pebble
    # timeouts will break the client-server pipe of ROS services (see
    # rospy.impl.tcpros_base:657)
    #
    s = rospy.Service(
        '/contact_stability/pendular/compute_support_area',
        PendularArea, compute_support_area)
    rospy.spin()
    rospy.signal_shutdown("[pendular_server.py] shutdown")

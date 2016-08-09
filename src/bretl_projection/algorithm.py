#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 Quang-Cuong Pham <cuong.pham@normalesup.org>
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

from numpy import array, cos, pi, sin
from numpy.random import random
from optim import OptimizeDirection
from polygon import Polygon
from vertex import Vertex


def ComputePolygon(lp):
    """Expand a polygon iteratively."""
    theta = pi * random()
    d1 = array([cos(theta), sin(theta)])
    d2 = array([cos(theta + 2 * pi / 3), sin(theta + 2 * pi / 3)])
    d3 = array([cos(theta + 4 * pi / 3), sin(theta + 4 * pi / 3)])
    res, z1 = OptimizeDirection(d1, lp)
    if not res:
        return False, d1
    res, z2 = OptimizeDirection(d2, lp)
    if not res:
        return False, d2
    res, z3 = OptimizeDirection(d3, lp)
    if not res:
        return False, d3
    v1 = Vertex(z1)
    v2 = Vertex(z2)
    v3 = Vertex(z3)
    P0 = Polygon()
    P0.fromVertices(v1, v2, v3)
    P0.iter_expand(lp, 1000)
    return True, P0

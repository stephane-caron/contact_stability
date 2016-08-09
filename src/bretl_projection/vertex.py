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

from numpy import array, cross
from optim import OptimizeDirection
from pylab import plot
from scipy.linalg import norm


class Vertex:

    def __init__(self, p):
        """Create new vertex from iterable.

        p -- iterable (e.g. list or numpy.ndarray)
        """
        self.x = p[0]
        self.y = p[1]
        self.next = None
        self.expanded = False

    def length(self):
        return norm([self.x-self.next.x, self.y-self.next.y])

    def expand(self, lp):
        v1 = self
        v2 = self.next
        v = array([v2.y-v1.y, v1.x-v2.x])  # orthogonal direction to edge
        v = 1 / norm(v) * v
        res, z = OptimizeDirection(v, lp)
        if not res:
            self.expanded = True
            return False, None
        xopt, yopt = z
        if abs(cross([xopt-v1.x, yopt-v1.y], [v1.x-v2.x, v1.y-v2.y])) < 1e-2:
            self.expanded = True
            return False, None
        else:
            vnew = Vertex([xopt, yopt])
            vnew.next = self.next
            self.next = vnew
            self.expanded = False
            return True, vnew

    def Plot(self):
        plot([self.x, self.next.x], [self.y, self.next.y])

    def Print(self):
        print self.x, self.y, "to", self.next.x, self.next.y

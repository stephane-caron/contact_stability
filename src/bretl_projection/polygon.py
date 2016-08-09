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

import StringIO
import pylab
import scipy

from vertex import Vertex


class Polygon:

    def __init__(self):
        pass

    def fromVertices(self, v1, v2, v3):
        v1.next = v2
        v2.next = v3
        v3.next = v1
        self.vertices = [v1, v2, v3]

    def fromString(self, s):
        buff = StringIO.StringIO(s)
        self.vertices = []
        while(True):
            l = buff.readline()
            l = l.strip(" \n")
            if len(l) < 2:
                break
            x, y = [pylab.double(x) for x in l.split(' ')]
            vnew = Vertex([x, y])
            self.vertices.append(vnew)

        for i in range(len(self.vertices)-1):
            self.vertices[i].next = self.vertices[i+1]
        self.vertices[-1].next = self.vertices[0]

    def all_expanded(self):
        for v in self.vertices:
            if not v.expanded:
                return False
        return True

    def iter_expand(self, qpconstraints, maxiter=10):
        """
        Returns true if there's a edge that can be expanded, and expands that
        edge, otherwise returns False.
        """
        niter = 0
        v = self.vertices[0]
        while not self.all_expanded() and niter < maxiter:
            if not v.expanded:
                res, vnew = v.expand(qpconstraints)
                if res:
                    self.vertices.append(vnew)
                    niter += 1
            else:
                v = v.next

    def sort_vertices(self):
        """
        Export the vertices starting from the left-most and going clockwise.

        Assumes every vertices are on the positive halfplane.
        """
        minsd = 1e10
        ibottom = 0
        for i in range(len(self.vertices)):
            v = self.vertices[i]
            if (v.y + v.next.y) < minsd:
                ibottom = i
                minsd = v.y + v.next.y
        for v in self.vertices:
            v.checked = False
        vcur = self.vertices[ibottom]
        newvertices = []
        while not vcur.checked:
            vcur.checked = True
            newvertices.append(vcur)
            vcur = vcur.next
        newvertices.reverse()
        vfirst = newvertices.pop(-1)
        newvertices.insert(0, vfirst)
        self.vertices = newvertices

    def export_vertices(self, threshold=1e-2):
        export_list = [self.vertices[0]]
        for i in range(1, len(self.vertices)-1):
            vcur = self.vertices[i]
            vlast = export_list[-1]
            if scipy.linalg.norm([vcur.x-vlast.x, vcur.y-vlast.y]) > threshold:
                export_list.append(vcur)
        # always add last vertex
        export_list.append(self.vertices[-1])
        return export_list

    def Plot(self):
        pylab.hold("on")
        for v in self.vertices:
            v.Plot()

    def Print(self):
        print "Polygon contains vertices"
        for v in self.vertices:
            v.Print()

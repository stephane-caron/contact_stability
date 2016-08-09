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

import cvxopt
import cvxopt.solvers

from numpy import array
from warnings import warn


cvxopt.solvers.options['show_progress'] = False
debug = False


def OptimizeDirection(vdir, lp):
    """Optimize in one direction."""
    lp_q, lp_Gextended, lp_hextended, lp_A, lp_b = lp
    lp_q[-2] = -vdir[0]
    lp_q[-1] = -vdir[1]
    try:
        sol = cvxopt.solvers.lp(lp_q, lp_Gextended, lp_hextended, lp_A, lp_b)
        if sol['status'] == 'optimal':
            z = sol['x']
            z = array(z).reshape((lp_q.size[0], ))
            return True, z[-2:]
        else:
            if debug:
                warn("Failed with status %s\n" % sol['status'])
            return False, 0
    except Exception as inst:
        if debug:
            warn("Exception: %s" % repr(inst))
        return False, 0

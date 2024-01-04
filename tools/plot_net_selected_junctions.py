#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# # Copyright (C) 2008-2018 German Aerospace Center (DLR) and others.
# # This program and the accompanying materials
# # are made available under the terms of the Eclipse Public License v2.0
# # which accompanies this distribution, and is available at
# # http://www.eclipse.org/legal/epl-v20.html
# # SPDX-License-Identifier: EPL-2.0
#
# # @file    plot_net_selected_junctions.py
# # @author  Daniel Krajzewicz
# # @author  Michael Behrisch
# # @date    2013-10-28
# # @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
# Import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools"))
    sys.path.append(
        os.path.join(os.environ.get("SUMO_HOME", os.path.join(os.path.dirname(__file__), "..", "..", "..")), "tools"))
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "missing declare environment variable 'SUMO_HOME' as the root directory of SUMO installation "
        "(it should contain folders 'bin', 'tools' and 'docs')")

import sumolib  # noqa
from sumolib.visualization import helpers  # noqa
import matplotlib.pyplot as plt  # noqa


def get_space_separated_args(value):
    new_value = value.split(' ')
    return new_value

def main(args=None):
    """The main function; parses options and plots"""
    # ---------- build and read options ----------
    from optparse import OptionParser
    optParser = OptionParser()
    optParser.add_option("-n", "--net", dest="net", metavar="FILE",
                         help="Defines the network to read")
    optParser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                         default=False, help="If set, the script says what it's doing")
    optParser.add_option("-j", "--junctions",
                             dest="junctions",
                             help="Junctions to be selected. "
                                  "Defaults to all traffic light controlled",
                             type="string",
                             default="")
    optParser.add_option("-w", "--width", dest="width",
                         type="float", default=7, help="Defines the width of the dots")
    optParser.add_option("-c", "--color", dest="color",
                         default='#0050ff', help="Defines the dot color")
    optParser.add_option("--edge-width", dest="defaultWidth",
                         type="float", default=1, help="Defines the edge width")
    optParser.add_option("--edge-color", dest="defaultColor",
                         default='#9e9e9e', help="Defines the edge color")
    # standard plot options
    helpers.addInteractionOptions(optParser)
    helpers.addPlotOptions(optParser)
    # parse
    options, remaining_args = optParser.parse_args(args=args)

    if options.net is None:
        print("Error: a network to load must be given.")
        return 1
    if options.verbose:
        print("Reading network from '%s'" % options.net)
    net = sumolib.net.readNet(options.net)

    jctsn = {}
    if options.junctions == "":
        for jid in net._id2tls:
            j = net._id2tls[jid]
            jctsn[jid] = set()
            for c in j._connections:
                n = c[0].getEdge().getToNode()
                jctsn[jid].add(n)

        jctspX = []
        jctspY = []
        for jid in jctsn:
            x = 0
            y = 0
            n = 0
            for node in jctsn[jid]:
                x += node._coord[0]
                y += node._coord[1]
                n = n + 1
            x = x / n
            y = y / n
            jctspX.append(x)
            jctspY.append(y)

    else:
        jctspX = []
        jctspY = []
        selected_junctions = get_space_separated_args(options.junctions)
        jct_children = [net.getNode(jct) for jct in selected_junctions]
        for jct_child in jct_children:
            jctspX.append(float(jct_child.getCoord()[0]))
            jctspY.append(float(jct_child.getCoord()[1]))

    fig, ax = helpers.openFigure(options)
    ax.set_aspect("equal", None, 'C')
    helpers.plotNet(net, {}, {}, options, "edge")
    plt.plot(jctspX, jctspY, options.color, linestyle='None',
             marker='o', markersize=options.width)
    options.nolegend = True
    helpers.closeFigure(fig, ax, options)


if __name__ == "__main__":
    sys.exit(main(sys.argv))

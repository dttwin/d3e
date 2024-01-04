#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2008-2018 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
# SPDX-License-Identifier: EPL-2.0

# @file    plot_net_selected_edgesLanes.py
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @date    2014-02-19
# @version $Id$

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


def main(args=None):
    """The main function; parses options and plots"""
    # ---------- build and read options ----------
    from optparse import OptionParser
    optParser = OptionParser()
    optParser.add_option("-n", "--net", dest="net", metavar="FILE",
                         help="Defines the network to read")
    optParser.add_option("-i", "--selection", dest="selection", metavar="FILE",
                         help="Defines the selection to read either edges or lanes")
    optParser.add_option("--selected-width", dest="selectedWidth",
                         type="float", default=1.25, help="Defines the width of selected edges/lanes")
    optParser.add_option("--color", "--selected-color", dest="selectedColor",
                         default='#0050ff', help="Defines the color of selected edges/lanes")
    optParser.add_option("--edge-width", dest="defaultWidth",
                         type="float", default=1.25, help="Defines the width of not selected edges/lanes")
    optParser.add_option("--edge-color", dest="defaultColor",
                         default='#9e9e9e', help="Defines the color of not selected edges/lanes")
    optParser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                         default=False, help="If set, the script says what it's doing")
    # standard plot options
    helpers.addInteractionOptions(optParser)
    helpers.addPlotOptions(optParser)
    # parse
    options, remaining_args = optParser.parse_args(args=args)

    if options.net is None:
        print("Error: a network to load must be given.")
        return 1
    # if options.selection is None:
    #     print("Error: a selection to load must be given.")
    #     return 1
    if options.verbose:
        print("Reading network from '%s'" % options.net)
    net = sumolib.net.readNet(options.net)

    colors = {}
    widths = {}
    if options.selection is None:
        obj_type = "edge"
    else:
        selection = sumolib.files.selection.read(options.selection, lanes2edges=False)
        try:
            for e in selection["edge"]:
                colors[e] = options.selectedColor
                widths[e] = options.selectedWidth
            obj_type = "edge"
        except KeyError:
            for e in selection["lane"]:
                colors[e] = options.selectedColor
                widths[e] = options.selectedWidth
            obj_type = "lane"

    fig, ax = helpers.openFigure(options)
    ax.set_aspect("equal", None, 'C')
    helpers.plotNet(net, colors, widths, options, obj_type)
    options.nolegend = True
    helpers.closeFigure(fig, ax, options)


if __name__ == "__main__":
    sys.exit(main(sys.argv))

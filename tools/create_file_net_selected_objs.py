#!/usr/bin/env python
"""

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2009-2017 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import
from __future__ import print_function

import logging
import optparse
import os
import sys
import openpyxl

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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


def get_net_file_directory(net_file):
    """ Returns the directory containing the net file given. """

    dirname = os.path.split(net_file)[0]
    return dirname

def get_space_separated_args(value):
    new_value = value.split(' ')
    return new_value

if __name__ == "__main__":
    # pylint: disable-msg=C0103

    logging.basicConfig(level="INFO")

    option_parser = optparse.OptionParser()
    option_parser.add_option("-n", "--net-file",
                             dest="net_file",
                             help="Network file to work with. Mandatory.",
                             type="string",
                             default="lust.net.xml")
    option_parser.add_option("-j", "--junctions",
                             dest="junctions",
                             help="Junctions to be selected. "
                                  "Defaults to output all junctions",
                             type="string",
                             default = "")
    option_parser.add_option("-t", "--tls",
                             dest="tls",
                             help="call it outputs only traffic light controlled junctions, "
                                  "instead of all junctions, as default."
                                  "Not with --junctions/-j",
                             action="store_true",
                             default=False)
    option_parser.add_option("--onlyjcts",
                             dest="onlyJunctions",
                             help="If it is desired only the junctions to be put in output file. "
                                  "Not with -l/--lanes and -e/--edges"
                                  "Defaults to false, so it will outputs only edges of the inputted junctions",
                             action="store_true",
                             default=False)
    option_parser.add_option("-e", "--edges",
                             dest="edges",
                             help="Edges to be selected. Don't use with --junctions/-j neither --onlyjcts and -l/--lanes"
                                  "Defaults to edges of inputted junctions",
                             type="string",
                             default="")
    option_parser.add_option("-l", "--lanes",
                             dest="lanes",
                             help="Lanes to be selected. Don't use with --junctions/-j neither --onlyjcts and -e/--edges"
                                  "Defaults to edges of inputted junctions",
                             type="string",
                             default="")
    option_parser.add_option("--incoming",
                             dest="incoming",
                             help="call it to get incoming edges of inputted junctions, "
                                  "it can be used with --outgoing and --connections",
                             action="store_true",
                             default=False)
    option_parser.add_option("--outgoing",
                             dest="outgoing",
                             help="call it to get outgoing edges of inputted junctions,"
                                  "it can be used with --incoming and --connections",
                             action="store_true",
                             default=False)
    option_parser.add_option("--connections",
                             dest="connections",
                             help="call it to get connection edges of inputted junctions,"
                                  "it can be used with --incoming and --outgoing",
                             action="store_true",
                             default=False)
    option_parser.add_option("-r", "--results-file",
                             dest="results",
                             help="The name of the file with all the selected lanes ingressing and egressing the junctions. "
                                  "Possible xlsx or txt (SUMO selection format) file. Defaults to managed_lanes.xlsx.",
                             type="string",
                             default="allEdges_modelled_area.xlsx")

    (options, args) = option_parser.parse_args()
    if not options.net_file:
        print("Missing arguments")
        option_parser.print_help()
        exit()

    logging.info("Reading net...")
    net = sumolib.net.readNet(options.net_file)

    global net_obj_ids

    if options.connections:
        from xml.etree import ElementTree
        # Read Network File
        tree_network = ElementTree.parse(options.net_file)
        internal_edgeIDs = {edge.get("id") for edge in list(tree_network.iter('edge'))
                            if edge.get('function') == "internal"}

    if options.edges != "" or options.lanes != "":
        if options.edges != "":
            net_obj_ids = options.edges.split(",")
        if options.lanes != "":
            net_obj_ids = options.lanes.split(",")
    else:
        if options.onlyJunctions:
            net_obj_ids = []
            if options.tls:
                for jct_child in net._tlss:
                    net_obj_ids.append(jct_child.getID())
            elif options.junctions == "":
                for jct_child in net.getNodes():
                    net_obj_ids.append(jct_child.getID())
            else:
                net_obj_ids = get_space_separated_args(options.junctions)
        else:
            edges_id = set()
            if options.junctions == "":
                if options.tls:
                    jct_children = net._tlss
                else:
                    jct_children = net.getNodes()
            else:
                selected_junctions = get_space_separated_args(options.junctions)
                jct_children = [net.getNode(jct) for jct in selected_junctions]

            for jct_child in jct_children:
                if options.connections:
                    copied_internal_edgeIDs = internal_edgeIDs.copy()
                    jctID = jct_child.getID()
                    for internal_edgeID in copied_internal_edgeIDs:
                        if jctID in internal_edgeID:
                            internal_edgeIDs.remove(internal_edgeID)
                            edges_id.add(internal_edgeID)

                if options.incoming or options.outgoing:
                    for connection_child in jct_child.getConnections():
                        if options.incoming:
                            from_edge_child = connection_child.getFrom()
                            edges_id.add(from_edge_child.getID())
                        if options.outgoing:
                            to_edge_child = connection_child.getTo()
                            edges_id.add(to_edge_child.getID())

            net_obj_ids = edges_id


    # Create a file to input a network element selection
    if options.results.split(".")[-1] == "xlsx":
        wb = openpyxl.Workbook()
        sheet = wb.active
        if options.onlyJunctions:
            sheet.title = "Selected Junctions"
        elif options.lanes != "":
            sheet.title = "Selected Lanes"
        else:
            sheet.title = "Selected Edges"
        for obj_ind, obj_id in enumerate(net_obj_ids):
            sheet.cell(row=obj_ind + 1, column=1).value = obj_id

        # Finally, save the file and give it a name
        wb.save(options.results)
    else:
        # assume txt file in the format of SUMO selection
        with open(options.results, 'a') as the_file:
            if options.onlyJunctions:
                for jct_id in net_obj_ids:
                    the_file.write("junction:" + jct_id + "\n")
            elif options.lanes != "":
                for lane_id in net_obj_ids:
                    the_file.write("lane:" + lane_id + "\n")
            else:
                for edge_id in net_obj_ids:
                    the_file.write("edge:" + edge_id + "\n")
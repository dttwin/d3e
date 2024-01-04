## Simulation module to be connected to another main script. It contains the simulator interface and the statistics of
# the simulation.
# @author  Andre Maia Pereira
# @date    2021-01-27

## IMPORT MODULES
from __future__ import absolute_import
from __future__ import print_function

# Import SUMO class modules
import sys

# Import modules for all classes
import os
from xml.etree import ElementTree

from collections import Counter
import numpy as np

from Applications.base import commonFunctions

## SCRIPT FUNCTIONS
class SUMO:
    """Interaction between main script and the simulation using SUMO"""

    def __init__(self,
                 cfg_obj,  # configScenarioName class object
                 name):  # object name
        """Start SUMO as a server, then connect and run the main script, afterwards init. this class instances"""

        # Import python modules from the $SUMO_HOME/tools directory
        try:
            sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools"))
            sys.path.append(
                os.path.join(os.environ.get("SUMO_HOME", os.path.join(os.path.dirname(__file__), "..", "..", "..")),
                             "tools"))
            import sumolib  # noqa
        except ImportError:
            sys.exit(
                "missing declare environment variable 'SUMO_HOME' as the root directory of SUMO installation "
                "(it should contain folders 'bin', 'tools' and 'docs')")

        # this script has been called from the command line. It will start sumo as a server, then connect and run
        try:
            if cfg_obj.SIMUL_INTERFACE == "libsumo":
                import libsumo
                self.traci = libsumo
                # As GUI is not working yet with libsumo, use only without GUI
                sumoBinary = sumolib.checkBinary('sumo')
                cfg_obj.SIMUL_INTERFACE = "libsumo"
            elif cfg_obj.SIMUL_INTERFACE == "traci":
                raise ImportError
            else:
                sys.exit("Interface that connects the simulator with this script is not defined")
        except ImportError:
            import traci
            self.traci = traci
            cfg_obj.SIMUL_INTERFACE = "traci"
            if cfg_obj.options.nogui:
                sumoBinary = sumolib.checkBinary('sumo')
            else:
                sumoBinary = sumolib.checkBinary('sumo-gui')

        # Network Files
        self.NET_FILE = cfg_obj.options.net_filename
        self.sumolibNet = sumolib.net.readNet(self.NET_FILE, withInternal=True)

        # Define SUMO additional files
        if cfg_obj.options.fixed_additional_files != "":
            add_filenames = cfg_obj.options.fixed_additional_files.split(",")
        else:
            add_filenames = []
        if cfg_obj.options.dyn_additional_files != "":
            add_filenames.extend([".".join(dynf.split('.')[:-2])
                                  + "." + cfg_obj.options.config_name + "."
                                  + ".".join(dynf.split('.')[-2:])
                                  for dynf in cfg_obj.options.dyn_additional_files.split(",")])
        self.additional_files = ",".join(add_filenames)

        # Define SUMO route files
        if cfg_obj.options.fixed_route_files != "":
            route_filenames = cfg_obj.options.fixed_route_files.split(",")
        else:
            route_filenames = []
        if cfg_obj.options.dyn_route_files != "":
            route_filenames.extend([".".join(dynf.split('.')[:-2])
                                    + "." + cfg_obj.options.config_name + "."
                                    + ".".join(dynf.split('.')[-2:])
                                    for dynf in cfg_obj.options.dyn_route_files.split(",")])
        self.route_files = ",".join(route_filenames)

        # Start Sumo as a subprocess and then connect it to the python script that runs Traci based on file config
        params = cfg_obj.defSUMOparams(sumoBinary, self.additional_files, self.route_files) + cfg_obj.args
        self.traci.start(params)

        # Object name
        self.name = name

        # Get the version of SUMO
        try:
            SUMOversion = self.traci.getVersion()[1]
            if SUMOversion == "SUMO UNKNOWN":
                raise AttributeError
            else:
                if "_" in SUMOversion:
                    self.SUMO_RELEASE = int(SUMOversion.split("_")[0][-1])
                    self.SUMO_PATCH = int(SUMOversion.split("_")[1])
                else:
                    self.SUMO_RELEASE = int(SUMOversion.split(".")[0][-1])
                    self.SUMO_PATCH = int(SUMOversion.split(".")[1])
        except (KeyError, AttributeError):
            # If module has no getVersion() or version is unkown, assumes that version is not earlier than 1.1.0
            self.SUMO_RELEASE = 1
            self.SUMO_PATCH = 1

        # Class default constants
        self.NOGUI = cfg_obj.options.nogui
        self.SEL_EDGES = cfg_obj.options.sel_edges
        self.SEL_LANES = cfg_obj.options.sel_lanes
        self.SEL_JCTS = cfg_obj.options.sel_jcts
        if cfg_obj.options.net_xlim == "" or cfg_obj.options.net_ylim == "":
            xmin, ymin, xmax, ymax = self.sumolibNet.getBoundary()
            self.NET_XLIM = str(xmin) + "," + str(xmax)
            self.NET_YLIM = str(ymin) + "," + str(ymax)
        else:
            self.NET_XLIM = cfg_obj.options.net_xlim
            self.NET_YLIM = cfg_obj.options.net_ylim
        self.NET_XLIM_ZOOMED = cfg_obj.options.net_xlim_zoomed.split(";")
        self.NET_YLIM_ZOOMED = cfg_obj.options.net_ylim_zoomed.split(";")

        # Define type of vehicles to be excluded from the modelled area, so the lanes that only allow these vehicles
        # will not be modelled by the system
        self.EXCLUDING_TYPES = set(cfg_obj.options.excluding_types.split(","))

        # Initialize and constants for controlling actuated traffic lights
        self.LAST_STEP_TIME_SINCE_DETECTION = self.traci.constants.LAST_STEP_TIME_SINCE_DETECTION
        self.LAST_STEP_VEHICLE_NUMBER = self.traci.constants.LAST_STEP_VEHICLE_NUMBER
        self.TL_CURRENT_PROGRAM = self.traci.constants.TL_CURRENT_PROGRAM
        self.TL_CURRENT_PHASE = self.traci.constants.TL_CURRENT_PHASE
        self.TL_PHASE_DURATION = self.traci.constants.TL_PHASE_DURATION
        self.TL_NEXT_SWITCH = self.traci.constants.TL_NEXT_SWITCH

        # Initialize and constants for generating vehicles according to detector measurements
        self.VAR_CURRENT_TRAVELTIME = self.traci.constants.VAR_CURRENT_TRAVELTIME

        # Define files with traffic light plans
        if cfg_obj.options.tlights_filename != "":
            self.MOV_GROUPS_FILE = cfg_obj.options.tlights_filename
        else:
            self.MOV_GROUPS_FILE = self.NET_FILE

        # Define files with vehicle composition/distribution
        if cfg_obj.options.vtypes_filename != "":
            self.VTYPES_FILE = cfg_obj.options.vtypes_filename
        else:
            self.VTYPES_FILE = cfg_obj.options.flows_filename

        # Define files for collecting network information using turning rates, flow, or trips information
        self.TURN_DEFS_FILE = cfg_obj.options.turndefs_filename
        self.FLOWS_FILE = cfg_obj.options.flows_filename
        self.TRIPS_FILE = cfg_obj.options.trips_filename
        self.DETECTORS_FILE = cfg_obj.options.detectors_filename
        self.DETECTORS_DATA_FILE = cfg_obj.options.detectors_data_filename

        # Define files and number of periods for collecting network information using traffic detectors data
        # Vehicles detected at the end of the lane (according to the direction)
        self.END_DETECTORS_FILE = cfg_obj.options.path_det_data + "det_end_output." \
                                  + cfg_obj.CONFIG_NAME.replace(cfg_obj.CONFIG_NAME.split(".")[0]
                                                                + ".", "") + ".xml"
        # Vehicles detected at the begin of the lane (according to the direction)
        self.BEGIN_DETECTORS_FILE = cfg_obj.options.path_det_data + "det_begin_output." \
                                    + cfg_obj.CONFIG_NAME.replace(cfg_obj.CONFIG_NAME.split(".")[0]
                                                                  + ".", "") + ".xml"
        # Information of each detector
        self.INF_DETECTORS_FILE = cfg_obj.options.path_det_data + "detectors." \
                                  + cfg_obj.CONFIG_NAME.replace(cfg_obj.CONFIG_NAME.split(".")[0]
                                                                + ".", "") + ".xml"

    def setSimulIntances(self,
                         cfg_obj, # configScenarioName class object
                         *args, **kwargs):

        """Define simulation instances"""

        # Get Traffic Light Program
        self.lane_attrs = dict()
        self.tlLink2lanes = dict()
        self.tlProgramGreenPhasesParams = dict()
        self.tlProgramPhase2NextGreenPhases = dict()
        self.tlProgramParams = dict()
        self.tlProgramPhasesDur = dict()

        for tlID in self.traci.trafficlight.getIDList():
            self.tlLink2lanes.update({tlID: []})
            self.tlProgramPhase2NextGreenPhases.update({tlID: dict()})
            self.tlProgramGreenPhasesParams.update({tlID: dict()})
            self.tlProgramParams.update({tlID: dict()})
            self.tlProgramPhasesDur.update({tlID: dict()})

            for link, connections in enumerate(self.traci.trafficlight.getControlledLinks(tlID)):
                self.tlLink2lanes[tlID].append([])
                for connection in connections:
                    laneID = connection[0]
                    self.tlLink2lanes[tlID][link].append(laneID)
                    self.lane_attrs.update({laneID: {"t_without_vehicle": 0, "detectors": []}})
            self.traci.trafficlight.subscribe(tlID, [self.TL_CURRENT_PROGRAM,
                                                     self.TL_CURRENT_PHASE,
                                                     self.TL_PHASE_DURATION,
                                                     self.TL_NEXT_SWITCH])

            tl_programs = self.traci.trafficlight.getAllProgramLogics(tlID)
            for program in tl_programs:
                if program.type == self.traci.constants.TRAFFICLIGHT_TYPE_ACTUATED:
                    self.tlProgramGreenPhasesParams[tlID].update({program.programID: dict()})
                    self.tlProgramPhase2NextGreenPhases[tlID].update({program.programID: []})
                    self.tlProgramPhasesDur[tlID].update({program.programID: []})

                    try:
                        max_gap = float(program.getParameters()['max-gap'])
                    except (AttributeError, KeyError):
                        max_gap = 5
                    try:
                        cycle = float(program.getParameters()['inactive-threshold'])
                    except (AttributeError, KeyError):
                        cycle = 80
                    self.tlProgramParams[tlID].update({program.programID: {'max_gap': max_gap,
                                                                           'cycle': cycle}})
                    phases_info = program.getPhases()
                    num_phases = len(phases_info)
                    for phase in range(0, len(phases_info)):
                        phaseDef = phases_info[phase].state

                        assert len(phaseDef) == len(self.tlLink2lanes[tlID])

                        minDur = phases_info[phase].minDur
                        maxDur = phases_info[phase].maxDur
                        self.tlProgramPhase2NextGreenPhases[tlID][program.programID].append(dict())
                        self.tlProgramPhasesDur[tlID][program.programID].append({"minDur": minDur,
                                                                                 "maxDur": maxDur})
                        nonRedDef = set(phaseDef).difference(["r", "O"])
                        if len(nonRedDef) > 0 and len(nonRedDef.intersection(["u", "y"])) == 0:
                            green_links = [link for link, linkState in enumerate(phaseDef) if linkState == "G"]
                            self.tlProgramGreenPhasesParams[tlID][program.programID].update(
                                {phase: {"lastServedTime": cfg_obj.BEGIN_T,
                                         "greenLinks": green_links}})
                        next_phases = list(phases_info[phase].next)
                        if len(next_phases) == 0:
                            if phase == num_phases - 1:
                                next_phases = [0]
                            else:
                                next_phases = [phase + 1]

                        for next_phase in next_phases:
                            # Next phase is either 1) green or blinking amber, or 2) containing amber or 3) red+amber
                            # Check if next_phase is type 1)
                            phaseDef = phases_info[next_phase].state
                            nonRedDef = set(phaseDef).difference(["r", "O"])
                            if len(nonRedDef) > 0 and len(nonRedDef.intersection(["u", "y"])) == 0:
                                # It is a green phase only (or blinking amber): no transition
                                self.tlProgramPhase2NextGreenPhases[tlID][program.programID] \
                                    [phase].update({next_phase: [next_phase]})
                            else:
                                # next_phase is a transition phase, either 2) containing amber or 3) red+amber
                                # Find the next green phase from next_phase
                                after_next_phases = list(phases_info[next_phase].next)
                                if len(after_next_phases) == 0:
                                    if next_phase == num_phases - 1:
                                        after_next_phases = [0]
                                    else:
                                        after_next_phases = [next_phase + 1]
                                while len(after_next_phases) > 0:
                                    # Check if each after_next_phase is green or blinking amber
                                    further_next_phases = []
                                    for after_next_phase in after_next_phases:
                                        phaseDef = phases_info[after_next_phase].state
                                        nonRedDef = set(phaseDef).difference(["r", "O"])
                                        if len(nonRedDef) > 0 and len(nonRedDef.intersection(["u", "y"])) == 0:
                                            # It is a green phase only (or blinking amber): only next_phase is a transition
                                            try:
                                                self.tlProgramPhase2NextGreenPhases[tlID][program.programID] \
                                                    [phase][next_phase].append(after_next_phase)
                                            except KeyError:
                                                self.tlProgramPhase2NextGreenPhases[tlID][program.programID] \
                                                    [phase].update({next_phase: [after_next_phase]})
                                        else:
                                            # Both next_phase and after_next_phase are transitions,
                                            # check for green phases on further phases
                                            further_next_phases = list(phases_info[after_next_phase].next)
                                            if len(further_next_phases) == 0:
                                                if after_next_phase == num_phases - 1:
                                                    after_next_phases.append(0)
                                                else:
                                                    after_next_phases.append(after_next_phase + 1)
                                    after_next_phases = further_next_phases[:]

        # Initialize traffic detectors
        for detID in self.traci.lanearea.getIDList():
            # Cameras
            self.traci.lanearea.subscribe(detID, [self.LAST_STEP_VEHICLE_NUMBER])
            try:
                controllingLaneID = detID.split("-")[1]
            except IndexError:
                controllingLaneID = "_".join(detID.split("LaneAreaOn")[1:])[1:]
            if len(controllingLaneID) > 0:
                try:
                    self.lane_attrs[controllingLaneID]["detectors"].append(detID)
                except KeyError:
                    print("WARNING: Found detector", detID, "for traffic light", detID.split("-")[0],
                          "at lane", controllingLaneID + ",", "but this lane is not incoming into the traffic light,"
                                                              "so this detector will not automatically influence the "
                                                              "phase changing. "
                                                              "The detector can be placed elsewhere but its ID must "
                                                              "contain the ID of the lane which is controlled by the "
                                                              "traffic light!")
                    placedLaneID = self.traci.lanearea.getLaneID(detID)
                    try:
                        self.lane_attrs[placedLaneID]["detectors"].append(detID)
                    except KeyError:
                        self.lane_attrs.update({placedLaneID: {"t_without_vehicle": 0, "detectors": [detID]}})

        for detID in self.traci.inductionloop.getIDList():
            # Induction loops
            self.traci.inductionloop.subscribe(detID, [self.LAST_STEP_TIME_SINCE_DETECTION])
            try:
                controllingLaneID = detID.split("-")[1]
            except IndexError:
                controllingLaneID = "_".join(detID.split("InductLoopOn")[1:])[1:]
            if len(controllingLaneID) > 0:
                try:
                    self.lane_attrs[controllingLaneID]["detectors"].append(detID)
                except KeyError:
                    print("WARNING: Found detector", detID, "for traffic light", detID.split("-")[0],
                          "at lane", controllingLaneID + ",", "but this lane is not incoming into the traffic light,"
                                                              "so this detector will not automatically influence the "
                                                              "phase changing. "
                                                              "The detector can be placed elsewhere but its ID must "
                                                              "contain the ID of the lane which is controlled by the "
                                                              "traffic light!")
                    placedLaneID = self.traci.inductionloop.getLaneID(detID)
                    try:
                        self.lane_attrs[placedLaneID]["detectors"].append(detID)
                    except KeyError:
                        self.lane_attrs.update({placedLaneID: {"t_without_vehicle": 0, "detectors": [detID]}})

        # get vehicle probabilities from Vehicle Composition File
        tree_vtypes = ElementTree.parse(os.path.abspath(os.path.join(self.VTYPES_FILE)))
        # Get probability of each vehicle type
        self.veh_probabilities = {vType.get("id"): float(vType.get("probability"))
                                  for vType in tree_vtypes.findall("vType")
                                  if vType.get("probability") != None}

        # Get all edges in the simulation
        self.all_InOut_edges = []
        self.all_fringe_edges = []
        all_edges = self.sumolibNet.getEdges()
        num_edges = len(all_edges)
        for edge_ind,edge_obj in enumerate(all_edges):
            if edge_obj.getFunction() == '' and edge_obj.allows("private"):
                edge_id = edge_obj.getID()
                self.all_InOut_edges.append(edge_id)
                # Subscribe for edge travel time
                self.traci.edge.subscribe(edge_id, [self.VAR_CURRENT_TRAVELTIME])

            sys.stdout.write('\r' + "Reading SUMO network " + str(edge_ind) + "/" + str(num_edges))

        for jct_obj in self.sumolibNet.getNodes():
            if jct_obj.getType() == 'dead_end':
                for edge_obj in jct_obj.getIncoming():
                    if edge_obj.getFunction() == '' and edge_obj.allows("private"):
                        self.all_fringe_edges.append(edge_obj.getID())

        # Read training flow file to get the historical departures per edge,
        # and prepare for test flow for online estimations
        FLOWS_FILE = os.path.abspath(self.FLOWS_FILE)
        self.hist_deps = dict()
        self.est_deps = dict()
        self.diff_deps = dict()
        flows_XML = ElementTree.parse(FLOWS_FILE)
        interval_children = {float(interval_child.get("begin")): interval_child.findall("edge")
                             for interval_child in flows_XML.iter('interval')}
        for interval in list(interval_children):
            interval_child = interval_children[interval]
            self.hist_deps[interval] = dict()
            self.est_deps[interval] = dict()
            self.diff_deps[interval] = dict()
            for edge_child in interval_child:
                edge_id = edge_child.get("id")
                edge_entered = edge_child.get("entered")
                if self.sumolibNet.getEdge(edge_id).allows("private"):
                    self.hist_deps[interval][edge_id] = int(edge_entered)
                    self.diff_deps[interval][edge_id] = np.nan
        for interval in self.est_deps:
            for edge_id in self.all_InOut_edges:
                self.est_deps[interval][edge_id] = 0

        # Read training routes file to get list of possible routes and their number of vehicles using each route
        TRIPS_FILE = os.path.abspath(self.TRIPS_FILE)
        self.edge2routes = dict()
        self.routes_data = dict()
        for edge_id in self.all_InOut_edges:
            self.edge2routes[edge_id] = set()
        routes_XML = ElementTree.parse(TRIPS_FILE)
        my_routes = Counter()
        route_edges = []
        route_sumo_ids = dict()
        for route_child in routes_XML.iter('route'):
            edges = route_child.get("edges")
            route_edges.append(edges)
            route_sumo_ids[edges] = route_child.get("id")
        my_routes.update(route_edges)
        self.routes_data["edges"] = {route_id: tuple(route.split(" ")) for route_id, route in
                                     enumerate(my_routes)}
        self.routes_data["number"] = {route_id: my_routes[route] for route_id, route in
                                      enumerate(my_routes)}
        self.routes_data["sumo_id"] = {route_id: route_sumo_ids[route] for route_id, route in
                                       enumerate(my_routes)}
        for route_id in self.routes_data["edges"]:
            for edge_id in self.routes_data["edges"][route_id]:
                self.edge2routes[edge_id].add(route_id)

        # Read the network file to get the edges between an edge with detectors (origin) to other nearest edges with
        # detectors or sink edge (destination) in addition to the proportion of flows from OD.
        all_det_edges = set()
        for detID in self.traci.inductionloop.getIDList():
            # Induction Loops
            if detID.split(".")[-1] == "0":
                # Get only first detectors (closest to the intersection)
                try:
                    controllingLaneID = detID.split("-")[1]
                except IndexError:
                    controllingLaneID = "_".join(detID.split("InductLoopOn")[1:])[1:]
                if len(controllingLaneID) > 0:
                    all_det_edges.add(controllingLaneID.split("_")[0])

        for detID in self.traci.lanearea.getIDList():
            # Cameras
            if detID.split(".")[-1] == "0":
                # Get only first detectors (closest to the intersection)
                try:
                    controllingLaneID = detID.split("-")[1]
                except IndexError:
                    controllingLaneID = "_".join(detID.split("LaneAreaOn")[1:])[1:]
                if len(controllingLaneID) > 0:
                    all_det_edges.add(controllingLaneID.split("_")[0])

        # Get the edges (a route) between one edge with detectors to another edge with detectors
        self.detedge_next_edge = dict()
        self.oriedge_prev_edge = dict()
        done_num = 1
        num_det_edges = len(all_det_edges)
        for edge_id in all_det_edges:
            edge_obj = self.sumolibNet.getEdge(edge_id)
            candidates = [cand_edge_obj for cand_edge_obj in edge_obj.getOutgoing()
                          if cand_edge_obj.allows("private")]
            explored_routes = []
            num_completed = len([candidate for candidate in candidates if candidate is not None])
            num_loops = 1
            while num_completed > 0:
                num_cadidates = len(candidates)
                num_missing = num_cadidates - num_completed
                new_candidates = [None] * num_cadidates
                for cand_ind,cand_edge_obj in enumerate(candidates):
                    sys.stdout.write('\r' + "Getting routes between edges with detector "
                                     + str(done_num) + "/" + str(num_det_edges) + " - "
                                     + str(num_missing) + "/" + str(num_cadidates) + " - Loop " + str(num_loops))
                    if cand_edge_obj is not None:
                        cand_edge_id = cand_edge_obj.getID()
                        try:
                            explored_routes[cand_ind].append(cand_edge_id)
                        except IndexError:
                            explored_routes.append([cand_edge_id])
                        if cand_edge_id in all_det_edges or cand_edge_id in self.all_fringe_edges:
                            # The section below assumes just one route between edge_id and cand_edge_id.
                            # Thus, if self.detedge_next_edge[edge_id][cand_edge_id] already exists, it is necessary
                            # to choose the shortest route
                            try:
                                self.detedge_next_edge[edge_id][cand_edge_id] = dict()
                            except KeyError:
                                self.detedge_next_edge[edge_id] = dict()
                                self.detedge_next_edge[edge_id][cand_edge_id] = dict()
                            self.detedge_next_edge[edge_id][cand_edge_id]["edges"] = [edge_id] + explored_routes[cand_ind]
                            self.detedge_next_edge[edge_id][cand_edge_id]["proportion"] = np.nan
                            try:
                                self.oriedge_prev_edge[cand_edge_id].append(edge_id)
                            except KeyError:
                                self.oriedge_prev_edge[cand_edge_id] = [edge_id]
                            new_candidates[cand_ind] = None
                        else:
                            next_cands = [next_cand_edge_obj for next_cand_edge_obj in cand_edge_obj.getOutgoing()
                                          if next_cand_edge_obj.allows("private")
                                          and next_cand_edge_obj.getID() not in explored_routes[cand_ind]]

                            num_cands = len(next_cands)
                            if num_cands == 0:
                                new_candidates[cand_ind] = None
                            elif num_cands == 1:
                                new_candidates[cand_ind] = next_cands[0]
                            else:
                                for next_cand in next_cands:
                                    new_candidates.append(next_cand)
                                    explored_routes.append(explored_routes[cand_ind][:])
                                del new_candidates[cand_ind]
                                del explored_routes[cand_ind]

                candidates = new_candidates[:]
                num_completed = len([candidate for candidate in candidates if candidate is not None])
                num_loops += 1

            done_num += 1

        # Calculate the proportion of flows between each edge with detector origin to edge with detector destination
        for from_edge_id in self.detedge_next_edge:
            sum_routes_origin = len(self.edge2routes[from_edge_id])
            for to_edge_id in self.detedge_next_edge[from_edge_id]:
                sum_routes_od = len(self.edge2routes[from_edge_id].intersection(self.edge2routes[to_edge_id]))
                try:
                    self.detedge_next_edge[from_edge_id][to_edge_id]["proportion"] = sum_routes_od / sum_routes_origin
                except ZeroDivisionError:
                    self.detedge_next_edge[from_edge_id][to_edge_id]["proportion"] = 0

    def SUMOsimulationStep(self,
                           cfg_obj):  # configScenarioName class object
        """Make SUMO simulation step"""

        if cfg_obj.SIMUL_INTERFACE in ("traci", "libsumo"):
            # Do simulation step using Traci
            self.traci.simulationStep()
            cfg_obj.ACTUAL_T = round(float(self.traci.simulation.getTime()), 1)

        if self.NOGUI or cfg_obj.SIMUL_INTERFACE == "libsumo":
            sys.stdout.write('\r' + "Step " + str(cfg_obj.ACTUAL_T))

    def controlActuatedTLs(self,
                           ACTUAL_T):
        """Algorithm for controlling actuated traffic lights using traffic detectors (cameras and/or induction loops)"""

        dets_detections = commonFunctions.merge_two_dicts(self.traci.lanearea.getAllSubscriptionResults(),
                                                          self.traci.inductionloop.getAllSubscriptionResults())

        for laneID in self.lane_attrs:
            for detID in self.lane_attrs[laneID]["detectors"]:
                try:
                    if dets_detections[detID][self.LAST_STEP_TIME_SINCE_DETECTION] == 0:
                        self.lane_attrs[laneID]["t_without_vehicle"] = 0
                        break
                except KeyError:
                    if dets_detections[detID][self.LAST_STEP_VEHICLE_NUMBER] > 0:
                        self.lane_attrs[laneID]["t_without_vehicle"] = 0
                        break
            else:
                self.lane_attrs[laneID]["t_without_vehicle"] += 1

        tl_data = self.traci.trafficlight.getAllSubscriptionResults()
        for tlID in tl_data:
            current_program = tl_data[tlID][self.TL_CURRENT_PROGRAM]
            if self.tlProgramGreenPhasesParams[tlID].__contains__(current_program):
                max_gap = self.tlProgramParams[tlID][current_program]['max_gap']
                cycle = self.tlProgramParams[tlID][current_program]['cycle']
                current_phase = tl_data[tlID][self.TL_CURRENT_PHASE]
                phase_duration = tl_data[tlID][self.TL_PHASE_DURATION]
                tl_next_switch = tl_data[tlID][self.TL_NEXT_SWITCH]
                remaining_t = tl_next_switch - ACTUAL_T
                elapsed_t = ACTUAL_T - tl_next_switch + phase_duration
                maintain_phase = 0
                if remaining_t > 0:
                    if self.tlProgramGreenPhasesParams[tlID][current_program].__contains__(current_phase) \
                            and elapsed_t > self.tlProgramPhasesDur[tlID][current_program][current_phase]["minDur"]:
                        green_links = \
                            self.tlProgramGreenPhasesParams[tlID][current_program][current_phase]["greenLinks"]
                        if len(green_links) == 0:
                            # Blinking Amber
                            maintain_phase = 1
                        else:
                            for link in green_links:
                                for laneID in self.tlLink2lanes[tlID][link]:
                                    if laneID != [] and self.lane_attrs[laneID]["t_without_vehicle"] <= max_gap:
                                        maintain_phase = 1
                                        break
                                if maintain_phase == 1:
                                    break
                    else:
                        maintain_phase = 1
                if maintain_phase == 0:
                    highestPriority_NextPhase = dict()
                    num_links = len(self.tlLink2lanes[tlID])
                    for next_phase in self.tlProgramPhase2NextGreenPhases[tlID][current_program][current_phase]:
                        priority = 0
                        for greenPhase in self.tlProgramPhase2NextGreenPhases[tlID][current_program][current_phase][
                            next_phase]:
                            starvation_t = ACTUAL_T - self.tlProgramGreenPhasesParams[tlID][current_program] \
                                [greenPhase]["lastServedTime"]
                            if starvation_t >= cycle:
                                priority = max(cycle + num_links + starvation_t, priority)
                            else:
                                num_links_active_dets = 0
                                green_links = self.tlProgramGreenPhasesParams[tlID][current_program][greenPhase][
                                    "greenLinks"]
                                for link in green_links:
                                    for laneID in self.tlLink2lanes[tlID][link]:
                                        if laneID != [] and self.lane_attrs[laneID]["t_without_vehicle"] == 0:
                                            num_links_active_dets += 1
                                            priority = max(cycle + num_links_active_dets, priority)
                                        else:
                                            priority = max(starvation_t, priority)
                        highestPriority_NextPhase.update({next_phase: priority})

                    chosen_phase = sorted(highestPriority_NextPhase.items(), key=lambda sort_it: sort_it[1])[-1][0]
                    try:
                        self.tlProgramGreenPhasesParams[tlID][current_program][chosen_phase]["lastServedTime"] = \
                            ACTUAL_T
                    except KeyError:
                        notGreenPhase = 1
                    self.traci.trafficlight.setPhase(tlID, chosen_phase)
                    self.traci.trafficlight.setPhaseDuration(tlID,
                                                             self.tlProgramPhasesDur[tlID][current_program]
                                                                                    [chosen_phase]["maxDur"])

    def getNet(self):
        """Get the XML children of each network element from a SUMO network"""

        # Read Network File
        tree_network = ElementTree.parse(os.path.abspath(os.path.join(self.NET_FILE)))

        # Read Additional File (For Movement Groups)
        tree_mov_groups = ElementTree.parse(os.path.abspath(os.path.join(self.MOV_GROUPS_FILE)))

        # Read Vehicle Composition File
        tree_vtypes = ElementTree.parse(os.path.abspath(os.path.join(self.VTYPES_FILE)))

        # Define the SUMO XML junction, edge, connection, mov. groups and veh. distributions children used for the area
        net_jct_children = {jct.get("id"): jct for jct in list(tree_network.iter('junction'))}
        net_conn_children = [conn for conn in list(tree_network.iter('connection'))
                             if conn.get('via') != None]  # still takes any in_lane to int_lane and int_lane to int_lane
        tls_signal_children = {signal.get('id'): signal for signal in list(tree_mov_groups.iter('tlLogic'))}
        vTypeDistribution_children = list(tree_vtypes.iter('vTypeDistribution'))
        # get distribution with id = all. Only in case vTypes are in the vTypeDistribution
        # vTypeDistribution_child = [vTypeDistribution_child for vTypeDistribution_child in vTypeDistribution_children
        #                            if vTypeDistribution_child.get("id") == "all"][0].get("vTypes").split(" ")
        # When the vTypes are not inside the vTypeDistribution
        vehComposition_children = {vType.get("id"): vType for vType in tree_vtypes.findall("vType")
                                   if vType.get("probability") != None}
        net_edg_children = []
        net_internal_edg_children = []
        for edge in list(tree_network.iter('edge')):
            if edge.get('function') == "internal":
                net_internal_edg_children.append(edge)
            else:
                net_edg_children.append(edge)

        return net_jct_children, net_conn_children, tls_signal_children, net_edg_children, net_internal_edg_children, \
               vehComposition_children

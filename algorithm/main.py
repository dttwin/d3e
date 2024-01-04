## Digital Twin Modelling
# @author  Andre Maia Pereira
# @author  Jan Přikryl
# @date    2024-

## IMPORT SYSTEM MODULES
import os.path
import sys
import optparse

from xml.etree import ElementTree
import numpy as np

## IMPORT LOCAL MODULES
from application import config
from application.base import simulation


def get_current_detector_measurements(current_measurements, actual_t):
    """
    Retrieve the current detector measurements for a given time.

    :param current_measurements: A dictionary containing detector measurements indexed by time.
    :type current_measurements: dict
    :param actual_t: The current time as a string.
    :type actual_t: str
    :return: A list of measurements for the given time. If no measurements are found, an empty list is returned.
    :rtype: list
    """
    try:
        measurements = current_measurements[str(actual_t)]
    except KeyError:
        measurements = []

    return measurements


def read_test_detector_measurements(sumo_net, sumo_det_file, detectors_measurements_file):
    """
    Read the test detector measurements from the file.

    :param sumo_net: The SUMO network object.
    :type sumo_net: sumolib.net.Net
    :param sumo_det_file: The path to the SUMO detector file.
    :type sumo_det_file: str
    :param detectors_measurements_file: The path to the file containing detector measurements.
    :type detectors_measurements_file: str
    :return: A dictionary containing the test detector measurements.
    :rtype: dict
    """

    detf = ElementTree.parse(sumo_det_file)
    e1_edges = {det_child.get("id"): det_child.get("lane").split("_")[0]
                for det_child in detf.iter('inductionLoop')}
    e2_edges = {det_child.get("id"): det_child.get("lane").split("_")[0]
                for det_child in detf.iter('laneAreaDetector')}

    all_det_edges = dict()
    all_det_edges.update(e1_edges)
    all_det_edges.update(e2_edges)
    dm_abspath = os.path.abspath(detectors_measurements_file)
    with open(dm_abspath, mode='r') as flowf:
        det_test_measurements = dict()
        all_lines = flowf.readlines()
        count = 0
        for l_ind,line in enumerate(all_lines):
            count += 1
            if l_ind > 0:
                det_info = line.rstrip().split(";")
                det_id = det_info[0]
                det_begin = det_info[1]
                det_end = det_info[2]
                det_entered = int(det_info[3])
                det_speed = int(det_info[4])
                det_edge = all_det_edges[det_id]
                edge_obj = sumo_net.getEdge(det_edge)
                if edge_obj.getFunction() == '' and edge_obj.allows("private"):
                    interval = det_begin
                    if interval not in det_test_measurements.keys():
                        det_test_measurements[interval] = dict()
                    if det_edge in det_test_measurements[interval].keys():
                        current_ent_val = int(det_test_measurements[interval][det_edge]["entered"])
                        new_ent_val = det_entered
                        sum_ent = current_ent_val + new_ent_val
                        current_speed_val = int(det_test_measurements[interval][det_edge]["speed"])
                        new_speed_val = det_speed
                        try:
                            weigh_ave = int(sum([current_ent_val * current_speed_val,
                                                 new_ent_val * new_speed_val]) / sum_ent)
                        except ZeroDivisionError:
                            weigh_ave = 0
                        det_test_measurements[interval][det_edge]["entered"] = sum_ent
                        det_test_measurements[interval][det_edge]["speed"] = weigh_ave
                    else:
                        det_test_measurements[interval][det_edge] = {"entered": det_entered,
                                                                     "speed": det_speed}

    return det_test_measurements


if __name__ == "__main__":
    # Define the algorithm configuration
    cfg_setup = config.Base(optparse.OptionParser())

    # If simulating:
    if cfg_setup.SIMUL_INTERFACE in ("traci", "libsumo"):
        # Simulation Statistics Module
        simul_setup = simulation.SUMO(cfg_setup, "simul_setup")

        # Define simulation constants
        cfg_setup.ACTUAL_T = round(float(simul_setup.traci.simulation.getTime()), 1)
        simul_setup.setSimulIntances(cfg_setup)
    else:
        sys.exit("Interface to get current time in seconds, and also to communicate with the simulator "
                 "if doing simulation, are not defined")

    print("Begin\n")
    cfg_setup.logger.info('Begin')

    # Initialize
    deps_intervals = sorted(list(simul_setup.est_deps.keys()))
    error_measurements = dict()
    model_measurements = dict()
    for interv in simul_setup.est_deps:
        for edge_id in simul_setup.est_deps[interv]:
            try:
                model_measurements[interv][edge_id] = 0
                error_measurements[interv][edge_id] = np.nan
            except KeyError:
                model_measurements[interv] = {edge_id: 0}
                error_measurements[interv] = {edge_id: np.nan}

    global interval, interval_ind, adjusted_interval_ind, adjusted_interval, max_interval_ind
    interval = None
    interval_ind = 0
    max_interval_ind = 0
    NUM_INTERV_AVE = 3
    MIN_COMMON_ROUTES = 5

    det_test_measurements = read_test_detector_measurements(simul_setup.sumolibNet,
                                                            simul_setup.DETECTORS_FILE,
                                                            simul_setup.DETECTORS_DATA_FILE)

    # Constant loop communicating
    while cfg_setup.ACTUAL_T < cfg_setup.END_T + cfg_setup.STEP_LENGTH:
        # Get current time in seconds
        if cfg_setup.SIMUL_INTERFACE in ("libsumo", "traci"):
            # If simulating, also do a simulation step
            simul_setup.SUMOsimulationStep(cfg_setup)
        else:
            sys.exit("Interface to get current time in seconds, and to do simulation step and reset simulation "
                     "instances if doing simulation, are not defined")

        # Get inputs from and send outputs to the simulation
        if cfg_setup.SIMUL_INTERFACE in ("libsumo", "traci"):
            # Control Actuated Traffic Lights (it runs only if there is an actuated program at moment)
            simul_setup.controlActuatedTLs(cfg_setup.ACTUAL_T)
        else:
            sys.exit("Interface to get and send orders is not defined")

        # Get current interval readings of detectors (TEST DATASET) as TRAINING DATASED is used for historical data
        det_measurements = get_current_detector_measurements(det_test_measurements, cfg_setup.ACTUAL_T)

        if len(det_measurements) > 0:
            # Define the interval and initialize estimated departures
            for ind, interval in enumerate(deps_intervals[interval_ind:]):
                interval_ind += ind
                try:
                    next_interval = deps_intervals[interval_ind + 1]
                except IndexError:
                    next_interval = float("inf")
                if cfg_setup.ACTUAL_T < next_interval:
                    break
            else:
                interval_ind = 0
                interval = deps_intervals[interval_ind]
            for interv in deps_intervals[interval_ind:max_interval_ind + 1]:
                for edge_id in simul_setup.est_deps[interv]:
                    simul_setup.est_deps[interv][edge_id] = 0

            # Calculate the error between the model measurements and detector measurements
            for det_edge_id in det_measurements:
                try:
                    error_measurements[interval][det_edge_id] = \
                         det_measurements[det_edge_id]["entered"] - model_measurements[interval][det_edge_id]
                except TypeError:
                    error_measurements[interval][det_edge_id] = np.nan

            # Retrieve travel times of the model
            travel_times = simul_setup.traci.edge.getAllSubscriptionResults()

            # Estimate the flow on each edge with detectors based on the flow and speeds of incoming links
            for origin_edge_id in det_measurements:
                # Calculate moving average of the error at origin_edge_id
                if interval_ind < NUM_INTERV_AVE:
                    errors = [error_measurements[prev_interval][origin_edge_id]
                              for prev_interval in deps_intervals[0:interval_ind + 1]]
                    if np.all(np.isnan(errors)):
                        ave_error_origin_edge = 0
                    else:
                        ave_error_origin_edge = np.nanmean(errors)
                else:
                    errors = [error_measurements[prev_interval][origin_edge_id]
                              for prev_interval in deps_intervals[interval_ind + 1 - NUM_INTERV_AVE:interval_ind + 1]]
                    if np.all(np.isnan(errors)):
                        ave_error_origin_edge = 0
                    else:
                        ave_error_origin_edge = np.nanmean(errors)
                for destination_edge_id in simul_setup.detedge_next_edge[origin_edge_id]:
                    adjusted_interval = interval
                    adjusted_interval_ind = interval_ind
                    # According to the historical proportion of vehicle from origin_edge_id to destination_edge_id,
                    # define how much should be the number of vehicles on each edge on the way
                    ratio_entered = (simul_setup.detedge_next_edge[origin_edge_id][destination_edge_id]["proportion"]
                                     * det_measurements[origin_edge_id]["entered"])
                    # Adjust the ratio according to the moving average error at origin_edge_id and destination_edge_id
                    if interval_ind < NUM_INTERV_AVE:
                        errors = [error_measurements[prev_interval][destination_edge_id]
                                  for prev_interval in deps_intervals[0:interval_ind + 1]]
                        if np.all(np.isnan(errors)):
                            ave_error_destination_edge = 0
                        else:
                            ave_error_destination_edge = np.nanmean(errors)
                    else:
                        errors = [error_measurements[prev_interval][destination_edge_id]
                                  for prev_interval in deps_intervals[interval_ind + 1 - NUM_INTERV_AVE:interval_ind + 1]]
                        if np.all(np.isnan(errors)):
                            ave_error_destination_edge = 0
                        else:
                            ave_error_destination_edge = np.nanmean(errors)
                    factor_error_origin_edge = ave_error_origin_edge \
                                               * (1 / len(simul_setup.detedge_next_edge[origin_edge_id]))
                    factor_error_destination_edge = ave_error_destination_edge \
                                                    * (1 / len(simul_setup.oriedge_prev_edge[destination_edge_id]))
                    ratio_entered += (factor_error_origin_edge + factor_error_destination_edge) / 2
                    # Adjust the interval according to the arrival time on the edges in the route from origin to destination
                    for edge_id in simul_setup.detedge_next_edge[origin_edge_id][destination_edge_id]["edges"][1:]:
                        travel_time = travel_times[edge_id][simul_setup.VAR_CURRENT_TRAVELTIME]
                        adjusted_interval += travel_time
                        try:
                            simul_setup.est_deps[adjusted_interval][edge_id] += np.round(ratio_entered)
                        except KeyError:
                            for ind, interv in enumerate(deps_intervals[adjusted_interval_ind:]):
                                adjusted_interval_ind += ind
                                try:
                                    next_interv = deps_intervals[adjusted_interval_ind + 1]
                                except IndexError:
                                    next_interv = float("inf")
                                if adjusted_interval < next_interv:
                                    adjusted_interval = interv
                                    break
                            else:
                                adjusted_interval = deps_intervals[0]
                            simul_setup.est_deps[adjusted_interval][edge_id] += np.round(ratio_entered)
                        except TypeError:
                            simul_setup.est_deps[adjusted_interval][edge_id] = np.round(ratio_entered)
                    max_interval_ind = max(max_interval_ind, adjusted_interval_ind)


            # Calculate the difference between historical and estimate departures. If no estimated value, use historical
            for interv in deps_intervals[interval_ind:max_interval_ind + 1]:
                for edge_id in simul_setup.hist_deps[interv]:
                    try:
                        simul_setup.diff_deps[interv][edge_id] = \
                                simul_setup.est_deps[interv][edge_id] - simul_setup.hist_deps[interv][edge_id]
                    except TypeError:
                        simul_setup.est_deps[interv][edge_id] = simul_setup.hist_deps[interv][edge_id]
                        simul_setup.diff_deps[interv][edge_id] = 0

            # Calculate the positive difference between estimated departures and modelled departures
            model_diff = dict()
            for edge_id in simul_setup.est_deps[interval]:
                model_diff[edge_id] = 0
            for interv in deps_intervals[interval_ind:max_interval_ind + 1]:
                for edge_id in simul_setup.est_deps[interv]:
                    model_diff[edge_id] += simul_setup.est_deps[interv][edge_id] - model_measurements[interv][edge_id]

            # Generate vehicles by selecting routes that pass over the edges missing certain number of vehicles using
            # the popularity of routes
            gen_vehs = True
            num_added = 1
            while gen_vehs is True:
                # While there are vehicles to be generated
                same_diff_group_edges = dict()
                for edge_id in model_diff:
                    # Classify edges according to the number of missing vehicles
                    num_veh = int(model_diff[edge_id])
                    if num_veh > 0:
                        try:
                            same_diff_group_edges[num_veh].append(edge_id)
                        except KeyError:
                            same_diff_group_edges[num_veh] = [edge_id]
                if len(same_diff_group_edges) == 0:
                    gen_vehs = False
                for num_veh in sorted(same_diff_group_edges.keys()):
                    # For each number of missing vehicles, try to get routes that will pass to the needed edges for num_veh
                    edge_ids = same_diff_group_edges[num_veh]
                    all_routes = set()
                    for edge_id in edge_ids:
                        all_routes.update(simul_setup.edge2routes[edge_id])
                    num_all_routes = len(all_routes)
                    edge_route_coverage = []
                    # To avoid no routes passing to all needed edges, sort the edges that have most common routes and get
                    # eligible routes that filter out passing all edges as much as possible
                    for edge_id in edge_ids:
                        edge_route_coverage.append((edge_id, len(simul_setup.edge2routes[edge_id]) / num_all_routes))
                    edge_route_coverage = sorted(edge_route_coverage, key=lambda lst: (lst[1]))[::-1]
                    most_covered_edge_id = edge_route_coverage[0][0]
                    eligible_routes = simul_setup.edge2routes[most_covered_edge_id]
                    for edge_id,coverage in edge_route_coverage[1:]:
                        intersect_routes = eligible_routes.intersection(simul_setup.edge2routes[edge_id])
                        if len(intersect_routes) >= MIN_COMMON_ROUTES:
                            eligible_routes.intersection_update(simul_setup.edge2routes[edge_id])
                        else:
                            break
                    # Select the routes according to their "popularity"
                    eligible_routes = list(eligible_routes)
                    routes_popularity = [simul_setup.routes_data["number"][route_id] for route_id in eligible_routes]
                    routes_popularity = [route_num/sum(routes_popularity) for route_num in routes_popularity]
                    interval_len = deps_intervals[interval_ind + 1] - deps_intervals[interval_ind]
                    num_veh_p_seconds = num_veh / interval_len
                    headway_times = np.random.default_rng().exponential(scale=1/num_veh_p_seconds, size=num_veh)
                    for veh_ind in range(num_veh):
                        chosen_route_id = np.random.choice(eligible_routes, p=routes_popularity)
                        origin_edge_id = simul_setup.routes_data["edges"][chosen_route_id][0]
                        adjusted_interval = interval
                        adjusted_interval_ind = interval_ind
                        for edge_id in simul_setup.routes_data["edges"][chosen_route_id]:
                            model_diff[edge_id] -= 1
                            travel_time = travel_times[edge_id][simul_setup.VAR_CURRENT_TRAVELTIME]
                            adjusted_interval += travel_time
                            try:
                                model_measurements[adjusted_interval][edge_id] += 1
                            except KeyError:
                                for ind, interv in enumerate(deps_intervals[adjusted_interval_ind:]):
                                    adjusted_interval_ind += ind
                                    try:
                                        next_interv = deps_intervals[adjusted_interval_ind + 1]
                                    except IndexError:
                                        next_interv = float("inf")
                                    if adjusted_interval < next_interv:
                                        adjusted_interval = interv
                                        break
                                model_measurements[adjusted_interval][edge_id] += 1
                        max_interval_ind = max(max_interval_ind, adjusted_interval_ind)
                        veh_id = str(interval) + "_" + str(num_added)
                        veh_type = np.random.choice(list(simul_setup.veh_probabilities.keys()),
                                                    p=list(simul_setup.veh_probabilities.values()))
                        arr_t = interval + min(headway_times[veh_ind], interval_len)

                        # Generate the vehicle in the model
                        sumo_route_id = simul_setup.routes_data["sumo_id"][chosen_route_id]
                        simul_setup.traci.vehicle.add(veh_id, sumo_route_id, typeID=veh_type,
                                                      depart=str(arr_t), departLane='random', departPos='base', departSpeed='max',
                                                      arrivalLane='current', arrivalPos='max', arrivalSpeed='current')

                        num_added += 1

            prev_interval = interval

    if cfg_setup.SIMUL_INTERFACE in ("libsumo", "traci"):
        # Close Traci/Libsumo connection
        simul_setup.traci.close()

    cfg_setup.logger.info('End')
    sys.stdout.write('\n')
    sys.stdout.flush()
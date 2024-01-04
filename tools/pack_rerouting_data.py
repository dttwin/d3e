import csv
import numpy as np
# import matplotlib
# matplotlib.use('agg')
import matplotlib.pyplot as plt
from collections import defaultdict


DO_HISTOGRAM = True
DO_LLR_TEST = False


class RouteData(object):

    def __init__(self, my_route,
                 my_old_tt=None, my_new_tt=None, my_old_len=None, my_new_len=None,
                 my_route_diff=None):
        self.old_tt = my_old_tt
        self.new_tt = my_new_tt
        self.old_len = my_old_len
        self.new_len = my_new_len
        self.route = my_route
        self.route_diff = my_route_diff


class VehicleData(object):

    def __init__(self, my_veh_id, my_time_stamp, my_initial_route):
        self.veh_id = my_veh_id
        self.routes = {my_time_stamp: RouteData(my_initial_route)}
        self.travel_time = None
        self.entrance_time = my_time_stamp

    def append_route(self, my_time_stamp, my_route,
                     my_old_tt=None, my_new_tt=None, my_old_len=None, my_new_len=None,
                     my_route_diff=None):
        self.routes[my_time_stamp] = RouteData(my_route,
                                               my_old_tt, my_new_tt, my_old_len, my_new_len,
                                               my_route_diff)

    def set_travel_time(self, my_travel_time):
        self.travel_time = my_travel_time

    def get_rerouting_info(self, my_parallel_edges):
        route_count = len(self.routes)
        route_duplicates = 0
        visited_routes = []
        for tid in sorted(self.routes):
            route = self.routes[tid]
            route_is_new = True
            if visited_routes:
                for vsr in visited_routes:
                    if route.route == vsr.route:
                        route_is_new = False
                        break
                    rr = set(route.route)
                    tr = set(vsr.route)
                    trd = rr.symmetric_difference(tr)
                    # if len(trd) < 8:
                    #    print('trd=', trd)
                    if all([edge in my_parallel_edges for edge in trd]):
                        # Vehicle rerouted to parallel edges, the route is identical
                        # print('vehicle rerouted to parallel edge:\n  {:s}\n  {:s}'.format(
                        #     str(route.route), str(vsr.route)))
                        route_is_new = False
                        break
            if route_is_new:
                visited_routes.append(route)
            else:
                route_duplicates += 1

        return route_count, route_duplicates, visited_routes


class VehicleStats(object):

    def __init__(self, my_file_name):
        self.veh_data = {}
        with open(my_file_name, 'r', newline='') as rf:
            rreader = csv.reader(rf)
            for row in rreader:
                time_stamp = float(row[0])
                my_veh_id = row[1]
                event = row[2]
                if event == 'entered_simulation':
                    initial_route = row[3:]
                    self.veh_data[my_veh_id] = VehicleData(my_veh_id, time_stamp, initial_route)
                elif event == 'rerouted':
                    data_idx = row.index('old_tt')
                    new_route = row[3:data_idx]
                    old_tt = float(row[data_idx+1])
                    new_tt = float(row[data_idx+3])
                    old_len = float(row[data_idx+5])
                    new_len = float(row[data_idx + 7])
                    route_diff = row[data_idx+9:]
                    self.veh_data[my_veh_id].append_route(time_stamp, new_route,
                                                          old_tt, new_tt, old_len, new_len,
                                                          route_diff)
                elif event == 'left_simulation':
                    travel_time = float(row[3])
                    self.veh_data[my_veh_id].set_travel_time(travel_time)

    def travel_time_gain(self, baseline_data):
        """

        :type baseline_data: VehicleStats
        """
        res = defaultdict(list)
        for my_veh_id, my_veh_record in self.veh_data.items():
            try:
                baseline_record = baseline_data.veh_data[my_veh_id]
            except KeyError:
                print(f'vehicle {my_veh_id} did not enter baseline simulation')
                continue
            baseline_tt = baseline_record.travel_time
            if baseline_tt is None:
                print(f'vehicle {my_veh_id} did not finish baseline simulation')
                continue
            this_tt = my_veh_record.travel_time
            if this_tt is None:
                print(f'vehicle {my_veh_id} did not finish LLR simulation')
                continue
            gain_tt = this_tt - baseline_tt
            res[gain_tt].append(my_veh_id)
        return res

    def get_rerouting_info(self, my_veh_id, my_parallel_edges):
        return self.veh_data[my_veh_id].get_rerouting_info(my_parallel_edges)

    def get_time_slot(self, my_veh_id):
        vd = self.veh_data[my_veh_id]
        return vd.entrance_time, vd.entrance_time + vd.travel_time


if __name__ == "__main__":
    PARALLEL_EDGES = [
        'gneE45', 'gneE46', 'gneE47', 'gneE59', 'gneE60',
        'gneE143', 'gneE144', 'gneE145', 'gneE147', 'gneE148', 'gneE149',
        'gneE240.50', 'gneE241.49',
        'gneE246', 'gneE245', 'gneE248', 'gneE249']
    ROUTES_SHOW = {
        'dz_gneE26': ['fl_448565742.11', 'fl_22743640#0.11'],
        'dz_gneE2': ['or_gneE52', 'or_-482263304#0', '-gneE260'],
        'dz_79094527#1_0': ['or_gneE52', 'or_-482263304#0', '-gneE260'],
        'gneE57_0': ['fl_448565742.11', 'fl_22743640#0.11'] + [],
        'fl_156262933#4_1': ['fl_gneE60', 'dz_gneE27'],
        'gneE247_0': ['fl_gneE60', 'dz_gneE27']
    }
    # Not imported but executed
    # ref_data = VehicleStats('notts_routes_55800.csv')
    # llr_data = VehicleStats('rerouted_55800.csv')
    ref_data = VehicleStats(r'notts\notts_routes_55800.csv')
    llr_data = VehicleStats(r'llr\llr_rerouted_20190529T113150_55800.csv')
    travel_gains = llr_data.travel_time_gain(ref_data)
    #
    if DO_HISTOGRAM:
        print(sorted(travel_gains))
        # Make a histogram of gains
        travel_gain_tuples = [(x, len(travel_gains[x])) for x in sorted(travel_gains)]
        gain_data = np.asarray(travel_gain_tuples)
        plt.plot(gain_data[:, 0], gain_data[:, 1])
        plt.xlim([-240, -20])
        plt.show()
    # Select vehicle ids that are 20 to 120 seconds faster
    llr_faster = []
    llr_gains = {}
    time_entered = defaultdict(list)
    time_left = {}
    for gain, veh_ids in travel_gains.items():
        if -240.0 < gain < -20.0:
            llr_faster += veh_ids
            for veh_id in veh_ids:
                llr_gains[veh_id] = gain
                ref_min, ref_max = ref_data.get_time_slot(veh_id)
                llr_min, llr_max = llr_data.get_time_slot(veh_id)
                if veh_id == '3.0':
                    print(f'vehicle {veh_id}: '
                          f'ref_min={ref_min:.1f}, ref_max={ref_max:.1f}, '
                          f'llr_min={llr_min:.1f}, llr_max={llr_max:.1f}')
                t_min = min(ref_min, llr_min)
                t_max = max(ref_max, llr_max)
                time_entered[t_min].append(veh_id)
                time_left[veh_id] = t_max
    candidate_end_time = 0.0
    candidate_veh_id = None
    candidate_list = []
    for entrance_time in sorted(time_entered):
        if candidate_end_time > entrance_time:
            continue
        veh_ids = time_entered[entrance_time]
        candidate_length = 0
        candidate_time = -1e10
        for veh_id in veh_ids:
            rc, rd, vr = llr_data.get_rerouting_info(veh_id, PARALLEL_EDGES)
            # The first element in `vr` is the original route
            num_reroutes = len(vr) - 1
            if num_reroutes > 0:
                # Vehicle has been rerouted at least once to a different route
                if rd == 0:
                    # And the vehicle has not been rerouted back to its origina route
                    gain = llr_gains[veh_id]
                    print(f'vehicle {veh_id} with time gain {gain} rerouted {num_reroutes} times')
                    if num_reroutes >= 1:
                        route_data = vr[1]
                        route_begin = route_data.route[0]
                        route_end = route_data.route[-1]
                        print(f'  vehicle goes from {route_begin} to {route_end}')
                        route_len_diff = route_data.new_len - route_data.old_len
                        route_tt_diff = route_data.new_tt - route_data.old_tt
                        if route_len_diff > 0:
                            print(f'  new route is {route_len_diff:.1f} metres longer')
                        else:
                            print(f'  new route is {-route_len_diff:.1f} metres shorter')
                        if route_tt_diff > 0:
                            print(f'  new route takes {route_tt_diff:.1f} sec less')
                        else:
                            print(f'  new route takes {-route_tt_diff:.1f} sec more')
                        if route_begin in ROUTES_SHOW.keys() and \
                                route_end in ROUTES_SHOW[route_begin] and \
                                route_len_diff > candidate_length:
                            candidate_veh_id = veh_id
                            candidate_list.append(veh_id)
                        candidate_length = max(candidate_length, route_len_diff)
                        candidate_time = max(candidate_time, route_tt_diff)
                        if num_reroutes >= 2:
                            print('  -- second reroute')
                            route_data2 = vr[2]
                            route_len_diff = route_data2.new_len - route_data.old_len
                            route_tt_diff = route_data2.new_tt - route_data.old_tt
                            if route_len_diff < 0:
                                print(f'  new route is {route_len_diff:.1f} metres longer than the first one')
                            else:
                                print(f'  new route is {-route_len_diff:.1f} metres shorter than the first one')
                            if route_tt_diff < 0:
                                print(f'  new route takes {route_tt_diff:.1f} sec less than the first one')
                            else:
                                print(f'  new route takes {-route_tt_diff:.1f} sec more than the first one')
        if candidate_veh_id is not None:
            candidate_end_time = time_left[candidate_veh_id]
            print(f'** will track vehicle {candidate_veh_id} from time {entrance_time} to {candidate_end_time}')
            candidate_veh_id = None

    print('----- candidate list -----\n', candidate_list)

    # Construct route data
    routes = {}
    for veh_id in candidate_list:
        is_first = True
        for time_stamp, routing_data in llr_data.veh_data[veh_id].routes.items():
            row = {}
            if is_first:
                row['action'] = 'entered_simulation'
                row['distance gain'] = -1
                row['time gain'] = -1
                is_first = False
            else:
                row['action'] = 'rerouted'
                row['distance gain'] = routing_data.new_len - routing_data.old_len
                row['time gain'] = routing_data.new_tt - routing_data.old_tt
            if time_stamp in routes:
                raise KeyError(f'duplicate key {time_stamp}')
            row['timestamp'] = time_stamp
            row['id'] = veh_id
            row['route'] = routing_data.route
            routes[time_stamp] = row
    # Now erite the CSV
    fieldnames = [
        'timestamp', 'id', 'action', 'distance gain', 'time gain', 'route'
    ]
    with open('fcd_routes.csv', 'w', newline='') as cf:
        dw = csv.writer(cf)
        dw.writerow(fieldnames)
        for time_stamp in sorted(routes):
            r = routes[time_stamp]
            dl = [
                r['timestamp'], r['id'], r['action'], r['distance gain'], r['time gain']
                ] + r['route']
            dw.writerow(dl)


    if DO_LLR_TEST:
        for veh_id in llr_faster:
            rc, rd, vr = llr_data.get_rerouting_info(veh_id, PARALLEL_EDGES)
            # The first element in `vr` is the original route
            num_reroutes = len(vr) - 1
            if num_reroutes > 0:
                # Vehicle has been rerouted at least once to a different route
                if rd > 0:
                    print(f'vehicle {veh_id} rerouted back to some of the original routes')
                else:
                    gain = llr_gains[veh_id]
                    print(f'vehicle {veh_id} with time gain {gain} rerouted {num_reroutes} times')
                    if num_reroutes == 1:
                        route_data = vr[1]
                        if route_data.new_len > route_data.old_len:
                            print(f'  new route is {route_data.new_len-route_data.old_len:.1f} metres longer')
                        else:
                            print(f'  new route is {route_data.old_len-route_data.new_len:.1f} metres shorter')
                        if route_data.new_tt > route_data.old_tt:
                            print(f'  new route takes {route_data.new_tt-route_data.old_tt:.1f} sec less')
                        else:
                            print(f'  new route takes {route_data.old_tt-route_data.new_tt:.1f} sec more')


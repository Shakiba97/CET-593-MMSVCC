## The topology settings for a single intersection
import collections
import math


def set_common_paras(paras):
    ## Model parameters
    # Prediction horizon of the MPC problem, i.e., T^{s}.
    paras["num_predict_steps"] = 6
    # Number of slower steps used to construct the faster scale problem, i.e., h in Equation (24).
    paras["faster_steps_from_slower"] = 3
    # Length of each slower step, i.e., \Delta T_{s}.
    paras["delta_T"] = 5
    # Length of each faster step, i.e., \Delta T_{f}.
    paras["delta_T_faster"] = 0.5
    # Time headway of the car-following model, i.e., \tau_{h}.
    paras["tau_cf"] = 1
    # Safety distance of the car-following model, i.e., d_{0}.
    paras["d_0_cf"] = 7
    # Discount ratio used to stabalize the MPC problem.
    paras["discount_ratio"] = 0.95

    ## IDM model parameters, see Equation (11) in the second paper.
    # Maximum acceleration that the vechiles can reach, in m/s^{2}.
    paras["max_acc"] = 3.5
    # Minimum acceleration, i.e., the maximum braking capability, in m/s^{2}.
    paras["min_acc"] = -5
    # Comfortable acceleration for each vehicle.
    paras["comf_acc"] = 1.67
    # The constant acceleration exponent.
    paras["delta_idm"] = 4

    ## Simulation parameters.
    # Peneration rate of CAVs.
    paras["penetration"] = 1
    # Random seed used to generate the volume.
    paras["random_seed"] = 1
    # simulation duration.
    paras["simulation_duration"] = 600
    # Simulation steps.
    paras["simulation_steps"] = paras["simulation_duration"] // paras["delta_T"]
    # Signal yellow time added between conflicting phases.
    paras["yellow_time"] = 5
    paras["all_red_time"] = 2
    # Left turn and right turn ratios. We only explicitly set the main road, i,e, through traffic, volumes. The volume of other movements are set as ratios to their main movements.
    paras["left_right_ratio"] = 1 / 6
    # Speed limit of all roads, in m/s.
    paras["speed_limit"] = 13
    # Vehicle length.
    paras["vehicle_length"] = 5
    # Communication range, in m.
    paras["comminication_range"] = 200
    # Number of lanes for each road.
    paras["num_lanes_each_road"] = 3
    # Total number of lanes around this intersection
    paras["num_lanes_intersection"] = 12
    # Road length, in m.
    paras["distance_from_upstream_intersections"] = 200
    # Number of signal phases.
    paras["num_phases"] = 9
    # The lowest volume, in veh/h.
    paras["low_volume"] = 200
    # The highest volume, in veh/h.
    paras["high_volume"] = 400
    # We simulate the volumes in a wave feature. This parameter represents the half-period of such waves. In seconds.
    paras["time_interval_seconds"] = int(paras["simulation_duration"]/6)


    ## pedestrian parameters:
    paras['crossing_length']=3.2*5 #m
    paras['X_crossing_length']=3.2*5*math.sqrt(2)
    paras["crossing_width"]=3.5 #m
    paras['ped_speed']=1 #m/s
    paras['num_cross']=6
    # weighting factor for each mode (for the slower scale problem optimization)
    paras["weight(Vehicles/Pedestrians)"] = (0.5, 0.5)


def set_network_topology_paras(paras):
    """Generate the network topology based on paras and network type.

    This function will add a "traffic_graph" to the paras.
    The graph includes multiple nodes, each of which represents an intersection.
    There are six features for each intersection.
        pos: The position of this intersection in an x-y plane.
        adj: A list of (x,y) tuples where x is the node connecting to the current node,
            and y is the number of lanes from the current node to this neighbor node.
        distance_from_upstream_intersections: The distance from this neighbor to the current node.
        num_lanes_each_road: The number of lanes from the neighbor node to the current node.
        range: Communication range.
        num_phase: Number of phases for the signal at the current node.
    """
    comminication_range = paras["comminication_range"]
    num_lanes_each_road = paras["num_lanes_each_road"]
    distance_from_upstream_intersections = paras["distance_from_upstream_intersections"]
    num_phases = paras["num_phases"]
    network_type = paras["network_type"]

    if network_type == "single_intersection":
        paras["traffic_graph"] = {
            1: {
                "pos": (0, 0),
                "adj": [],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            }
        }
    elif network_type == "corridor":
        paras["traffic_graph"] = {
            1: {
                "pos": (0, 0),
                "adj": [(2, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            2: {
                "pos": (200, 0),
                "adj": [(1, num_lanes_each_road), (3, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            3: {
                "pos": (400, 0),
                "adj": [(2, num_lanes_each_road), (4, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            4: {
                "pos": (600, 0),
                "adj": [(3, num_lanes_each_road), (5, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            5: {
                "pos": (800, 0),
                "adj": [(4, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
        }
    elif network_type == "4_4_network":
        paras["traffic_graph"] = {
            1: {
                "pos": (0, 0),
                "adj": [(2, num_lanes_each_road), (5, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            2: {
                "pos": (200, 0),
                "adj": [
                    (1, num_lanes_each_road),
                    (3, num_lanes_each_road),
                    (6, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            3: {
                "pos": (400, 0),
                "adj": [
                    (2, num_lanes_each_road),
                    (4, num_lanes_each_road),
                    (7, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            4: {
                "pos": (600, 0),
                "adj": [(3, num_lanes_each_road), (8, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            5: {
                "pos": (0, 200),
                "adj": [
                    (1, num_lanes_each_road),
                    (6, num_lanes_each_road),
                    (9, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            6: {
                "pos": (200, 200),
                "adj": [
                    (2, num_lanes_each_road),
                    (5, num_lanes_each_road),
                    (7, num_lanes_each_road),
                    (10, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            7: {
                "pos": (400, 200),
                "adj": [
                    (3, num_lanes_each_road),
                    (6, num_lanes_each_road),
                    (8, num_lanes_each_road),
                    (11, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            8: {
                "pos": (600, 200),
                "adj": [
                    (4, num_lanes_each_road),
                    (7, num_lanes_each_road),
                    (12, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            9: {
                "pos": (0, 400),
                "adj": [
                    (5, num_lanes_each_road),
                    (10, num_lanes_each_road),
                    (13, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            10: {
                "pos": (200, 400),
                "adj": [
                    (6, num_lanes_each_road),
                    (9, num_lanes_each_road),
                    (11, num_lanes_each_road),
                    (14, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            11: {
                "pos": (400, 400),
                "adj": [
                    (7, num_lanes_each_road),
                    (10, num_lanes_each_road),
                    (12, num_lanes_each_road),
                    (15, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            12: {
                "pos": (600, 400),
                "adj": [
                    (8, num_lanes_each_road),
                    (11, num_lanes_each_road),
                    (16, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            13: {
                "pos": (0, 600),
                "adj": [(9, num_lanes_each_road), (14, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            14: {
                "pos": (200, 600),
                "adj": [
                    (10, num_lanes_each_road),
                    (13, num_lanes_each_road),
                    (15, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            15: {
                "pos": (400, 600),
                "adj": [
                    (11, num_lanes_each_road),
                    (14, num_lanes_each_road),
                    (16, num_lanes_each_road),
                ],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
            16: {
                "pos": (600, 600),
                "adj": [(12, num_lanes_each_road), (15, num_lanes_each_road)],
                "distance_from_upstream_intersections": distance_from_upstream_intersections,
                "num_lanes_each_road": num_lanes_each_road,
                "range": comminication_range,
                "num_phases": num_phases,
            },
        }


def set_signal_phases_paras(paras):
    paras["actions"] = {
        "1": "GGGGrrrrrrrrGGGGrrrrrrrr",
        "2": "GGGGGGGrrrrrrrrrrrrrrrrr",
        "3": "rrrrrrrrrrrrGGGGGGGrrrrr",
        "4": "rrrrGGGrrrrrrrrrGGGrrrrr",
        "5": "rrrrrrGGGGrrrrrrrrGGGGrr",
        "6": "GrrrrrrrrrrrrrrrrrGGGGGG",
        "7": "rrrrrrGGGGGGGrrrrrrrrrrr",
        "8": "GrrrrrrrrrGGGrrrrrrrrrGG",
        "9": "rrrrrrrrrrrrrrrrrrrrrrrr",
    }
    paras["actions_full"] = {
        "1": "GGGGrrrrrrrrGGGGrrrrrrrr",
        "1_to_2_yellow": "GGGGrrrrrrrryyyyrrrrrrrr",
        "1_to_2_red": "GGGGrrrrrrrrrrrrrrrrrrrr",
        "1_to_3_yellow": "yyyyrrrrrrrrGGGGrrrrrrrr",
        "1_to_3_red": "yyyyrrrrrrrrGGGGrrrrrrrr",
        "1_to_4_yellow": "yyyyrrrrrrrryyyyrrrrrrrr",
        "1_to_4_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "1_to_5_yellow": "yyyyrrrrrrrryyyyrrrrrrrr",
        "1_to_5_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "1_to_6_yellow": "yyyyrrrrrrrryyyyrrrrrrrr",
        "1_to_6_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "1_to_7_yellow": "yyyyrrrrrrrryyyyrrrrrrrr",
        "1_to_7_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "1_to_8_yellow": "yyyyrrrrrrrryyyyrrrrrrrr",
        "1_to_8_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "2": "GGGGGGGrrrrrrrrrrrrrrrrr",
        "2_to_1_yellow": "GGGGyyyrrrrrrrrrrrrrrrrr",
        "2_to_1_red": "GGGGrrrrrrrrrrrrrrrrrrrr",
        "2_to_3_yellow": "yyyyyyyrrrrrrrrrrrrrrrrr",
        "2_to_3_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "2_to_4_yellow": "yyyyGGGrrrrrrrrrrrrrrrrr",
        "2_to_4_red": "rrrrGGGrrrrrrrrrrrrrrrrr",
        "2_to_5_yellow": "yyyyyyyrrrrrrrrrrrrrrrrr",
        "2_to_5_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "2_to_6_yellow": "yyyyyyyrrrrrrrrrrrrrrrrr",
        "2_to_6_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "2_to_7_yellow": "yyyyyyyrrrrrrrrrrrrrrrrr",
        "2_to_7_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "2_to_8_yellow": "yyyyyyyrrrrrrrrrrrrrrrrr",
        "2_to_8_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "3": "rrrrrrrrrrrrGGGGGGGrrrrr",
        "3_to_1_yellow": "rrrrrrrrrrrrGGGGyyyrrrrr",
        "3_to_1_red": "rrrrrrrrrrrrGGGGrrrrrrrr",
        "3_to_2_yellow": "rrrrrrrrrrrryyyyyyyrrrrr",
        "3_to_2_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "3_to_4_yellow": "rrrrrrrrrrrryyyyGGGrrrrr",
        "3_to_4_red": "rrrrrrrrrrrrrrrrGGGrrrrr",
        "3_to_5_yellow": "rrrrrrrrrrrryyyyyyyrrrrr",
        "3_to_5_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "3_to_6_yellow": "rrrrrrrrrrrryyyyyyyrrrrr",
        "3_to_6_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "3_to_7_yellow": "rrrrrrrrrrrryyyyyyyrrrrr",
        "3_to_7_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "3_to_8_yellow": "rrrrrrrrrrrryyyyyyyrrrrr",
        "3_to_8_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "4": "rrrrGGGrrrrrrrrrGGGrrrrr",
        "4_to_1_yellow": "rrrryyyrrrrrrrrryyyrrrrr",
        "4_to_1_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "4_to_2_yellow": "rrrrGGGrrrrrrrrryyyrrrrr",
        "4_to_2_red": "rrrrGGGrrrrrrrrrrrrrrrrr",
        "4_to_3_yellow": "rrrryyyrrrrrrrrrGGGrrrrr",
        "4_to_3_red": "rrrrrrrrrrrrrrrrGGGrrrrr",
        "4_to_5_yellow": "rrrryyyrrrrrrrrryyyrrrrr",
        "4_to_5_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "4_to_6_yellow": "rrrryyyrrrrrrrrryyyrrrrr",
        "4_to_6_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "4_to_7_yellow": "rrrryyyrrrrrrrrryyyrrrrr",
        "4_to_7_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "4_to_8_yellow": "rrrryyyrrrrrrrrryyyrrrrr",
        "4_to_8_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "5": "rrrrrrGGGGrrrrrrrrGGGGrr",
        "5_to_1_yellow": "rrrrrryyyyrrrrrrrryyyyrr",
        "5_to_1_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "5_to_2_yellow": "rrrrrryyyyrrrrrrrryyyyrr",
        "5_to_2_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "5_to_3_yellow": "rrrrrryyyyrrrrrrrryyyyrr",
        "5_to_3_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "5_to_4_yellow": "rrrrrryyyyrrrrrrrryyyyrr",
        "5_to_4_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "5_to_6_yellow": "rrrrrryyyyrrrrrrrrGGGGrr",
        "5_to_6_red": "rrrrrrrrrrrrrrrrrrGGGGrr",
        "5_to_7_yellow": "rrrrrrGGGGrrrrrrrryyyyrr",
        "5_to_7_red": "rrrrrrGGGrrrrrrrrrrrrrrr",
        "5_to_8_yellow": "rrrrrryyyyrrrrrrrryyyyrr",
        "5_to_8_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "6": "GrrrrrrrrrrrrrrrrrGGGGGG",
        "6_to_1_yellow": "yrrrrrrrrrrrrrrrrryyyyyy",
        "6_to_1_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "6_to_2_yellow": "yrrrrrrrrrrrrrrrrryyyyyy",
        "6_to_2_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "6_to_3_yellow": "yrrrrrrrrrrrrrrrrryyyyyy",
        "6_to_3_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "6_to_4_yellow": "yrrrrrrrrrrrrrrrrryyyyyy",
        "6_to_4_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "6_to_5_yellow": "yrrrrrrrrrrrrrrrrrGGGGyy",
        "6_to_5_red": "rrrrrrrrrrrrrrrrrrGGGGrr",
        "6_to_7_yellow": "yrrrrrrrrrrrrrrrrryyyyyy",
        "6_to_7_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "6_to_8_yellow": "yrrrrrrrrrrrrrrrrryyyyGG",
        "6_to_8_red": "rrrrrrrrrrrrrrrrrrrrrrGG",
        "7": "rrrrrrGGGGGGGrrrrrrrrrrr",
        "7_to_1_yellow": "rrrrrryyyyyyyrrrrrrrrrrr",
        "7_to_1_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "7_to_2_yellow": "rrrrrryyyyyyyrrrrrrrrrrr",
        "7_to_2_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "7_to_3_yellow": "rrrrrryyyyyyyrrrrrrrrrrr",
        "7_to_3_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "7_to_4_yellow": "rrrrrryyyyyyyrrrrrrrrrrr",
        "7_to_4_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "7_to_5_yellow": "rrrrrrGGGGyyyrrrrrrrrrrr",
        "7_to_5_red": "rrrrrrGGGGrrrrrrrrrrrrrrr",
        "7_to_6_yellow": "rrrrrryyyyyyyrrrrrrrrrrr",
        "7_to_6_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "7_to_8_yellow": "rrrrrryyyyGGyrrrrrrrrrrr",
        "7_to_8_red": "rrrrrrrrrrGGrrrrrrrrrrrr",
        "8": "GrrrrrrrrrGGGrrrrrrrrrGG",
        "8_to_1_yellow": "yrrrrrrrrryyyrrrrrrrrryy",
        "8_to_1_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "8_to_2_yellow": "yrrrrrrrrryyyrrrrrrrrryy",
        "8_to_2_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "8_to_3_yellow": "yrrrrrrrrryyyrrrrrrrrryy",
        "8_to_3_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "8_to_4_yellow": "yrrrrrrrrryyyrrrrrrrrryy",
        "8_to_4_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "8_to_5_yellow": "yrrrrrrrrryyyrrrrrrrrryy",
        "8_to_5_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
        "8_to_6_yellow": "yrrrrrrrrryyyrrrrrrrrrGG",
        "8_to_6_red": "rrrrrrrrrrrrrrrrrrrrrrGG",
        "8_to_7_yellow": "yrrrrrrrrrGGyrrrrrrrrryy",
        "8_to_7_red": "rrrrrrrrrrGGrrrrrrrrrrrr",
        "all_red": "rrrrrrrrrrrrrrrrrrrrrrrr",
    }


def set_volume_paras(paras):
    low_volume = paras["low_volume"]
    high_volume = paras["high_volume"]
    time_interval_seconds = paras["time_interval_seconds"]
    num_interval = (
        paras["simulation_steps"] * paras["delta_T"] // paras["time_interval_seconds"]
    )
    volume_type = paras["volume_type"]

    if volume_type == "symmetric":
        paras["depart_rate"] = {
            "time_interval": time_interval_seconds,
            "vol_we_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_ew_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_ns_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_sn_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
        }
    elif volume_type == "asymmetric":
        paras["depart_rate"] = {
            "time_interval": time_interval_seconds,
            "vol_we_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_ew_main": [
                low_volume / 2
                + (high_volume / 2 - low_volume / 2) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_ns_main": [
                low_volume / 2
                + (high_volume / 2 - low_volume / 2) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_sn_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
        }
    else:
        raise TypeError("Unknown volume type!")


def set_parameters(network_type, volume_type):
    """Get the parameters for the entire simulation.

    Args:
        network_type: Should be one of (single_intersection, corridor, 4_4_network)

    Returns:
        paras: The parameters used to build the SUMO/MPC models, and all about this project.
    """
    paras = collections.defaultdict()
    paras["network_type"] = network_type
    paras["volume_type"] = volume_type

    ## Common parameters
    set_common_paras(paras)

    ## Network topology
    set_network_topology_paras(paras)

    # Signal actions
    #set_signal_phases_paras(paras)

    # Volume
    set_volume_paras(paras)
    return paras

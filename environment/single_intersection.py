## The single intersection environment.
import traci
import collections
import os
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
import math

from sumolib import checkBinary

from .sumo_network_builder import SumoNetworkBuilder
from .sumo_routes_generator import SumoRoutesGenerator


class SingleIntersection:
    def __init__(self, paras):
        self.paras = paras
        print("--------Building SUMO network...")
        network_builder = SumoNetworkBuilder(self.paras)
        self.paras["neighbors"] = network_builder.build()
        print("--------SUMO network built.")

        print("--------Generating vehicles and routes")
        routes_generator = SumoRoutesGenerator(self.paras)
        cav_ids, hdv_ids = routes_generator.generate_routes_for_single_intersection()
        self.paras["ped_ids"] = set()
        self.paras["cav_ids"] = cav_ids
        self.paras["hdv_ids"] = hdv_ids
        self.fuel_total_cav_external_model = 0
        self.fuel_total_hdv_external_model = 0
        self.fuel_total_cav_sumo = 0
        self.fuel_total_hdv_sumo = 0
        self.phase_list_fix_act =[]
        self.dur_list_fix_act =[]
        self.greenTimeSofar = 0
        self.next_time_to_change_to_actuated = 0
        self.stopped_peds = set()
        self.cross_map = {':1_c0': 'N', ':1_c2': 'E', ':1_c4': 'S', ':1_c5': 'W', ':1_c3': 'NWSE', ':1_c1': 'NESW'}
        self.walking_areas = [':1_w3', ':1_w2', ':1_w1', ':1_w0']
        self.cur_phase = None
        self.previous_phase = None
        self.yellow_duration=0
        self.phase_avg = None
        self.phase_ntimes = None
        self.right_turn_conflicts = {}
        print("--------Vehicles and routes generated.")

    def start_sumo(self, show_gui, control_type, network_type, volume_type):
        penetration = self.paras["penetration"]
        ## GUI setting.
        if show_gui:
            sumoBinary = checkBinary("sumo-gui")
        else:
            sumoBinary = checkBinary("sumo")

        if control_type not in ["multi_scale", "actuated", "fixed_time"]:
            raise TypeError("unknown control_type!")

        ## Check if the SUMO model exists or not.
        model_dir = os.path.dirname(os.path.realpath(__file__)) + "/network_model"
        if not os.path.exists(model_dir):
            raise TypeError("Network model is not built yet.")
        model_file_name = (
            model_dir + "/" + network_type + "_" + control_type + ".sumocfg"
        )

        ## Create folder to store simulation data.
        data_dir = os.path.dirname(os.path.realpath(__file__)) + "/simulation_data"
        if network_type in ["single_intersection", "corridor", "4_4_network"]:
            data_dir_next = data_dir + "/" + network_type
        else:
            raise TypeError("unknown traffic network!")
        if not os.path.exists(data_dir_next):
            os.mkdir(data_dir_next)

        queue_file = (
            data_dir_next
            + "/queues_"
            + control_type
            + "_"
            + volume_type
            + "_pene_"
            + str(int(penetration * 100))
            + ".xml"
        )
        trip_file = (
            data_dir_next
            + "/tripinfo_"
            + control_type
            + "_"
            + volume_type
            + "_pene_"
            + str(int(penetration * 100))
            + ".xml"
        )

        ## Start SUMO.
        traci.start(
            [
                sumoBinary,
                "-c",
                model_file_name,
                "--start",
                "--queue-output",
                queue_file,
                "--tripinfo-output",
                trip_file,
                "--step-length",
                "0.5",
            ]
        )
        if control_type=="actuated":
            traci.trafficlight.setProgram("1", "actuated_program")

    def is_active(self):
        return traci.simulation.getMinExpectedNumber() > 0

    def get_state_cur_intersection(self, cur_step):
        ## Get the parameters needed to get the network state.
        neighbors = self.paras["neighbors"]
        traffic_graph = self.paras["traffic_graph"]
        speed_limit = self.paras["speed_limit"]
        delta_T = self.paras["delta_T"]
        delta_T_faster = self.paras["delta_T_faster"]
        num_predict_steps = self.paras["num_predict_steps"]
        cav_ids = self.paras["cav_ids"]
        right_turning_vehs=["WS", "SE", "EN", "NW"]

        self.network_state = collections.defaultdict()
        ## Iterate through each intersection.
        for inter_id in traffic_graph:
            ## State representations for each intersection
            signal_phase = int(traci.trafficlight.getPhase(str(inter_id)))
            self.phase_list_fix_act.append(signal_phase)
            self.dur_list_fix_act.append(cur_step)

            pos_vehicles, speed_vehicles, wt_vehicles, veh_id = [], [], [], []
            pos_peds, speed_peds, wt_peds, ped_id = [], [], [], []
            arrival_times = []
            num_vehicles_max = 0
            lane_id = []
            lane_length = []
            sidewalk_id=[]
            cross_demand={}
            cross_demand_all={}
            delta_T=self.paras["delta_T"]
            delta_T_faster=self.paras["delta_T_faster"]
            num_predict_steps=self.paras["num_predict_steps"]
            another_map= {':1_c0': 4, ':1_c2': 3, ':1_c4': 2, ':1_c5': 1, ':1_c1': 5, ':1_c3': 6} #CounterClockwise starting from the Westbound

            Xcrossing = False  # True or False
            if Xcrossing:
                map_diag= {':1_w0':[':1_c3'], ':1_w1': [':1_c1'], ':1_w2': [':1_c3'], ':1_w3':[':1_c1']}
            else:
                map_diag= {':1_w0':[':1_c0', ':1_c5'], ':1_w1': [':1_c0', ':1_c2'], ':1_w2': [':1_c2',':1_c4'], ':1_w3':[':1_c4', ':1_c5']}


            for i in self.cross_map.keys():
                cross_demand[i]=set()
                cross_demand_all[another_map[i]] = {1:[], 2:[], 3:[], 4:[], 5:[], 6:[], 7:[]}

            communication_range = traffic_graph[inter_id]["range"]
            ## Get static information.
            for direction in ["west", "south", "east", "north"]:
                node, len_lane, num_lane = neighbors[inter_id][direction]
                for temp in range(num_lane, 0, -1):
                    lane_id.append("%i_%i_%i" % (node, inter_id, temp))
                    lane_length.append(len_lane)
                for _ in range(int(num_lane/3-1), -1, -1):
                    sidewalk_id.append("%i_%i" % (node, inter_id))
            sidewalk_id.extend(self.walking_areas)

            ## Get pedestrian info:
            for i in range(len(sidewalk_id)):
                ## note that you are not taking account of the
                # second crossing demand yet and they would only
                # immediately be added to the current demand of the
                # second crossing but not include in the 10s, 20s,
                # 30s, 40s analysis of that second crossing
                # and I don't tink we can consider that because we don't
                # know if the signal for the first crossing gonna be green or not
                # but unfortunately this would make the optimization not be as accurate as we want
                pos_peds.append([])
                speed_peds.append([])
                wt_peds.append([])
                ped_id.append([])
                peds_lane = traci.edge.getLastStepPersonIDs(sidewalk_id[i]) #Get the IDs of the pedestrians on the lane
                self.paras["ped_ids"] = self.paras["ped_ids"].union(set(peds_lane))

                for j in range(len(peds_lane)):
                    pos=traci.person.getLanePosition(peds_lane[j])
                    speed=traci.person.getSpeed(peds_lane[j])

                    waiting_time=traci.person.getWaitingTime(peds_lane[j])
                    route=traci.person.getEdges(peds_lane[j],)
                    next_edge=traci.person.getNextEdge(peds_lane[j])
                    current_edge=traci.person.getLaneID(peds_lane[j])[0:5]
                    #print("pedestrian: ", peds_lane[j])
                    #print("route: ",route)
                    #print("next_edge: ",next_edge)

                    pos_peds[i%4].append(pos)
                    speed_peds[i%4].append(speed)
                    wt_peds[i%4].append(waiting_time)
                    ped_id[i%4].append(peds_lane[j])

                    if next_edge in self.cross_map.keys():
                        if "diag" in peds_lane[j]:
                            for cross in map_diag[current_edge]:
                                cross_demand[cross].add(peds_lane[j])
                                cross_demand_all[another_map[cross]][1].append(peds_lane[j])
                        else:
                            cross_demand[next_edge].add(peds_lane[j])
                            cross_demand_all[another_map[next_edge]][1].append(peds_lane[j])
                        #print("cross_demand[West]: ",cross_demand[':1_c5'])
                        #print("cross_demand[East]: ", cross_demand[':1_c2'])
                        #print("cross_demand[North]: ", cross_demand[':1_c0'])
                        #print("cross_demand[South]: ", cross_demand[':1_c4'])
                        #print("cross_demand[NESW]: ", cross_demand[':1_c1'])
                        #print("cross_demand[NWSE]: ", cross_demand[':1_c3'])
                        # traci.person.setColor(peds_lane[j], (255, 0, 0, 255))
                    elif next_edge in self.walking_areas and speed!=0:
                        if "diag" in peds_lane[j]:
                            for cross in map_diag[next_edge]:
                                for step in range(2, num_predict_steps + 2):
                                    if (186.4 - pos) / speed <= (step - 1) * delta_T and (186.4 - pos) / speed >= (step - 2) * delta_T:
                                        cross_demand_all[another_map[cross]][step].append(peds_lane[j])
                                        traci.person.setColor(peds_lane[j], (255, 0, 0, 255))
                        else:
                            crossing = route[2]
                            for step in range(2, num_predict_steps+2):
                                if (186.4-pos)/speed <= (step-1)*delta_T and (186.4-pos)/speed >= (step-2)*delta_T:
                                    cross_demand_all[another_map[crossing]][step].append(peds_lane[j])
                                    traci.person.setColor(peds_lane[j], (255, 0, 0, 255))


                        # if (186.4-pos)/speed <10:
                        #     cross_demand_10s[crossing].add(peds_lane[j])
                        # elif (186.4-pos)/speed <20:
                        #     cross_demand_20s[crossing].add(peds_lane[j])
                        #     traci.person.setColor(peds_lane[j], (255, 0, 0, 255))
                        # elif (186.4-pos)/speed <30:
                        #     cross_demand_30s[crossing].add(peds_lane[j])
                        #     traci.person.setColor(peds_lane[j], (255, 0, 0, 255))
                        # elif (186.4-pos)/speed <40:
                        #     cross_demand_40s[crossing].add(peds_lane[j])
                        #     traci.person.setColor(peds_lane[j], (255, 0, 0, 255))

                    # for cross in self.cross_map.keys():
                    #     cross_demand_next5s.setdefault(self.cross_map[cross], set())
                    #     if cross in route:
                    #         d=self.cross_map[cross]
                    #         cross_demand.setdefault(d,set())
                    #         cross_demand[d].add(peds_lane[j])
                    #         if (speed!=0 and (186.4-pos)/speed <5 or pos<5) : ## could be adjusted for predicting the next 5, 10 or 20 seconds demand
                    #             cross_demand_next5s[d].add(peds_lane[j])
                    #             traci.person.setColor(peds_lane[j], (255, 0, 0, 255))

                ## Count the Conflicts between right turning vehicles and pedestrians:
                all_cars=traci.vehicle.getIDList()
                all_peds=traci.person.getIDList()
                for car in all_cars:
                    for ped in all_peds:
                        for dir in right_turning_vehs:
                            if (car.startswith(dir) and traci.vehicle.getLaneID(car).startswith(":") and traci.person.getLaneID(ped) in [key + "_0" for key in self.cross_map.keys()]):
                                self.right_turn_conflicts.setdefault(car,set())
                                position_car=traci.vehicle.getPosition(car)
                                position_ped=traci.person.getPosition(ped)
                                if math.dist(position_car, position_ped) < 2:
                                    self.right_turn_conflicts[car].add(ped)
                                    #print(self.right_turn_conflicts)



            ## Get dynamic information. Use linear interpolation to estimate HDV's state.
            for i in range(len(lane_id)):
                pos_vehicles.append([])
                speed_vehicles.append([])
                wt_vehicles.append([])
                arrival_times.append([])
                veh_id.append([])
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])


                # Number of vehicles in the current lane and also in the communication range.
                num_vehicles_lane = 0
                # Number of HDVs between two CAVs. Will be set to 0 when we find a CAV.
                current_num_hdv = 0
                # HDV ids between two CAVs. Will be set to empty when we find a CAV.
                current_ids_hdv = []
                for j in range(len(cars_lane) - 1, -1, -1):
                    if cars_lane[j] in cav_ids["all"]:
                        ## This is a CAV, we need to first check if there are HDVs in front of it.
                        if current_num_hdv == 0:
                            ## There is no HDV between the current CAV and the CAV ahead.
                            if (
                                lane_length[i]
                                - traci.vehicle.getLanePosition(cars_lane[j])
                                - 13
                                <= communication_range
                            ):
                                ## The current CAV is within the communication range. Note that the value
                                ## 13 accounts for the intersection length. Simply add the CAV's state.
                                pos_vehicles[-1].append(
                                    -(
                                        lane_length[i]
                                        - traci.vehicle.getLanePosition(cars_lane[j])
                                        - 13
                                    )
                                )
                                speed_vehicles[-1].append(
                                    traci.vehicle.getSpeed(cars_lane[j])
                                )
                                wt_vehicles[-1].append(
                                    traci.vehicle.getWaitingTime(cars_lane[j])
                                )
                                veh_id[-1].append(cars_lane[j])
                                num_vehicles_lane += 1
                            else:
                                ## The current CAV is outside the communication range. We need to check if
                                ## it will arrive within the MPC's prediction horizon. If so, add its arrival time.
                                temp = (
                                    lane_length[i]
                                    - traci.vehicle.getLanePosition(cars_lane[j])
                                    - 13
                                    - communication_range
                                ) / speed_limit
                                if temp < delta_T * num_predict_steps:
                                    arrival_times[-1].append(
                                        cur_step + int(temp / delta_T_faster)
                                    )
                                    veh_id[-1].append(cars_lane[j])
                                    num_vehicles_lane += 1
                        else:
                            ## There are some HDVs in front of the current CAV. Use interpolation to estimate their states.
                            current_ids_hdv.append(cars_lane[j])
                            # Find the right most vehicle's state.
                            no_cav_ahead = False
                            if not pos_vehicles[-1]:
                                # This is the first CAV in the lane. All the vehicles in front of it are HDVs.
                                r_pos, r_spd, r_wt = -0.1, 0, 0
                                pos_vehicles[-1].append(r_pos)
                                speed_vehicles[-1].append(r_spd)
                                wt_vehicles[-1].append(r_wt)
                                veh_id[-1].append(current_ids_hdv[0])
                                current_num_hdv -= 1
                                num_vehicles_lane += 1
                                no_cav_ahead = True
                            else:
                                # There are CAVs ahead. Simply get the values.
                                r_pos, r_spd, r_wt = (
                                    pos_vehicles[-1][-1],
                                    speed_vehicles[-1][-1],
                                    wt_vehicles[-1][-1],
                                )
                            # Find the left most vehicle's state, i.e., the current vehicle's state.
                            l_pos = -(
                                lane_length[i]
                                - traci.vehicle.getLanePosition(cars_lane[j])
                                - 13
                            )
                            l_spd = traci.vehicle.getSpeed(cars_lane[j])
                            l_wt = traci.vehicle.getWaitingTime(cars_lane[j])
                            # Estimate HDVs' states and record them if in communication range or will arrive during MPC's prediction horizon.
                            for k in range(current_num_hdv + 1):
                                t_pos = r_pos + (k + 1) / (current_num_hdv + 1) * (
                                    l_pos - r_pos
                                )
                                t_spd = r_spd + (k + 1) / (current_num_hdv + 1) * (
                                    l_spd - r_spd
                                )
                                t_wt = r_wt + (k + 1) / (current_num_hdv + 1) * (
                                    l_wt - r_wt
                                )
                                if -t_pos <= communication_range:
                                    pos_vehicles[-1].append(t_pos)
                                    speed_vehicles[-1].append(t_spd)
                                    wt_vehicles[-1].append(t_wt)
                                    if no_cav_ahead:
                                        veh_id[-1].append(current_ids_hdv[k + 1])
                                    else:
                                        veh_id[-1].append(current_ids_hdv[k])
                                    num_vehicles_lane += 1
                                else:
                                    temp = (-t_pos - communication_range) / speed_limit
                                    if temp < delta_T * num_predict_steps:
                                        arrival_times[-1].append(temp)
                                        if no_cav_ahead:
                                            veh_id[-1].append(current_ids_hdv[k + 1])
                                        else:
                                            veh_id[-1].append(current_ids_hdv[k])
                                        num_vehicles_lane += 1
                        current_num_hdv = 0
                        current_ids_hdv = []
                    else:
                        ## This is a HDV. We can't get the state. Juest record the ids and update the number of HDVs.
                        current_ids_hdv.append(cars_lane[j])
                        current_num_hdv += 1

                ## There could be a corner case where the last several vehicles in a lane are all HDVs.
                ## We need to estimate the information of those vehicles. The methods are the same as above.
                if current_num_hdv != 0:
                    no_cav_ahead = False
                    if not pos_vehicles[-1]:
                        r_pos, r_spd, r_wt = -0.1, 0, 0
                        pos_vehicles[-1].append(r_pos)
                        speed_vehicles[-1].append(r_spd)
                        wt_vehicles[-1].append(r_wt)
                        veh_id[-1].append(current_ids_hdv[0])
                        current_num_hdv -= 1
                        num_vehicles_lane += 1
                        no_cav_ahead = True
                    else:
                        r_pos, r_spd, r_wt = (
                            pos_vehicles[-1][-1],
                            speed_vehicles[-1][-1],
                            wt_vehicles[-1][-1],
                        )
                    l_pos = -(
                        lane_length[i]
                        - traci.vehicle.getLanePosition(cars_lane[0])
                        - 13
                    )
                    l_spd = traci.vehicle.getSpeed(cars_lane[0])
                    l_wt = traci.vehicle.getWaitingTime(cars_lane[0])
                    for k in range(current_num_hdv):
                        t_pos = r_pos + (k + 1) / (current_num_hdv) * (l_pos - r_pos)
                        t_spd = r_spd + (k + 1) / (current_num_hdv) * (l_spd - r_spd)
                        t_wt = r_wt + (k + 1) / (current_num_hdv) * (l_wt - r_wt)
                        if -t_pos <= communication_range:
                            pos_vehicles[-1].append(t_pos)
                            speed_vehicles[-1].append(t_spd)
                            wt_vehicles[-1].append(t_wt)
                            if no_cav_ahead:
                                veh_id[-1].append(current_ids_hdv[k + 1])
                            else:
                                veh_id[-1].append(current_ids_hdv[k])
                            num_vehicles_lane += 1
                        else:
                            temp = (-t_pos - communication_range) / speed_limit
                            if temp < delta_T * num_predict_steps:
                                arrival_times[-1].append(
                                    cur_step + int(temp / delta_T_faster)
                                )
                                if no_cav_ahead:
                                    veh_id[-1].append(current_ids_hdv[k + 1])
                                else:
                                    veh_id[-1].append(current_ids_hdv[k])
                                num_vehicles_lane += 1
                num_vehicles_max = max(num_vehicles_max, num_vehicles_lane)

            self.network_state[inter_id] = {
                "num_vehicles_max": num_vehicles_max,
                "pos_vehicles": pos_vehicles,
                "speed_vehicles": speed_vehicles,
                "wt_vehicles": wt_vehicles,
                "arrival_times": arrival_times,
                "signal_phase": signal_phase,
                "num_lane": len(lane_id),
                "lane_length": lane_length,
                "lane_id": lane_id,
                "vehicle_id": veh_id,
                "pedestrian_demand_current": cross_demand,
                "pedestrian_demand": cross_demand_all,
            }
        return self.network_state

    def apply_control_commands(
        self, should_update_signal, next_signal_phase, speed_commands
    ):
        cur_phase = int(traci.trafficlight.getPhase("1"))
        ## Signal phase control.
        if should_update_signal:
            if next_signal_phase == -1:
                self.yellow_duration = self.paras["delta_T_faster"]
                traci.trafficlight.setPhase("1", cur_phase + 9)
            else:
                traci.trafficlight.setPhase("1", next_signal_phase)
                self.yellow_duration = 0
        elif next_signal_phase == -1:
            self.yellow_duration += self.paras["delta_T_faster"]

        # print("simulation time: ",traci.simulation.getTime())
        # print("current phase: ", cur_phase)
        # print("yellow duration: ", self.yellow_duration)
        if self.yellow_duration == self.paras['yellow_time']:
            traci.trafficlight.setPhase("1", 17)
        ## Vehicles control
        network_vehs=traci.vehicle.getIDList()
        for veh_id in speed_commands:
            if veh_id in network_vehs:
                traci.vehicle.setSpeed(veh_id, speed_commands[veh_id])

    def move_one_step_forward(self):
        traci.simulationStep()

    def performance_results(self, phase_list_multi, duration_list_multi, network_type, volume_type, control_type, step):
        data_dir = os.path.dirname(os.path.realpath(__file__)) + "/simulation_data"
        data_dir_next = data_dir + "/" + network_type
        queue_file = (data_dir_next + "/queues_" + control_type + "_" + volume_type + "_pene_" + str(int(self.paras["penetration"] * 100)) + ".xml")
        tripinfo_file = (data_dir_next + "/tripinfo_" + control_type + "_" + volume_type + "_pene_" + str(int(self.paras["penetration"] * 100)) + ".xml")
        self.get_average_delay_endtime(tripinfo_file)
        self.get_average_queue_length_endtime(queue_file)
        self.get_average_phase_duration(phase_list_multi, duration_list_multi, control_type)
        right_conflicts=self.right_turn_conflicts_measure()
        print(f"average fuel consumption (external model) for {control_type} (in mg): ",
              self.fuel_total_cav_external_model / len(self.paras["cav_ids"]["all"]))
        print(f"average fuel consumption (SUMO output) for {control_type} (in mg): ",
              self.fuel_total_cav_sumo / len(self.paras["cav_ids"]["all"]))
        print(f"average waiting time for {control_type} (in s): ",
              self.waiting_time_avg)
        print(f"average time loss for {control_type} (in s): ",
              self.lost_time_avg)
        print(f"average queue for {control_type} length (in m): ",
              self.queue_avg)
        print(f"average pedestrian time loss for {control_type} (in s): ",
              self.lost_time_avg_ped)
        print(f"number of right-turn conflicts between vehicles and pedestrians: {right_conflicts}")
        print(f"number of CAVs passing through the specific intersection for {control_type}: ",
              len(self.paras["cav_ids"]["all"]))
        print(f"number of pedestrians passing through the specific intersection for {control_type}: ",
              len(self.paras["ped_ids"]))
        print(f"The time of simulation termination in {control_type} scenario:",step/2 )
        with open(f"Metrics_Results_{control_type}_scenario.txt", 'w') as file:
            file.write(f"average fuel consumption for {control_type} scenario (external model) (in mg): {self.fuel_total_cav_external_model / len(self.paras['cav_ids']['all'])}\n")
            file.write(f"average fuel consumption for {control_type} scenario (SUMO output) (in mg): {self.fuel_total_cav_sumo / len(self.paras['cav_ids']['all'])}\n")
            file.write(f"average waiting time for {control_type} scenario (in s): {self.waiting_time_avg}\n")
            file.write(f"average time loss for {control_type} scenario (in s): {self.lost_time_avg}\n")
            file.write(f"average queue length for {control_type} scenario (in m): {self.queue_avg}\n")
            file.write(f"average pedestrian time loss for {control_type} scenario (in s): {self.lost_time_avg_ped}\n")
            file.write(f"number of right-turn conflicts between vehicles and pedestrians: {right_conflicts}\n")
            file.write(f"number of CAVs passing through the specific intersection in {control_type} scenario: {len(self.paras['cav_ids']['all'])}\n")
            file.write(f"number of pedestrians passing through the specific intersection in {control_type} scenario: {len(self.paras['ped_ids'])}\n")
            file.write(f"The time of simulation termination in {control_type} scenario: {step/2}\n")
            file.write(f"average phase lengths:  {self.phase_avg}\n")
            file.write(f"number of times each phase happened: {self.phase_ntimes}\n")

    def right_turn_conflicts_measure(self):
        a=0
        for key in self.right_turn_conflicts.keys():
            a+=len(self.right_turn_conflicts[key])
        return a
    def get_average_phase_duration(self, phase_list_multi, duration_list_multi, control_type):
        phase_dict = {}
        if control_type!="multi_scale":
            change_step=0
            for i, phase in enumerate(self.phase_list_fix_act[:-1]):
                next_phase=self.phase_list_fix_act[i+1]
                if next_phase!=phase:
                    phase_dict.setdefault(phase,[])
                    phase_dict[phase].append((i-change_step)*0.5)
                    change_step = i
            x = np.array(self.dur_list_fix_act)
            widths = np.diff(np.concatenate(([0], x)))
            plt.figure(figsize=(15, 7))
            phase_list_fix_act_1 = [phase + 1 for phase in self.phase_list_fix_act]
            plt.bar(x - widths, phase_list_fix_act_1, width=widths, align='edge')
            plt.gca().xaxis.set_minor_locator(plt.MultipleLocator(100))
            plt.xlabel('Time Duration')
            plt.ylabel('Phase Number')
            plt.title('Phases over Duration')
            plt.savefig("Phases_Over_Duration.png")
            plt.show()

            self.phase_avg={}
            self.phase_ntimes={}
            for phase in phase_dict.keys():
                self.phase_avg[phase]=sum(phase_dict[phase])/len(phase_dict[phase])
                self.phase_ntimes[phase]=sum(phase_dict[phase])/5
            print("average phase lengths: ", self.phase_avg)
            print("number of times each phase happened: ", self.phase_ntimes)


        else:
            change_index=0
            for i, phase in enumerate(phase_list_multi[:-1]):
                next_phase = phase_list_multi[i+1]
                if phase != next_phase:
                    phase_dict.setdefault(phase, [])
                    phase_dict[phase].append(sum(duration_list_multi[change_index:i+1]))
                    change_index = i+1
            x = np.cumsum(duration_list_multi)
            widths = np.diff(np.concatenate(([0], x)))
            plt.figure(figsize=(15, 7))
            phase_list_multi_1 = [phase + 1 for phase in phase_list_multi]
            plt.bar(x - widths, phase_list_multi_1, width=widths, align='edge')
            plt.xlabel('Time Duration')
            plt.ylabel('Phase Number')
            plt.title('Phases over Duration (Multi-scale)')
            plt.savefig("Phases_Over_Duration.png")
            plt.show()

            with open('output_phase.txt', 'w') as file:
                for item1, item2 in zip(phase_list_multi, duration_list_multi):
                    file.write(f"{item1}\t{item2}\n")

            self.phase_avg={}
            self.phase_ntimes={}
            for phase in phase_dict.keys():
                self.phase_avg[phase]=sum(phase_dict[phase])/len(phase_dict[phase])
                self.phase_ntimes[phase]=sum(phase_dict[phase])/5
            print("average phase lengths: ", self.phase_avg)
            print("number of times each phase happened: ", self.phase_ntimes)


    def get_average_delay_endtime(self,file):
        tree = ET.parse(file)
        root = tree.getroot()
        cnt = 0
        cnt_ped=0
        wt = 0
        tl = 0
        tl_ped=0
        wt_max = 0
        for stu in root:
            if stu.tag == "tripinfo":
                wt += float(stu.attrib["waitingTime"])
                wt_max = max(wt_max, float(stu.attrib["waitingTime"]))
                tl += float(stu.attrib["timeLoss"])
                cnt += 1
            elif stu.tag == "personinfo":
                walk=stu.find("walk")
                tl_ped +=float (walk.attrib["timeLoss"])
                cnt_ped += 1
        #print("wt cnt: ", cnt)
        #print("max wt: ", wt_max)
        self.waiting_time_avg = wt / len(self.paras["cav_ids"]["all"])
        self.lost_time_avg = tl / len(self.paras["cav_ids"]["all"])
        self.lost_time_avg_ped = tl_ped / len(self.paras["ped_ids"])

    def get_average_queue_length_endtime(self, file):
        tree = ET.parse(file)
        root = tree.getroot()
        cnt = 0
        queue = 0
        for time in root:
            for lanes in time:
                for lane in lanes:
                    queue += float(lane.attrib["queueing_length"])
                    cnt += 1
        #print("queue cnt: ", cnt)
        self.queue_avg = queue / cnt


    def calculate_extra_metrics(self):
        temp_cav, temp_hdv = self.get_instant_fuel_external_model()
        self.fuel_total_cav_external_model += temp_cav
        self.fuel_total_hdv_external_model += temp_hdv
        temp_cav, temp_hdv = self.get_instant_fuel_sumo()
        self.fuel_total_cav_sumo += temp_cav
        self.fuel_total_hdv_sumo += temp_hdv

    def get_instant_fuel_external_model(self):
        fuel_cav = 0
        fuel_hdv = 0
        for inter_id in self.paras["traffic_graph"]:
            lane_id = []
            for direction in ["west", "south", "east", "north"]:
                node, len_lane, num_lane = self.paras["neighbors"][inter_id][direction]
                for temp in range(num_lane, 0, -1):
                    lane_id.append("%i_%i_%i" % (node, inter_id, temp))
            for i in range(len(lane_id)):
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])
                for j in range(len(cars_lane) - 1, -1, -1):
                    speed = traci.vehicle.getSpeed(cars_lane[j])
                    acc = traci.vehicle.getAcceleration(cars_lane[j])
                    fuel_temp = (
                        0.2736
                        + 0.0599 * speed
                        + 0.3547 * acc
                        - 0.0058 * speed**2
                        + 0.0179 * speed * acc
                        + 0.0663 * acc**2
                        + 0.0002 * speed**3
                        + 0.002 * speed**2 * acc
                        + 0.0245 * speed * acc**2
                        - 0.0489 * acc**3
                    )
                    if cars_lane[j] in self.paras["cav_ids"]["all"]:
                        fuel_cav += fuel_temp
                    else:
                        fuel_hdv += fuel_temp
        return fuel_cav, fuel_hdv

    def get_instant_fuel_sumo(self):
        fuel_cav = 0
        fuel_hdv = 0
        for inter_id in self.paras["traffic_graph"]:
            lane_id = []
            for direction in ["west", "south", "east", "north"]:
                node, len_lane, num_lane = self.paras["neighbors"][inter_id][direction]
                for temp in range(num_lane, 0, -1):
                    lane_id.append("%i_%i_%i" % (node, inter_id, temp))
            for i in range(len(lane_id)):
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])
                for j in range(len(cars_lane) - 1, -1, -1):
                    fuel_temp = traci.vehicle.getFuelConsumption(cars_lane[j])
                    if cars_lane[j] in self.paras["cav_ids"]["all"]:
                        fuel_cav += fuel_temp
                    else:
                        fuel_hdv += fuel_temp
        return fuel_cav, fuel_hdv

    def close_sumo_simulation(self):
        traci.close()


    def pedestrian_actuation(self):
        # Only runs in Actuated Scenario
        self.previous_phase=self.cur_phase
        self.cur_phase = int(traci.trafficlight.getPhase("1"))
        if self.cur_phase != self.previous_phase:
            self.greenTimeSofar = 0
        logic = traci.trafficlight.getCompleteRedYellowGreenDefinition("1")
        Vehicle_Phases = [0, 2, 4, 6]
        ped_demand={}
        Gp=0
        ped_phase_map = {0: ['E', "W"], 2: [], 4: ['N', 'S'], 6: []}
        for d in self.cross_map.keys():
            ped_demand[self.cross_map[d]] = len(self.network_state[1]['pedestrian_demand_current'][d])

        if self.cur_phase in Vehicle_Phases:
            for program in logic:
                if program.programID == "actuated_program":
                    for index, phase in enumerate(program.phases):
                        if self.cur_phase == index:
                            #phase.minDur=10
                            min_dur = phase.minDur
                            max_dur = phase.maxDur
                            # for dir in ped_phase_map[self.cur_phase]:
                            #     if ped_demand[dir] > 0:
                            #         minimum_green = 3.2 + self.paras['crossing_length'] / self.paras[
                            #             'ped_speed'] + 2.7 * ped_demand[dir] / (self.paras['crossing_width'] * 3.28)
                            #         print("minimum g: ", minimum_green)
                            #         minimum_green=20
                            #         Gp = max(Gp, minimum_green)
                            # #print("Gp: ", Gp)
                            # min_dur = max(Gp, min_dur)
                            # #print("min_dur: ", min_dur)
                            # phase.minDur=min_dur
                            # traci.trafficlight.setCompleteRedYellowGreenDefinition("1", program)
                            break

            self.greenTimeSofar += 0.5
            if self.next_time_to_change_to_actuated == self.greenTimeSofar:
                traci.trafficlight.setProgram("1", "actuated_program")
                traci.trafficlight.setPhase("1", self.cur_phase)
                ped_demand, current_phase_extension, opposite_phase_delay = self.checkPresentPersons()
                if not current_phase_extension or self.greenTimeSofar==max_dur: # stop the extensions
                    traci.trafficlight.setPhase("1", self.cur_phase + 1)
                self.next_time_to_change_to_actuated = 0
            ped_demand, current_phase_extension, opposite_phase_delay = self.checkPresentPersons()
            # if current_phase_extension and self.greenTimeSofar<max_dur and self.greenTimeSofar>=min_dur:
            #     print("current phase needs extension")
            #     print("status of opposite phase delays: ", opposite_phase_delay)
            if current_phase_extension and not opposite_phase_delay:  # if current phase needs extension (gap-based) and the opposite phase does not have too much delay (delay-based)
                if self.greenTimeSofar<max_dur and self.greenTimeSofar>=min_dur:
                    # stay at the current green
                    print("current phase needs extension")
                    if traci.trafficlight.getProgram("1")=="actuated_program":
                        self.next_time_to_change_to_actuated = min(max_dur, self.greenTimeSofar + 10) # note that here at least 10 seconds (minimum duration) is added to the traffic signal
                        traci.trafficlight.setProgram("1", "fixed_program")
                        traci.trafficlight.setPhase("1", self.cur_phase)



    def checkPresentPersons(self):
        current_phase_extension = False
        opposite_phase_delay = False
        ped_demand={0: 0, 2: 0, 4: 0, 6: 0}
        ped_phase_map = {0: ['E', "W"], 2: [], 4: ['N', 'S'], 6: []}
        ## the plan is to extend the phase if there are any pedestrian for the current phase but not if the opposing phase is facing too much pedestrian delay
        for walking_area in self.walking_areas:
            peds=traci.edge.getLastStepPersonIDs(walking_area)
            for ped in peds:
                next_edge = traci.person.getNextEdge(ped)
                directions= ped_phase_map[self.cur_phase]
                for dir in directions:  # check gap-based extension of current phase
                    if next_edge in self.cross_map.keys() and self.cross_map[next_edge] == dir:
                        ped_demand[self.cur_phase] += 1
                        current_phase_extension = True
                opposite_directions = ped_phase_map[(self.cur_phase+4)%8]
                for opposite_dir in opposite_directions: # check delay-based termination according to opposite phase
                    if next_edge in self.cross_map.keys() and self.cross_map[next_edge] == opposite_dir:
                        ped_demand[(self.cur_phase + 4) % 8] += 1
                        if traci.person.getWaitingTime(ped)>=25:  # if the pedestrians on the opposite phase are waiting for at least 15s, don't do the extension
                            #print("delay: ", traci.person.getWaitingTime(ped))
                            opposite_phase_delay = True

        return ped_demand, current_phase_extension, opposite_phase_delay


    # def pedestrian_movement_control(self):
    #     # only runs in Multi-Scale scenario (for now)
    #     self.cur_phase = int(traci.trafficlight.getPhase("1"))
    #     if should_update_signal:
    #         self.greenTimeSofar = 0
    #         for ped in self.stopped_peds:
    #             traci.person.setSpeed(ped, -1)
    #         self.stopped_peds=set()
    #
    #     Veh_phases_multiscale = [0, 1, 2, 3, 4, 5, 6, 7]
    #     ped_phase_map = {0:['E','W'], 1:['W'], 2:['E'], 3:[], 4:['N','S'], 5:['S'], 6:['N'], 7:[]}
    #
    #     if self.cur_phase in Veh_phases_multiscale:
    #         self.greenTimeSofar += 0.5
    #         for walking_area in self.walking_areas:
    #             peds = traci.edge.getLastStepPersonIDs(walking_area)
    #             directions = ped_phase_map[self.cur_phase]
    #             for ped in peds:
    #                 if ped not in self.stopped_peds:
    #                     next_edge = traci.person.getNextEdge(ped)
    #                     for dir in directions:
    #                         if next_edge in self.cross_map.keys() and self.cross_map[next_edge] == dir and self.greenTimeSofar>=20:
    #                             traci.person.setSpeed(ped, 0)
    #                             self.stopped_peds.add(ped)
    #
    #                             ## a problem with the behavior control is that we might assign 0 to the speed of the corresponding
    #                             # phase pedestrians but then the phase gets extended (which is unreasonable for pedestrians to stop)
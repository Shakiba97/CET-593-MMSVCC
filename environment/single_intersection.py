## The single intersection environment.
import random

import traci
import collections
import os
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
import math

from sumolib import checkBinary

from .sumo_network_reader import SumoNetworkReader
from .sumo_routes_generator import SumoRoutesGenerator


class SingleIntersection:
    def __init__(self, paras):
        self.paras = paras
        print("--------Building SUMO network...")
        network_reader = SumoNetworkReader(self.paras)
        self.network_graph = network_reader.read()
        print("--------SUMO network built.")

        print("--------Generating vehicles and routes")
        # routes_generator = SumoRoutesGenerator(self.paras)   ## for a generalized case, we assume that the route file is already given to us
        # cav_ids, hdv_ids = routes_generator.generate_routes_for_single_intersection()
        self.paras["ped_ids"] = set()
        self.paras["cav_ids"] = set()
        self.paras["hdv_ids"] = set()
        self.fuel_total_cav_external_model = 0
        self.fuel_total_hdv_external_model = 0
        self.fuel_total_cav_sumo = 0
        self.fuel_total_hdv_sumo = 0
        self.phase_list_fix_act =[]
        self.dur_list_fix_act =[]
        self.greenTimeSofar = 0
        self.next_time_to_change_to_actuated = 0
        self.stopped_peds = set()

        self.map_diag = {}
        for inter_id in self.network_graph:
            self.map_diag.setdefault(inter_id, {})
            if self.paras["ped_phasing"] == "Exclusive":
                for walkingarea in self.network_graph[inter_id]["walkingarea"]:
                    for crossing in self.network_graph[inter_id]["walkingarea"][walkingarea]["adjacent"]:
                        if self.network_graph[inter_id]["crossing"][crossing]["phasing"] == "Diagonal":
                            self.map_diag[inter_id].setdefault(walkingarea, set())
                            self.map_diag[inter_id][walkingarea].add(crossing)
            else:
                for walkingarea in self.network_graph[inter_id]["walkingarea"]:
                    for crossing in self.network_graph[inter_id]["walkingarea"][walkingarea]["adjacent"]:
                        if self.network_graph[inter_id]["crossing"][crossing]["phasing"] == "Straight":
                            self.map_diag[inter_id].setdefault(walkingarea, set())
                            self.map_diag[inter_id][walkingarea].add(crossing)

        self.cur_phase = None
        self.previous_phase = None
        self.yellow_duration=0
        self.phase_avg = None
        self.phase_ntimes = None
        self.right_turn_conflicts = {}
        print("--------Vehicles and routes generated.")

    def start_sumo(self, show_gui, control_type, network_type, volume_type):
        penetration = self.paras["penetration"]
        pedestrian_phasing = self.paras["ped_phasing"]
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
        if control_type != "multi_scale":
            model_file_name = (
                model_dir + "/" + network_type + "_" + control_type + ".sumocfg"
            )
        else:
            model_file_name = (
                model_dir + "/" + network_type + "_" + control_type + "_" + pedestrian_phasing + ".sumocfg"
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
        traffic_graph = self.network_graph
        speed_limit = self.paras["speed_limit"]
        delta_T = self.paras["delta_T"]
        delta_T_faster = self.paras["delta_T_faster"]
        num_predict_steps = self.paras["num_predict_steps"]
        right_turning_vehs = ["WS", "SE", "EN", "NW"]

        self.network_state = collections.defaultdict()
        ## Iterate through each intersection.
        for inter_id in traffic_graph:
            ## State representations for each intersection
            signal_phase = int(traci.trafficlight.getPhase(str(inter_id)))
            self.phase_list_fix_act.append(signal_phase)
            self.dur_list_fix_act.append(cur_step)

            pos_vehicles, speed_vehicles, wt_vehicles, veh_id = [], [], [], []
            arrival_times = []
            num_vehicles_max = 0
            lane_id = []
            lane_length = []
            sidewalk_id=[]
            cross_demand_current={}
            cross_demand_all_steps={}
            delta_T=self.paras["delta_T"]
            delta_T_faster=self.paras["delta_T_faster"]
            num_predict_steps=self.paras["num_predict_steps"]
            crossing_number_map={}
            for crossing in self.paras["network_graph"][inter_id]["crossing"]:
                crossing_number_map[crossing]= int(crossing[-1])+1


            for cc in self.paras["network_graph"][inter_id]["crossing"]:
                cross_demand_current[cc] = set()  ## demand on different crossings of the current step
                for i in range(1, num_predict_steps +2):
                    cross_demand_all_steps.setdefault(crossing_number_map[cc], {})
                    cross_demand_all_steps[crossing_number_map[cc]][i]=set()

            communication_range = self.paras["communication_range"]
            ## Get static information.
            for lane in traffic_graph[inter_id]["incoming_veh"]:
                lane_id.append(lane)
                lane_length.append(traffic_graph[inter_id]["incoming_veh"][lane]["length"])
            for sidewalk in traffic_graph[inter_id]["incoming_ped"]:
                sidewalk_id.append(sidewalk)
            sidewalk_id.extend(list(traffic_graph[inter_id]["walkingarea"]))

            ## Get pedestrian info (Pedestrian Demand at current step):
            for i in range(len(sidewalk_id)):
                peds_lane = traci.edge.getLastStepPersonIDs(sidewalk_id[i]) #Get the IDs of the pedestrians on the whole edge
                self.paras["ped_ids"] = self.paras["ped_ids"].union(set(peds_lane))

                for j in range(len(peds_lane)):
                    pos=traci.person.getLanePosition(peds_lane[j])
                    speed=traci.person.getSpeed(peds_lane[j])
                    waiting_time=traci.person.getWaitingTime(peds_lane[j])
                    route=traci.person.getEdges(peds_lane[j],)
                    next_edge=traci.person.getNextEdge(peds_lane[j])
                    current_edge=sidewalk_id[i]
                    # print(route)
                    # print(current_edge)
                    # edge_index_in_route = route.index(current_edge)
                    # print(peds_lane[j])
                    # print(route)
                    # print(next_edge)
                    # print(current_edge)

                    if next_edge in self.paras["network_graph"][inter_id]["crossing"]:
                        for cross in self.map_diag[inter_id][current_edge]:
                            cross_demand_current[cross].add(peds_lane[j])
                            cross_demand_all_steps[crossing_number_map[cross]][1].add(peds_lane[j])
                    elif next_edge in self.paras["network_graph"][inter_id]["walkingarea"] and speed!=0:
                        for cross in self.map_diag[inter_id][next_edge]:
                            for step in range(2, num_predict_steps + 2):
                                sidewalk_length = self.paras["network_graph"][inter_id]["incoming_ped"][current_edge]["length"]
                                if (sidewalk_length - pos) / speed <= (step - 1) * delta_T and (sidewalk_length - pos) / speed >= (step - 2) * delta_T:
                                    cross_demand_all_steps[crossing_number_map[cross]][step].add(peds_lane[j])
                                    traci.person.setColor(peds_lane[j], (255, 0, 0, 255))   ## TODO: check if this is working correctly later

            ## Count the Conflicts between right turning vehicles and pedestrians passing on the crossings:
            all_cars=traci.vehicle.getIDList()
            all_peds=traci.person.getIDList()
            for car in all_cars:
                for dir in right_turning_vehs:
                    if dir in car and traci.vehicle.getLaneID(car).startswith(":"):
                        for ped in all_peds:
                            if traci.lane.getEdgeID(traci.person.getLaneID(ped)) in self.paras["network_graph"][inter_id]["crossing"]:
                                position_car=traci.vehicle.getPosition(car)
                                position_ped=traci.person.getPosition(ped)
                                speed_car=traci.vehicle.getSpeed(car)
                                speed_ped=traci.person.getSpeed(ped)
                                if math.dist(position_car, position_ped) < 5 and math.dist(position_car, position_ped)/max(1,(speed_car+speed_ped))<2: ## if TTC less than 2 seconds
                                    self.right_turn_conflicts.setdefault(car, set())
                                    self.right_turn_conflicts[car].add(ped)

            ## Get dynamic information. Use linear interpolation to estimate HDV's state.
            for i in range(len(lane_id)):
                pos_vehicles.append([])
                speed_vehicles.append([])
                wt_vehicles.append([])
                arrival_times.append([])
                veh_id.append([])
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])
                # for car in cars_lane:
                #     if car.startswith("HDV"):
                #         self.paras["hdv_ids"] = self.paras["hdv_ids"].union({car})
                #     else:
                #         self.paras["cav_ids"] = self.paras["cav_ids"].union({car})

                for car in cars_lane:
                    if car not in self.paras["cav_ids"].union(self.paras["hdv_ids"]):
                        if random.uniform(0,1) <= self.paras["penetration"]:
                            self.paras["cav_ids"] = self.paras["cav_ids"].union({car})
                        else:
                            self.paras["hdv_ids"] = self.paras["hdv_ids"].union({car})


                # Number of vehicles in the current lane and also in the communication range.
                num_vehicles_lane = 0
                # Number of HDVs between two CAVs. Will be set to 0 when we find a CAV.
                current_num_hdv = 0
                # HDV ids between two CAVs. Will be set to empty when we find a CAV.
                current_ids_hdv = []
                for j in range(len(cars_lane) - 1, -1, -1):
                    if cars_lane[j] in self.paras["cav_ids"]:
                        ## This is a CAV, we need to first check if there are HDVs in front of it.
                        if current_num_hdv == 0:
                            ## There is no HDV between the current CAV and the CAV ahead.
                            if (
                                lane_length[i]
                                - traci.vehicle.getLanePosition(cars_lane[j])
                                <= communication_range
                            ):
                                ## The current CAV is within the communication range. Note that the value
                                ## 13 accounts for the intersection length. Simply add the CAV's state.
                                pos_vehicles[-1].append(
                                    -(
                                        lane_length[i]
                                        - traci.vehicle.getLanePosition(cars_lane[j])
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
                "pedestrian_demand_current": cross_demand_current,
                "pedestrian_demand": cross_demand_all_steps,
            }
        return self.network_state

    def apply_control_commands(
        self, should_update_signal, next_signal_phase, speed_commands
    ):
        for inter_id in self.network_graph:
            cur_phase = int(traci.trafficlight.getPhase(inter_id))
            ## Signal phase control.
            if should_update_signal:
                if next_signal_phase == -1:
                    self.yellow_duration = self.paras["delta_T_faster"]
                    traci.trafficlight.setPhase(inter_id, cur_phase + self.network_graph[inter_id]["num_phases"])
                else:
                    traci.trafficlight.setPhase(inter_id, next_signal_phase)
                    self.yellow_duration = 0
            elif next_signal_phase == -1:
                self.yellow_duration += self.paras["delta_T_faster"]
            if self.yellow_duration == self.paras['yellow_time']:
                traci.trafficlight.setPhase(inter_id, self.paras["all_red_index"])
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
        print(f"average CAV fuel consumption (external model) for {control_type} (in mg): ",
              self.fuel_total_cav_external_model / len(self.paras["cav_ids"]))
        # print(f"average fuel consumption (SUMO output) for {control_type} (in mg): ",
        #       self.fuel_total_cav_sumo / len(self.paras["cav_ids"]["all"]))
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
              len(self.paras["cav_ids"]))
        print(f"number of pedestrians passing through the specific intersection for {control_type}: ",
              len(self.paras["ped_ids"]))
        print(f"The time of simulation termination in {control_type} scenario:",step/2 )

        print("HDVs: ", len(self.paras['hdv_ids']))
        print("CAVs: ", len(self.paras['cav_ids']))
        with open(f"Results/Metrics_Results_{control_type}_scenario.txt", 'w') as file:
            file.write(f"average CAV fuel consumption for {control_type} scenario (external model) (in mg): {self.fuel_total_cav_external_model / len(self.paras['cav_ids'])}\n")
            #file.write(f"average fuel consumption for {control_type} scenario (SUMO output) (in mg): {self.fuel_total_cav_sumo / len(self.paras['cav_ids']['all'])}\n")
            file.write(f"average waiting time for {control_type} scenario (in s): {self.waiting_time_avg}\n")
            file.write(f"average time loss for {control_type} scenario (in s): {self.lost_time_avg}\n")
            file.write(f"average queue length for {control_type} scenario (in m): {self.queue_avg}\n")
            file.write(f"average pedestrian time loss for {control_type} scenario (in s): {self.lost_time_avg_ped}\n")
            file.write(f"number of right-turn conflicts between vehicles and pedestrians: {right_conflicts}\n")
            file.write(f"number of CAVs passing through the specific intersection in {control_type} scenario: {len(self.paras['cav_ids'])}\n")
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

            self.phase_avg={}
            self.phase_ntimes={}
            for phase in phase_dict.keys():
                self.phase_avg[phase]=sum(phase_dict[phase])/len(phase_dict[phase])
                self.phase_ntimes[phase]=sum(phase_dict[phase])/5
            print("average phase lengths: ", self.phase_avg)
            print("number of times each phase happened: ", self.phase_ntimes)


    def get_average_delay_endtime(self, file):
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
        self.waiting_time_avg = wt / cnt
        self.lost_time_avg = tl / cnt
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
        # print("queue cnt: ", cnt)
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
        for inter_id in self.paras["network_graph"]:
            lane_id = []
            for lane in self.paras["network_graph"][inter_id]["incoming_veh"]:
                lane_id.append(lane)
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
                    if cars_lane[j] in self.paras["cav_ids"]:
                        fuel_cav += fuel_temp
                    else:
                        fuel_hdv += fuel_temp
        return fuel_cav, fuel_hdv

    def get_instant_fuel_sumo(self):
        fuel_cav = 0
        fuel_hdv = 0
        for inter_id in self.paras["network_graph"]:
            lane_id = []
            for lane in self.paras["network_graph"][inter_id]["incoming_veh"]:
                lane_id.append(lane)
            for i in range(len(lane_id)):
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])
                for j in range(len(cars_lane) - 1, -1, -1):
                    fuel_temp = traci.vehicle.getFuelConsumption(cars_lane[j])
                    if cars_lane[j] in self.paras["cav_ids"]:
                        fuel_cav += fuel_temp
                    else:
                        fuel_hdv += fuel_temp
        return fuel_cav, fuel_hdv

    def close_sumo_simulation(self):
        traci.close()


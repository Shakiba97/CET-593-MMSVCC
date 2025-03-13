""" The MPC based agent. Currently, we only support the single_intersection scenario. """

import os
import sys
import traci

from gams import *


class MpcAgent:
    def __init__(self, paras, intersection_type):
        """Initialize the MPC agent.

        Args:
            intersection_type: Currently, we manually build one GAMS model for one intersection type
                because the geometry and signal settings are different. This intersection_type
                is used to grab the corresponding GMAS model.
        """
        self.phase_list_multi=[]
        self.duration_list_multi=[]
        self.ped_phase_map={}
        self.veh_phase_map={}
        self.paras=paras
        self.extract_signal_info()

        ## Check if the intersection type is supported.
        if intersection_type not in ["unified_four_legs_three_lanes"]:
            raise TypeError("Unknown intersection type")

        ## Set up the GAMS workspace
        self.models_dir = os.path.dirname(os.path.realpath(__file__)) + "/gams_models"
        if not os.path.exists(self.models_dir):
            os.mkdir(self.models_dir)
        #gams_dir = "/Library/Frameworks/GAMS.framework/Versions/49/Resources/"
        #self.ws = GamsWorkspace(self.models_dir, system_directory=gams_dir, debug=1)
        self.ws = GamsWorkspace(self.models_dir, debug=1)
        if paras["ped_phasing"] == "Concurrent":
            self.gams_file_slower = (self.models_dir + "/" + intersection_type + "_slower_Pedestrians.gms")
        elif paras["ped_phasing"] == "Exclusive":
            self.gams_file_slower = (self.models_dir + "/" + intersection_type + "_slower_Pedestrians (Exclusive).gms")
        self.gams_file_faster = (
            self.models_dir + "/" + intersection_type + "_faster.gms"
        )

        ## We apply the solutions for one slower scale step size after solving the MPC problems.
        ## However, the SUMO simulation step size equals to the faster scale step size. We need to
        ## know when is the next global time step to solve the MPC problems again. It is updated
        ## every time we solve the MPC problems. If the next phase is not the same as the current
        ## one, we add yellow time. Otherwise, we add a slower scale step size.
        self.next_global_step_to_re_solve_the_netwok = 0

        ## Related to self.next_global_step_to_re_solve_the_netwok, we also need to record where
        ## the current step is in the faster scale solutions. Will be reset to 1 after solving
        ## the MPC problems.
        self.current_step_in_faster_scale_solution = 1

        self.slower_scale_solution = {
            "s_vehicles_slower": [],
            "following_phases": [],
        }
        self.faster_scale_solution = {
            "a_vehicles_faster": [],
            "v_vehicles_faster": [],
            "s_vehicles_faster": [],
        }

        self.f=[]
        self.f_throughput=[]
        self.f_dist=[]
        self.f_transit=[]
        self.f_delay=[]
        self.f_ped_throughput=[]
        self.extension_steps=1
        self.is_extended = False
        self.ped_times_veh=[]


    def extract_signal_info(self):
        for inter_id in self.paras["network_graph"]:
            for logic in traci.trafficlight.getAllProgramLogics(inter_id):
                if logic.programID == "my_program":
                    for index, phase in enumerate(logic.phases):
                        all_red = True
                        signal_state = phase.state
                        controlled_links = traci.trafficlight.getControlledLinks(inter_id)
                        green_lanes = []
                        green_crossings = []
                        # Iterate over signal states and match with controlled links
                        for i, state in enumerate(signal_state[:-1]):
                            if state == 'G' and i < len(controlled_links):  # Ensure index is valid
                                all_red = False
                                for link_tuple in controlled_links[i]:  # Each entry is (incoming_lane, outgoing_lane, crossing)
                                    incoming_lane = link_tuple[0]
                                    crossing = traci.lane.getEdgeID(link_tuple[1])
                                    # Store vehicle lanes and pedestrian crossings
                                    if crossing in self.paras["network_graph"][inter_id][
                                        "crossing"].keys():  # SUMO crossings start with ":"
                                        green_crossings.append(crossing)
                                    if incoming_lane in self.paras["network_graph"][inter_id]["incoming_veh"].keys():
                                        green_lanes.append(incoming_lane)
                        if all_red:
                            self.paras["all_red_index"] = index
                        if self.paras["ped_phasing"]=="Exclusive" and len(green_crossings) == len(self.paras["network_graph"][inter_id]["crossing"].keys()):
                            self.exclusive_phase_ind=index
                        self.ped_phase_map[index] = green_crossings
                        self.veh_phase_map[index] = green_lanes


    def reset_solutions(self):
        self.slower_scale_solution = {
            "s_vehicles_slower": [],
            "following_phases": [],
        }
        self.faster_scale_solution = {
            "a_vehicles_faster": [],
            "v_vehicles_faster": [],
            "s_vehicles_faster": [],
        }
        self.vehicle_ids = []

    def gather_static_paras_single_intersection(self, inter_id, paras):
        """Gather static parameters that used for constructing the GAMS model

        Args:
            paras: The parameters defined all the way here.
        """
        self.num_phases = paras["network_graph"][inter_id]["num_phases"]
        self.num_lanes = len(paras["network_graph"][inter_id]["incoming_veh"].keys())
        self.len_lane = []
        for dict in paras["network_graph"][inter_id]["incoming_veh"].values():
            self.len_lane.append(dict['length'])
        self.num_steps = paras["num_predict_steps"]
        self.delta_T = paras["delta_T"]
        self.delta_T_faster = paras["delta_T_faster"]
        self.tau_cf = paras["tau_cf"]
        self.d_0_cf = paras["d_0_cf"]
        self.discount_ratio = paras["discount_ratio"]
        self.faster_steps_from_slower = paras["faster_steps_from_slower"]
        self.yellow_time = paras["yellow_time"]
        self.all_red_time = paras['all_red_time']
        ## Parameters used for the idm
        self.network_cav_ids = paras["cav_ids"]
        self.network_speed_limit = paras["speed_limit"]
        self.max_acc = paras["max_acc"]
        self.comf_acc = paras["comf_acc"]
        self.delta_idm = paras["delta_idm"]
        self.vehicle_length = paras["vehicle_length"]

        #self.cross_map = {':1_c0': 'N', ':1_c2': 'E', ':1_c4': 'S', ':1_c5': 'W', ':1_c3': 'NWSE', ':1_c1': 'NESW'}
        self.num_cross=len(paras["network_graph"][inter_id]["crossing"].keys())

    def set_slower_scale_constant_paras_single_intersection(self):
        """Set the constant parameters of the slower scale model"""

        j = self.db_slower.add_set("j", 1, "lanes")
        for ind in range(1, self.num_lanes + 1):
            j.add_record(str(ind))

        l = self.db_slower.add_set("l", 1, "signal phases")
        for ind in range(1, self.num_phases + 1):
            l.add_record(str(ind))

        k = self.db_slower.add_set("k", 1, "prediction steps")
        for ind in range(1, self.num_steps + 2):
            k.add_record(str(ind))

        m = self.db_slower.add_set("m", 1, 'crossings')
        for ind in range(1, self.num_cross + 1):
            m.add_record(str(ind))

        h = self.db_slower.add_parameter(
            "h", 0, "length of the time interval, i.e., the step size"
        )
        h.add_record().value = self.delta_T

        d_0 = self.db_slower.add_parameter("d_0", 0, "d_0 in the car-following model")
        d_0.add_record().value = self.d_0_cf

        k_dynamics = self.db_slower.add_parameter_dc(
            "k_dynamics",
            [k],
            "dynamic prediction steps used to calculate the differences",
        )
        k_dynamics.add_record("1").value = 0
        for ind in range(2, self.num_steps + 2):
            k_dynamics.add_record(str(ind)).value = 1

        gamma = self.db_slower.add_parameter_dc("gamma", [k], "discount ratio")
        for ind in range(1, self.num_steps + 2):
            gamma.add_record(str(ind)).value = self.discount_ratio**ind

        lane_length = self.db_slower.add_parameter_dc(
            "lane_length", [j], "length of lane j"
        )
        for ind in range(self.num_lanes):
            lane_length.add_record(str(ind + 1)).value = self.len_lane[ind]

    def set_slower_scale_dynamic_paras_single_intersection(self, intersection_state):
        """Set the dynamic parameters of the slower scale model

        Args that we will get from intersection_state:
            pos_vehicles: Positions of each vehicle, 1 * m list where m is the number of lanes,
                each element is a 1*n list where each element is the position of each vehicle.
            wt_vehicles: Waiting time, others are the same as pos_vehicles.
            cur_phase: Current signal phase, int.
            num_vehicles_max: The maximum number of existing and incoming vehicles on each lane.
                Currently, it should be equal to max([len(pos_vehicles[i]) for i in len(pos_vehicles)]).
        """

        j = self.db_slower.get_set("j")
        l = self.db_slower.get_set("l")
        m = self.db_slower.get_set("m")
        k= self.db_slower.get_set("k")

        pos_vehicles = intersection_state["pos_vehicles"]
        wt_vehicles = intersection_state["wt_vehicles"]
        cur_phase = intersection_state["signal_phase"]
        num_vehicles_max = intersection_state["num_vehicles_max"]
        pedestrian_demand=intersection_state["pedestrian_demand"]
        weights= self.paras["weight(Vehicles/Pedestrians)"]

        i = self.db_slower.add_set("i", 1, "vehicles")
        for ind in range(1, num_vehicles_max + 1):
            i.add_record(str(ind))

        vehicle_indi = self.db_slower.add_set(
            "vehicle_indi",
            2,
            "indicators showing that whether there is a vehicle i at lane j",
        )
        s_init = self.db_slower.add_parameter_dc(
            "s_init", [i, j], "initial position of vehicle i at lane j"
        )
        wt_init = self.db_slower.add_parameter_dc(
            "wt_init", [i, j], "initial waiting time of vehicle i at lane j"
        )
        p_init = self.db_slower.add_parameter_dc("p_init", [l], "initial phases")
        veh_dynamics = self.db_slower.add_parameter_dc(
            "veh_dynamics",
            [i, j],
            "indicator of vehicles to calculate the car-following distances",
        )
        ped_Demand = self.db_slower.add_parameter_dc("ped_Demand", [m, k], "Pedestrian demand for each crossing m for the next k steps of prediction horizon")

        extension_steps = self.db_slower.add_parameter(
            "Gp", 0, "length of the time interval, i.e., the step size"
        )
        extension_steps.add_record().value = self.extension_steps

        veh_weight = self.db_slower.add_parameter(
            "Wv", 0, "Weighting factor for Vehicles"
        )
        ped_weight = self.db_slower.add_parameter(
            "Wp", 0, "Weighting factor for Pedestrians"
        )

        for ind_lane in range(len(pos_vehicles)):
            for ind_vehicle in range(len(pos_vehicles[ind_lane])):
                vehicle_indi.add_record((str(ind_vehicle + 1), str(ind_lane + 1)))
                s_init.add_record((str(ind_vehicle + 1), str(ind_lane + 1))).value = (
                    pos_vehicles[ind_lane][ind_vehicle]
                )
                wt_init.add_record((str(ind_vehicle + 1), str(ind_lane + 1))).value = (
                    wt_vehicles[ind_lane][ind_vehicle]
                )
                if ind_vehicle != len(pos_vehicles[ind_lane]) - 1:
                    veh_dynamics.add_record(
                        (str(ind_vehicle + 1), str(ind_lane + 1))
                    ).value = 1

        for ind_signal in range(self.num_phases):
            if ind_signal == cur_phase:
                p_init.add_record((str(ind_signal + 1))).value = 1
            else:
                p_init.add_record((str(ind_signal + 1))).value = 0

        for ind_cross in range(self.num_cross):
            cum=0
            for ind_step in range(1, self.num_steps + 2):
                cum+=len(pedestrian_demand[ind_cross+1][ind_step])
                ped_Demand.add_record((str(ind_cross+1),str(ind_step))).value = cum ## the current demand should be considered for the q(m, 3) which is the phase after the current phase

        veh_weight.add_record().value = weights[0]
        ped_weight.add_record().value = weights[1]


    def run_gams_to_solve_slower_scale_problem_single_intersection(self):
        """Run the corresponding GAMS model to solve the slower scale problem"""
        self.model_slower = self.ws.add_job_from_file(self.gams_file_slower)
        opt_slower = self.ws.add_options()
        opt_slower.defines["gdxincname"] = self.db_slower.name
        print(self.db_slower.name)
        self.model_slower.run(opt_slower, databases=self.db_slower)
        if self.model_slower.out_db["model_status"][()].value not in [1, 2, 8]:
            raise RuntimeError("Fail to solve the slower scale problem.")

    def collect_solution_from_slower_scale_problem_single_intersection(
        self, inter_id, intersection_state
    ):
        """Collect solutions from the solower scale problem"""
        num_vehicles_max = intersection_state["num_vehicles_max"]
        p_gams = [[] for _ in range(self.num_phases)]
        s_vehicles_slower = [
            [[] for _ in range(num_vehicles_max)]
            for _ in range(intersection_state["num_lane"])
        ]
        following_phases = []
        ped_demand={}
        ped_demand_all={}
        for d in intersection_state['pedestrian_demand_current'].keys():
            ped_demand[d] = len(intersection_state['pedestrian_demand_current'][d])

        ## Collect solutions
        for item in self.model_slower.out_db["f"]:
            self.f.append(item.level)
        for item in self.model_slower.out_db["f_throughput"]:
            self.f_throughput.append(item.level)
        # print("f_throughput: ",sum(self.f_throughput)/len(self.f_throughput))
        for item in self.model_slower.out_db["f_dist"]:
            self.f_dist.append(item.level)
        #print("f_dist: ",sum(self.f_dist)/len(self.f_dist))
        for item in self.model_slower.out_db["f_transit"]:
            self.f_transit.append(item.level)
        #print("f_transit: ",sum(self.f_transit)/len(self.f_transit))
        for item in self.model_slower.out_db["f_delay"]:
            self.f_delay.append(item.level)
        # print("f_delay: ",sum(self.f_delay)/len(self.f_delay))
        for item in self.model_slower.out_db["f_ped_throughput"]:
            self.f_ped_throughput.append(item.level)
        #print("f_ped_throughput: ", sum(self.f_ped_throughput)/len(self.f_ped_throughput))
        #
        # Vehicle_cost=sum(self.f_throughput)/len(self.f_throughput) + sum(self.f_dist)/len(self.f_dist)/100 + sum(self.f_transit)/len(self.f_transit)*5 + sum(self.f_delay)/len(self.f_delay)/50
        # Pedestrian_cost=sum(self.f_ped_throughput)/len(self.f_ped_throughput)*5
        #
        # self.ped_times_veh.append(Pedestrian_cost/Vehicle_cost)
        # print("Vehicle Cost: ", sum(self.f_throughput)/len(self.f_throughput) +
        #                         sum(self.f_dist)/len(self.f_dist)/100 +
        #                         sum(self.f_transit)/len(self.f_transit)*5 +
        #                         sum(self.f_delay)/len(self.f_delay)/50)
        # print("pedestrian Cost: ",  sum(self.f_ped_throughput)/len(self.f_ped_throughput)*5)
        # print("Mean of ped times veh values: ", sum(self.ped_times_veh)/len(self.ped_times_veh))
        # print("Vehicle cost at this point: ", self.f_throughput[-1]+self.f_dist[-1]/100+self.f_transit[-1]*5+self.f_delay[-1]/50)
        # print("Pedestrian cost at this point: ", self.f_ped_throughput[-1]*5/5)
        # print("objective function at this point:", self.f[-1])

        for rec in self.model_slower.out_db["p"]:
            p_gams[int(rec.key(0)) - 1].append(round(rec.level))
        for rec in self.model_slower.out_db["s"]:
            s_vehicles_slower[int(rec.key(1)) - 1][int(rec.key(0)) - 1].append(
                rec.level
            )
        for k in range(1, self.num_steps + 1):
            ## yellow phase is identified by not finding any p_gams[:][k] = 1.
            is_yellow = True
            for l in range(self.num_phases):
                if int(p_gams[l][k]) == 1:
                    following_phases.append(l)
                    is_yellow = False
            if is_yellow:
                following_phases.append(-1)

        current_phase=traci.trafficlight.getPhase(inter_id)
        if following_phases[0] == -1:
            if self.paras["ped_phasing"]=="Exclusive" and current_phase == self.exclusive_phase_ind:
                prv_step = self.next_global_step_to_re_solve_the_netwok
                for cross in self.paras["network_graph"][inter_id]["crossing"]:
                    if self.paras["network_graph"][inter_id]["crossing"][cross]["phasing"]=="Diagonal":
                        crossing_length = self.paras["network_graph"][inter_id]["crossing"][cross]["length"]
                        all_red_clearance= int(crossing_length/self.paras['ped_speed'])
                all_red_clearance=15
                self.next_global_step_to_re_solve_the_netwok += int(
                    all_red_clearance / self.delta_T_faster
                )
                self.is_extended = False
                self.extension_steps = 1
            else:
                prv_step = self.next_global_step_to_re_solve_the_netwok
                self.next_global_step_to_re_solve_the_netwok += int(
                    (self.yellow_time+self.all_red_time) / self.delta_T_faster
                )
                self.is_extended = False
                self.extension_steps = 1
        else:
            # check for pedestrian extension:
            if self.paras["ped_phasing"]=="Exclusive":
                Gp = 0
                next = following_phases[0]

                self.extension_steps = max(1, self.extension_steps - 1)
                if self.extension_steps > 1:
                    self.is_extended = True
                # print("is previously extended: ", self.is_extended)
                if next == self.exclusive_phase_ind:
                    if self.is_extended == False:
                        for cross in self.ped_phase_map[next]:
                            if ped_demand[cross] > 0:
                                crossing_length = self.paras["network_graph"][inter_id]["crossing"][cross]["length"]
                                crossing_width = self.paras["network_graph"][inter_id]["crossing"][cross]["width"]
                                minimum_green = 3.2 + crossing_length / self.paras['ped_speed'] + 2.7 * ped_demand[cross] / (crossing_width * 3.28)
                                Gp = max(Gp, minimum_green)
                        # print("Gp: ", Gp)
                        self.extension_steps = max(int(Gp / self.delta_T) + 1, 1)
                        # print("extension steps just calculated: ", self.extension_steps)
                else:
                    Gp = 0
                    # Gp = min(Gp, 20)
                    # print("ultimate Gp: ", Gp)
                prv_step = self.next_global_step_to_re_solve_the_netwok
                self.next_global_step_to_re_solve_the_netwok += int(
                    self.delta_T / self.delta_T_faster
                )
            elif self.paras["ped_phasing"]=="Concurrent":
                # check for pedestrian extension:
                Gp = 0
                next = following_phases[0]
                after_next = following_phases[1]
                two_after_next = following_phases[2]

                self.extension_steps = max(1, self.extension_steps - 1)
                if self.extension_steps > 1:
                    self.is_extended = True
                #print("is previously extended: ", self.is_extended)
                if next != after_next and self.is_extended == False:
                    # if (next != after_next or current_phase==8) and self.is_extended==False:
                    for cross in self.ped_phase_map[next]:
                        # print(f"pedestrian demand for {dir} direction is:", ped_demand[dir])
                        if ped_demand[cross] > 0:
                            # print(f"pedestrian demand for {dir} direction is:", ped_demand[dir])
                            crossing_length = self.paras["network_graph"][inter_id]["crossing"][cross]["length"]
                            crossing_width = self.paras["network_graph"][inter_id]["crossing"][cross]["width"]
                            minimum_green = 3.2 + crossing_length / self.paras['ped_speed'] + 2.7 * ped_demand[cross] / (crossing_width * 3.28)
                            Gp = max(Gp, minimum_green)

                    #print("Gp: ", Gp)
                    Gp= 0
                    self.extension_steps = max(int(Gp / self.delta_T) + 1, 1)
                    #print("extension steps just calculated: ", self.extension_steps)
                    Gp = 0
                    # Gp = min(Gp, 20)
                    # print("ultimate Gp: ", Gp)
                prv_step = self.next_global_step_to_re_solve_the_netwok
                self.next_global_step_to_re_solve_the_netwok += int(
                    max(self.delta_T, Gp) / self.delta_T_faster
                )
        self.phase_list_multi.append(following_phases[0])
        self.duration_list_multi.append((self.next_global_step_to_re_solve_the_netwok-prv_step)*0.5)
        # if ((self.next_global_step_to_re_solve_the_netwok-prv_step)*0.5)>30:
        #     print("error")
        ## Reset the index of faster scale solutions.
        self.current_step_in_faster_scale_solution = 1
        ## Gather slower scale solution.
        self.slower_scale_solution = {
            "s_vehicles_slower": s_vehicles_slower,
            "following_phases": following_phases,
        }

    def prepare_faster_scale_input_from_slower_scale_solution(self, intersection_state):
        """Collect the critical points from the slower scale solution.
        Args:
            num_vehicles_max: From intersection_state. See set_slower_scale_dynamic_paras_single_intersection for explanation.

        Returns:
            critical_points: The critical points in the faster scale.
            pos_vehicles_point: Position of vehicles at critical points
            steps_faster: Total number of faster scale steps corresponding to the entire slower scale horizon.

        """
        num_vehicles_max = intersection_state["num_vehicles_max"]
        steps_faster = 0
        critical_points = []
        s_vehicles_point = [
            [[] for _ in range(num_vehicles_max)] for _ in range(self.num_lanes)
        ]
        temp = 0
        while temp < self.faster_steps_from_slower:
            # if self.slower_scale_solution["following_phases"][temp] == -1:
            #     steps_faster += int(self.yellow_time / self.delta_T_faster)
            # else:
            steps_faster += int( self.delta_T / self.delta_T_faster)
            critical_points.append(steps_faster)
            for i in range(self.num_lanes):
                for j in range(num_vehicles_max):
                    if self.slower_scale_solution["s_vehicles_slower"][i][j]:
                        s_vehicles_point[i][j].append(
                            self.slower_scale_solution["s_vehicles_slower"][i][j][
                                temp + 1
                            ]
                        )
            temp += 1
        return (critical_points, s_vehicles_point, steps_faster)

    def set_faster_scale_dynamic_paras_single_lane(
        self,
        critical_points,
        pos_vehicles_point,
        pos_vehicles_init,
        speed_vehicles_init,
        steps_faster,
    ):
        """Set the dynamic parameters of the faster scale model

        Args:
            critical_points: The critical points in the faster scale.
            pos_vehicles_point: Position of vehicles at critical points.
            pos_vehicles_init: Initial position of each vehicle, 1 * n list where n is the number of vehicles.
            speed_vehicles_init: Initial speed of each vehicle, 1 * n list where n is the number of vehicles.
            steps_faster: Number of steps in the fast scale.
        """

        k = self.db_faster.add_set("k", 1, "prediction steps")
        for ind in range(1, steps_faster + 2):
            k.add_record(str(ind))

        h = self.db_faster.add_parameter("h", 0, "length of the time interval")
        h.add_record().value = self.delta_T_faster

        tau = self.db_faster.add_parameter("tau", 0, "tau in the car-following model")
        tau.add_record().value = self.tau_cf - 0.4

        d_0 = self.db_faster.add_parameter("d_0", 0, "d_0 in the car-following model")
        d_0.add_record().value = self.d_0_cf - 0.1

        k_dynamics = self.db_faster.add_parameter_dc(
            "k_dynamics",
            [k],
            "dynamic prediction steps used to calculate the differences",
        )
        k_dynamics.add_record("1").value = 0
        for ind in range(2, steps_faster + 2):
            k_dynamics.add_record(str(ind)).value = 1

        k_critical_points = self.db_faster.add_parameter_dc(
            "k_critical_points",
            [k],
            "the position where there should be critical points",
        )
        for ind in range(1, steps_faster + 2):
            if ind - 1 in critical_points:
                k_critical_points.add_record(str(ind)).value = 1
            else:
                k_critical_points.add_record(str(ind)).value = 0

        i = self.db_faster.add_set("i", 1, "vehicles")
        for ind in range(1, len(pos_vehicles_init) + 1):
            i.add_record(str(ind))
        s_critical = self.db_faster.add_parameter_dc(
            "s_critical", [i, k], "critical position points of vehicle i at time k"
        )
        s_init = self.db_faster.add_parameter_dc(
            "s_init", [i], "initial position of vehicle i"
        )
        v_init = self.db_faster.add_parameter_dc(
            "v_init", [i], "initial speed of vehicle i"
        )
        veh_dynamics = self.db_faster.add_parameter_dc(
            "veh_dynamics",
            [i],
            "indicator of vehicles to calculate the car-following distances",
        )
        for ind_i in range(len(pos_vehicles_init)):
            s_init.add_record((str(ind_i + 1))).value = pos_vehicles_init[ind_i]
            v_init.add_record((str(ind_i + 1))).value = speed_vehicles_init[ind_i]
            if ind_i != len(pos_vehicles_init) - 1:
                veh_dynamics.add_record((str(ind_i + 1))).value = 1
            for ind_j in range(len(critical_points)):
                s_critical.add_record(
                    (str(ind_i + 1), str(critical_points[ind_j] + 1))
                ).value = pos_vehicles_point[ind_i][ind_j]

    def run_gams_to_solve_faster_scale_problem_single_lane(self):
        """Run the corresponding GAMS model to solve the faster scale problem"""
        # create gams model for the MINLP and solve it
        self.model_faster = self.ws.add_job_from_file(self.gams_file_faster)
        opt_faster = self.ws.add_options()
        opt_faster.defines["gdxincname"] = self.db_faster.name
        self.model_faster.run(opt_faster, databases=self.db_faster)
        if self.model_faster.out_db["model_status"][()].value not in [1, 2, 8]:
            raise RuntimeError("Fail to solve the faster scale problem.")

    def collect_solution_from_faster_scale_problem_single_lane(
        self, ind_lane, a_vehicles_faster, v_vehicles_faster, s_vehicles_faster
    ):
        """Collect solutions from the faster scale problem"""
        for rec in self.model_faster.out_db["a"]:
            a_vehicles_faster[int(rec.key(1)) - 1][ind_lane].append(rec.level)
        for rec in self.model_faster.out_db["v"]:
            v_vehicles_faster[int(rec.key(1)) - 1][ind_lane].append(rec.level)
        for rec in self.model_faster.out_db["s"]:
            s_vehicles_faster[int(rec.key(1)) - 1][ind_lane].append(rec.level)

    def solve_slower_scale_problem_single_intersection(self, inter_id, intersection_state):
        """Solve one slwoer scale problem"""
        self.db_slower = self.ws.add_database()
        self.set_slower_scale_constant_paras_single_intersection()
        self.set_slower_scale_dynamic_paras_single_intersection(intersection_state)
        self.run_gams_to_solve_slower_scale_problem_single_intersection()
        self.collect_solution_from_slower_scale_problem_single_intersection(inter_id, intersection_state
        )

    def solve_faster_scale_problem_single_intersection(
        self, critical_points, s_vehicles_point, steps_faster, intersection_state
    ):
        """Solve one faster scale problem for one intersection"""
        a_vehicles_faster = [
            [[] for _ in range(self.num_lanes)] for _ in range(steps_faster + 1)
        ]
        v_vehicles_faster = [
            [[] for _ in range(self.num_lanes)] for _ in range(steps_faster + 1)
        ]
        s_vehicles_faster = [
            [[] for _ in range(self.num_lanes)] for _ in range(steps_faster + 1)
        ]

        ## Iterate through each lane.
        for i in range(self.num_lanes):
            # Get the initial speed of vehicles on this lane.
            speed_vehicles_init = intersection_state["speed_vehicles"][i]
            if speed_vehicles_init:
                pos_vehicles_init = intersection_state["pos_vehicles"][i]
                pos_vehicles_point = s_vehicles_point[i]
                self.db_faster = self.ws.add_database()
                self.set_faster_scale_dynamic_paras_single_lane(
                    critical_points,
                    pos_vehicles_point,
                    pos_vehicles_init,
                    speed_vehicles_init,
                    steps_faster,
                )
                self.run_gams_to_solve_faster_scale_problem_single_lane()
                self.collect_solution_from_faster_scale_problem_single_lane(
                    i, a_vehicles_faster, v_vehicles_faster, s_vehicles_faster
                )
        self.faster_scale_solution = {
            "a_vehicles_faster": a_vehicles_faster,
            "v_vehicles_faster": v_vehicles_faster,
            "s_vehicles_faster": s_vehicles_faster,
        }

    def clear_redundant_gams_files(self):
        ## clear redundant gams files
        for file in os.listdir(self.models_dir):
            if file[0:6] == "_gams_":
                try:
                    os.remove(self.models_dir + "/" + file)
                except PermissionError as e:
                    print(f"Failed to delete {file}: {e}")

    def solve_single_intersection(self, inter_id, paras, intersection_state):
        self.vehicle_ids = intersection_state["vehicle_id"]
        self.gather_static_paras_single_intersection(inter_id, paras)
        self.solve_slower_scale_problem_single_intersection(inter_id, intersection_state)
        (
            critical_points,
            s_vehicles_point,
            steps_faster,
        ) = self.prepare_faster_scale_input_from_slower_scale_solution(
            intersection_state
        )
        self.solve_faster_scale_problem_single_intersection(
            critical_points, s_vehicles_point, steps_faster, intersection_state
        )

    def adjust_faster_scale_speeds_based_on_idm(self, network_state):
        """Adjust the speeds generated by the faster scale problem based on the intelligent driver model
        so that the commands satisfy safety guarantees."""

        # print(len(self.faster_scale_solution["v_vehicles_faster"]))
        # print(self.current_step_in_faster_scale_solution)
        v_vehicles_faster = self.faster_scale_solution["v_vehicles_faster"][
            self.current_step_in_faster_scale_solution
        ]
        all_vehicles=traci.vehicle.getIDList()
        vehicle_ids = self.vehicle_ids
        speed_commands = {}
        for inter_id in network_state:
            if network_state[inter_id]["num_vehicles_max"] > 0:
                for j in range(network_state[inter_id]["num_lane"]):
                    for k in range(len(v_vehicles_faster[j])):
                        veh_id = vehicle_ids[j][k]
                        if veh_id not in all_vehicles:
                            continue
                        if veh_id in self.network_cav_ids:
                            # default command is the speed limit.
                            v_command = self.network_speed_limit
                            if (
                                traci.vehicle.getLaneID(veh_id)
                                in network_state[inter_id]["lane_id"]
                            ):
                                if k > 0 and vehicle_ids[j][k - 1] in all_vehicles:
                                    front_veh_id = vehicle_ids[j][k - 1]
                                    speed_fv = traci.vehicle.getSpeed(front_veh_id)
                                    pos_fv = -(
                                        network_state[inter_id]["lane_length"][j]
                                        - traci.vehicle.getLanePosition(front_veh_id)
                                        - 13
                                    )
                                    speed_cv = traci.vehicle.getSpeed(veh_id)
                                    pos_cv = -(
                                        network_state[inter_id]["lane_length"][j]
                                        - traci.vehicle.getLanePosition(veh_id)
                                        - 13
                                    )
                                    s_star = (
                                        2
                                        + 1.5 * speed_cv
                                        + (speed_cv * (speed_cv - speed_fv))
                                        / (
                                            2
                                            * (
                                                self.max_acc
                                                * self.comf_acc
                                            )
                                            ** 0.5
                                        )
                                    )
                                    a_ref = self.max_acc * (
                                        1
                                        - (speed_cv / self.network_speed_limit)
                                        ** self.delta_idm
                                        - (
                                            s_star
                                            / (
                                                pos_fv
                                                - pos_cv
                                                - self.vehicle_length
                                            )
                                        )
                                        ** 2
                                    )
                                    v_ref = (
                                        speed_cv + a_ref * self.delta_T_faster
                                    )
                                    if v_ref >= v_vehicles_faster[j][k]:
                                        v_command = v_vehicles_faster[j][k]
                                    elif 0 < v_ref < v_vehicles_faster[j][k]:
                                        v_command = v_ref
                                    else:
                                        v_command = 0
                                else:
                                    v_command = v_vehicles_faster[j][k]
                            speed_commands[veh_id] = v_command
        return speed_commands

    def get_control_commands(self, network_state, cur_step):
        """Get control commands from the agent.

        Args:
            paras: Parameters that has been built all the way.
            network_state: See the environment for details.
            cur_step: The current step.

        Returns:
            should_update_signal: Whether we should update the signal phase. Set to be true
                each time we solve new MPC problems.
            next_signal_phase: The next signal phase that we can use if should_update_signal = True.
            speed_commands: Speeds that need to be applied to all CAVs in the network. A dictionary
                with keys as the vehicles ids and values as the adjusted speed commands.
        """

        if len(self.paras["network_graph"]) != 1:
            raise TypeError("The MPC agent only supports single_intersection scenario.")
        should_update_signal = False
        if cur_step >= self.next_global_step_to_re_solve_the_netwok:
            print("--------It's time to update MPC solutions!")
            self.reset_solutions()
            for inter_id in network_state:
                if network_state[inter_id]["num_vehicles_max"] > 0:
                    print("--------Solve intersection: ", inter_id)
                    self.solve_single_intersection(inter_id, self.paras, network_state[inter_id])
                    should_update_signal = True
        else:
            self.current_step_in_faster_scale_solution += 1

        if self.vehicle_ids:
            speed_commands = self.adjust_faster_scale_speeds_based_on_idm(network_state)
            return (
                self.next_global_step_to_re_solve_the_netwok,
                self.phase_list_multi,
                self.duration_list_multi,
                should_update_signal,
                self.slower_scale_solution["following_phases"][0],
                speed_commands,
            )
        else:
            return (self.next_global_step_to_re_solve_the_netwok, self.phase_list_multi, self.duration_list_multi, False, -1, {})

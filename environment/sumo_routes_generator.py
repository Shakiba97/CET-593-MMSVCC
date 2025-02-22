import random
import os


class SumoRoutesGenerator:
    def __init__(self, paras):
        self.paras = paras

    def generate_routes_for_single_intersection(self):
        """Generate vehicles and corresponding routes.

        Returns:
            cav_ids and hdv_ids, each of which contains four types of information.
                we: Ids of all west-to-east vehicles.
                ew: Ids of all east-to-west vehicles.
                others: Ids of all non we and ew vehicles.
                all: Ids of all vehicles.
        """
        N = (
            self.paras["simulation_steps"] * self.paras["delta_T"]
        )  # number of time steps

        penetration = self.paras["penetration"]
        depart_rate = self.paras["depart_rate"]
        left_right_ratio = self.paras["left_right_ratio"]
        seed = self.paras["random_seed"]
        random.seed(seed)  # make tests reproducible

        cav_WEid = set()
        cav_EWid = set()
        cav_others = set()

        hdv_WEid = set()
        hdv_EWid = set()
        hdv_others = set()
        # demand per second from different directions
        model_dir = os.path.dirname(os.path.realpath(__file__)) + "/network_model"
        if not os.path.exists(model_dir):
            os.mkdir(model_dir)
        file_name = model_dir + "/single_intersection.rou.xml"
        poisson_gamma=0.04 #average number of pedestrian at each second
        # high:0.08 medium=0.08*0.5 low=0.01
        scenario = "Asymmetric" # Asymmetric or Symmetric
        if scenario == "Asymmetric":
            division=[6,3,2,1]
        else:
            division=[3,3,3,3]

        with open(file_name, "w") as routes:
            print(
                f"""<routes>
            <vType id="cars" accel="3" decel="5" sigma="1" length="5" minGap="2" maxSpeed="13" \
            guiShape="passenger"/>

            <route id="WE" edges="2_1 1_4" />
            <route id="WN" edges="2_1 1_5" />
            <route id="WS" edges="2_1 1_3" />
            <route id="SN" edges="3_1 1_5" />
            <route id="SW" edges="3_1 1_2" />
            <route id="SE" edges="3_1 1_4" />
            <route id="EW" edges="4_1 1_2" />
            <route id="ES" edges="4_1 1_3" />
            <route id="EN" edges="4_1 1_5" />
            <route id="NS" edges="5_1 1_3" />
            <route id="NE" edges="5_1 1_4" />
            <route id="NW" edges="5_1 1_2" />

            <route id="NS_ped" edges="5_1 :1_w0 :1_c5 :1_w3 1_3" />
            <route id="SN_ped" edges="3_1 :1_w2 :1_c2 :1_w1 1_5" />
            <route id="WE_ped" edges="2_1 :1_w3 :1_c4 :1_w2 1_4" />
            <route id="EW_ped" edges="4_1 :1_w1 :1_c0 :1_w0 1_2" />
            <route id="WN_ped_wn" edges="2_1 :1_w3 :1_c5 :1_w0 :1_c0 :1_w1 1_5" />
            <route id="WN_ped_se" edges="2_1 :1_w3 :1_c4 :1_w2 :1_c2 :1_w1 1_5" />
            <route id="SW_ped_sw" edges="3_1 :1_w2 :1_c4 :1_w3 :1_c5 :1_w0 1_2" />
            <route id="SW_ped_en" edges="3_1 :1_w2 :1_c2 :1_w1 :1_c0 :1_w0 1_2" />
            <route id="ES_ped_es" edges="4_1 :1_w1 :1_c2 :1_w2 :1_c4 :1_w3 1_3" />
            <route id="ES_ped_nw" edges="4_1 :1_w1 :1_c0 :1_w0 :1_c5 :1_w3 1_3" />
            <route id="NE_ped_ne" edges="5_1 :1_w0 :1_c0 :1_w1 :1_c2 :1_w2 1_4" />
            <route id="NE_ped_ws" edges="5_1 :1_w0 :1_c5 :1_w3 :1_c4 :1_w2 1_4" />
            <route id="WN_ped_diag" edges="2_1 :1_w3 :1_c1 :1_w1 1_5" />
            <route id="SW_ped_diag" edges="3_1 :1_w2 :1_c3 :1_w0 1_2" />
            <route id="ES_ped_diag" edges="4_1 :1_w1 :1_c1 :1_w3 1_3" />
            <route id="NE_ped_diag" edges="5_1 :1_w0 :1_c3 :1_w2 1_4" />

            
            <personFlow id="person_WE_s" begin="0" end="900" period="exp({poisson_gamma*division[1]/2})" departPos="100">
                <walk route="WE_ped"/>
             </personFlow>
            <personFlow id="person_EW_n" begin="0" end="900" period="exp({poisson_gamma*division[3]/2})" departPos="100">
                <walk route="EW_ped"/>
            </personFlow>
            <personFlow id="person_NS_w" begin="0" end="900" period="exp({poisson_gamma*division[0]/2})" departPos="100">
                <walk route="NS_ped"/>
            </personFlow>
            <personFlow id="person_SN_e" begin="0" end="900" period="exp({poisson_gamma*division[2]/2})" departPos="100">
                <walk route="SN_ped"/>
            </personFlow>   
            <personFlow id="person_WN_diag" begin="0" end="900" period="exp({poisson_gamma*1.5})" departPos="100">
                <walk from="2_1" to="1_5"/>
            </personFlow>
            <personFlow id="person_SW_diag" begin="0" end="900" period="exp({poisson_gamma*1.5})" departPos="100">
                <walk from="3_1" to="1_2"/>
            </personFlow>   
            <personFlow id="person_ES_diag" begin="0" end="900" period="exp({poisson_gamma*1.5})" departPos="100">
                <walk from="4_1" to="1_3"/>
            </personFlow>       
            <personFlow id="person_NE_diag" begin="0" end="900" period="exp({poisson_gamma*1.5})" departPos="100">
                <walk from="5_1" to="1_4"/>
            </personFlow>      
   
            """,
                file=routes,
            )

            vehNr_WE = 0
            vehNr_EW = 0
            vehNr_others = 0
            for i in range(N):
                pEW = (
                    depart_rate["vol_ew_main"][i // depart_rate["time_interval"]] / 3600
                )
                pWE = (
                    depart_rate["vol_we_main"][i // depart_rate["time_interval"]] / 3600
                )
                pNS = (
                    depart_rate["vol_ns_main"][i // depart_rate["time_interval"]] / 3600
                )
                pSN = (
                    depart_rate["vol_sn_main"][i // depart_rate["time_interval"]] / 3600
                )
                pES = pEW * left_right_ratio
                pEN = pEW * left_right_ratio
                pSW = pEW * left_right_ratio
                pSE = pEW * left_right_ratio
                pWN = pWE * left_right_ratio
                pWS = pWE * left_right_ratio
                pNE = pWE * left_right_ratio
                pNW = pWE * left_right_ratio
                if random.uniform(0, 1) < pWE:
                    if random.uniform(0, 1) < penetration:
                        cav_WEid.add("WE_%i" % vehNr_WE)
                        print(
                            '    <vehicle id="WE_%i" type="cars" route="WE" depart="%i" departLane="2" color="0,1,0"/>'
                            % (vehNr_WE, i),
                            file=routes,
                        )
                    else:
                        hdv_WEid.add("WE_%i" % vehNr_WE)
                        print(
                            '    <vehicle id="WE_%i" type="cars" route="WE" depart="%i" departLane="2"/>'
                            % (vehNr_WE, i),
                            file=routes,
                        )
                    vehNr_WE += 1
                if random.uniform(0, 1) < pEW:
                    if random.uniform(0, 1) < penetration:
                        cav_EWid.add("EW_%i" % vehNr_EW)
                        print(
                            '    <vehicle id="EW_%i" type="cars" route="EW" depart="%i" departLane="2" color="0,1,0"/>'
                            % (vehNr_EW, i),
                            file=routes,
                        )
                    else:
                        hdv_EWid.add("EW_%i" % vehNr_EW)
                        print(
                            '    <vehicle id="EW_%i" type="cars" route="EW" depart="%i" departLane="2" />'
                            % (vehNr_EW, i),
                            file=routes,
                        )
                    vehNr_EW += 1

                if random.uniform(0, 1) < pWN:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("WN_%i" % vehNr_others)
                        print(
                            '    <vehicle id="WN_%i" type="cars" route="WN" depart="%i" departLane="3" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("WN_%i" % vehNr_others)
                        print(
                            '    <vehicle id="WN_%i" type="cars" route="WN" depart="%i" departLane="3" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pWS:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("WS_%i" % vehNr_others)
                        print(
                            '    <vehicle id="WS_%i" type="cars" route="WS" depart="%i" departLane="1" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("WS_%i" % vehNr_others)
                        print(
                            '    <vehicle id="WS_%i" type="cars" route="WS" depart="%i" departLane="1" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pES:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("ES_%i" % vehNr_others)
                        print(
                            '    <vehicle id="ES_%i" type="cars" route="ES" depart="%i" departLane="3" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("ES_%i" % vehNr_others)
                        print(
                            '    <vehicle id="ES_%i" type="cars" route="ES" depart="%i" departLane="3" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pEN:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("EN_%i" % vehNr_others)
                        print(
                            '    <vehicle id="EN_%i" type="cars" route="EN" depart="%i" departLane="1" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("EN_%i" % vehNr_others)
                        print(
                            '    <vehicle id="EN_%i" type="cars" route="EN" depart="%i" departLane="1" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pNS:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("NS_%i" % vehNr_others)
                        print(
                            '    <vehicle id="NS_%i" type="cars" route="NS" depart="%i" departLane="2" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("NS_%i" % vehNr_others)
                        print(
                            '    <vehicle id="NS_%i" type="cars" route="NS" depart="%i" departLane="2" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pNE:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("NE_%i" % vehNr_others)
                        print(
                            '    <vehicle id="NE_%i" type="cars" route="NE" depart="%i" departLane="3" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("NE_%i" % vehNr_others)
                        print(
                            '    <vehicle id="NE_%i" type="cars" route="NE" depart="%i" departLane="3" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pNW:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("NW_%i" % vehNr_others)
                        print(
                            '    <vehicle id="NW_%i" type="cars" route="NW" depart="%i" departLane="1" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("NW_%i" % vehNr_others)
                        print(
                            '    <vehicle id="NW_%i" type="cars" route="NW" depart="%i" departLane="1" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pSN:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("SN_%i" % vehNr_others)
                        print(
                            '    <vehicle id="SN_%i" type="cars" route="SN" depart="%i" departLane="2" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("SN_%i" % vehNr_others)
                        print(
                            '    <vehicle id="SN_%i" type="cars" route="SN" depart="%i" departLane="2" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pSW:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("SW_%i" % vehNr_others)
                        print(
                            '    <vehicle id="SW_%i" type="cars" route="SW" depart="%i" departLane="3" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("SW_%i" % vehNr_others)
                        print(
                            '    <vehicle id="SW_%i" type="cars" route="SW" depart="%i" departLane="3" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
                if random.uniform(0, 1) < pSE:
                    if random.uniform(0, 1) < penetration:
                        cav_others.add("SE_%i" % vehNr_others)
                        print(
                            '    <vehicle id="SE_%i" type="cars" route="SE" depart="%i" departLane="1" color="0,1,0"/>'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    else:
                        hdv_others.add("SE_%i" % vehNr_others)
                        print(
                            '    <vehicle id="SE_%i" type="cars" route="SE" depart="%i" departLane="1" />'
                            % (vehNr_others, i),
                            file=routes,
                        )
                    vehNr_others += 1
            print("</routes>", file=routes)
            cav_ids = {
                "we": cav_WEid,
                "ew": cav_EWid,
                "others": cav_others,
                "all": set.union(cav_WEid, cav_EWid, cav_others),
            }
            hdv_ids = {
                "we": hdv_WEid,
                "ew": hdv_EWid,
                "others": hdv_others,
                "all": set.union(hdv_WEid, hdv_EWid, hdv_others),
            }
            # total_number = vehNr_WE + vehNr_WN + vehNr_WS + vehNr_SN + vehNr_SE + vehNr_SW + vehNr_EW + vehNr_EN + vehNr_ES + vehNr_NS + vehNr_NE + vehNr_NW
        return cav_ids, hdv_ids

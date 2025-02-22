## The entry point of the Multiscale SVCC algorithm.
import traci
from configs.set_parameters import set_parameters
from environment.single_intersection import SingleIntersection
from agent.mpc_agent import MpcAgent


def main(network_type, volume_type, control_type):
    print("----Get parameters...")
    paras = set_parameters(network_type, volume_type)

    print("----Build single intersection environments...")
    env_single_intersection = SingleIntersection(paras)

    print("----Start SUMO...")
    env_single_intersection.start_sumo(True, control_type, network_type, volume_type)

    print("----Initializing the agent...")
    agent_unified_four_legs_three_lanes = MpcAgent("unified_four_legs_three_lanes")
    agent_unified_four_legs_three_lanes.clear_redundant_gams_files()

    phase_list_multi=[]
    duration_list_multi=[]
    step = 0
    while env_single_intersection.is_active():

        # print("----Get network state")
        network_state = env_single_intersection.get_state_cur_intersection(step)

        if control_type == "multi_scale":


            # print("position of vehicles: ", network_state[1]['pos_vehicles'])
            # print("speed of vehicles: ", network_state[1]['speed_vehicles'])
            # print("arrival times: ", network_state[1]['arrival_times'])
            # print("signal phase: ", network_state[1]['signal_phase'])
            # print("pedestrian demand: ", network_state[1]['pedestrian_demand'])

            # print("----Get control commands from the agent")
            (next_global_step_to_re_solve_the_network, phase_list_multi, duration_list_multi, should_update_signal, next_signal_phase, speed_commands) = (
                agent_unified_four_legs_three_lanes.get_control_commands(
                    paras, network_state, step
                )
            )

            # print("should_update_signal: ", should_update_signal)
            # print("next_signal_phase: ", next_signal_phase)
            # print("speed_commands: ", speed_commands)
            #print("time (s): ", step*0.5)

            # print("----Apply the control commands to the environment")
            env_single_intersection.apply_control_commands(
                should_update_signal, next_signal_phase, speed_commands
            )

        elif control_type == "actuated":
             env_single_intersection.pedestrian_actuation()



            #env_single_intersection.pedestrian_movement_control()
        #print("----Calculate extra metrics")
        env_single_intersection.calculate_extra_metrics()
        #print("----Move one step forward")
        env_single_intersection.move_one_step_forward()
        step += 1

    env_single_intersection.close_sumo_simulation()
    env_single_intersection.performance_results(phase_list_multi, duration_list_multi, network_type, volume_type, control_type, step)
    agent_unified_four_legs_three_lanes.clear_redundant_gams_files()


if __name__ == "__main__":
    main("single_intersection", "symmetric", "actuated")
    # control_type: "multi_scale", "actuated", "fixed_time"
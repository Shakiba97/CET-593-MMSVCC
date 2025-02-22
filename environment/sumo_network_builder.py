import collections
import os


class SumoNetworkBuilder:
    def __init__(self, paras):
        self.paras = paras

    def build(self):
        traffic_graph = self.paras["traffic_graph"]
        network_type = self.paras["network_type"]
        neighbors = collections.defaultdict(dict)
        visited_node = set()
        visited_edge = set()

        model_dir = os.path.dirname(os.path.realpath(__file__)) + "/network_model"
        if not os.path.exists(model_dir):
            os.mkdir(model_dir)

        if network_type in ["single_intersection", "corridor", "4_4_network"]:
            file_name = model_dir + "/" + network_type
        else:
            raise TypeError("unknown traffic network")

        with open(file_name + ".nod.xml", "w") as nodes, open(
            file_name + ".edg.xml", "w"
        ) as edges:
            print("""<?xml version="1.0" encoding="UTF-8"?>""", file=nodes)
            print("""<nodes>""", file=nodes)
            print("""<?xml version="1.0" encoding="UTF-8"?>""", file=edges)
            print("""<edges>""", file=edges)
            for inter in traffic_graph:
                print(
                    """   <node id="%i" x="%f" y="%f"  type="traffic_light"/>"""
                    % (
                        inter,
                        traffic_graph[inter]["pos"][0],
                        traffic_graph[inter]["pos"][1],
                    ),
                    file=nodes,
                )
                visited_node.add(traffic_graph[inter]["pos"])
                for node, num_lane in traffic_graph[inter]["adj"]:
                    if (inter, node) not in visited_edge:
                        print(
                            """   <edge id="%i_%i" from="%i" to="%i" priority="10" numLanes="%i" speed="20" />"""
                            % (inter, node, inter, node, num_lane),
                            file=edges,
                        )
                        visited_edge.add((inter, node))
                    if (node, inter) not in visited_edge:
                        print(
                            """   <edge id="%i_%i" from="%i" to="%i" priority="10" numLanes="%i" speed="20" />"""
                            % (node, inter, node, inter, num_lane),
                            file=edges,
                        )
                        visited_edge.add((node, inter))
                    x_1, y_1 = traffic_graph[inter]["pos"]
                    x_2, y_2 = traffic_graph[node]["pos"]
                    if x_1 == x_2:
                        if y_1 > y_2:
                            neighbors[inter]["south"] = (node, y_1 - y_2, num_lane)
                            neighbors[node]["north"] = (inter, y_1 - y_2, num_lane)
                        else:
                            neighbors[inter]["north"] = (node, y_2 - y_1, num_lane)
                            neighbors[node]["south"] = (inter, y_2 - y_1, num_lane)
                    else:
                        if x_1 > x_2:
                            neighbors[inter]["west"] = (node, x_1 - x_2, num_lane)
                            neighbors[node]["east"] = (inter, x_1 - x_2, num_lane)
                        else:
                            neighbors[inter]["east"] = (node, x_2 - x_1, num_lane)
                            neighbors[node]["west"] = (inter, x_2 - x_1, num_lane)
            for inter in traffic_graph:
                dx, dy = [], []
                if "west" not in neighbors[inter]:
                    dx.append(
                        -traffic_graph[inter]["distance_from_upstream_intersections"]
                    )
                    dy.append(0)
                if "south" not in neighbors[inter]:
                    dx.append(0)
                    dy.append(
                        -traffic_graph[inter]["distance_from_upstream_intersections"]
                    )
                if "east" not in neighbors[inter]:
                    dx.append(
                        traffic_graph[inter]["distance_from_upstream_intersections"]
                    )
                    dy.append(0)
                if "north" not in neighbors[inter]:
                    dx.append(0)
                    dy.append(
                        traffic_graph[inter]["distance_from_upstream_intersections"]
                    )
                # dx, dy = [0, traffic_graph[inter]["distance_from_upstream_intersections"], 0, -traffic_graph[inter]["distance_from_upstream_intersections"]], [traffic_graph[inter]["distance_from_upstream_intersections"], 0, -traffic_graph[inter]["distance_from_upstream_intersections"], 0]
                for k in range(len(dx)):
                    x, y = (
                        traffic_graph[inter]["pos"][0] + dx[k],
                        traffic_graph[inter]["pos"][1] + dy[k],
                    )
                    # if (x, y) not in visited_node:
                    print(
                        """   <node id="%i" x="%f" y="%f"  type="priority"/>"""
                        % (len(visited_node) + 1, x, y),
                        file=nodes,
                    )
                    print(
                        """   <edge id="%i_%i" from="%i" to="%i" priority="10" numLanes="%i" speed="20" />"""
                        % (
                            len(visited_node) + 1,
                            inter,
                            len(visited_node) + 1,
                            inter,
                            traffic_graph[inter]["num_lanes_each_road"],
                        ),
                        file=edges,
                    )
                    print(
                        """   <edge id="%i_%i" from="%i" to="%i" priority="10" numLanes="%i" speed="20" />"""
                        % (
                            inter,
                            len(visited_node) + 1,
                            inter,
                            len(visited_node) + 1,
                            traffic_graph[inter]["num_lanes_each_road"],
                        ),
                        file=edges,
                    )
                    visited_node.add((x, y))
                    if dy[k] > 0:
                        neighbors[inter]["north"] = (
                            len(visited_node),
                            traffic_graph[inter][
                                "distance_from_upstream_intersections"
                            ],
                            traffic_graph[inter]["num_lanes_each_road"],
                        )
                        neighbors[len(visited_node)]["south"] = (
                            inter,
                            traffic_graph[inter][
                                "distance_from_upstream_intersections"
                            ],
                            traffic_graph[inter]["num_lanes_each_road"],
                        )
                    elif dx[k] > 0:
                        neighbors[inter]["east"] = (
                            len(visited_node),
                            traffic_graph[inter][
                                "distance_from_upstream_intersections"
                            ],
                            traffic_graph[inter]["num_lanes_each_road"],
                        )
                        neighbors[len(visited_node)]["west"] = (
                            inter,
                            traffic_graph[inter][
                                "distance_from_upstream_intersections"
                            ],
                            traffic_graph[inter]["num_lanes_each_road"],
                        )
                    elif dy[k] < 0:
                        neighbors[inter]["south"] = (
                            len(visited_node),
                            traffic_graph[inter][
                                "distance_from_upstream_intersections"
                            ],
                            traffic_graph[inter]["num_lanes_each_road"],
                        )
                        neighbors[len(visited_node)]["north"] = (
                            inter,
                            traffic_graph[inter][
                                "distance_from_upstream_intersections"
                            ],
                            traffic_graph[inter]["num_lanes_each_road"],
                        )
                    else:
                        neighbors[inter]["west"] = (
                            len(visited_node),
                            traffic_graph[inter][
                                "distance_from_upstream_intersections"
                            ],
                            traffic_graph[inter]["num_lanes_each_road"],
                        )
                        neighbors[len(visited_node)]["east"] = (
                            inter,
                            traffic_graph[inter][
                                "distance_from_upstream_intersections"
                            ],
                            traffic_graph[inter]["num_lanes_each_road"],
                        )
            print("""</nodes>""", file=nodes)
            print("""</edges>""", file=edges)

        ## connection
        with open(file_name + ".con.xml", "w") as conections:
            print("""<?xml version="1.0" encoding="iso-8859-1"?>""", file=conections)
            print("""<connections>""", file=conections)
            for inter in traffic_graph:
                west, east, south, north = (
                    neighbors[inter]["west"][0],
                    neighbors[inter]["east"][0],
                    neighbors[inter]["south"][0],
                    neighbors[inter]["north"][0],
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="2" toLane="2"/>"""
                    % (west, inter, inter, north),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="2" toLane="1"/>"""
                    % (west, inter, inter, north),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="1" toLane="2"/>"""
                    % (west, inter, inter, east),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="1" toLane="1"/>"""
                    % (west, inter, inter, east),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="0" toLane="0"/>"""
                    % (west, inter, inter, east),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="0" toLane="0"/>"""
                    % (west, inter, inter, south),
                    file=conections,
                )

                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="2" toLane="2"/>"""
                    % (south, inter, inter, west),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="2" toLane="1"/>"""
                    % (south, inter, inter, west),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="1" toLane="2"/>"""
                    % (south, inter, inter, north),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="1" toLane="1"/>"""
                    % (south, inter, inter, north),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="0" toLane="0"/>"""
                    % (south, inter, inter, north),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="0" toLane="0"/>"""
                    % (south, inter, inter, east),
                    file=conections,
                )

                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="2" toLane="2"/>"""
                    % (east, inter, inter, south),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="2" toLane="1"/>"""
                    % (east, inter, inter, south),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="1" toLane="2"/>"""
                    % (east, inter, inter, west),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="1" toLane="1"/>"""
                    % (east, inter, inter, west),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="0" toLane="0"/>"""
                    % (east, inter, inter, west),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="0" toLane="0"/>"""
                    % (east, inter, inter, north),
                    file=conections,
                )

                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="2" toLane="2"/>"""
                    % (north, inter, inter, east),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="2" toLane="1"/>"""
                    % (north, inter, inter, east),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="1" toLane="2"/>"""
                    % (north, inter, inter, south),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="1" toLane="1"/>"""
                    % (north, inter, inter, south),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="0" toLane="0"/>"""
                    % (north, inter, inter, south),
                    file=conections,
                )
                print(
                    """   <connection from="%i_%i" to="%i_%i" fromLane="0" toLane="0"/>"""
                    % (north, inter, inter, west),
                    file=conections,
                )
            print("""</connections>""", file=conections)

        ## traffic signal
        with open(file_name + ".add.xml", "w") as additionals:
            print("""<additional>""", file=additionals)
            for inter in traffic_graph:
                print(
                    """   <tlLogic id="%i" type="static" programID="my_program" offset="15">"""
                    % (inter),
                    file=additionals,
                )
                print(
                    """
            <phase duration="30" state="gGGrrrrrgGGrrrrrrGrGrr"/>
            <phase duration="10" state="gGGGgrrrrrrrrrrrrrrGrr"/>
            <phase duration="10" state="rrrrrrrrgGGGgrrrrGrrrr"/>
            <phase duration="10" state="rrrGgrrrrrrGgrrrrrrrrr"/>
            <phase duration="30" state="rrrrgGGrrrrrgGGrGrGrrr"/>
            <phase duration="10" state="grrrrrrrrrrrgGGGrrGrrr"/>
            <phase duration="10" state="rrrrgGGGgrrrrrrrGrrrrr"/>
            <phase duration="10" state="grrrrrrGgrrrrrrGrrrrrr"/>
            <phase duration="10" state="rrrrrrrrrrrrrrrrGGGGGG"/>
            <phase duration="5" state="yyyrrrrryyyrrrrrrrrrrr"/>
            <phase duration="5" state="yyyyrrrrrrrrrrrrrrrrrr"/>
            <phase duration="5" state="rrrrrrrryyyyyrrrrrrrrr"/>
            <phase duration="5" state="rrryyrrrrrryyrrrrrrrrr"/>
            <phase duration="5" state="rrrryyyrrrrryyyrrrrrrr"/>
            <phase duration="5" state="yrrrrrrrrrrryyyyrrrrrr"/>
            <phase duration="5" state="rrrryyyyyrrrrrrrrrrrrr"/>
            <phase duration="5" state="yrrrrrryyrrrrrryrrrrrr"/>
            <phase duration="20" state="rrrrrrrrrrrrrrrrrrrrrr"/>
                """,
                    file=additionals,
                )
                print("""   </tlLogic>""", file=additionals)
            print("""</additional>""", file=additionals)

        ind = 0
        with open(file_name + ".add_fixed_time.xml", "w") as additionals:
            print("""<additional>""", file=additionals)
            for inter in traffic_graph:
                print(
                    """   <tlLogic id="%i" type="static" programID="my_program" offset="%i">"""
                    % (
                        inter,
                        int(
                            traffic_graph[inter]["distance_from_upstream_intersections"]
                            * ind
                            / 13
                        ),
                    ),
                    file=additionals,
                )
                print(
                    """
            <phase duration="30" state="gGGrrrrrgGGrrrrrrGrGrr"/>
            <phase duration="5" state="yyyrrrrryyyrrrrrrrrrrr"/>
            <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
            <phase duration="10" state="rrrGgrrrrrrGgrrrrrrrrr"/>
            <phase duration="5" state="rrryyrrrrrryyrrrrrrrrr"/>
            <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
            <phase duration="30" state="rrrrgGGrrrrrgGGrGrGrrr"/>
            <phase duration="5" state="rrrryyyrrrrryyyrrrrrrr"/>
            <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
            <phase duration="10" state="grrrrrrGgrrrrrrGrrrrrr"/>
            <phase duration="5" state="yrrrrrryyrrrrrryrrrrrr"/>
            <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
                """,
                    file=additionals,
                )
                print("""   </tlLogic>""", file=additionals)
                ind += 1
            print("""</additional>""", file=additionals)

        with open(file_name + ".add_actuated.xml", "w") as additionals:
            print("""<additional>""", file=additionals)
            for inter in traffic_graph:
                print(
                    """   <tlLogic id="%i" type="actuated" programID="actuated_program" offset="%i">"""
                    % (inter, 15 * inter),
                    file=additionals,
                )
                print(
                    """
            <param key="max-gap" value="3.0"/>
            <param key="detector-gap" value="2.0"/>
            <param key="show-detectors" value="false"/>
            <param key="file" value="NULL"/>
            <param key="freq" value="300"/>

            <phase duration="33" minDur="20" maxDur="45"  state="gGGrrrrrgGGrrrrrrGrGrr"/>
            <phase duration="5" state="yyyrrrrryyyrrrrrrrrrrr"/>
            <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
            <phase duration="33" minDur="10" maxDur="45"  state="rrrGgrrrrrrGgrrrrrrrrr"/>
            <phase duration="5" state="rrryyrrrrrryyrrrrrrrrr"/>
            <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
            <phase duration="33" minDur="20" maxDur="45"  state="rrrrgGGrrrrrgGGrGrGrrr"/>
            <phase duration="5" state="rrrryyyrrrrryyyrrrrrrr"/>
            <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
            <phase duration="33" minDur="10" maxDur="45"  state="grrrrrrGgrrrrrrGrrrrrr"/>
            <phase duration="5" state="yrrrrrryyrrrrrryrrrrrr"/>
            <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
                """,
                    file=additionals,
                )
                print("""   </tlLogic>""", file=additionals)

                for inter in traffic_graph:
                    print(
                        """   <tlLogic id="%i" type="static" programID="fixed_program" offset="%i">"""
                        % (
                            inter,
                            int(
                                traffic_graph[inter]["distance_from_upstream_intersections"]
                                * ind
                                / 13
                            ),
                        ),
                        file=additionals,
                    )
                    print(
                        """
                <phase duration="30" state="gGGrrrrrgGGrrrrrrGrGrr"/>
                <phase duration="5" state="yyyrrrrryyyrrrrrrrrrrr"/>
                <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
                <phase duration="10" state="rrrGgrrrrrrGgrrrrrrrrr"/>
                <phase duration="5" state="rrryyrrrrrryyrrrrrrrrr"/>
                <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
                <phase duration="30" state="rrrrgGGrrrrrgGGrGrGrrr"/>
                <phase duration="5" state="rrrryyyrrrrryyyrrrrrrr"/>
                <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
                <phase duration="10" state="grrrrrrGgrrrrrrGrrrrrr"/>
                <phase duration="5" state="yrrrrrryyrrrrrryrrrrrr"/>
                <phase duration="2" state="rrrrrrrrrrrrrrrrrrrrrr"/>
                    """,
                        file=additionals,
                    )
                    print("""   </tlLogic>""", file=additionals)
                    ind += 1





            print("""</additional>""", file=additionals)

        ## build .net file
        # os.system(
        #     "netconvert --node-files "
        #     + file_name
        #     + ".nod.xml --edge-files "
        #     + file_name
        #     + ".edg.xml --connection-files "
        #     + file_nameS
        #     + ".con.xml -o "
        #     + file_name
        #     + ".net.xml"
        # )

        return neighbors

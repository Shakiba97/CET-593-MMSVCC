import collections
import os
import xml.etree.ElementTree as ET

class SumoNetworkReader:
    def __init__(self, paras):
        self.paras = paras
        self.network_graph = collections.defaultdict()
        self.paras["network_graph"] = self.network_graph
    def read(self):
        data_dir = os.path.dirname(os.path.realpath(__file__)) + "/network_model/"
        network_file = (data_dir + "single_intersection_pedestrian_X.net.xml")
        if self.paras["ped_phasing"] == "Concurrent":
            signal_file = (data_dir + "single_intersection_Concurrent.add.xml")
        elif self.paras["ped_phasing"] == "Exclusive":
            signal_file = (data_dir + "single_intersection_Exclusive.add.xml")
        tree = ET.parse(network_file)
        root = tree.getroot()

        # Extract the junctions of study
        for element in root:
            if element.tag == "junction" and element.attrib["type"].startswith("traffic_light"):
                inter_id = element.attrib["id"]
                self.network_graph[inter_id]=collections.defaultdict(dict)

        # Extract incoming and outgoing lanes to and from the intersection
        for element in root:
            if element.tag == "edge":
                if "function" not in element.attrib:
                    if element.attrib["to"] in self.network_graph.keys():
                        inter_id = element.attrib["to"]
                        self.network_graph[inter_id].setdefault("neighbors", set())
                        self.network_graph[inter_id]["neighbors"].add(element.attrib["from"])
                        for lane in element.findall("lane"):
                            if "allow" in lane.attrib and "pedestrian" in lane.attrib["allow"]:  #this is a sidewalk
                                self.network_graph[inter_id].setdefault("incoming_ped", {})
                                self.network_graph[inter_id]["incoming_ped"][element.attrib["id"]] = {"length": float(lane.attrib["length"])}
                            elif "disallow" in lane.attrib and "pedestrian" in lane.attrib["disallow"]:  # what if bike lane?
                                self.network_graph[inter_id].setdefault("incoming_veh", {})
                                self.network_graph[inter_id]["incoming_veh"][lane.attrib["id"]] = {"length": float(lane.attrib["length"])}
                            else:
                                self.network_graph[inter_id].setdefault("incoming_veh", {})
                                self.network_graph[inter_id]["incoming_veh"][lane.attrib["id"]] = {"length": float(lane.attrib["length"])}

                    elif element.attrib["from"] in self.network_graph.keys():
                        inter_id = element.attrib["from"]
                        for lane in element.findall("lane"):
                            if "allow" in lane.attrib and "pedestrian" in lane.attrib["allow"]:  #this is a sidewalk
                                self.network_graph[inter_id].setdefault("outgoing_ped", {})
                                self.network_graph[inter_id]["outgoing_ped"][element.attrib["id"]] = {
                                    "length": float(lane.attrib["length"])}
                            elif "disallow" in lane.attrib and "pedestrian" in lane.attrib["disallow"] : # what if bike lane?
                                self.network_graph[inter_id].setdefault("outgoing_veh", {})
                                self.network_graph[inter_id]["outgoing_veh"][lane.attrib["id"]] = {"length": float(lane.attrib["length"])}
                            else:
                                self.network_graph[inter_id].setdefault("outgoing_veh", {})
                                self.network_graph[inter_id]["outgoing_veh"][lane.attrib["id"]] = {"length": float(lane.attrib["length"])}

                elif element.attrib["function"] == "walkingarea":
                    for inter_id in self.network_graph.keys():
                        if element.attrib["id"].startswith(f":{inter_id}"):
                            for lane in element:
                                self.network_graph[inter_id].setdefault("walkingarea", {})
                                self.network_graph[inter_id]["walkingarea"][element.attrib["id"]]={"shape": lane.attrib["shape"].split()}

                elif element.attrib["function"] == "crossing":
                    for inter_id in self.network_graph.keys():
                        if element.attrib["id"].startswith(f":{inter_id}"):
                            for lane in element:
                                self.network_graph[inter_id].setdefault("crossing", {})
                                if len(element.attrib["crossingEdges"].split()) >2:
                                    self.network_graph[inter_id]["crossing"].setdefault(element.attrib["id"], {})
                                    self.network_graph[inter_id]["crossing"][element.attrib["id"]]["phasing"] = "Diagonal"
                                else:
                                    self.network_graph[inter_id]["crossing"].setdefault(element.attrib["id"], {})
                                    self.network_graph[inter_id]["crossing"][element.attrib["id"]]["phasing"] = "Straight"
                                self.network_graph[inter_id]["crossing"][element.attrib["id"]]["shape"]=lane.attrib["shape"].split()
                                self.network_graph[inter_id]["crossing"][element.attrib["id"]]["length"] = float(lane.attrib["length"])
                                self.network_graph[inter_id]["crossing"][element.attrib["id"]]["width"] = float(lane.attrib["width"])


        # Extract signal info for the intersections based on the additional file for the multiscale scenario
        tree = ET.parse(signal_file)
        root = tree.getroot()
        for element in root:
            if element.tag == "tlLogic" and element.attrib["id"] in self.network_graph.keys():
                inter_id = element.attrib["id"]
                num_phases=0
                for phase in element.findall("phase"):
                    if "g" in phase.attrib["state"] or "G" in phase.attrib["state"]:
                        num_phases += 1
                self.network_graph[inter_id]["num_phases"] = num_phases

        # find walking areas and crossings adjacent:
        for inter_id in self.network_graph:
            for crossing in self.network_graph[inter_id]["crossing"]:
                for walkingarea in self.network_graph[inter_id]["walkingarea"]:
                    for cross_shape in self.network_graph[inter_id]["crossing"][crossing]["shape"]:
                        cross_shape = tuple(map(float, cross_shape.split(',')))
                        for walk_shape in self.network_graph[inter_id]["walkingarea"][walkingarea]["shape"]:
                            walk_shape = tuple(map(float, walk_shape.split(',')))
                            if ((cross_shape[0]-walk_shape[0])**2+(cross_shape[1]-walk_shape[1])**2)**0.5 < 2:
                                self.network_graph[inter_id]["walkingarea"][walkingarea].setdefault("adjacent", set())
                                self.network_graph[inter_id]["walkingarea"][walkingarea]["adjacent"].add(crossing)


        return self.network_graph
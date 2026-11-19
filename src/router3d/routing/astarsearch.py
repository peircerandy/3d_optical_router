import math
from collections import Counter
from copy import deepcopy
from functools import partial
from typing import Dict, List, Optional, Tuple, Union

import gdsfactory as gf
import numpy as np
import shapely.geometry as sg

from router3d.database.schematic import CustomSchematic, Nets
from router3d.drc.drcmanager import DrcManager
from router3d.utils.route_bundle_sbend import route_bundle_sbend
from router3d.utils.smooth import smooth

from ..queue.heapdict import heapdict
from .drgridroute import DrGridRoute
from .utils import EulerDist, customH, manhattanDH


class GridAstarNode:
    __slots__ = [
        "_pos",
        "_cost_g",
        "_cost_f",
        "_visited",
        "_parent",
        "_neighbors",
        "_crossing_budget",
        "_crossing_net",
        "_violatedNet",
        "_bviolated",
        "_straightCount",
    ]

    def __init__(
        self,
        pos: List = (0, 0, 0, 0),   # (x, y, orientation, layer)
        cost_g: Union[float, int] = float("inf"),
        cost_f: Union[float, int] = float("inf"),
        # bend_count: Union[float, int] = float("inf"),
        visited: bool = False,
        parent: Optional[object] = None,
        neighbors: Dict = {},
        crossing_budget: int = 0,
        crossing_net: str = None,
        violatedNet: set = set(),
        bviolated: bool = False,
        straight_count: int = 0,
    ) -> None:
        # Support both 3-tuple (legacy 2D) and 4-tuple (3D)
        if len(pos) == 3:
            self._pos = (pos[0], pos[1], pos[2], 0)
        else:
            self._pos = tuple(pos)
        self._cost_g = cost_g
        self._cost_f = cost_f
        self._visited = visited
        self._parent = parent
        self._neighbors = neighbors
        self._crossing_budget = crossing_budget
        self._crossing_net = crossing_net
        self._violatedNet = violatedNet
        self._bviolated = bviolated
        self._straightCount = straight_count

    def reset(self):
        self._cost_g = float("inf")
        self._cost_f = float("inf")
        # self._bend_count = float("inf")
        self._visited = False
        self._parent = None

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value):
        self._pos = value

    @property
    def x(self) -> int:
        return self._pos[0]

    @property
    def y(self) -> int:
        return self._pos[1]

    @property
    def ori(self) -> int:
        return self._pos[2]

    @property
    def layer(self) -> int:
        return self._pos[3] if len(self._pos) > 3 else 0

    @property
    def costG(self):
        return self._cost_g

    @costG.setter
    def costG(self, value):
        self._cost_g = value

    @property
    def costF(self):
        return self._cost_f

    @costF.setter
    def costF(self, value):
        self._cost_f = value

    @property
    def straightCount(self):
        return self._straightCount

    @straightCount.setter
    def straightCount(self, value):
        self._straightCount = value

    @property
    def visited(self):
        return self._visited

    @visited.setter
    def visited(self, value):
        self._visited = value

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        self._parent = value

    @property
    def neighbors(self):
        return self._neighbors

    # @neighbors.setter
    # def neighbors(self, value):
    #     self._neighbors = value

    def addNeighbors(self, key, value):
        self._neighbors[key] = value

    ### This __eq__ and __lt__ function is to make the Node object comparable, so the priority queue knows how to sort nodes.
    ### A node with smaller cost_f will have higher priority.
    ### If two nodes have the same cost_f, a node with smaller bend_count will have higher priotity
    ### If two nodes have the same cost_f and same bend_count. The one pushed into the queue first will have higher priority.
    ### You do not need to explicitly call __eq__ or __lt__. The priority queue will handle the 'sorting' internally. You just need to push a GridAstarNode into the queue 'queue.put(node)', and pop the lowest cost node out of the queue: 'node = queue.get()'.
    # def __eq__(self, other):
    #     return (self._cost_f, self._bend_count) == (other._cost_f, other._bend_count)

    # def __lt__(self, other):
    #     return (self._cost_f, self._bend_count) < (other._cost_f, other._bend_count)

    def __eq__(self, other):
        return self._cost_f == other._cost_f

    def __lt__(self, other):
        return self._cost_f < other._cost_f

    def __hash__(self):
        return hash(self._pos)


class AstarSearch:
    def __init__(
        self,
        Dr: DrGridRoute,
        CirDB: CustomSchematic,
        CurrentNet: Nets,
        Drc: DrcManager,
        config: dict = None,
        net_config: dict = None,
        bStrictDRC: bool = True,
        groups: dict = None,
    ):
        self._Dr = Dr
        self._cirdb = CirDB
        self._net = CurrentNet
        self._drcmgr = Drc
        self._config = config
        self._net_config = net_config
        self._bStrictDRC = bStrictDRC
        # self.taperLength = 10
        self.connect_th = math.ceil(
            config.dr.bend_radius * 2 / config.dr.grid_resolution
        )
        self._crossing_nets = set()
        self.resolution = config.dr.grid_resolution
        self.repel = False
        if self._config.dr.grid_resolution < 20:  # To be continued:
            self.repel = True
        self.groups = groups
        self.unrouted = max(2, min(10, Dr.unrouted))  # why 10?
        self.check_region = self.unrouted * config.dr.grid_resolution

        self.vAllNodesMap = {}

        # ======================================================================
        # Multi-Tier Layer Stack & Loss Ingestion
        # ======================================================================
        self.layers = self._config.dr.get("layers", None)
        if self.layers is None:
            # Fallback for single-layer / legacy 2D configs
            self.layers = {
                0: {
                    "name": "Si",
                    "loss_propagation": self._config.dr.get("loss_propagation", 1.5),
                    "loss_bending": self._config.dr.get("loss_bending", 0.03),
                    "bend_radius": self._config.dr.get("bend_radius", 5.0),
                    "loss_crossing": self._config.dr.get("loss_crossing", 0.18),
                    "allow_crossings": True,
                }
            }

        self.transitions = self._config.dr.get("transitions", {})

        # Propagation loss per layer: convert dB/cm to dB/um (1 cm = 10,000 um)
        self.propagation_losses = {
            int(z): float(self.layers[z].get("loss_propagation", 1.5)) * 1e-4
            for z in self.layers
        }
        self.alpha = self.propagation_losses  # alias for mathematical notation
        self.bend_losses = {
            int(z): float(self.layers[z].get("loss_bending", 0.03))
            for z in self.layers
        }
        self.bend_radii = {
            int(z): float(self.layers[z].get("bend_radius", 5.0))
            for z in self.layers
        }
        self.crossing_losses = {
            int(z): float(self.layers[z].get("loss_crossing", 0.18))
            for z in self.layers
        }

        # Transition (3D via / adiabatic taper) parameters
        default_trans = self.transitions.get("0-1", {})
        self.taper_loss = float(default_trans.get("loss_taper", 0.08))
        self.taper_length = float(default_trans.get("taper_length", 55.0))
        self.taper_steps = max(1, round(self.taper_length / self.resolution))

        self.penalty = {}
        self.penalty["propagation_loss"] = self.propagation_losses[0] * 1e4  # back in dB/cm for legacy scaling
        self.penalty["bending_loss"] = self.bend_losses[0]
        self.penalty["crossing_loss"] = self.crossing_losses[0]
        self.penalty["congestion"] = float(self._config.dr.get("loss_congestion", 1.0))
        self.penalty["history"] = float(self._config.dr.get("historyCost", 2.0))

        resolution = self._config.dr.grid_resolution
        if net_config:
            self.routing_bound = np.array(net_config["region"]) / resolution
        else:
            scale_factor = (
                config.dr.net_bound_scale_factor + (self._net.failed_count // 2) * 0.5
            )
            x1, y1 = self._net.port1.center
            x2, y2 = self._net.port2.center
            x_mean = (x1 + x2) / 2
            y_mean = (y1 + y2) / 2

            boundx = (
                max(
                    abs(x1 - x2) * scale_factor,
                    resolution * config.dr.net_default_bound,
                )
                / 2
            )
            boundy = (
                max(
                    abs(y1 - y2) * scale_factor,
                    resolution * config.dr.net_default_bound,
                )
                / 2
            )
            self.routing_bound = [
                [
                    max(1, math.ceil((x_mean - boundx) / resolution)),
                    max(1, math.ceil((y_mean - boundy) / resolution)),
                ],
                [
                    min(
                        self._drcmgr.bitMap.width - 1,
                        int((x_mean + boundx) / resolution),
                    ),
                    min(
                        self._drcmgr.bitMap.height - 1,
                        int((y_mean + boundy) / resolution),
                    ),
                ],
            ]

        CurrentNet.current_budget = CurrentNet.crossing_budget
        self.crossing_budget = CurrentNet.crossing_budget

        self.nodepq = heapdict()

        if net_config and not net_config["customH"]:
            self.costH = self.scaledMDist
        elif config.dr.enable_45_neighbor:
            self.costH = lambda x, y, bending_loss=self.penalty[
                "bending_loss"
            ], res=self.resolution: customH(x, y, bending_loss, resolution=res)
        else:
            self.costH = self.scaledMDist

        # Initialize multi-tier state machine (precomputing nextSteps and stepG per layer)
        self.nextSteps = {}
        self.stepG = {}
        self.init_state_machine()

    def _build_state_machine_for_layer(
        self, layer_id: int, radius: float, alpha: float, loss_bending: float
    ):
        resolution = self.resolution
        grid_radius = math.ceil(radius / resolution)
        bend45_1 = math.ceil(radius * 0.415 / resolution)
        bend45_2 = math.ceil(radius * 0.3 / resolution)

        nextSteps = {
            0: [
                (grid_radius, -grid_radius, 270, "bend_90"),
                (1, 0, 0, "straight_0"),
                (grid_radius, grid_radius, 90, "bend_90"),
            ],
            90: [
                (grid_radius, grid_radius, 0, "bend_90"),
                (0, 1, 90, "straight_0"),
                (-grid_radius, grid_radius, 180, "bend_90"),
            ],
            180: [
                (-grid_radius, grid_radius, 90, "bend_90"),
                (-1, 0, 180, "straight_0"),
                (-grid_radius, -grid_radius, 270, "bend_90"),
            ],
            270: [
                (-grid_radius, -grid_radius, 180, "bend_90"),
                (0, -1, 270, "straight_0"),
                (grid_radius, -grid_radius, 0, "bend_90"),
            ],
            45: [
                (bend45_1 + bend45_2, bend45_2, 0, "bend_45_2"),
                (1, 1, 45, "straight_45"),
                (bend45_2, bend45_1 + bend45_2, 90, "bend_45_2"),
            ],
            135: [
                (-bend45_2, bend45_2 + bend45_1, 90, "bend_45_2"),
                (-1, 1, 135, "straight_45"),
                (-bend45_2 - bend45_1, bend45_2, 180, "bend_45_2"),
            ],
            225: [
                (-bend45_2 - bend45_1, -bend45_2, 180, "bend_45_2"),
                (-1, -1, 225, "straight_45"),
                (-bend45_2, -bend45_2 - bend45_1, 270, "bend_45_2"),
            ],
            315: [
                (bend45_2, -bend45_2 - bend45_1, 270, "bend_45_2"),
                (1, -1, 315, "straight_45"),
                (bend45_2 + bend45_1, -bend45_2, 0, "bend_45_2"),
            ],
        }

        if self._net.enable_45 and self._config.dr.enable_45_neighbor:
            nextSteps[0].append((bend45_2 + bend45_1, -bend45_2, 315, "bend_45_1"))
            nextSteps[0].append((bend45_2 + bend45_1, bend45_2, 45, "bend_45_1"))
            nextSteps[90].append((-bend45_2, bend45_2 + bend45_1, 135, "bend_45_1"))
            nextSteps[90].append((bend45_2, bend45_2 + bend45_1, 45, "bend_45_1"))
            nextSteps[180].append((-bend45_2 - bend45_1, bend45_2, 135, "bend_45_1"))
            nextSteps[180].append((-bend45_2 - bend45_1, -bend45_2, 225, "bend_45_1"))
            nextSteps[270].append((-bend45_2, -bend45_2 - bend45_1, 225, "bend_45_1"))
            nextSteps[270].append((bend45_2, -bend45_2 - bend45_1, 315, "bend_45_1"))

        # Precompute step costs (stepG) in physical decibels
        stepG = {
            "straight_0": resolution * alpha,
            "straight_45": np.sqrt(2) * resolution * alpha,
        }

        # 45-degree bend
        points45 = np.array(
            [
                (0, 0),
                (bend45_1 * resolution, 0),
                ((bend45_1 + bend45_2) * resolution, bend45_2 * resolution),
            ]
        )
        P45, _, _ = smooth(points=points45, radius=radius - 1e-3, bend=gf.path.arc)
        stepG["bend_45_1"] = P45.length() * alpha + loss_bending / 2
        stepG["bend_45_2"] = stepG["bend_45_1"]

        # 90-degree bend
        points90 = np.array(
            [
                (0, 0),
                (grid_radius * resolution, 0),
                (grid_radius * resolution, grid_radius * resolution),
            ]
        )
        P90, _, _ = smooth(points=points90, radius=radius - 1e-3, bend=gf.path.arc)
        stepG["bend_90"] = P90.length() * alpha + loss_bending

        return nextSteps, stepG, grid_radius, bend45_1, bend45_2

    def init_state_machine(self):
        resolution = self.resolution
        self.radius = self.bend_radii.get(0, self._config.dr.bend_radius)
        self.grid_radius = math.ceil(self.radius / resolution)

        self.nextSteps = {}
        self.stepG = {}
        for z in self.layers:
            z_int = int(z)
            r = self.bend_radii[z_int]
            a = self.alpha[z_int]
            b_loss = self.bend_losses[z_int]
            nSteps, sG, g_rad, b45_1, b45_2 = self._build_state_machine_for_layer(
                z_int, r, a, b_loss
            )
            self.nextSteps[z_int] = nSteps
            self.stepG[z_int] = sG
            if z_int == 0:
                self.bend45_1 = b45_1
                self.bend45_2 = b45_2
                self.grid_radius = g_rad

        predict_length = round(self._net.half_size / resolution)
        self.set_DrcManager(self.grid_radius, self.bend45_1, self.bend45_2, predict_length)
        self.his_index = {0: 0, 45: 1, 90: 2, 135: 3, 180: 4, 225: 5, 270: 6, 315: 7}

        """
        offset neighbor 1
        points = np.array([(0,0), (self.grid_45*res,1), (self.grid_45*res+1,1)])
        P, _, _ = smooth(
            points=points,
            radius=radius - 1e-3,
            bend=gf.path.arc, #gf.path.euler / gf.path.arc
            # use_eff=True
        )
        self.stepG["offset_1"] = P.length()

        # offset neighbor 2
        points = np.array([(0,0), (self.grid_45*res,2), (self.grid_45*res+2,2)])
        P, _, _ = smooth(
            points=points,
            radius=radius - 1e-3,
            bend=gf.path.arc, #gf.path.euler / gf.path.arc
            # use_eff=True
        )
        self.stepG["offset_2"] = P.length()
        """

    def set_DrcManager(self, bend90, bend45_1, bend45_2, predict_length):
        self._drcmgr.ben_45_part1 = bend45_1
        self._drcmgr.ben_45_part2 = bend45_2
        self._drcmgr.predict_length = predict_length
        self._drcmgr.radius = bend90

    def findAcsPts(self):
        """To do: only support positive index now"""
        if self._config.dr.group:
            if self._net.reverse:
                self.start_port = self._net.NetPort2
                self.end_port = self._net.NetPort1
            else:
                self.end_port = self._net.NetPort2
                self.start_port = self._net.NetPort1

            if self._net.earlyaccess:  ### To be continued
                access_grid = int(self.radius / self.resolution)
                endPos = self.end_port.port_grids[-access_grid]
            else:
                endPos = self.end_port.port_grids[0]
        else:
            self.start_port = self._net.NetPort1
            self.end_port = self._net.NetPort2
            endPos = self.end_port.port_grids[0]

        self.access_points = set(self.end_port.port_grids)

        self.endNode = GridAstarNode(
            pos=(endPos[0], endPos[1], self.end_port.orientation), crossing_budget=0
        )

        """To do: find start point considering the taper length
        """
        self.startNode = []

        startPos = self.start_port.port_grids[0]
        self.startNode.append(
            GridAstarNode(
                pos=(startPos[0], startPos[1], self.start_port.orientation),
                cost_g=0,
                cost_f=self.costH(startPos, endPos),
                straight_count=0,
                neighbors={},
                crossing_budget=self.crossing_budget,
            )
        )
        return True

    def connection_check(self, port1, port2):
        center1 = port1.center
        ori1 = round(port1.orientation)
        if ori1 == 360:
            ori1 = 0
        center2 = port2.center
        ori2 = round(port2.orientation)
        if ori2 == 360:
            ori2 = 0
        angle = abs(ori1 - ori2)
        if angle > 180:
            angle -= 180

        dx = abs(center1[0] - center2[0])
        dy = abs(center1[1] - center2[1])

        """ To do: Parametric coding
            """
        # gdsfactory do not support 180 bending
        if angle == 0:
            if ori1 == 0 or ori1 == 180:  ### lack 45 / 135 case
                if dy > 2.5 * self.radius:
                    return True, "all_angle", 180
                else:
                    return False, "None", 0
            elif dx > 2.5 * self.radius:
                return True, "all_angle", 180

        elif angle == 180:
            if ori1 == 90 or ori1 == 270:
                if dy > np.sqrt(dx * (4 * self.radius - dx)):
                    return True, "Sbend_y", 0
                else:
                    return False, "None", 0
            if ori1 == 0 or ori1 == 180:
                if dx > np.sqrt(dy * (4 * self.radius - dy)):
                    return True, "Sbend_x", 0
                else:
                    return False, "None", 0
        elif angle == 90:
            if dx >= self.radius and dy >= self.radius:
                ori1_rad = np.deg2rad(ori1)
                ori2_rad = np.deg2rad(ori2)
                dx1 = 100 * np.cos(ori1_rad)
                dy1 = 100 * np.sin(ori1_rad)
                dx2 = 100 * np.cos(ori2_rad)
                dy2 = 100 * np.sin(ori2_rad)
                center1_far = np.asarray(center1) + [dx1, dy1]
                l1 = sg.LineString([center1, center1_far])

                center2_far = np.asarray(center2) + [dx2, dy2]
                l2 = sg.LineString([center2, center2_far])

                intersect = l1.intersection(l2)
                if isinstance(intersect, sg.Point):
                    return True, "all_angle", 90
                return False, "None", 0
            else:
                return False, "None", 0

        elif angle == 45 or angle == 135:
            if ori1 == 45 or ori1 == 135 or ori1 == 225 or ori1 == 315:
                if ori2 == 0 or ori2 == 180:
                    if dy >= 0.3 * self.radius and dx >= dy + 0.4 * self.radius:
                        ori1_rad = np.deg2rad(ori1)
                        ori2_rad = np.deg2rad(ori2)
                        dx1 = 100 * np.cos(ori1_rad)
                        dy1 = 100 * np.sin(ori1_rad)
                        dx2 = 100 * np.cos(ori2_rad)
                        dy2 = 100 * np.sin(ori2_rad)
                        center1_far = np.asarray(center1) + [dx1, dy1]
                        l1 = sg.LineString([center1, center1_far])

                        center2_far = np.asarray(center2) + [dx2, dy2]
                        l2 = sg.LineString([center2, center2_far])

                        intersect = l1.intersection(l2)
                        if isinstance(intersect, sg.Point):
                            return True, "all_angle", 45

                        return False, "None", 0

                elif ori2 == 90 or ori2 == 270:
                    if dx >= 0.3 * self.radius and dy >= dx + 0.4 * self.radius:
                        ori1_rad = np.deg2rad(ori1)
                        ori2_rad = np.deg2rad(ori2)
                        dx1 = 100 * np.cos(ori1_rad)
                        dy1 = 100 * np.sin(ori1_rad)
                        dx2 = 100 * np.cos(ori2_rad)
                        dy2 = 100 * np.sin(ori2_rad)
                        center1_far = np.asarray(center1) + [dx1, dy1]
                        l1 = sg.LineString([center1, center1_far])

                        center2_far = np.asarray(center2) + [dx2, dy2]
                        l2 = sg.LineString([center2, center2_far])

                        intersect = l1.intersection(l2)
                        if isinstance(intersect, sg.Point):
                            return True, "all_angle", 45

                        return False, "None", 0

                return False, "None", 0
        else:
            return False, "None", 0

    def hard_connection(self, currentNode, radius=5):
        path, origin_path, violatedNets = self.backTrack(currentNode)
        """To be continued
        """
        if path is None:
            for node in origin_path[1:]:
                self.vAllNodesMap[node].visited = False
            return False

        Plength = len(path)
        # bend_circular has some problem...
        bend_circular = partial(gf.components.bend_circular, radius=radius)
        bend_euler = partial(gf.components.bend_euler, radius=radius)
        bend = bend_euler
        for _ in range(self.radius):
            try:
                P, segments, accumulated_bend = smooth(
                    points=path[0:Plength],
                    radius=radius - 1e-3,
                    bend=gf.path.euler,
                    # bend=gf.path.arc,
                    use_eff=True,
                    alignment=False,
                )
            except:
                Plength -= 1
                continue

            """can change the orther
            """
            # only support 0.5 width cross section now
            route = gf.path.extrude(P, width=0.5, layer=(1, 0))  ### To be continued

            rect_route = self.fast_split_polygon_to_rectangles(
                route, self._drcmgr.resolution
            )
            # save result into net and update the bitmap
            if self.resolution == 1:
                expanded_route = gf.path.extrude(P, width=2, layer=(1, 0))
                expanded_rect_route = self.fast_split_polygon_to_rectangles(
                    expanded_route, self._drcmgr.resolution
                )
                rect_union = self.sets_union(expanded_rect_route, rect_route)
                self._drcmgr.updateBitmap(rect_route, self._net.netName)
            else:
                pass

            """ gf_ports are used for access waveguide gerneration
            """
            if self._bStrictDRC:
                # align = alignment(segments, self.start_port.port, self.end_port.port, radius=radius-1e-3, bend=gf.path.euler)
                self._net.rect_route = rect_route
                # if align is not None:
                # segments = align
                self._net.wg_component.append(route)
                # self._net.wg_component.append(temp1.flatten())
                # self._net.wg_component.append(temp2.flatten())
                self._net.routed_path = [
                    [segments.tolist(), self.start_port.gf_port, self.end_port.gf_port]
                ]
                self._net.crossing_nets = self._crossing_nets
                # self._net.current_budget -= len(self._crossing_nets)
                self._net.bending = accumulated_bend  # + connect_angle
                self._net.wirelength = P.length()
                self._net.crossing_num = len(self._crossing_nets)
            else:
                self._net.rect_route = rect_route  # save for updating bitmap
                self._net.origin_path = origin_path  # save for updating historyCost
                # self._drcmgr.updateBitmap(rect_route, self._net.netName)
                self._net.wg_component.append(route)
                # self._net.wg_component.append(temp1.flatten())
                # self._net.wg_component.append(temp2.flatten())
                self._net.routed_path = [
                    [segments.tolist(), self.start_port.gf_port, self.end_port.gf_port]
                ]
                self._net.wirelength = P.length()
                self._net.bending = accumulated_bend
                self._Dr.mark_conflict_nets(violatedNets, self._net.netName)
                self._net.vioNets = violatedNets
                self._net.vionets = len(violatedNets)
            return True

    def sets_union(self, sets1, sets2):
        curve_rects_1, straight_90_1, straight_0_1, straight_45_1, straight_135_1 = (
            sets1
        )
        curve_rects_2, straight_90_2, straight_0_2, straight_45_2, straight_135_2 = (
            sets2
        )
        curve_rects = curve_rects_1.union(curve_rects_2)
        straight_90 = straight_90_1.union(straight_90_2)
        straight_0 = straight_0_1.union(straight_0_2)
        straight_45 = straight_45_1.union(straight_45_2)
        straight_135 = straight_135_1.union(straight_135_2)
        return (curve_rects, straight_90, straight_0, straight_45, straight_135)

    def route(self):
        if (
            self._net.distance["Euler"] < 10
            and abs(self.startNode[0].pos[2] - self.endNode.pos[2]) == 180
        ):
            access_waveguide = gf.Component()
            waveguide_length = route_bundle_sbend(
                access_waveguide,
                [self.start_port.gf_port],
                [self.end_port.gf_port],
            )
            self._net.wirelength += waveguide_length
            self._net.wg_component.append(access_waveguide)
            self._cirdb.layout.add_ref(access_waveguide)
            return True

        if not self.findAcsPts():
            return False
        for node in self.startNode:
            # self.nodepq.put(node)
            self.nodepq[node] = node.costF

        while not self.nodepq.empty():  # Need to check the _param.maxExplore
            current_node, _ = self.nodepq.popitem()
            current_node.visited = True

            if self.check_connection(
                current_node.pos, self.endNode.pos
            ):  # self.connect_th:
                if self.hard_connection(
                    current_node, radius=self._config.dr.bend_radius
                ):
                    self._net.routed = True
                    return True
                else:
                    current_node.visited = False

            # find the current node's neightbors and loop through the neighbors
            if not current_node.neighbors:
                self.findNeighbors(current_node)

            for neighbor, nb_type in current_node.neighbors.items():
                if neighbor.visited:
                    continue
                costF = 0
                layer = current_node.layer
                if nb_type == "via_3d":
                    costG = current_node.costG + self.taper_loss
                elif nb_type in {"crossing_0", "crossing_45"}:
                    costG = current_node.costG
                    costG += self.crossing_losses.get(layer, self.penalty["crossing_loss"])
                else:
                    stepG_layer = self.stepG.get(layer, self.stepG[0])
                    costG = current_node.costG + stepG_layer[nb_type]

                if neighbor._bviolated:
                    cv = current_node._violatedNet
                    nv = neighbor._violatedNet
                    union_nets = cv | nv
                    netnum = len(union_nets)
                    for net in nv:
                        if net in cv:
                            costG += 0.1 * self.penalty["crossing_loss"]
                        else:
                            costG += netnum * 10 * self.penalty["crossing_loss"]

                x, y, ori = neighbor.pos[:3]

                if self.groups:
                    dis = EulerDist(neighbor.pos, self.endNode.pos) * self.resolution
                    # congestion_penalty = self.penalty["bending_loss"] * 0.1
                    congestion_penalty = self.penalty["congestion"]
                    if dis >= self.check_region:  # compatible with NOC and computing
                        if (
                            self.repel
                            and (ori + self.end_port.orientation) % 180 != 0
                            and self._net.failed_count > 1
                        ):  #
                            count = 0
                            if nb_type == "straight_0":
                                count = self._drcmgr.checkSpacing(neighbor, 5, {})
                                costG += count * congestion_penalty
                            elif nb_type == "straight_45":
                                count = self._drcmgr.checkSpacing(neighbor, 4, {})
                                costG += count * congestion_penalty

                        if not self.repel:
                            count = 0
                            if nb_type == "straight_0":
                                count = self._drcmgr.checkSpacing(
                                    neighbor, self.unrouted, self.groups
                                )
                            elif nb_type == "straight_45":
                                count = self._drcmgr.checkSpacing(
                                    neighbor, self.unrouted, self.groups
                                )
                            if count > 0:
                                costG += count * congestion_penalty

                if self.bNeedUpdate(neighbor, costG):
                    costF += costG + self.propagation_losses.get(neighbor.layer, self.propagation_losses[0]) * self.costH(
                        neighbor.pos, self.endNode.pos
                    )

                    if self._bStrictDRC:
                        his = self._Dr.historyMap[x][y][self.his_index[ori]]
                        if his:
                            costF += his

                    if nb_type in {"straight_0", "straight_45"}:
                        neighbor.straightCount = current_node.straightCount + 1
                    elif nb_type in {"bend_45_1", "bend_45_2"}:
                        neighbor.straightCount = 0

                    neighbor.costG = costG
                    neighbor.costF = costF
                    neighbor.parent = current_node
                    self.nodepq[neighbor] = neighbor.costF

        return False

    def findNeighbors(self, currentNode: GridAstarNode):
        currentX, currentY, orientation = currentNode.pos[:3]
        layer = currentNode.layer
        nextStep = self.nextSteps.get(layer, self.nextSteps[0])[orientation]
        end_node = self.endNode.pos
        th = self.connect_th
        for step in nextStep:
            enable_prediction = True
            neighbor_position = (currentX + step[0], currentY + step[1], step[2], layer)
            dis = manhattanDH(currentNode.pos, end_node)
            if dis <= th:
                enable_prediction = False
            routing_bound = self.routing_bound
            if (
                neighbor_position[0] <= routing_bound[0][0]
                or neighbor_position[0] >= routing_bound[1][0]
                or neighbor_position[1] <= routing_bound[0][1]
                or neighbor_position[1] >= routing_bound[1][1]
            ):
                continue
            ans = self._drcmgr.bViolateDRC(
                currentNode,
                step,
                self._bStrictDRC,
                self._net.netName,
                enable_prediction,
            )
            ans_1 = ans[1]

            vAllNodesMap = self.vAllNodesMap
            if not ans[0]:
                tmp_neighbor = vAllNodesMap.get(neighbor_position, None)
                if tmp_neighbor is None:
                    tmp_neighbor = vAllNodesMap[neighbor_position] = GridAstarNode(
                        pos=neighbor_position,
                        parent=None,
                        neighbors={},
                        crossing_budget=currentNode._crossing_budget,
                    )
                currentNode.addNeighbors(tmp_neighbor, step[3])

            elif ans_1 is not None:  # crossing neighbor
                cross_pos = (*ans_1[:3], layer)
                tmp_neighbor = vAllNodesMap.get(cross_pos, None)
                if tmp_neighbor is None:
                    tmp_neighbor = vAllNodesMap[cross_pos] = GridAstarNode(
                        pos=cross_pos,
                        parent=None,
                        neighbors={},
                        crossing_budget=currentNode._crossing_budget - 1,
                        crossing_net=ans[3],
                        # bviolated=ans[4]
                    )
                currentNode.addNeighbors(tmp_neighbor, ans[2])

            elif not self._bStrictDRC and ans[2] not in {"blk", "port"}:
                tmp_neighbor = vAllNodesMap.get(neighbor_position, None)
                if tmp_neighbor is None:
                    tmp_neighbor = vAllNodesMap[neighbor_position] = GridAstarNode(
                        pos=neighbor_position,
                        parent=None,
                        neighbors={},
                        bviolated=True,
                        violatedNet=ans[3],
                    )
                currentNode.addNeighbors(tmp_neighbor, step[3])

        # Inter-tier vertical via candidate (3D transition)
        if len(self.layers) > 1:
            for target_layer in self.layers:
                target_z = int(target_layer)
                if target_z != layer:
                    dx, dy = 0, 0
                    if orientation == 0:
                        dx = self.taper_steps
                    elif orientation == 90:
                        dy = self.taper_steps
                    elif orientation == 180:
                        dx = -self.taper_steps
                    elif orientation == 270:
                        dy = -self.taper_steps
                    elif orientation == 45:
                        dx = dy = max(1, round(self.taper_steps / 1.414))
                    elif orientation == 135:
                        dx = -max(1, round(self.taper_steps / 1.414))
                        dy = max(1, round(self.taper_steps / 1.414))
                    elif orientation == 225:
                        dx = dy = -max(1, round(self.taper_steps / 1.414))
                    elif orientation == 315:
                        dx = max(1, round(self.taper_steps / 1.414))
                        dy = -max(1, round(self.taper_steps / 1.414))

                    via_pos = (currentX + dx, currentY + dy, orientation, target_z)
                    routing_bound = self.routing_bound
                    if (
                        via_pos[0] > routing_bound[0][0]
                        and via_pos[0] < routing_bound[1][0]
                        and via_pos[1] > routing_bound[0][1]
                        and via_pos[1] < routing_bound[1][1]
                    ):
                        tmp_via = self.vAllNodesMap.get(via_pos, None)
                        if tmp_via is None:
                            tmp_via = self.vAllNodesMap[via_pos] = GridAstarNode(
                                pos=via_pos,
                                parent=None,
                                neighbors={},
                                crossing_budget=currentNode._crossing_budget,
                            )
                        currentNode.addNeighbors(tmp_via, "via_3d")

    def scaledMDist(self, pos1, pos2):
        x = abs(pos1[0] - pos2[0]) * self.resolution
        y = abs(pos1[1] - pos2[1]) * self.resolution
        if x > 0 and y > 0:
            return x + y + self.resolution
        else:
            return x + y

    def check_connection(self, pos1, pos2):
        x1, y1, ori1 = pos1
        x2, y2, ori2 = pos2
        if (x1, y1) in self.access_points and abs(ori1 - ori2) == 180:
            return True
        else:
            return False

    def backTrack(self, node: GridAstarNode):
        if not self._bStrictDRC:
            violatedNets = set()
            path = [node.pos[0:]]
            if node._violatedNet:
                violatedNets = violatedNets | node._violatedNet
            while node.parent is not None:
                path.append(node.parent.pos[0:])
                if node._violatedNet:
                    violatedNets = violatedNets | node._violatedNet
                node = node.parent
            path = path[::-1]
            origin_path = deepcopy(path)
            path = np.array(self._process_bend(path))
            path[:, 0:2] *= self._drcmgr.resolution
            path[:, 0:2] += self._drcmgr.resolution / 2
            return path[:, 0:2], origin_path, violatedNets
        else:
            flag = False  # used to check whether the net is crossed twice
            path = [node.pos[0:]]
            self._crossing_nets.clear()
            if node._crossing_net:
                self._crossing_nets.add(node._crossing_net)
            while node.parent is not None:
                path.append(node.parent.pos[0:])
                node = node.parent
                if node._crossing_net:
                    if node._crossing_net in self._crossing_nets:
                        flag = True
                    else:
                        self._crossing_nets.add(node._crossing_net)
            path = path[::-1]
            origin_path = deepcopy(path)
            if flag:
                return None, origin_path, None
            path = np.array(self._process_bend(path))
            path[:, 0:2] *= self._drcmgr.resolution
            path[:, 0:2] += self._drcmgr.resolution / 2
            return path[:, 0:2], origin_path, None

    def _process_bend(self, path):
        vec = []
        if len(path) == 1:
            vec.append(path[0][0:])
            return vec
        if len(path) == 2:
            vec.append(path[0][0:])
            vec.append(path[1][0:])
            return vec

        k = len(path)
        for i in range(0, k):
            vec.append(path[i][0:])
            if i == k - 1:
                break
            elif path[i][2] != path[i + 1][2]:  # next point
                ori = path[i][2]
                angle = abs(path[i + 1][2] - ori)
                if angle > 180:
                    angle = 360 - angle
                if angle == 45:
                    match ori:
                        case 0:
                            vec.append((path[i][0] + self.bend45_1, path[i][1], ori))
                        case 90:
                            vec.append((path[i][0], path[i][1] + self.bend45_1, ori))
                        case 180:
                            vec.append((path[i][0] - self.bend45_1, path[i][1], ori))
                        case 270:
                            vec.append((path[i][0], path[i][1] - self.bend45_1, ori))
                        case 45:
                            vec.append(
                                (
                                    path[i][0] + self.bend45_2,
                                    path[i][1] + self.bend45_2,
                                    ori,
                                )
                            )
                        case 135:
                            vec.append(
                                (
                                    path[i][0] - self.bend45_2,
                                    path[i][1] + self.bend45_2,
                                    ori,
                                )
                            )
                        case 225:
                            vec.append(
                                (
                                    path[i][0] - self.bend45_2,
                                    path[i][1] - self.bend45_2,
                                    ori,
                                )
                            )
                        case 315:
                            vec.append(
                                (
                                    path[i][0] + self.bend45_2,
                                    path[i][1] - self.bend45_2,
                                    ori,
                                )
                            )

                else:
                    match ori:
                        case 0:
                            vec.append((path[i][0] + self.grid_radius, path[i][1], ori))
                        case 90:
                            vec.append((path[i][0], path[i][1] + self.grid_radius, ori))
                        case 180:
                            vec.append((path[i][0] - self.grid_radius, path[i][1], ori))
                        case 270:
                            vec.append((path[i][0], path[i][1] - self.grid_radius, ori))
        return vec

    def _mergePath(self, path: List[Tuple[int]]) -> List[Tuple[int]]:
        vec = []
        if len(path) == 1:
            vec.append(path[0][0:2])
            return vec
        if len(path) == 2:
            vec.append(path[0][0:2])
            vec.append(path[1][0:2])
            return vec

        i = 0
        k = len(path)
        for j in range(1, k + 1):
            if j != k:
                if i == j:
                    continue
                v1 = path[j]
                # same direction
                if path[i][2] == v1[2]:
                    continue

                angle = abs(v1[2] - path[i][2])
                if angle > 180:
                    angle = 360 - angle
                if angle == 45:
                    vec.append(path[i][0:2])
                    # more than two node in the same direction, add the second point of the segment
                    if i != j - 1:
                        vec.append(path[j - 1][0:2])
                        i = j
                    else:
                        i = j
                # angle == 90, add the inflection node, ignore the intermediate node
                elif path[i][2] == 180:
                    vec.append(path[i][0:2])
                    vec.append((path[j - 1][0] - 5, path[j - 1][1]))
                    i = j
                elif path[i][2] == 0:
                    vec.append(path[i][0:2])
                    vec.append((path[j - 1][0] + 5, path[j - 1][1]))
                    i = j
                elif path[i][2] == 90:
                    vec.append(path[i][0:2])
                    vec.append((path[j - 1][0], path[j - 1][1] + 5))
                    i = j
                else:
                    vec.append(path[i][0:2])
                    vec.append((path[j - 1][0], path[j - 1][1] - 5))
                    i = j
            else:  # The last node
                if i == k - 1:
                    vec.append(path[i][0:2])
                else:
                    vec.append(path[i][0:2])
                    vec.append(path[k - 1][0:2])

        return vec

    def hasBend(self, currentNode: GridAstarNode, neighbor: GridAstarNode):
        return currentNode.pos[2] != neighbor.pos[2]

    def _findDirection(self, currentNode: GridAstarNode, neighbor: GridAstarNode):
        if currentNode.pos[0] == neighbor.pos[0]:
            assert currentNode.pos[1] != neighbor.pos[1]
            return "n" if currentNode.pos[1] < neighbor.pos[1] else "s"
        elif currentNode.pos[1] == neighbor.pos[1]:
            return "e" if currentNode.pos[0] < neighbor.pos[0] else "w"
        elif currentNode.pos[0] < neighbor.pos[0]:
            return "ne" if currentNode.pos[1] < neighbor.pos[1] else "se"
        else:
            return "nw" if currentNode.pos[1] < neighbor.pos[1] else "sw"

    def bNeedUpdate(self, neighbor: GridAstarNode, costG):
        if neighbor.costG > costG:
            return True
        return False

    def fast_split_polygon_to_rectangles(self, component: gf.Component, resolution=1):
        """using the GDSFactory polygons"""
        points = component.get_polygons_points()[1][0]
        # assert component.polygons is not None
        # points = component.polygons[0].points
        int_points = (points[0:] / resolution).astype(np.int32).tolist()
        int_points = tuple(map(tuple, int_points))
        dpoint = points[0:-1, :] - points[1:, :]
        slope = dpoint[:, 1] / dpoint[:, 0]
        grid_dict = {}
        for i, num in enumerate(dpoint):
            abs_num = abs(num)
            if abs_num[0] > 0.5 or abs_num[1] > 0.5:  # Interpolation
                num_points = round(max(abs_num[0], abs_num[1]) / 0.1)
                x = (
                    np.linspace(
                        points[i, 0] / resolution,
                        points[i + 1, 0] / resolution,
                        num_points,
                    )
                    .astype(np.int32)
                    .tolist()
                )
                y = (
                    np.linspace(
                        points[i, 1] / resolution,
                        points[i + 1, 1] / resolution,
                        num_points,
                    )
                    .astype(np.int32)
                    .tolist()
                )
                for grid in tuple(zip(x, y)):
                    if grid_dict.get(grid) is None:
                        grid_dict[grid] = []
                    grid_dict[grid].append(slope[i])

        curve_rects = set()
        straight_0 = set()
        straight_45 = set()
        straight_90 = set()
        straight_135 = set()

        for i, num in enumerate(slope):
            if grid_dict.get(int_points[i]) is None:
                grid_dict[int_points[i]] = []
            grid_dict[int_points[i]].append(slope[i])

        for grid, slope in grid_dict.items():
            mean_slope = np.mean(slope)
            abs_slope = np.abs(slope)
            abs_mean_slope = np.mean(abs_slope)
            max_slope = np.max(abs_slope)
            min_slope = np.min(abs_slope)
            grid_dict[grid] = mean_slope
            # contain inf
            if (
                np.isnan(mean_slope)
                or mean_slope == float("inf")
                or mean_slope == float("-inf")
            ):  # [inf, -inf] / [inf, ...] / [-inf, ...]
                if min_slope == float("inf"):  # [-inf,..., inf]
                    straight_90.add((grid[0], grid[1]))
                    continue
                result = Counter(abs_slope)
                if len(result) == 2:
                    if result[0] > result[float("inf")]:
                        straight_0.add((grid[0], grid[1]))
                    else:
                        straight_90.add((grid[0], grid[1]))
                else:
                    curve_rects.add((grid[0], grid[1]))
            # 45/135
            elif abs(abs_mean_slope - 1) < 1e-2:
                if mean_slope > 0:
                    straight_45.add((grid[0], grid[1]))
                if mean_slope < 0:
                    straight_135.add((grid[0], grid[1]))
            # 0/180
            elif abs(mean_slope) < 1e-3:
                straight_0.add((grid[0], grid[1]))
            # 90/270
            elif abs(mean_slope) > 3e3:
                straight_90.add((grid[0], grid[1]))
            else:
                curve_rects.add((grid[0], grid[1]))

        return (curve_rects, straight_90, straight_0, straight_45, straight_135)

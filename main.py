from collections import OrderedDict
from collections import deque
from priority_queue import *
import numpy as np


class Graph:
    def __init__(self, graph_dict=None):
        self.graph_dict = graph_dict or {}

    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)


class Problem:

    def __init__(self, initial, goal, graph):
        self.initial = initial
        self.goal = goal
        self.graph = graph

    def actions(self, A):
        return list(self.graph.get(A).keys())

    def path_cost(self, cost_so_far, A, action, B):
        return cost_so_far + (self.graph.get(A, B) or np.inf)

    def goal_test(self, state):
        return state == self.goal

    def find_min_edge(self):
        """Find minimum value of edges."""
        m = np.inf
        for d in self.graph.graph_dict.values():
            local_min = min(d.values())
            m = min(m, local_min)

        return m

    def result(self, action):
        return action


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def solution(self):
        return [node.action for node in self.path()[1:]]

    def path(self):
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))


def expand(node, problem):
    nodes = []
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(action)
        cost = problem.path_cost(node.path_cost, s, action, s1)
        next_node = Node(s1, node, action, cost)
        nodes.append(next_node)
    return nodes


def uniform_cost_search(problem):
    node = Node(problem.initial)
    frontier = PriorityQueue()
    frontier.insert((node, node.path_cost), node.path_cost)
    reached = dict()
    reached[problem.initial] = node
    while frontier:
        node, costo = frontier.remove()
        print("Elemento removido de frontier: " + str(node.state))
        print("Nodo actual: " + str(node.state))
        # if node.parent:
        #     #print("Nodo padre: " + str(node.parent.state))
        # else:
        #     #print("Nodo padre: No tiene")
        nodes = []
        for action in problem.actions(node.state):
            nodes.append(action)
        # print("Nodos hijos: " + str(nodes))
        if problem.goal_test(node.state):
            return node
        for child in expand(node, problem):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.insert((child, child.path_cost), child.path_cost)
                print("Elemento añadido a frontier: " + str(child.state))

    return None


    # def a_star_search():



arbol = Graph(OrderedDict(Ellensburg=OrderedDict(Pendleton=168, Spokane=175),
                          Spokane=OrderedDict(BonnersFerry=112, Missoula=199),
                          BonnersFerry=OrderedDict(WestGlacier=176),
                          Missoula=OrderedDict(Helena=111),
                          WestGlacier=OrderedDict(Havre=231, GreatFalls=111, Helena=243),
                          Havre=OrderedDict(GreatFalls=115),
                          GreatFalls=OrderedDict(Helena=91),
                          Helena=OrderedDict(Butte=65)))


problema = Problem("Ellensburg", "Havre", arbol)

bfs = uniform_cost_search(problema)

print(bfs.solution())

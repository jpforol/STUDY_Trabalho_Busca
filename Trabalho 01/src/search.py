
from math import inf

import sys

from collections import deque
from typing import Any, List, Union, Tuple

from src.problems import ProblemInterface
from src.viewer import ViewerInterface


class Node:
    # The output path is generated backwards starting from
    # the goal node, hence the need to store the parent in
    # the node.
    def __init__(self, state: Any, action=None, previous_node=None):
        self.state = state
        self.action = action
        self.previous_node = previous_node

    def __repr__(self):
        return f"Node(state={self.state}, action={self.action}, previous_node={self.previous_node})"

    def __eq__(self, n) -> bool:
        return (self.state == n.state)

    # method necessary for easily checking if nodes
    # have already been added to sets or used as keys
    # in dictionaries.
    def __hash__(self):
        return hash(self.state)


def breadth_first_search(problem: ProblemInterface, viewer: ViewerInterface) -> Tuple[List[Any], float]:
    # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()

    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())
    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None

    # Repeat while we haven't found the goal and still have
    # nodes to expand. If there aren't further nodes
    # to expand in breadth-first search, the goal is
    # unreachable.
    while (len(to_explore) > 0) and (goal_found is None):
        # select next node or expansion
        state_node = to_explore.popleft()

        neighbors = _generate_neighbors(state_node, problem)

        for n in neighbors:
            if (n not in expanded) and (n not in to_explore):
                if problem.is_goal(n.state):
                    goal_found = n
                    break
                to_explore.append(n)

        expanded.add(state_node)

        #viewer.update(state_node.state,generated=to_explore,expanded=expanded)

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)
    n_expandidos = len(expanded)

    return path, cost, n_expandidos


def deep_first_search(problem: ProblemInterface, viewer: ViewerInterface) -> Tuple[List[Any], float]:
    # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()

    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())
    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None

    # Repeat while we haven't found the goal and still have
    # nodes to expand. If there aren't further nodes
    # to expand in breadth-first search, the goal is
    # unreachable.
    while (len(to_explore) > 0) and (goal_found is None):
        # select next node or expansion
        state_node = to_explore[-1]

        neighbors = _generate_neighbors(state_node, problem)

        aux = 0

        for n in neighbors:
            if (n not in expanded) and (n not in to_explore):
                if problem.is_goal(n.state):
                    goal_found = n
                    break
                else:
                    aux += 1
                    auxv = n
                    break
        if aux != 0:
            to_explore.append(auxv)
        else:
            expanded.add(state_node)
            to_explore.pop()

        #viewer.update(state_node.state,generated=to_explore, expanded=expanded)

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)
    n_expandidos = len(expanded)

    return path, cost, n_expandidos

def uniform_cost_search(problem: ProblemInterface, viewer: ViewerInterface) -> Tuple[List[Any], float]:
    # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()


    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())
    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None

    # Repeat while we haven't found the goal and still have
    # nodes to expand. If there aren't further nodes
    # to expand in breadth-first search, the goal is
    # unreachable.
    while (len(to_explore) > 0) and (goal_found is None):
        # select next node or expansion
        state_node = _min_value(to_explore, problem)

        neighbors = _generate_neighbors(state_node, problem)


        for n in neighbors:
            if (n not in expanded) and (n not in to_explore):
                if problem.is_goal(n.state):
                    goal_found = n
                    break
                else:
                    to_explore.append(n)

        expanded.add(state_node)

        #viewer.update(state_node.state,generated=to_explore,expanded=expanded)

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)
    n_expandidos = len(expanded)

    return path, cost, n_expandidos

def astar_search(problem: ProblemInterface, viewer: ViewerInterface) -> Tuple[List[Any], float]:
    # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()

    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())
    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None
    
    # Repeat while we haven't found the goal and still have
    # nodes to expand. If there aren't further nodes
    # to expand in breadth-first search, the goal is
    # unreachable.
    while (len(to_explore) > 0) and (goal_found is None):
        # select next node or expansion
        state_node = _min_value_astar(to_explore, problem)

        neighbors = _generate_neighbors(state_node, problem)


        for n in neighbors:
            if (n not in expanded) and (n not in to_explore):
                if problem.is_goal(n.state):
                    goal_found = n
                    break
                else:
                    to_explore.append(n)

        expanded.add(state_node)

        #viewer.update(state_node.state,generated=to_explore,expanded=expanded)

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)
    n_expandidos = len(expanded)

    return path, cost, n_expandidos

def _path_cost(problem: ProblemInterface, path: List[Node]) -> float:
    if len(path) == 0:
        return inf
    cost = 0
    for i in range(1, len(path)):
        cost += problem.step_cost(path[i].previous_node.state,
                                  path[i].action,
                                  path[i].state)
    return cost


def _extract_path(goal: Union[Node, None]) -> List[Node]:
    path = []
    state_node = goal
    while state_node is not None:
        path.append(state_node)
        state_node = state_node.previous_node
    path.reverse()
    return path


def _generate_neighbors(state_node: Node, problem: ProblemInterface) -> List[Node]:
    # generate neighbors of the current state
    neighbors = []
    state = state_node.state
    available_actions = problem.actions(state)
    for action in available_actions:
        next_state = problem.transition(state, action)
        neighbors.append(Node(next_state, action, state_node))
    return neighbors
    
#Função que retorna o menor nó da fronteira para a busca uniform-cost
def _min_value(fronteira: List[Node], problem: ProblemInterface) -> List[Node]:
    custoa = 1000
    for i in fronteira:
        custo = _path_cost(problem, _extract_path(i))
        if custo < custoa:
            node = i
            custoa = custo
    fronteira.remove(node)
    return node

#Função que retorna o menor nó da fronteira para a busca A*
def _min_value_astar(fronteira: List[Node], problem: ProblemInterface) -> List[Node]:
    custoa = 1000
    for i in fronteira:
        custo = _path_cost(problem, _extract_path(i)) + problem.heuristic_cost(i.state)
        if custo < custoa:
            node = i
            custoa = custo
    fronteira.remove(node)
    return node

import heapq  # smallest element is always at the root
import copy
import time

# Define the problem


class Problem:
    def __init__(self, initial, goal, size):
        self.initial = initial
        self.goal = goal
        self.size = size

    def is_goal(self, state):
        return state["pawns"] == []

    def state_track(self, state):
        return (
            state["bishop"],
            state["rook"],
            tuple(sorted(state["pawns"])),
            tuple(sorted(state["obstacles"]))
        )

        # First heuristic function

    def h1(self, states):
        pawn_number = 0
        h1_cost = 0
        x, y = states["rook"]
        for i, j in states["pawns"]:
            if i == x or j == y:
                pawn_number += 1
            else:
                pawn_number = len(states["pawns"]) + 1
        h1_cost = pawn_number*8
        return h1_cost

    # Second heuristic function

    def h2(self, states):
        # ignore the obstacles and restriction of the movements, so that the function doesn't overestimate
        # first collect the nearest pawn, then continue with the second nearest
        bishop = states["bishop"]
        rook = states["rook"]
        pawns = states["pawns"]
        if not pawns:
            return 0  # goal state, heuristic is zero
        dist_pawn_bishop = [
            abs(bishop[0]-pawn[0]) + abs(bishop[1]-pawn[1]) for pawn in pawns]
        dist_pawn_rook = [
            abs(rook[0]-pawn[0]) + abs(rook[1]-pawn[1]) for pawn in pawns]
        nearest_pawn_bishop = min(dist_pawn_bishop)
        nearest_pawn_rook = min(dist_pawn_rook)
        return nearest_pawn_bishop + nearest_pawn_rook

    def h_combined(self, states):
        cost_h1 = self.h1(states)
        cost_h2 = self.h2(states)
        if states["rook"]:
            return cost_h1 + cost_h2
        else:
            return cost_h2

    # Expand the nodes
    def expand(self, state, moves, cost):
        successors = []
        for piece, action in moves.items():
            state_list = action(self.size, state, cost)
            for new_state, last_cost in state_list:
                g_cost = cost + last_cost  # real cost
                h2_cost = self.h2(new_state)  # estimated cost
                if new_state["rook"]:
                    h1_cost = self.h1(new_state)
                    h_cost = h1_cost + h2_cost
                else:
                    h_cost = h2_cost
                f_cost = g_cost + h_cost  # total cost
                successors.append((new_state, g_cost, h_cost, f_cost))
        return successors


# Graph Search


def uniform_cost_search(problem, moves):
    closed = set()
    fringe = []  # create a heap
    expanded_nodes = 0
    # (cumulative cost, state, previous state)
    heapq.heappush(fringe, (0, problem.initial))

    while fringe:
        expanded_nodes += 1
        # remove and return the smallest element
        cost, state = heapq.heappop(fringe)
        print("Exploring state:")
        board(problem.size, state)

        if problem.is_goal(state):
            print(f"Expanded: {expanded_nodes}")
            print(f"Path-cost: {cost}")
            print("Goal reached!")
            return state  # if the solution is found

        closed.add(problem.state_track(state))

        for new_state, new_cost, _, _ in problem.expand(state, moves, cost):
            if problem.state_track(new_state) not in closed:
                closed.add(problem.state_track(new_state))
                heapq.heappush(
                    fringe, (new_cost, new_state))
    print("Final board state:")
    board(problem.size, state)
    return "No solution."  # if there is no solution


def greedy_search(problem, moves):
    closed = set()
    fringe = []
    expanded_nodes = 0
    initial_h1_cost = problem.h1(problem.initial)
    initial_h2_cost = problem.h2(problem.initial)
    print(f"h1: {initial_h1_cost}")
    print(f"h2: {initial_h2_cost}")

    heapq.heappush(fringe, (initial_h1_cost +
                   initial_h2_cost, problem.initial))

    while fringe:
        expanded_nodes += 1
        cost, state = heapq.heappop(fringe)
        print("Exploring state:")
        board(problem.size, state)

        if problem.is_goal(state):
            print(f"Expanded: {expanded_nodes}")
            print(f"Path-cost: {cost}")
            print("Goal reached!")
            return state

    closed.add(problem.state_track(state))

    for new_state, new_cost, _, _ in problem.expand(state, moves, cost):
        if problem.state_track not in closed:
            closed.add(problem.state_track(state))
            heapq.heappush(
                fringe, (new_cost, new_state))
    print("Final board state:")
    board(problem.size, state)
    return "No solution."


def a_star_search(problem, moves):
    closed = set()
    fringe = []
    expanded_nodes = 0
    initial_g_cost = 0
    initial_h1_cost = problem.h1(problem.initial)
    initial_h2_cost = problem.h2(problem.initial)
    print(f"h1: {initial_h1_cost}")
    print(f"h2: {initial_h2_cost}")
    state_tuple = problem.state_track(problem.initial)
    heapq.heappush(
        fringe, (initial_h1_cost+initial_h2_cost, initial_g_cost, state_tuple, problem.initial))

    while fringe:
        expanded_nodes += 1
        f_cost, g_cost, _, state = heapq.heappop(fringe)
        print("Exploring state:")
        board(problem.size, state)

        if problem.is_goal(state):
            print(f"Expanded: {expanded_nodes}")
            print(f"Path-cost: {g_cost}")
            print("Goal reached!")
            return state
        closed.add(problem.state_track(state))
        # total cost is computed after expanding the node, based on the new state's general cost and its corresponding heuristic cost
        for new_state, new_g_cost, _, _ in problem.expand(state, moves, g_cost):
            key = problem.state_track(new_state)
            if key not in closed:
                closed.add(key)
                h_cost = problem.h_combined(new_state)
                new_f_cost = h_cost + new_g_cost
                heapq.heappush(fringe, (new_f_cost, new_g_cost, key, new_state))
    print("Final board state:")
    board(problem.size, state)
    return "No solution."

# Display board


def board(N, state):
    print(state)
    board = [["." for i in range(N)] for j in range(N)]
    if 0 <= state["bishop"][0] < N and 0 <= state["bishop"][1] < N:
        board[state["bishop"][0]][state["bishop"][1]] = "B"

    if 0 <= state["rook"][0] < N and 0 <= state["rook"][1] < N:
        board[state["rook"][0]][state["rook"][1]] = "R"

    for i, place in enumerate(state["pawns"]):
        if 0 <= place[0] < N and 0 <= place[1] < N:
            board[place[0]][place[1]] = str(i + 1)

    for place in state["obstacles"]:
        if 0 <= place[0] < N and 0 <= place[1] < N:
            board[place[0]][place[1]] = "X"
    for row in board:
        print(" ".join(row))

# In order to create nodes


def move_bishop(N, states, cost):
    new_states = []
    initial_x, initial_y = states["bishop"]
    directions = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    # Movement
    for dx, dy in directions:
        x, y = initial_x, initial_y
        # Borders
        while 0 <= x < N and 0 <= y < N:
            # Move in the same direction
            x += dx
            y += dy
            # Stop when there is a stone other than a pawn
            if (x, y) in states["obstacles"]:
                break
            if (x, y) == states["rook"]:
                continue
            current_cost = cost + 10
            # When encountered, capture the pawn
            new_state = copy.deepcopy(states)
            if (x, y) in states["pawns"]:
                new_state["pawns"].remove((x, y))
                new_state["bishop"] = (x, y)
                new_states.append((new_state, current_cost))
                break

            new_state["bishop"] = (x, y)
            new_states.append((new_state, current_cost))
            # print(f"Cost: {current_cost}")
    return new_states


def move_rook(N, states, cost):
    new_states = []
    initial_x, initial_y = states["rook"]
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    # Movement
    for dx, dy in directions:
        x, y = initial_x, initial_y
        # Borders
        while 0 <= x < N and 0 <= y < N:
            x += dx
            y += dy
            # Stop when there is a stone other than a pawn
            if (x, y) in states["obstacles"]:
                break
            if (x, y) == states["bishop"]:
                continue
            current_cost = cost + 8
            # When encountered, delete the pawn
            new_state = copy.deepcopy(states)
            if (x, y) in states["pawns"]:
                new_state["pawns"].remove((x, y))
                new_state["rook"] = (x, y)
                new_states.append((new_state, current_cost))
                break

            new_state["rook"] = (x, y)
            new_states.append((new_state, current_cost))
            # print(f"Cost: {current_cost}")
    return new_states


moves = {
    "bishop": move_bishop,
    "rook": move_rook,
}

states = {
    "bishop": (0, 3),
    "rook": (0, 4),
    "pawns": [(1, 0), (1, 1), (2, 3), (3, 4), (5, 6), (7, 1), (8, 3), (8, 4)],
    "obstacles": [(0, 5), (1, 2), (1, 3), (1, 6), (3, 2), (3, 6), (6, 7)]
}

problem = Problem(initial=states, goal={"pawns": []}, size=9)
print("Initial state:")
board(9, states)
# uniform_cost_search(problem, moves)
# greedy_search(problem, moves)
a_star_search(problem, moves)
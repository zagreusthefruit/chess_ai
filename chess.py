import heapq  # smallest element is always at the root
import copy
import time


# Priority
stone_precedence = {
    "knight_precedence": 1,
    "bishop_precedence": 2,
    "rook_precedence": 3
}

direction_precedence = {
    "knight_precedence": {(-1, -2): 1,
                          (-2, -1): 2,
                          (-2, 1): 3,
                          (-1, 2): 4,
                          (1, 2): 5,
                          (2, 1): 6,
                          (2, -1): 7,
                          (1, -2): 8
                          },
    "bishop_precedence": {(-1, -1): 1,
                          (-1, 1): 2,
                          (1, 1): 3,
                          (1, -1): 4
                          },
    "rook_precedence": {(0, -1): 1,
                        (-1, 0): 2,
                        (0, 1): 3,
                        (1, 0): 4
                        }

}

# Define the problem


class Problem:
    def __init__(self, board_state):
        self.initial, self.pawns, self.size = self.parse_board(board_state)
        self.state_dictionary()

    @staticmethod
    def parse_board(board_state):
        states = {
            "bishop": None,
            "rook": None,
            "knight": None,
            "obstacles": []
        }
        with open(board_state, 'r') as f:
            lines = f.readlines()
        rows = [line.strip().split() for line in lines]
        pawn_states = []
        board_size = len(rows)
        for x in range(board_size):
            for y, rank in enumerate(rows[x]):
                if rank == 'B':
                    states["bishop"] = (x, y)
                elif rank == 'R':
                    states["rook"] = (x, y)
                elif rank == 'K':
                    states["knight"] = (x, y)
                elif rank == 'X' or rank == 'x':
                    states["obstacles"].append((x, y))
                elif rank.isdigit():
                    pawn_states.append((x, y))
        return states, pawn_states, board_size

    def state_dictionary(self):
        if "bishop" not in self.initial:
            self.initial["bishop"] = None
        if "rook" not in self.initial:
            self.initial["rook"] = None
        if "knight" not in self.initial:
            self.initial["knight"] = None

    def is_goal(self, pawns):
        return len(pawns) == 0

    def state_track(self, state):
        return (
            state["bishop"],
            state["rook"],
            state["knight"],
            tuple(sorted(state["obstacles"]))
        )

    # First heuristic function

    def h1(self, states, pawns):
        if states["rook"] == None:
            return 0
        pawn_number = 0
        h1_cost = 0
        x, y = states["rook"]
        for i, j in pawns:
            if i == x or j == y:
                pawn_number += 1
            else:
                pawn_number = len(pawns) + 1
        h1_cost = pawn_number*8
        return h1_cost

    # Second heuristic function

    def h2(self, states, pawns):
        # ignore the obstacles and restriction of the movements, so that the function doesn't overestimate
        # first collect the nearest pawn, then continue with the second nearest
        bishop = states["bishop"]
        rook = states["rook"]
        knight = states["knight"]
        if not pawns:
            return 0  # goal state, heuristic is zero
        if bishop:
            dist_pawn_bishop = [
                abs(bishop[0]-pawn[0]) + abs(bishop[1]-pawn[1]) for pawn in pawns]
            nearest_pawn_bishop = min(dist_pawn_bishop)
        else:
            nearest_pawn_bishop = 0
        if rook:
            dist_pawn_rook = [
                abs(rook[0]-pawn[0]) + abs(rook[1]-pawn[1]) for pawn in pawns]
            nearest_pawn_rook = min(dist_pawn_rook)
        else:
            nearest_pawn_rook = 0
        if knight:
            dist_pawn_knight = [
                abs(knight[0]-pawn[0]) + abs(knight[1]-pawn[1]) for pawn in pawns]
            nearest_pawn_knight = min(dist_pawn_knight)
        else:
            nearest_pawn_knight = 0
        return nearest_pawn_bishop + nearest_pawn_rook + nearest_pawn_knight

    def h_combined(self, states, pawn):
        cost_h1 = self.h1(states, pawn)
        cost_h2 = self.h2(states, pawn)
        if states["rook"]:
            return cost_h1 + cost_h2
        else:
            return cost_h2

    # Movements

    def move_bishop(self, N, state, pawns, cost):
        new_states = []
        if state["bishop"] is None:
            return new_states
        initial_x, initial_y = state["bishop"]
        directions = sorted(
            [(dx, dy) for dx, dy in direction_precedence["bishop_precedence"].items()], key=lambda x: x[1])
        # Movement
        for (dx, dy), _ in directions:
            x, y = initial_x, initial_y
            # Borders
            while 0 <= x + dx < N and 0 <= y + dy < N:
                x += dx
                y += dy
                # Stop when there is a stone other than a pawn
                if (x, y) not in state["obstacles"]:
                    last_cost = cost + 10
                    # When encountered, capture the pawn
                    new_state = copy.deepcopy(state)
                    new_pawns = pawns[:]
                    if (x, y) in new_pawns:
                        new_pawns.remove((x, y))

                    new_state["bishop"] = (x, y)
                    new_states.append((new_state, new_pawns, last_cost))
        return new_states

    def move_rook(self, N, state, pawns, cost):
        new_states = []
        if state["rook"] is None:
            return new_states
        initial_x, initial_y = state["rook"]
        directions = sorted(
            [(dx, dy) for dx, dy in direction_precedence["rook_precedence"].items()], key=lambda x: x[1])
        # Movement
        for (dx, dy), _ in directions:
            x, y = initial_x, initial_y
            # Borders
            while 0 <= x + dx < N and 0 <= y + dy < N:
                x += dx
                y += dy
                # Stop when there is a stone other than a pawn
                if (x, y) not in state["obstacles"]:
                    last_cost = cost + 8
                    # When encountered, delete the pawn
                    new_state = copy.deepcopy(state)
                    new_pawns = pawns[:]
                    if (x, y) in new_pawns:
                        new_pawns.remove((x, y))

                    new_state["rook"] = (x, y)
                    new_states.append((new_state, new_pawns, last_cost))
        return new_states

    def move_knight(self, N, state, pawns, cost):
        new_states = []
        if state["knight"] is None:
            return new_states
        initial_x, initial_y = state["knight"]
        directions = sorted(
            [(dx, dy) for dx, dy in direction_precedence["knight_precedence"].items()], key=lambda x: x[1])
        # Movement
        for (dx, dy), _ in directions:
            x, y = initial_x, initial_y
            # Borders
            if 0 <= x + dx < N and 0 <= y + dy < N:
                x += dx
                y += dy
                # Stop when there is a stone other than a pawn
                if (x, y) not in state["obstacles"]:
                    current_cost = cost + 8
                    # When encountered, delete the pawn
                    new_state = copy.deepcopy(state)
                    new_pawns = pawns[:]
                    if (x, y) in new_pawns:
                        new_pawns.remove((x, y))

                    new_state["knight"] = (x, y)
                    new_states.append((new_state, new_pawns, current_cost))
        return new_states

    # Expand the nodes
    def expand(self, state, pawns, cost):
        successors = []
        state_list = []
        pieces = sorted(stone_precedence.items(), key=lambda x: x[1])
        for piece, _ in pieces:
            if piece == "bishop_precedence" and state["bishop"] != None:
                state_list = self.move_bishop(
                    self.size, state, pawns, cost)
            elif piece == "rook_precedence" and state["rook"] != None:
                state_list = self.move_rook(
                    self.size, state, pawns, cost)
            if piece == "knight_precedence" and state["knight"] != None:
                state_list = self.move_knight(
                    self.size, state, pawns, cost)

            for new_state, last_pawn, last_cost in state_list:
                g_cost = cost + last_cost  # real cost
                h2_cost = self.h2(new_state, last_pawn)  # estimated cost
                if new_state["rook"]:
                    h1_cost = self.h1(new_state, last_pawn)
                    h_cost = h1_cost + h2_cost
                else:
                    h_cost = h2_cost
                f_cost = g_cost + h_cost  # total cost
                successors.append(
                    (new_state, last_pawn, g_cost, h_cost, f_cost))
        return successors


# Graph Search


def uniform_cost_search(problem, output):
    closed = set()
    fringe = []  # create a heap
    expanded_nodes = 0
    state_tuple = problem.state_track(problem.initial)
    start_time = time.time()
    # (cumulative cost, state, previous state)
    heapq.heappush(fringe, (0, state_tuple, problem.initial, problem.pawns))

    output_lines = []
    with open(output, 'w') as f:
        while fringe:
            expanded_nodes += 1
            # remove and return the smallest element
            cost, _, state, pawn = heapq.heappop(fringe)

            if problem.is_goal(pawn):
                output_lines.insert(0, f"Expanded nodes: {expanded_nodes}\n")
                output_lines.insert(1, f"Path-cost: {cost}\n")
                output_lines.append(board(state, pawn, problem))
                end_time = time.time()
                output_lines.append(
                    f"Execution time: {end_time - start_time:.4f} seconds\n")
                break

            closed.add(problem.state_track(state))

            for new_state, new_pawn, new_cost, _, _ in problem.expand(state, pawn, cost):
                key = problem.state_track(new_state)
                if key not in closed:
                    closed.add(key)
                    heapq.heappush(
                        fringe, (new_cost, key, new_state, new_pawn))
                    
            output_lines.append(board(state, pawn, problem))

    end_time = time.time()
    output_lines.append(
        f"Execution time: {end_time - start_time:.4f} seconds\n")
    output_lines.insert(0, f"Uniform Cost Search\nExpanded nodes: {expanded_nodes}\n")
    output_lines.insert(1, f"Path-cost: {cost}\n")
    with open(output, 'w') as f:
        f.writelines(output_lines)
    return None


def greedy_search(problem, output):
    closed = set()
    fringe = []
    expanded_nodes = 0
    initial_h1_cost = problem.h1(problem.initial, problem.pawns)
    initial_h2_cost = problem.h2(problem.initial, problem.pawns)
    state_tuple = problem.state_track(problem.initial)
    start_time = time.time()
    heapq.heappush(fringe, (initial_h1_cost +
                   initial_h2_cost, state_tuple, problem.initial, problem.pawns))
    output_lines = []
    output_lines.append(f"Initial h1: {initial_h1_cost}\n")
    output_lines.append(f"Initial h2: {initial_h2_cost}\n")

    with open(output, 'w') as f:
        while fringe:
            expanded_nodes += 1
            h_cost, _, state, pawn = heapq.heappop(fringe)
            if problem.is_goal(pawn):
                output_lines.insert(0, f"Expanded nodes: {expanded_nodes}\n")
                output_lines.insert(1, f"Path-cost: {h_cost}\n")
                output_lines.append(board(state, pawn, problem))
                end_time = time.time()
                output_lines.append(
                    f"Execution time: {end_time - start_time:.4f} seconds\n")
            break

    closed.add(problem.state_track(state))

    for new_state, new_pawn, new_cost, _, _ in problem.expand(state, pawn, h_cost):
        key = problem.state_track(new_state)
        if key not in closed:
            closed.add(key)
            heapq.heappush(
                fringe, (new_cost, key, new_state, new_pawn))

            output_lines.append(board(state, pawn, problem))

    end_time = time.time()
    output_lines.append(
        f"Execution time: {end_time - start_time:.4f} seconds\n")
    output_lines.insert(0, f"Greedy Search\nExpanded nodes: {expanded_nodes}\n")
    output_lines.insert(1, f"Path-cost: {h_cost}\n")
    with open(output, 'w') as f:
        f.writelines(output_lines)
    return None


def a_star_search(problem, output):
    closed = set()
    fringe = []
    expanded_nodes = 0
    initial_g_cost = 0
    initial_h1_cost = problem.h1(problem.initial, problem.pawns)
    initial_h2_cost = problem.h2(problem.initial, problem.pawns)
    state_tuple = problem.state_track(problem.initial)
    start_time = time.time()
    heapq.heappush(
        fringe, (initial_h1_cost+initial_h2_cost, initial_g_cost, state_tuple, problem.initial, problem.pawns))

    output_lines = []

    output_lines.append(f"Initial h1: {initial_h1_cost}\n")
    output_lines.append(f"Initial h2: {initial_h2_cost}\n")

    with open(output, 'w') as f:
        while fringe:
            expanded_nodes += 1
            f_cost, g_cost, _, state, pawn = heapq.heappop(fringe)
            if problem.is_goal(pawn):
                output_lines.insert(0, f"Expanded nodes: {expanded_nodes}\n")
                output_lines.insert(1, f"Path-cost: {g_cost}\n")
                output_lines.append(board(state, pawn, problem) + "\n")
                end_time = time.time()
                output_lines.append(
                    f"Execution time: {end_time - start_time:.4f} seconds\n")
                break

            closed.add(problem.state_track(state))
            # total cost is computed after expanding the node, based on the new state's general cost and its corresponding heuristic cost
            for new_state, new_pawn, new_g_cost, _, _ in problem.expand(state, pawn, g_cost):
                key = problem.state_track(new_state)
                if key not in closed:
                    closed.add(key)
                    h_cost = problem.h_combined(new_state, new_pawn)
                    new_f_cost = h_cost + new_g_cost
                    heapq.heappush(
                        fringe, (new_f_cost, new_g_cost, key, new_state, new_pawn))

            output_lines.append(board(state, pawn, problem))


    end_time = time.time()
    output_lines.append(
        f"Execution time: {end_time - start_time:.4f} seconds\n")
    output_lines.insert(0, f"A* Search:\nExpanded nodes: {expanded_nodes}\n")
    output_lines.insert(1, f"Path-cost: {g_cost}\n")
    with open(output, 'w') as f:
        f.writelines(output_lines)

    return None


# Display board


def board(state, pawn, problem):
    N = problem.size
    board_representation = []
    board = [["." for i in range(N)] for j in range(N)]
    pawn_represent = copy.deepcopy(problem.pawns)

    for location in pawn_represent:
        if location in pawn:
            board[location[0]][location[1]] = str(
                pawn_represent.index(location) + 1)
        else:
            board[location[0]][location[1]] = "."

    for place in state["obstacles"]:
        if 0 <= place[0] < N and 0 <= place[1] < N:
            board[place[0]][place[1]] = "X"

    if state["bishop"]:
        if 0 <= state["bishop"][0] < N and 0 <= state["bishop"][1] < N:
            board[state["bishop"][0]][state["bishop"][1]] = "B"

    if state["rook"]:
        if 0 <= state["rook"][0] < N and 0 <= state["rook"][1] < N:
            board[state["rook"][0]][state["rook"][1]] = "R"

    if state["knight"]:
        if 0 <= state["knight"][0] < N and 0 <= state["knight"][1] < N:
            board[state["knight"][0]][state["knight"][1]] = "K"

    for row in board:
        board_representation.append(" ".join(row))
    board_representation.append("*" * (2 * N - 1) + "\n")

    return "\n".join(board_representation)


board_path = "input-file.txt\\path"

problem = Problem(board_state=board_path)

moves = {
    "bishop": problem.move_bishop,
    "rook": problem.move_rook,
    "knight": problem.move_knight
}

output_file1 = "output_UCS.txt"
output_file2 = "output_GS.txt"
output_file3 = "output_AS.txt"

uniform_cost_search(problem, output_file1)
greedy_search(problem, output_file2)
a_star_search(problem, output_file3)

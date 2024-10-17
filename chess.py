import heapq  # smallest element is always at the root
import copy

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
    def __init__(self, initial, size):
        self.initial = initial
        self.size = size
        self.state_dictionary()

    def state_dictionary(self):
        if "bishop" not in self.initial:
            self.initial["bishop"] = None
        if "rook" not in self.initial:
            self.initial["rook"] = None
        if "knight" not in self.initial:
            self.initial["knight"] = None

    def is_goal(self, state):
        if state["pawns"] == []:
            return True

    def state_track(self, state):
        return (
            state["bishop"],
            state["rook"],
            state["knight"],
            tuple(sorted(state["pawns"])),
            tuple(sorted(state["obstacles"]))
        )

    # First heuristic function

    def h1(self, states):
        if states["rook"] == None:
            return 0
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
        knight = states["knight"]
        pawns = states["pawns"]
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

    def h_combined(self, states):
        cost_h1 = self.h1(states)
        cost_h2 = self.h2(states)
        if states["rook"]:
            return cost_h1 + cost_h2
        else:
            return cost_h2

    # Movements

    def move_bishop(self, N, state, cost):
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
            while 0 <= x < N and 0 <= y < N:
                x += dx
                y += dy
                # Stop when there is a stone other than a pawn
                if (x, y) in state["obstacles"]:
                    break
                if (x, y) == state["rook"] or (x, y) == state["knight"]:
                    continue
                current_cost = cost + 10
                # When encountered, capture the pawn
                new_state = copy.deepcopy(states)
                if (x, y) in state["pawns"]:
                    new_state["pawns"].remove((x, y))
                    new_state["bishop"] = (x, y)
                    new_states.append((new_state, current_cost))
                    break

                new_state["bishop"] = (x, y)
                new_states.append((new_state, current_cost))
                # print(f"Cost: {current_cost}")
        return new_states

    def move_rook(self, N, state, cost):
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
            while 0 <= x < N and 0 <= y < N:
                x += dx
                y += dy
                # Stop when there is a stone other than a pawn
                if (x, y) in state["obstacles"]:
                    break
                if (x, y) == state["bishop"] or (x, y) == state["knight"]:
                    continue
                current_cost = cost + 8
                # When encountered, delete the pawn
                new_state = copy.deepcopy(states)
                if (x, y) in state["pawns"]:
                    new_state["pawns"].remove((x, y))
                    new_state["rook"] = (x, y)
                    new_states.append((new_state, current_cost))
                    break

                new_state["rook"] = (x, y)
                new_states.append((new_state, current_cost))
                # print(f"Cost: {current_cost}")
        return new_states

    def move_knight(self, N, state, cost):
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
            while 0 <= x + dx < N and 0 <= y + dy < N:
                x += dx
                y += dy
                # Stop when there is a stone other than a pawn
                if (x, y) in state["obstacles"]:
                    break
                if (x, y) == state["bishop"] or (x, y) == state["rook"]:
                    continue
                current_cost = cost + 8
                # When encountered, delete the pawn
                new_state = copy.deepcopy(states)
                if (x, y) in state["pawns"]:
                    new_state["pawns"].remove((x, y))
                    new_state["knight"] = (x, y)
                    new_states.append((new_state, current_cost))
                    break

                new_state["knight"] = (x, y)
                new_states.append((new_state, current_cost))
                # print(f"Cost: {current_cost}")
        return new_states

    # Expand the nodes
    def expand(self, state, cost):
        successors = []
        pieces = sorted(stone_precedence.items(), key=lambda x: x[1])
        for piece, _ in pieces:
            if piece == "bishop_precedence" and state["bishop"] != None:
                state_list = self.move_bishop(
                    self.size, state, cost)
            elif piece == "rook_precedence" and state["rook"] != None:
                state_list = self.move_rook(
                    self.size, state, cost)
            if piece == "knight_precedence" and state["knight"] != None:
                state_list = self.move_knight(
                    self.size, state, cost)

            for new_state, last_cost in state_list:
                g_cost = cost + last_cost  # real cost
                h2_cost = self.h2(new_state)  # estimated cost
                if new_state["rook"]:
                    h1_cost = self.h1(new_state)
                    h_cost = h1_cost + h2_cost
                else:
                    h_cost = h2_cost
                f_cost = g_cost + h_cost  # total cost
                successors.append(
                    (new_state, g_cost, h_cost, f_cost))
        return successors


# Graph Search


def uniform_cost_search(problem):
    closed = set()
    fringe = []  # create a heap
    expanded_nodes = 0
    # (cumulative cost, state, previous state)
    heapq.heappush(fringe, (0, problem.initial))

    while fringe:
        expanded_nodes += 1
        # remove and return the smallest element
        cost, state = heapq.heappop(fringe)

        if problem.is_goal(state):
            print(f"Expanded: {expanded_nodes}")
            print(f"Path-cost: {cost}")
            return "Goal reached!"  # if the solution is found

        closed.add(problem.state_track(state))

        for new_state, new_cost, _, _ in problem.expand(state, cost):
            if problem.state_track(new_state) not in closed:
                closed.add(problem.state_track(new_state))
                heapq.heappush(
                    fringe, (new_cost, new_state))
        print("Exploring state:")
        board(state, problem)
    print("Final board state:")
    board(state, problem)
    return "No solution."  # if there is no solution


def greedy_search(problem):
    closed = set()
    fringe = []
    expanded_nodes = 0
    initial_h1_cost = problem.h1(problem.initial)
    initial_h2_cost = problem.h2(problem.initial)

    heapq.heappush(fringe, (initial_h1_cost +
                   initial_h2_cost, problem.initial))

    while fringe:
        expanded_nodes += 1
        # looks for order when there is a tie
        cost, state = heapq.heappop(fringe)

        if problem.is_goal(state):
            print(f"Expanded: {expanded_nodes}")
            print(f"Path-cost: {cost}")
            print(f"h1: {initial_h1_cost}")
            print(f"h2: {initial_h2_cost}")
            return "Goal reached!"

    closed.add(problem.state_track(state))

    for new_state, new_cost, _, _ in problem.expand(state, cost):
        if problem.state_track not in closed:
            closed.add(problem.state_track(state))
            heapq.heappush(
                fringe, (new_cost, new_state))
        print("Exploring state:")
        board(state, problem)
    print("Final board state:")
    board(state, problem)
    return "No solution."


def a_star_search(problem):
    closed = set()
    fringe = []
    expanded_nodes = 0
    initial_g_cost = 0
    initial_h1_cost = problem.h1(problem.initial)
    initial_h2_cost = problem.h2(problem.initial)
    state_tuple = problem.state_track(problem.initial)

    heapq.heappush(
        fringe, (initial_h1_cost+initial_h2_cost, initial_g_cost, state_tuple, problem.initial))

    while fringe:
        expanded_nodes += 1
        f_cost, g_cost, _, state = heapq.heappop(fringe)

        if problem.is_goal(state):
            print(f"Expanded: {expanded_nodes}")
            print(f"Path-cost: {g_cost}")
            print(f"h1: {initial_h1_cost}")
            print(f"h2: {initial_h2_cost}")
            return "Goal reached!"

        closed.add(problem.state_track(state))
        # total cost is computed after expanding the node, based on the new state's general cost and its corresponding heuristic cost
        for new_state, new_g_cost, _, _ in problem.expand(state, g_cost):
            key = problem.state_track(new_state)
            if key not in closed:
                closed.add(key)
                h_cost = problem.h_combined(new_state)
                new_f_cost = h_cost + new_g_cost
                heapq.heappush(
                    fringe, (new_f_cost, new_g_cost, key, new_state))
        print("Exploring state:")
        board(state, problem)
    print("Final board state:")
    board(state, problem)
    return "No solution."


# Display board



def board(state, problem):
    print(state)
    N = problem.size
    board = [["." for i in range(N)] for j in range(N)]
    pawn_represent = copy.deepcopy(problem.initial["pawns"])

    for _, location in enumerate(state["pawns"]):
        if location in pawn_represent:
            board[location[0]][location[1]] = str(pawn_represent.index(location) + 1)
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
        print(" ".join(row))
    print("*"*(N+5))


states = {
    "bishop": (0, 0),
    "rook": (0, 4),
    "knight": (2, 2),
    "pawns": [(4, 1), (4, 3)],
    "obstacles": [(2, 1), (3, 3)]
}

board_size = 5
problem = Problem(initial=states, size=board_size)

moves = {
    "bishop": problem.move_bishop,
    "rook": problem.move_rook,
    "knight": problem.move_knight
}

print("Initial state:")
board(states, problem)
# uniform_cost_search(problem)
# greedy_search(problem)
a_star_search(problem)

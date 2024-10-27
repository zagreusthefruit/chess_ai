

def n_queens_problem(n):
    variables = [f"Q[{i+1}]" for i in range(n)]
    domains = [(row, column) for row in range(n) for column in range(n)]
    constraints = []
    location = {q: () for q in variables}
    for q1, l1 in location.items():
        for q2, l2 in location.items():
            if q1 != q2:
                r1, r2 = int(l1[0]), int(l2[0])
                c1, c2 = int(l1[1]), int(l2[1])
                constraints.append(f"Rows, Columns and Diagonals: {q1} != {q2}")

    return variables, domains, constraints


def map_coloring_problem(n):
    variables = ["WA", "NT", "Q", "NSW", "V", "SA", "T"]
    domains = [f"c{i+1}" for i in range(n)]
    adjacent_regions = {
        "WA": ["NT", "SA"],
        "NT": ["WA", "SA", "Q"],
        "SA": ["WA", "NT", "Q", "NSW", "V"],
        "Q": ["NT", "SA", "NSW"],
        "NSW": ["SA", "Q", "V"],
        "V": ["SA", "NSW"],
        "T": []
    }
    constraints = [f"{region1} != {region2}" for region1 in adjacent_regions for region2 in adjacent_regions[region1]]

    return variables, domains, constraints


def to_to_for_problem(n):


def create_problem(type, n):
    if type == "P1":
        return n_queens_problem(n)
    if type == "P2":
        return map_coloring_problem(n)
    if type == "P3":
        return to_to_for_problem(n)
    else:
        raise ValueError("Problem type is not defined.")

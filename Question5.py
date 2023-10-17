import random

def checkAttacks(x1, y1, x2, y2):
    return y1 == y2 or x1 - y1 == x2 - y2 or x1 + y1 == x2 + y2

def heuristic_cost(chessboard):
    cost = 0
    for i in range(8):
        for j in range(i + 1, 8):
            if checkAttacks(i, chessboard[i], j, chessboard[j]):
                cost += 1
    return cost

def steepest_ascent_hill_climbing():
    current_state = [random.randint(0, 7) for _ in range(8)]
    current_cost = heuristic_cost(current_state)
    steps = 0

    while current_cost > 0:
        best_move = None
        best_cost = current_cost

        for i in range(8):
            for j in range(8):
                if current_state[i] != j:
                    new_state = list(current_state)
                    new_state[i] = j
                    new_cost = heuristic_cost(new_state)

                    if new_cost < best_cost:
                        best_move = (i, j)
                        best_cost = new_cost

        if best_move is None:
            break

        i, j = best_move
        current_state[i] = j
        current_cost = best_cost
        steps += 1

    return current_state, steps

success_count = 0
failure_count = 0
success_steps = []
failure_steps = []

for _ in range(1000):
    result, steps = steepest_ascent_hill_climbing()
    if heuristic_cost(result) == 0:
        success_count += 1
        success_steps.append(steps)
    else:
        failure_count += 1
        failure_steps.append(steps)

print(f"Successful runs: {success_count}")
print(f"Average steps for successful runs: {sum(success_steps) / success_count if success_count > 0 else 0}")
print(f"Failed runs: {failure_count}")
print(f"Average steps for failed runs: {sum(failure_steps) / failure_count if failure_count > 0 else 0}")


import random

def checkAttacks(x1, y1, x2, y2):
    return y1 == y2 or x1 - y1 == x2 - y2 or x1 + y1 == x2 + y2

def heuristic_cost(chessboard):
    cost = 0
    for i in range(8):
        for j in range(i + 1, 8):
            if checkAttacks(i, chessboard[i], j, chessboard[j]):
                cost += 1
    return cost

def steepest_ascent_hill_climbing_with_sideways_moves(max_sideways_moves):
    current_state = [random.randint(0, 7) for _ in range(8)]
    current_cost = heuristic_cost(current_state)
    steps = 0

    while current_cost > 0:
        best_moves = []
        best_cost = current_cost

        for i in range(8):
            for j in range(8):
                if current_state[i] != j:
                    new_state = list(current_state)
                    new_state[i] = j
                    new_cost = heuristic_cost(new_state)

                    if new_cost < best_cost:
                        best_moves = [(i, j)]
                        best_cost = new_cost
                    elif new_cost == best_cost:
                        best_moves.append((i, j))

        if not best_moves:
            break

        i, j = random.choice(best_moves)
        current_state[i] = j
        current_cost = best_cost
        steps += 1

        if steps >= max_sideways_moves:
            break

    return current_state, steps

success_count = 0
failure_count = 0
success_steps = []
failure_steps = []
max_sideways_moves = 100  # According to text book example

for _ in range(1000):
    result, steps = steepest_ascent_hill_climbing_with_sideways_moves(max_sideways_moves)
    if heuristic_cost(result) == 0:
        success_count += 1
        success_steps.append(steps)
    else:
        failure_count += 1
        failure_steps.append(steps)

print(f"Successful runs: {success_count}")
print(f"Average steps for successful runs: {sum(success_steps) / success_count if success_count > 0 else 0}")
print(f"Failed runs: {failure_count}")
print(f"Average steps for failed runs: {sum(failure_steps) / failure_count if failure_count > 0 else 0}")

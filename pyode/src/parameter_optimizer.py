__author__ = 'David Michelman'
__docformat__ = 'restructuredtext en'

import math
import random
import time
import simulation
import lookup_table_hopper
from subprocess import call


def start():
    random.seed(time.time())
    x = list((0.01, 0.0113, 0.0095, 0.012359187569444355, 0.06646608153231316, 0.01348791210060623, 17.04, 0.0389, 0.26385827625458874))
    # x = list((0.0100, 0.0125, 0.0091, 0.0098, 0.1000, 0.0090, 18.3585, 0.0388, 0.5000))
    # x = (0.1, 0.1, 0.1, 0.1, 0.1, 10.0, 10.0, 0.0000000, 0.1)
    fitness_old = fitness(x)
    print("starting fitness: " + str(fitness_old))
    while True:
        differences = list()  # make a new copy of x
        for i in range(2):  # change two values at once because aint nobody got time for one at a time
            index = random.randrange(0, len(x))
            # -.5 /2 +.1 gives random numbers between .5 and 1.5
            differences.append([((random.random() - .5)/1 + 1), index])
        x_new = list(x)
        for i in differences:
            x_new[i[1]] *= i[0]
            x_new[i[1]] = abs(x_new[i[1]])
        fitness_new = fitness(x_new)
        if fitness_new > fitness_old:
            print_new_fitness(x_new, fitness_new)
            fitness_old = fitness_new

            sub_differences, sub_fitness = try_all_combinations(x, differences)
            if sub_fitness > fitness_old:
                fitness_old = sub_fitness
                differences = sub_differences

                for i in differences:
                    x[i[1]] *= i[0]
                print_new_fitness(x, fitness_old)

        else:
            print("\ttried ", end=' ')
            lookup_table_hopper.print_tupple(x_new, new_line=False, precision=4)
            print((" had fitness of " + str(fitness_new)))


# TODO: test this
def try_all_combinations(x, differences):
    max_fitness = 0
    best_differences = differences
    for i in differences:
        i_index = differences.index(i)
        differences.remove(i)

        # try differences without i
        x_new = list(x)
        for j in differences:
            x_new[j[1]] *= j[0]
            x_new[j[1]] = abs(x_new[j[1]])
        new_fitness = fitness(x_new)
        if new_fitness > max_fitness:
            max_fitness = new_fitness
            best_differences = differences
            print("new sub x =", end=' ')
            call(["wall", str(x_new)])
        else:
            print("\ttried ", end=' ')
        lookup_table_hopper.print_tupple(x_new, precision=4)
        print("as sub with fitness of", new_fitness)

        # try all combinations without i
        if len(differences) > 0:  # don't do this if differences is empty
            sub_differences, sub_fitness = try_all_combinations(x, best_differences)
            if sub_fitness > max_fitness:
                max_fitness = sub_fitness
                best_differences = sub_differences

        differences.insert(i_index, i)  # put it back in the same place so the for loop doesn't repeat elements

    return best_differences, max_fitness


def fitness(weights):
    """
    Determines the fitness (darwinian) of the passed value. Higher return value is better
    :param weights:
    :return: The fitness of the passed value
    :rtype : float
    """
    run_time = 20
    fps = 50
    target_state = (math.pi - 90 * math.pi / 180, 0 * math.pi / 180, 0.0, 0.0)
    return_raw = simulation.run(starting_q1=90 * math.pi / 180, starting_q2=10 * math.pi / 180,
                                starting_q1v=0, starting_q2v=0, headless=True, maximum_seconds=run_time, passive=False,
                                return_state_vector=False, hopper_weights=weights, use_ode_as_table=False, fps=fps,
                                return_dump=True, exit_on_fail=True, target_state=target_state,
                                lookup_parameters=(2, 17, 1))
    part1 = 1.0 / lookup_table_hopper.dist_between(return_raw[0:4], target_state, weight_set=1)
    total = 0
    for i in return_raw[5]:
        total += lookup_table_hopper.dist_between(i, [math.pi / 2, 0, 0, 0], weight_set=1)
    total /= fps * run_time
    # print "***********************************************"
    # print return_raw[4]
    return part1 + (1.0 / total) + (return_raw[4])


def print_new_fitness(x_new, fitness_new):
    print("new x = ", end=' ')
    lookup_table_hopper.print_tupple(x_new, new_line=False, precision=4)
    print(" new fitness = " + str(fitness_new))
    # print "wall",  "\\" + str(x_new) + "\'"
    call(["wall", str(x_new)])


if __name__ == '__main__':
    start()

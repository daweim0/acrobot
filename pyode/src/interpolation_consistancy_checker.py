import lookup_table_hopper
import random
import lookup_table_hopper_helper
import numpy
import time
import sys

lookup_table_hopper.initialize(42)
table = lookup_table_hopper.table

time_samples = 0
time_total = 0

pruning_factor = 0.3

# for i in range(100):
    # random_tuple = (
    #     random.randint(-2, table.shape[0] + 2) / 0.9, random.randint(-2, table.shape[0] + 2) / 0.9,
    #     random.randint(-2, table.shape[0] + 2) / 0.9, random.randint(-2, table.shape[0] + 2) / 0.9,
    #     random.randint(-2, table.shape[0] + 2) / 0.9)
for a in range(table.shape[0]):
    if random.random() < pruning_factor:
        for b in range(table.shape[1]):
            if random.random() < pruning_factor:
                for c in range(table.shape[2]):
                    if random.random() < pruning_factor:
                        for d in range(table.shape[3]):
                            if random.random() < pruning_factor:
                                for e in range(table.shape[4]):
                                    if random.random() < pruning_factor:
                                        random_tuple = [a/0.79 - 2, b/0.79 - 2, c/0.79 - 2, d/0.79 - 2, e/0.79 - 2]
                                        start_time = time.time()
                                        new_result = lookup_table_hopper_helper.get_next_angles_interpolated(lookup_table_hopper.table, random_tuple)
                                        new_time = time.time() - start_time
                                        old_result = lookup_table_hopper.get_next_angles_interpolated(random_tuple)
                                        old_time = time.time() - start_time - new_time
                                        if numpy.sum(numpy.asarray(new_result) - numpy.asarray(old_result)) > 0.001:
                                            print("borked at", random_tuple)
                                            print("python implementation:", old_result)
                                            print("cython implementation:", new_result)
                                            sys.exit()
                                        else:
                                            print("    ok at", random_tuple, "with times (new, old) (", new_time * 1000, old_time * 1000, ") " + \
                                                  " (", old_time / new_time, "times speed up)")

                                            time_samples += 1
                                            time_total += old_time/new_time
print("finnished with no errors and average speedup " + str(time_total/time_samples) + " (times faster)")

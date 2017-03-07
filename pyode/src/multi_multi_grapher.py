'''
Created on Aug 12, 2014

@author: David
'''

import os 
import sys
import time
import multiprocessing
import multi_grapher

def main(threadded = True):
    
    #threadded
    if threadded:
        print("starting")
        threads = []
        
        for i in range(-15, 15):        
            emptydir("thread" + str(i))
            threads.append(multiprocessing.Process(target= multi_grapher.run, args = (True, 1, 
                            "left s curve friction" + str(chr(i + 10 + ord('a'))) + " = " + str(i), False,
                            "thread" + str(i), 0, i*80.0, "parent_log_balance_curve.csv", 0, "parent_log_balance_curve_left.csv")))
            
#             run(True, 0, "passive_right", True, "logging_test", 10, 0, "parent_log_balance_curve.csv", 0, 
#         "parent_log_balance_curve_right.csv")
            
# def run(headless, friction, save_name, threadded_terminal, logging_dir, maxTime, constant_torque,
#                      overlay_log, starting_torque, parent_log):
    #         threads.append(multiprocessing.Process(target = test, args = (i,)))
            threads[len(threads)-1].start()
        
        while len(threads) != 0:
            threads[0].join()
            threads.pop(0)
            print("joined thread, currently at " + str(len(threads)) + " running threads \n")
    
        print("done")




def emptydir(top):
    if(top == '/' or top == "\\"): return
    else:
        for root, dirs, files in os.walk(top, topdown=False):
            for name in files:
                os.remove(os.path.join(root, name))
            for name in dirs:
                os.rmdir(os.path.join(root, name))

def test(i):
    while True:
        print("worker" + str(i))
        time.sleep(.01)
        

if __name__ == '__main__':
    main()
    
    
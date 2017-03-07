'''
Created on Aug 5, 2014

@author: David
'''

from multiprocessing import Process, Queue
import os
import sys
import time

in_queue = Queue() 
out_queue = Queue()

def __init__():
    global in_queue
    global out_queue
    

    p = Process(target = thread)
    p.start()
#     p.join()


#     b = Process(target = run, args = ())
#     b.start()
#     b.join()

    run()
    
#     thread()
    
def thread():
    global in_queue
    global out_queue
    import simulation
    simulation.run(in_queue, out_queue)

    
def run():
    global in_queue
    global out_queue
    while True:
        if not out_queue.empty():
            command = out_queue.get()
            if command == "exit":
                sys.exit()
        input = input("simulation > ")
        in_queue.put(input)
    
if __name__ == "__main__":
    __init__()
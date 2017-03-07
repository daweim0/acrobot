'''
Created on Aug 28, 2014

@author: David
'''
import math
import simulation


def main():
    print("3d grapher starting")


#     for q1 in range(70,100):
#         for q2 in range(-30, 30):
#             print "hi " + str(q1) + " " +  str(q2)
#             home(q1,q2)
    home(80,10)


def home(q1,q2):
    done = False
    t = 0.0
    a = 100.0
    last_center_x = 0
    while not done:
        l1 = 0.5
        l2 = 0.75
        y1 = math.sin(math.radians(q1)) * l1
        x1 = math.cos(math.radians(q1)) * l1
        x2 = x1 + math.cos((math.radians(q1) + math.radians(q2))) * l2
        y2 = y1 + math.sin(math.radians(q1) + math.radians(q2)) * l2
        
        print("torque = " + str(t) + " a = " + str(a))
        
        
        center_X, out_q1, out_q2 = simulation.run(starting_x1 = x1, starting_x2 = x2,   #@UnusedVariable
                            starting_y1 = y1, starting_y2 = y2, exit_on_fail = True, starting_torque = t, 
                            passive = True, terminal = False, starting_goto = -999, fail_on_not_upright = True,
                            end_when_stable = False, pause_on_end = True)
    
        print("center of gravity = " + str(center_X)) 
        
        if -0.001 < center_X < 0.01:
            print("ended at " + str(t) + " with q1: " + str(out_q1) + " q2: " + str(out_q2))
            break
        elif center_X > 0.0:
            if last_center_x > 0.0:
                t += a
            else:
                a = a/2.0
                t+= -1.0 * a
                print("a flipped \n") 
        elif center_X < 0.0:
            if last_center_x < 0.0:
                t += -1.0 * a
            else:
                a = a/2.0
                t+= a
                print("a flipped \n") 
        last_center_x = center_X
        
    
    '''
    def run(starting_x1 = 0, starting_x2 = 0, starting_y1 = .5, starting_y2 = 1.25, starting_q1v = 0, 
            starting_q2v = 0, starting_damp1 = 0, starting_damp2 = 0, headless = False, terminal = True,
            logFromStart = False, logger_directory = "", fps = 50, exit_on_fail = False, max_seconds = 99999999999999,
            starting_goto = 0, delayed_start = True, passive = False, constant_torque = 0, q1_offset = 0, q2_offset = 0,
             starting_torque = .1):  
    '''



if __name__ == '__main__':
    main()
'''
Created on Aug 11, 2014

@author: David
'''

if __name__ == '__main__':
    lines_to_trim = 5
    input = open("parent-log.csv", "r")
    output = open("parent_log_trimmed.csv", "w")
    
    while True:
        next_line = input.readline()
        if next_line == "":
            break
        output.write(next_line)
        for i in range(0, lines_to_trim):
            input.readline()
    
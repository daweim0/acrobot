#!/usr/bin/python

import os,sys

file = open(sys.argv[1])

stack = []
linum = 1

for line in file.readlines():
    tokens = line.split()
    if len(tokens) > 1:
        cmd = tokens[0]
        item = tokens[1]
        if cmd == 'push':
            stack.append(item)
        elif cmd == 'pop':
            check = stack.pop()
            if check != item:
                print "Error %s <> %s at line %d"%(check,item,linum)
    linum += 1

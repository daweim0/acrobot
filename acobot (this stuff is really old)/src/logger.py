'''
Created on Jul 14, 2014

@author: David
'''
import os

class log:
    #if no filename is passed a sequentialy generated one will be used.
    #the default directory will be used
    def __init__(self, directory = "", filename = "auto", quiet = True, headder = ""):
        self.file_count = len(os.listdir(os.getcwd() + "/" + directory))


 #       file = open((filename + str(self.file_count)), "w")
        self.file = open( directory + "/" + filename + str(self.file_count + 1) + ".csv", "w")# <-- actualy a w
#         self.file = open("hi" + ".txt", "w")# <-- actualy a w
        self.file.write(headder)

    def write(self, data):
        self.file.write(str(data) + ",")


    def write_line(self):
        self.file.write("\n")
        
    def close(self):
        self.file.close()
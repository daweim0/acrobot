'''
Created on Jul 14, 2014

@author: David
'''
import os

class log:
    #if no filename is passed a sequentialy generated one will be used.
    #the default directory will be used
    def __init__(self, directory = "", filename = "auto", quiet = True, headder = ""):
        
        #make sure the directory exists
        if not os.path.exists(directory):
            os.makedirs(directory)
            print("made driectory")
        
        self.file_count = len(os.listdir(os.getcwd() + "/" + directory))
        self.file = open( directory + "/" + filename + str(self.file_count + 1) + ".csv", "w")
        self.file.write(headder)
        
        print("starting log at " + str(directory + "/" + filename + str(self.file_count + 1) + ".csv"))

#writes some data and a comma
    def write(self, data):
        self.file.write(str(data) + ",")

#writes a line
    def write_line(self):
        self.file.write("\n")
        
#closes a file
    def close(self):
        self.file.close()
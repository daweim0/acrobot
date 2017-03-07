from tkinter import *
import os
import subprocess, os

def main():
    print("hi")

#     subprocess.Popen('cmd.exe')
#     
#     os.system("cmd.exe pwd") 

    root = Tk()
    termf = Frame(root, height=400, width=500)
      
    termf.pack(fill=BOTH, expand=YES)
    wid = termf.winfo_id()
#     os.system('xterm -into %d -geometry 40x20 -sb &' % wid)
      
    root.mainloop()


if __name__ == "__main__":
    main()
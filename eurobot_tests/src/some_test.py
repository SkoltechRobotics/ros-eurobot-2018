from Tkinter import *

ROOT = Tk()
LABEL = Label(ROOT, text="Hello, world!")
LABEL.pack()
ROOT.mainloop()
LOOP_ACTIVE = True
while LOOP_ACTIVE:
    print("something")
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import time
import random
import numpy as np
from Tkinter import *
COLORS = np.array([[0, 123, 176], [208, 90, 40], [28, 28, 32], [96, 153, 59], [247, 181, 0]], dtype=np.uint8)
COLORS_NAME = ["blue", "orange", "black", "green", "yellow"]

class App:
    def __init__(self, master):

        frame = Frame(master, bg="white", colormap="new")
        frame.pack()
        frame0 = Frame(frame, bg="white", colormap="new")
        frame0.pack(side="top")
        frame1 = Frame(frame0, bg="white", colormap="new")
        frame1.pack(side="left")
        frame2 = Frame(frame0, bg="white", colormap="new")
        frame2.pack(side="left")
        frame3 = Frame(frame, bg="white", colormap="new")
        frame3.pack(side="top")

        Label(frame1, bg="white", height=1, width=8, font=("Helvetica", 32), text="POINTS").pack(side="top")
        self.points = StringVar()
        Label(frame1, bg="white", height=1, width=5, textvariable=self.points, font=("Helvetica", 128)).pack(side="top")
        self.points.set("0")

        Label(frame2, bg="white", height=1, width=13, font=("Helvetica", 32), text="SIDE").pack(side="top")
        Label(frame2, bg='#%02x%02x%02x' % tuple(COLORS[1]), height=1, width=13, font=("Helvetica", 32)).pack(side="top")
        Label(frame2, bg="white", height=1, width=13, font=("Helvetica", 32), text="COLORS PLAN").pack(side="top")
        self.w = Canvas(frame2, bg="white", width=300, height=100)
        self.rect1 = self.w.create_rectangle(0, 0, 100, 100, fill="red", outline='white')
        self.rect2 = self.w.create_rectangle(100, 0, 200, 100, fill="red", outline='white')
        self.rect3 = self.w.create_rectangle(200, 0, 300, 100, fill="red", outline='white')
        self.w.pack(side="top")
        self.set_plan('orange black green')

        self.wire_status = StringVar()
        self.wire_status.set("0")
        Label(frame3, bg="white", height=1, width=13, font=("Helvetica", 16), text="Startup wire").pack(side="top")
        self.wire_label = Label(frame3, bg="yellow", height=1, width=13, font=("Helvetica", 16),
                                textvariable=self.wire_status)
        self.wire_label.pack(side="top")

    def points_callback(self, data):
        self.points.set(data.data)

    def plan_callback(self, data):
        self.set_plan(data.data)

    def set_plan(self, plan):
        splitted_plan = plan.split()
        self.w.itemconfig(self.rect1, fill='#%02x%02x%02x' % tuple(COLORS[COLORS_NAME.index(splitted_plan[0])]))
        self.w.itemconfig(self.rect2, fill='#%02x%02x%02x' % tuple(COLORS[COLORS_NAME.index(splitted_plan[1])]))
        self.w.itemconfig(self.rect3, fill='#%02x%02x%02x' % tuple(COLORS[COLORS_NAME.index(splitted_plan[2])]))

    def wire_status_callback(self, data):
        self.wire_status.set(data.data)
        if data.data == "0":
            self.wire_label.configure(bg="yellow")
        elif data.data == "1":
            self.wire_label.configure(bg="green")


if __name__ == '__main__':
    rospy.init_node("display_node")
    root = Tk()
    root.title("Eurobot Reset")
    # root.geometry("500x400")
    app = App(root)

    rospy.Subscriber("/server/point", String, app.points_callback)
    rospy.Subscriber("/server/plan", String, app.plan_callback)
    rospy.Subscriber("/server/wire_status", String, app.wire_status_callback)
    rate = rospy.Rate(100)
    rospy.loginfo("Start display")


    # def quit(event):
    #     print "you pressed control c"
    #     root.quit()


    def check():
        root.after(50, check)  # 50 stands for 50 ms.
    root.after(50, check)

    # root.bind('<Control-c>', quit)

    try:
        root.mainloop()
    except:
        print "you pressed control c"
        sys.exit(0)






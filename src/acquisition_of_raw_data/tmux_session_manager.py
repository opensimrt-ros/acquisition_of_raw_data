#!/usr/bin/env python3

import subprocess
import tkinter as tk
from time import sleep
import libtmux
import rospy

window_dic_ ={"aaaa":["echo 1","echo 2", "echo 3"],
        "bbbb":["echo 4","echo 5"]}

start_directory="/catkin_ws/src/ros_biomech"

class TmuxManager:
    def __init__(self,session_name="test"):
        self.name = session_name
        self.srv = libtmux.Server()
    def create_session(self, session_name=None, initial_command=None):
        #self.srv.cmd('new-session', '-d', '-P', '-F#{session_id}','-n',self.name).stdout[0]
        if session_name:
            self.session = self.srv.new_session(session_name, start_directory=start_directory)
        else:
            self.session = self.srv.new_session(self.name,start_directory=start_directory)
        if initial_command:
            rospy.loginfo("initial_command:%s"%initial_command)
            self.session.active_window.active_pane.send_keys(initial_command, enter=True)
        else:
            rospy.logwarn_once("no initial command set")
        ## this doesnt work
        #self.srv.cmd('set-option' ,'-g', 'default-shell', '"/usr/bin/bash','--rcfile','~/.bashrc_ws.sh"')
    def new_tab(self,window_name=""):
        self.srv.cmd('new-window','-P','-n',f"{window_name}")
    def attach(self):
        self.srv.cmd('-2','a','-t',self.name)
        #self.srv.attach_session(self.name)
    def newsplit(self):

        self.session.cmd('split-window','-h')

    def default_splits4(self,num=0):

        pane0 = self.session.windows[num].active_pane
        pane1 = self.session.windows[num].split(direction=libtmux.constants.PaneDirection.Right)
        pane2 = self.session.windows[num].split(direction=libtmux.constants.PaneDirection.Right)
        pane3 = self.session.windows[num].split(direction=libtmux.constants.PaneDirection.Right)

        self.session.cmd('select-layout','even-horizontal')

    def default_splits8(self,num):
        self.default_splits4(num)
        for pane in self.session.windows[num].panes:
            pane.split(direction=libtmux.constants.PaneDirection.Below)

        return self.session.windows[num]

def create_some_windows(window_dic={},some_manager=TmuxManager()):
    for i, (keys, values) in enumerate(window_dic.items()):
        some_manager.new_tab(keys)
        #a.newsplit()
        pp = some_manager.default_splits8(i+1)
        print(pp)
        for j,cmd in enumerate(values):
            print(cmd)
            pp.panes[j].send_keys(cmd, enter=True)



if __name__ == "__main__":
    a = TmuxManager()
    a.create_session("test")
    create_some_windows(window_dic=window_dic_,some_manager=a)
    a.attach()
    #root.mainloop()


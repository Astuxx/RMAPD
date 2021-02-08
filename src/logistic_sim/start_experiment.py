#!/usr/bin/env python
#print 'bash -c \rosrun task_planner TaskPlanner'
    # os.system('gnome-terminal  --tab -e "bash -c \'rosrun task_planner TaskPlanner\'" &')    
    # os.system('sleep 5'
#

import Tkinter as tk
import tkMessageBox
import ConfigParser
import thread
from Tkinter import *
from ttk import *
import Image, tkFileDialog
import numpy as np
import sys, time, os, glob, shutil
from math import atan2, degrees, radians
import datetime
import rospkg

import os
dirname = rospkg.RosPack().get_path('logistic_sim')

Alg_names = [ 
        [ 'RAND', 'Random' ],
        [ 'CR',   'Conscientious_Reactive' ],
        [ 'HCR',  'Heuristic_Conscientious_Reactive' ],
        [ 'HPCC', 'Conscientious_Cognitive' ],
        [ 'CGG',  'Cyclic' ],
        [ 'MSP',  'MSP' ],
        [ 'GBS',  'GBS' ],
        [ 'SEBS', 'SEBS' ],
        [ 'CBLS', 'CBLS' ],
        [ 'DTAG', 'DTAGreedy' ],
        [ 'DTAP', 'DTASSIPart' ],
        [ 'LOGA', 'LOGA'],
        [ 'CYCLE','CYCLE'],
        [ 'CYCLEOPT','CYCLEOPT'],
        [ 'CFAGENT', 'CFAGENT'],
        [ 'TP',   'TP'],
        [ 'TokenAgent', 'TokenAgent'],
        [ 'DistrAgent', 'DistrAgent'],
        [ 'ConstAgent', 'ConstAgent'],
        [ 'SPartAgent','SPartAgent'],
        [ 'CFreeAgent','CFreeAgent'],
        [ 'GlobalAgent','GlobalAgent'],
        [ 'OnlineAgent','OnlineAgent']
     ]

TaskPlanner_names = [ 'TaskPlanner', 'SP_TaskPlanner', 'GlobalTaskPlanner', 'OnlineTaskPlanner' ]

TaskGeneration_names = ['uniform', 'non-uniform']

Map_names = ['cumberland','example','grid','1r5','broughton','DIAG_labs','DIAG_floor1','model1','model2','model3','model4','model5','model6','icelab','icelab_black']   

NRobots_list = ['1','2','4','6','8','10','12']

RobotsCapacity_list = ['3', '5', '7']

LocalizationMode_list = ['AMCL','fake_localization']

NavigationMode_list = ['ros','spqrel_navigation']

GWait_list = ['0','3','10']

CommDelay_list = ['0','0.2','1','2']

LostMsgRate_list = ['0','0.1','0.2','0.3']

Terminal_list = ['gnome-terminal','xterm']

initPoses = {}

COMMDELAY_DEFAULT = 0.0

INITPOS_DEFAULT = "default"

# return long name of the algorithm
def findAlgName(alg):
    r = 'None'
    for i in range(0,len(Alg_names)):
        if (Alg_names[i][0]==alg):
            r = Alg_names[i][1]
    return r

# load initial poses from configuration file
def loadInitPoses():
  try:
    ConfigIP = ConfigParser.ConfigParser()
    ConfigIP.read(dirname+"/params/initial_poses.txt")
    for option in ConfigIP.options("InitialPoses"):
        #print option
        initPoses[option] = ConfigIP.get("InitialPoses", option)
  except:
    print "Could not load initial poses file"


# get ROS time from /clock topic
def getROStime():
    os.system("rostopic echo -n 1 /clock > rostime.txt")
    f = open(dirname+"/rostime.txt",'r')
    t = 0
    for line in f:
        if (line[2:6]=='secs'):
            t = int(line[8:])
    f.close()
    return t

# get running simulation flag from /simulation_running param
def getSimulationRunning():
    os.system("rosparam get /simulation_running > simrun.txt")
    f = open(dirname+"/simrun.txt",'r')
    t = True
    line = f.readline();
    if (line[0:5]=='false'):
        t = False
    f.close()
    return t

# Run the experiment with the given arguments
# Terminates if simulation is stopped (/simulation_running param is false)
# or if timeout is reached (if this is >0)
# CUSTOM_STAGE: use of extended API for stage (requires custom stage and stage_ros).
def run_experiment(MAP, NROBOTS, INITPOS, ALG_SHORT, LOC_MODE, NAV_MODULE, GWAIT, COMMDELAY, TERM, TIMEOUT, CUSTOM_STAGE, SPEEDUP, CAPACITY, TP_NAME, GEN, MISSIONS_FILE):

    ALG = findAlgName(ALG_SHORT)
    print 'Run the experiment'
    print 'Loading map ',MAP
    print 'Initial pos ',INITPOS
    print 'N. robot ',NROBOTS
    print 'Algorithm ',ALG,'  ',ALG_SHORT
    print 'Localization Mode ',LOC_MODE
    print 'Navigation module ', NAV_MODULE
    print 'Goal wait time ', GWAIT
    print 'Communication delay ',COMMDELAY
    print 'Terminal ',TERM
    print 'Timeout ',TIMEOUT
    print 'Custom Stage ',CUSTOM_STAGE
    print 'Simulator speed-up ',SPEEDUP    
    print 'Capacity', CAPACITY

    if (TIMEOUT>0):
        TIMEOUT = TIMEOUT + 10 # Let's give more time to complete actions and logging

    loadInitPoses()
    
    scenario = MAP+"_"+NROBOTS

    if (INITPOS!='default'):
        scenario = scenario+"_"+INITPOS

    iposes = initPoses[scenario.lower()]
    print scenario,'   ',iposes
    
    if (TERM == 'xterm'):
        roscore_cmd = 'xterm -e roscore &'
    else:
        roscore_cmd = 'gnome-terminal -e "bash -c \'roscore\'" &'
    print roscore_cmd

    os.system(roscore_cmd)
    os.system('sleep 3')
    os.system('rosparam set /use_sim_time true')
    os.system("rosparam set /communication_delay "+str(COMMDELAY))
    os.system("rosparam set /navigation_module "+NAV_MODULE)
    os.system("rosparam set /initial_positions "+INITPOS)

    cmd = './setinitposes.py '+MAP+' "'+iposes+'"'
    os.system(cmd)
    print cmd    
    os.system('sleep 1')

    #cmd_monitor = 'rosrun logistic_sim monitor '+MAP+' '+ALG_SHORT+' '+NROBOTS  
    cmd_monitor = ''
    custom_stage = ''
    if (CUSTOM_STAGE=="true"):
      custom_stage = ' custom_stage:=true'
    cmd_stage = 'roslaunch logistic_sim map.launch map:='+MAP+custom_stage
    if (os.getenv('ROS_DISTRO')=='groovy'):
      cmd_stage = cmd_stage + " stage_pkg:=stage"
    print cmd_monitor
    print cmd_stage
    if (TERM == 'xterm'):
        os.system('xterm -e  "'+cmd_monitor+'" &') 
        os.system('xterm -e  "'+cmd_stage+'" &')
    else: 
        os.system('gnome-terminal --tab -e  "bash -c \''+cmd_monitor+'\'" --tab -e "bash -c \''+cmd_stage+'\'" &')
    
    os.system('sleep 3')
    
    # Start robots
    if (LOC_MODE == 'AMCL'):
        robot_launch = 'robot.launch'
    else:
        robot_launch = 'robot_fake_loc.launch'
    
    gcmd = 'gnome-terminal '
    for i in range(0,int(NROBOTS)):
        print 'Run robot ',i
        cmd = 'bash -c \'roslaunch logistic_sim '+robot_launch+' robotname:=robot_'+str(i)+' mapname:='+MAP+' '
        
        # Set navigation modules
        if (NAV_MODULE=="ros"):
           cmd = cmd + ' use_amcl:=true use_move_base:=true '
        elif (NAV_MODULE=="spqrel_navigation"):
           cmd = cmd + ' use_amcl:=false use_move_base:=false use_srrg_localizer:=true use_spqrel_planner:=true '
           
        cmd = cmd + "'"
        print cmd
        if (TERM == 'xterm'):
	     os.system('xterm -e  "'+cmd+'" &')
	     os.system('sleep 1')
        gcmd = gcmd + ' --tab -e "'+cmd+'" '
    gcmd = gcmd + '&'
    if (TERM == 'gnome-terminal'):
	    #print gcmd
	    os.system(gcmd)
    os.system('sleep 5')
    print 'bash -c \rosrun logistic_sim TaskPlanner'
    
    cmd_taskplanner = 'bash -c \'rosrun logistic_sim '+ TP_NAME+' '+ MAP+' '+ALG+' '+str(NROBOTS)+' '+GEN+' '+str(CAPACITY)+' '+str(MISSIONS_FILE)+'\''
    cmd_TP = 'gnome-terminal  --tab -e "'+cmd_taskplanner+'"&'
    
    if (TERM == 'xterm'):
        os.system('xterm -e "'+cmd_taskplanner+'" &')
    else:
        os.system(cmd_TP)
            
    # os.system('gnome-terminal  --tab -e "bash -c \'rosrun logistic_sim TaskPlanner \'" &')    
    os.system('sleep 5')   
        
    # Start patrol behaviors
    gcmd = 'gnome-terminal '
    for i in range(0,int(NROBOTS)):
        print 'Run patrol robot ',i
        if (ALG_SHORT=='MSP'):
            cmd = 'bash -c \'rosrun logistic_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' MSP/'+MAP+'/'+MAP+'_'+str(NROBOTS)+'_'+str(i)+' '+'\''
        elif (ALG_SHORT=='GBS' or ALG_SHORT=='SEBS' or ALG_SHORT=='CBLS'):
            cmd = 'bash -c \'rosrun logistic_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' '+str(NROBOTS)+'\''
        elif (ALG_SHORT=='TP'):
            cmd = 'bash -c \'rosrun logistic_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' robot_'+str(i)+' '+str(CAPACITY)+' '+str(NROBOTS)+'\''
        else:
            now = datetime.datetime.now()
            dateString = now.strftime("%Y-%m-%d-%H:%M")
            #cmd = 'bash -c \'rosrun logistic_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' > logs/'+ALG+'-'+dateString+'-robot'+str(i)+'.log \''
            # cmd = 'bash -c \'rosrun logistic_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+' robotname:=robot_'+str(i)+' '+str(CAPACITY)+' '+'\''
            # cmd = 'bash -c \'rosrun logistic_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' '+str(CAPACITY)+' '+'\''
            cmd = 'bash -c \'rosrun logistic_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' robot_'+str(i)+' '+str(CAPACITY)+' '+str(NROBOTS)+'\';bash'

        print cmd
        if (TERM == 'xterm'):
	     os.system('xterm -e  "'+cmd+'" &')
	     os.system('sleep 1')
        gcmd = gcmd + ' --tab -e "'+cmd+'" '
    gcmd = gcmd + '&'
    if (TERM == 'gnome-terminal'):
      #print gcmd
      os.system(gcmd)
    os.system('sleep '+NROBOTS)

    print 'Stage simulator footprints and speedup'
    os.system('rostopic pub /stageGUIRequest std_msgs/String "data: \'footprints\'"  --once')
    os.system('rostopic pub /stageGUIRequest std_msgs/String "data: \'speedup_%.1f\'"  --once' %(SPEEDUP))
    #os.system('rm ~/.ros/stage-000003.png')

    now = datetime.datetime.now()
    strinittime = now.strftime("%Y%m%d_%H%M%S")

    print 'Experiment started at ',strinittime
    # wait for termination
    run = True
    while (run):
        t = getROStime()
        #print "Elapsed time: ",t," sec Timeout = ",TIMEOUT
        if ((TIMEOUT>0 and t>TIMEOUT) or (not getSimulationRunning())):        
            run = False;
        os.system('sleep 1')

    #print "Taking a screenshot..."
    #os.system('rostopic pub /stageGUIRequest std_msgs/String "data: \'screenshot\'"  --once')
    #os.system('sleep 5')
    #cmd = 'mv ~/.ros/stage-000005.png results/screenshots/stage-%s.png' %(strinittime)
    #os.system(cmd)

    print "Terminating Experiment"
    os.system("./stop_experiment.sh")


class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.initUI()
        
    def initUI(self):
        self.loadOldConfig()
        
        
        self.parent.title("MRP Experiment Launcher")
        self.style = Style()
        self.style.theme_use("alt")
        self.parent.resizable(width=FALSE, height=FALSE)
        self.pack(fill=BOTH, expand=1)
        
        #self.columnconfigure(1, weight=1)
        #self.columnconfigure(3, pad=7)
        #self.rowconfigure(3, weight=1)
        #self.rowconfigure(7, pad=7)

        _row = 0
        
        lbl = Label(self, text="Map")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)
                
        self.map_name_list = Map_names
        self.map_ddm = StringVar(self)
        try:
            lastmap=self.oldConfigs["map"]
        except:
            lastmap=self.map_name_list[0]
        self.map_ddm.set(lastmap)
        tk.OptionMenu(self, self.map_ddm, *self.map_name_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)

        _row = _row + 1

        lbl = Label(self, text="N. Robots")
        lbl.grid(sticky=W, row=_row, column=0, pady=4, padx=5)

        self.robots_n_list = NRobots_list
        self.robots_ddm = StringVar(self)
        try:
            lastnrobots=self.oldConfigs["nrobots"]
        except:
            lastnrobots=self.robots_n_list[0]
        self.robots_ddm.set(lastnrobots)
        tk.OptionMenu(self, self.robots_ddm, *self.robots_n_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)

        _row = _row + 1

        lbl = Label(self, text="Algorithm")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)

        self.algorithm_list = []
        for i in range(0,len(Alg_names)):
            self.algorithm_list += [Alg_names[i][0]]

        self.alg_ddm = StringVar(self)
        try:
            lastalg=self.oldConfigs["algorithm"]
        except:
            lastalg=self.algorithm_list[0]
        self.alg_ddm.set(lastalg)
        tk.OptionMenu(self, self.alg_ddm, *self.algorithm_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)
        
        _row = _row + 1

        lbl = Label(self, text="Localization Mode")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)

        self.locmode_list = LocalizationMode_list
        self.locmode_ddm = StringVar(self)
        try:
            lastlocmode=self.oldConfigs["locmode"]
        except:
            lastlocmode=self.locmode_list[0]
        self.locmode_ddm.set(lastlocmode)
        tk.OptionMenu(self, self.locmode_ddm, *self.locmode_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)

        _row = _row + 1

        lbl = Label(self, text="Navigation Mode")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)

        self.navmode_list = NavigationMode_list
        self.navmode_ddm = StringVar(self)
        try:
            lastnavmode=self.oldConfigs["navmode"]
        except:
            lastnavmode=self.navmode_list[0]
        self.navmode_ddm.set(lastnavmode)
        tk.OptionMenu(self, self.navmode_ddm, *self.navmode_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)

        _row = _row + 1

        lbl = Label(self, text="Goal wait time")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)

        self.gwait_list = GWait_list
        self.gwait_ddm = StringVar(self)
        try:
            lastgwait=self.oldConfigs["gwait"]
        except:
            lastgwait=self.gwait_list[0]
        self.gwait_ddm.set(lastgwait)
        tk.OptionMenu(self, self.gwait_ddm, *self.gwait_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)

        _row = _row + 1

        lbl = Label(self, text="Capacity")
        lbl.grid(sticky=W, row=_row, column=0, pady=4, padx=5)

        self.capacity_list = RobotsCapacity_list
        self.capacity_ddm = StringVar(self)
        try:
            lastcapacity=self.oldConfigs["capacity"]
        except:
            lastcapacity=self.capacity_list[0]
        self.capacity_ddm.set(lastcapacity)
        tk.OptionMenu(self,self.capacity_ddm, *self.capacity_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)

        _row = _row + 1

        lbl = Label(self, text="Terminal")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)

        self.term_list = Terminal_list
        self.term_ddm = StringVar(self)
        try:
            lastterm=self.oldConfigs["term"]
        except:
            lastterm=self.term_list[0]
        self.term_ddm.set(lastterm)
        tk.OptionMenu(self, self.term_ddm, *self.term_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)
  
        _row = _row + 1

        lbl = Label(self, text="TaskPlanner")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)

        self.tp_list = TaskPlanner_names
        self.tp_ddm = StringVar(self)
        try:
            lasttp=self.oldConfigs["tp"]
        except:
            lasttp=self.tp_list[0]
        self.tp_ddm.set(lasttp)
        tk.OptionMenu(self, self.tp_ddm, *self.tp_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)

        _row = _row + 1

        lbl = Label(self, text="Generation")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)

        self.gen_list = TaskGeneration_names
        self.gen_ddm = StringVar(self)
        try:
            lasttp=self.oldConfigs["gen"]
        except:
            lasttp=self.gen_list[0]
        self.gen_ddm.set(lasttp)
        tk.OptionMenu(self, self.gen_ddm, *self.gen_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)
  
        _row = _row + 1

        launchButton = Button(self, text="Start Experiment",command=self.launch_script)
        launchButton.grid(sticky=W, row=_row, column=0, pady=4, padx=5)
        
        launchButton = Button(self, text="Stop Experiment",command=self.kill_demo)
        launchButton.grid(sticky=W, row=_row, column=1, pady=4, padx=5)
        
    
    def launch_script(self):
        self.saveConfigFile();
        thread.start_new_thread( run_experiment, (self.map_ddm.get(), self.robots_ddm.get(), INITPOS_DEFAULT, self.alg_ddm.get(),self.locmode_ddm.get(), self.navmode_ddm.get(), self.gwait_ddm.get(), COMMDELAY_DEFAULT, self.term_ddm.get(),0,"false",1.0, self.capacity_ddm.get(), self.tp_ddm.get(), self.gen_ddm.get()))

    
    def quit(self):
      self.parent.destroy()
      
    def kill_demo(self):
      os.system("rosparam set /simulation_running false")
      
      
    def saveConfigFile(self):
      f = open(dirname+"/lastConfigUsed", 'w')
      f.write("[Config]\n")
      f.write("map: %s\n"%self.map_ddm.get())
      f.write("nrobots: %s\n"%self.robots_ddm.get())
      f.write("algorithm: %s\n"%self.alg_ddm.get())
      f.write("locmode: %s\n"%self.locmode_ddm.get())
      f.write("navmode: %s\n"%self.navmode_ddm.get())
      f.write("gwait: %s\n"%self.gwait_ddm.get())
      f.write("capacity: %s\n"%self.capacity_ddm.get())
      f.write("term: %s\n"%self.term_ddm.get())
      f.write("tp: %s\n"%self.tp_ddm.get())
      f.write("gen: %s\n"%self.gen_ddm.get())
      f.close()


    def loadOldConfig(self):
      try:
        self.oldConfigs = {}
        self.Config = ConfigParser.ConfigParser()
        self.Config.read(dirname+"/lastConfigUsed")
        for option in self.Config.options("Config"):
          self.oldConfigs[option] = self.Config.get("Config", option)
      except:
        print "Could not load config file"


    


def main():

  if (len(sys.argv)==1):
    root = tk.Tk()
    DIP(root)
    root.geometry("300x430+0+0")
    root.mainloop()  

  elif (len(sys.argv)<17):
    print "Use: ",sys.argv[0]
    print " or  ",sys.argv[0],' <map> <n.robots> <init_pos> <alg_short> <loc_mode> <nav_module> <goal_wait_time> <communication_delay> <capacity> <tp_name> <terminal> <timeout> [<custom_stage_flag>|def:false] [<sim_speedup>|def:1.0]'

  else:
    MAP = sys.argv[1]
    NROBOTS = sys.argv[2]
    INITPOS = sys.argv[3]
    ALG_SHORT = sys.argv[4]
    LOC_MODE = sys.argv[5]
    NAV_MODULE = sys.argv[6]
    GWAIT = sys.argv[7]
    COMMDELAY = sys.argv[8]
    TERM = sys.argv[9]
    TIMEOUT = int(sys.argv[10])
    CUSTOM_STAGE = False
    SPEEDUP = float(1.0)
    CAPACITY = int(sys.argv[13])
    TP_NAME = sys.argv[14]
    GEN = sys.argv[15]
    MISSIONS_FILE = sys.argv[16]

    # MAP = model5
    # NROBOTS = 2
    # INITPOS = default
    # ALG_SHORT = TokenAgent
    # LOC_MODE = AMCL
    # NAV_MODULE = ros
    # GWAIT = 0
    # COMMDELAY= 0
    # TERM = xterm
    # TIMEOUT = 1800
    # CUSTOM_STAGE = False
    # SPEEDUP = 1.0
    # CAPACITY = 3



    # if (len(sys.argv)>=12):
    #   CUSTOM_STAGE = sys.argv[11]
    # if (len(sys.argv)>=13):
    #   SPEEDUP = float(sys.argv[12])

    print "param: ", MAP, NROBOTS, INITPOS, ALG_SHORT, LOC_MODE, NAV_MODULE, GWAIT, COMMDELAY, TERM, TIMEOUT, CUSTOM_STAGE,SPEEDUP,CAPACITY,TP_NAME,GEN,MISSIONS_FILE
    
    run_experiment(MAP, NROBOTS, INITPOS, ALG_SHORT, LOC_MODE, NAV_MODULE, GWAIT, COMMDELAY, TERM, TIMEOUT, CUSTOM_STAGE, SPEEDUP, CAPACITY, TP_NAME, GEN, MISSIONS_FILE)

 


if __name__ == '__main__':
    os.chdir(dirname)
    # MAP = "model5"
    # NROBOTS = 2
    # INITPOS = "default"
    # ALG_SHORT = "TokenAgent"
    # LOC_MODE = "AMCL"
    # NAV_MODULE = "ros"
    # GWAIT = 0
    # COMMDELAY= 0
    # TERM = "xterm"
    # TIMEOUT = 1800
    # CUSTOM_STAGE = False
    # SPEEDUP = 1.0
    # CAPACITY = 3

    # print "param: ", MAP, NROBOTS, INITPOS, ALG_SHORT, LOC_MODE, NAV_MODULE, GWAIT, COMMDELAY, TERM, TIMEOUT, CUSTOM_STAGE,SPEEDUP,CAPACITY
    main()

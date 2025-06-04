#!/usr/bin/env python3
import operator
import argparse
 
# Initialize parser
parser = argparse.ArgumentParser()

# Adding optional argument
parser.add_argument("-m", "--mode", help = "running scenario   0: others ), 1: turtle, 2: others, 3: others", nargs='?', const=1, type=int, default=1)
inputargs = parser.parse_args()
from statistics import mean
import sys
import csv
from ros2_interface_turtle import *
from terrain_senser import *
from gait_optimizer import *
sys.argv = [sys.argv[0]]


import threading
import time

import os
from kivy.core.window import Window
from kivy.config import Config
from kivy.metrics import dp
from kivy.lang import Builder
from kivy.properties import StringProperty
from kivy.graphics import Color, Line
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
from kivy.clock import Clock
from kivy.graphics import *
Config.set('input', '%(name)s', '')
# Config.set('graphics', 'resizable', 0)
Config.write()
from kivymd.app import MDApp
from kivymd.uix.card import MDCard
from kivymd.uix.list import OneLineIconListItem
from kivymd.uix.menu import MDDropdownMenu
from kivy.uix.widget import Widget
from kivymd.uix.tooltip import MDTooltip
from kivymd.uix.button import MDIconButton
from kivymd.uix.tab import MDTabs
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.tab import MDTabsBase
start_time = time.time()
import numpy as np
import matplotlib.pyplot as plt
DEBUG = False


class Tab(MDFloatLayout, MDTabsBase):
    pass

class turtle_tab(MDCard ):
    '''Implements a material design v3 card.'''
    text = StringProperty()
class turtle_optimize_tab(MDCard ):
    '''Implements a material design v3 card.'''
    text = StringProperty()



class IconListItem(OneLineIconListItem):
    icon = StringProperty()


class TooltipMDIconButton(MDIconButton, MDTooltip):
    pass

class TravelerApp(MDApp):
    def __init__(self, node, multi_camera=False, scale_size=1, scale_size_2=1):
        super().__init__()
        self.start_robot = False
        self.drag_traj = 5
        self.theme_cls.theme_style = "Light" 
        self.theme_cls.primary_palette  = "BlueGray"
        self.ground_height = 0.17
        self.multi_camera = multi_camera
        self.scale_size = scale_size
        self.scale_size_2 = scale_size_2
        self.ros_node = node
        self.updateplotflag = False
        self.frame = 0
        self.title = 'Turtle'
        # get the absolute path of the currently running script
        script_path = os.path.abspath(__file__)

        # get the absolute path of the parent directory of the script
        parent_path = os.path.dirname(script_path)

        # get the absolute path of the grandparent directory of the script
        grandparent_path = os.path.dirname(parent_path)
        target_path_1 = os.path.join(grandparent_path, 'resource/style.kv')

        print('target_path_1: ', target_path_1)
        self.screen = Builder.load_file(target_path_1)
        # generate the menu items
        menu_items = []
        self.tab_items = []
        for ii in range(len(self.ros_node.tab_control)):
            menu_items.append(
                {
                "viewclass": "IconListItem",
                "icon": "git",
                "text": self.ros_node.tab_control[ii],
                "height": dp(50),
                "on_release": lambda x=self.ros_node.tab_control[ii]: self.change_configure_tab(x)
                }
            )
       
        if(self.ros_node.id == "turtle"):
            self.turtle_tab = turtle_tab()
            self.turtle_optimize_tab = turtle_optimize_tab()
            self.screen.ids.configure_layout.add_widget(self.turtle_tab)
            self.current_tab = self.turtle_tab
    

        
            
        # # print(menu_items)
        self.screen.ids.drop_item.text = self.ros_node.tab_control[0]
        self.menu = MDDropdownMenu(
            caller=self.screen.ids.drop_item,
            items=menu_items,
            position="auto",
            width_mult=4,
            background_color="f9c6bc",
        )
        self.menu.bind()

        fp = node.get_fp()
        pp = node.get_pp()
        speed_p = node.get_speed_p()
        self.screen.ids.rawForcePlot.add_widget(FigureCanvasKivyAgg(fp.get_figure()))
        self.screen.ids.positionplot.add_widget(FigureCanvasKivyAgg(pp.get_figure()))
        self.screen.ids.speedplot.add_widget(FigureCanvasKivyAgg(speed_p.get_figure()))
       

        
        

    def change_configure_tab(self, type):
        self.set_item(type)
        if(type == "Preset_gait"): # mode 5
            self.drag_traj = 5
            self.screen.ids.configure_layout.clear_widgets()
            self.screen.ids.configure_layout.add_widget(self.turtle_tab)
            self.current_tab = self.turtle_tab
        elif("Optimized_gait"):
            self.drag_traj = 6
            self.screen.ids.configure_layout.clear_widgets()
            self.screen.ids.configure_layout.add_widget(self.turtle_optimize_tab)
            self.current_tab = self.turtle_optimize_tab

        

    
    def set_item(self, text_item):
        self.screen.ids.drop_item.text = text_item
        self.menu.dismiss()
        
    def build(self):
        Window.size = (1400,1000)

        Clock.schedule_interval(self.update_force_plot, 0.01)
        self.data_updator = threading.Thread(target=self.update_force_data)
        self.data_updator.daemon = True
        self.data_updator.start()
        self.errors_writer = threading.Thread(target=self.write_errors)
        self.errors_writer.daemon = True
        self.errors_writer.start()
        print(inputargs.mode == 3)
        if(inputargs.mode == 3):
            print(3)
            self.loadcell_calibrator = threading.Thread(target=self.ros_node.calibrate_loadcell)
            self.loadcell_calibrator.daemon = True
            self.loadcell_calibrator.start()


        return self.screen
    def update_force_plot(self, *args):
        self.ros_node.update_force_plot(self.screen.ids.if_real_time_plot.active, 
                                            self.updateplotflag)
        if(self.drag_traj == 7 and self.start_robot):
            height = -int(self.ros_node.toeposition_y * 1000)
            self.current_tab.ids.variable3_slider.value = height
            self.current_tab.ids.variable3.text = str(height)
    def update_force_data(self):
        
    
        self.run_time = 0
            # self.start_time_esle = time.time()
        while(1):
            current_time = time.time()
            self.ros_node.update_force_data(self.updateplotflag)
            self.frame = self.frame + 1
            time.sleep(0.001)




    def write_errors(self):
        while(True):
            rclpy.spin_once(self.ros_node)

    # Callback functions for turtle optimize tab
    def on_change_Optimize_Variable_1(self):
        self.current_tab.ids.Optimize_Variable_1.text = str(round(self.current_tab.ids.Slider_optimize_1.value))

    def on_change_Optimize_Variable_2(self):
        self.current_tab.ids.Optimize_Variable_2.text = str(round(self.current_tab.ids.Slider_optimize_2.value))

    def on_change_Optimize_Variable_3(self):
        self.current_tab.ids.Optimize_Variable_3.text = str(round(self.current_tab.ids.Slider_optimize_3.value))

    def on_change_Optimize_Variable_4(self):
        self.current_tab.ids.Optimize_Variable_4.text = str(round(self.current_tab.ids.Slider_optimize_4.value))

    def on_change_Optimize_Variable_5(self):
        self.current_tab.ids.Optimize_Variable_5.text = str(round(self.current_tab.ids.Slider_optimize_5.value))

    def on_change_Optimize_Variable_6(self):
        self.current_tab.ids.Optimize_Variable_6.text = str(round(self.current_tab.ids.Slider_optimize_6.value))

    def on_change_Optimize_Variable_7(self):
        self.current_tab.ids.Optimize_Variable_7.text = str(round(self.current_tab.ids.Slider_optimize_7.value, 2))

    def on_change_Optimize_Variable_8(self):
        self.current_tab.ids.Optimize_Variable_8.text = str(round(self.current_tab.ids.Slider_optimize_8.value, 2))
    def on_change_Optimize_Variable_9(self):
        self.current_tab.ids.Optimize_Variable_9.text = str(round(self.current_tab.ids.Slider_optimize_9.value, 2))
    def on_change_Optimize_Variable_10(self):
        self.current_tab.ids.Optimize_Variable_10.text = str(round(self.current_tab.ids.Slider_optimize_10.value, 2))






    # call back for turtle preset gait
    def on_change_Variable_1(self):
        self.current_tab.ids.Variable_1.text = str(round(self.current_tab.ids.Slider_1.value))
        
    def on_change_Variable_2(self):
        self.current_tab.ids.Variable_2.text = str(round(self.current_tab.ids.Slider_2.value))
        
    
    def on_change_Variable_3(self):    
        self.current_tab.ids.Variable_3.text = str(round(self.current_tab.ids.Slider_3.value))
        
    
    def on_change_Variable_4(self):    
        self.current_tab.ids.Variable_4.text = str(round(self.current_tab.ids.Slider_4.value))
        
    def on_change_Variable_5(self):    
        self.current_tab.ids.Variable_5.text = str(round(self.current_tab.ids.Slider_5.value))
       
    
    def on_change_Variable_6(self):
        self.current_tab.ids.Variable_6.text = str(round(self.current_tab.ids.Slider_6.value))
        
    def on_change_Variable_7(self):

        self.current_tab.ids.Variable_7.text = str(round(self.current_tab.ids.Slider_7.value))
        
    def on_change_Variable_8(self):
        self.current_tab.ids.Variable_8.text = str(round(self.turtle_tab.ids.Slider_8.value))
        
    def on_plus_speed_click(self, id, sign):
        # print(id)
        if(id == "moving_speed"):
            self.current_tab.ids.moving_speed_slider.value += sign
            self.current_tab.ids.moving_speed.text = str(round(self.current_tab.ids.moving_speed_slider.value))
        elif(id == "Variable_7"):
            self.current_tab.ids.Slider_7.value += sign
            self.current_tab.ids.Variable_7.text = str(round(self.current_tab.ids.Slider_7.value))
            
        elif(id == "Variable_5"):
            self.current_tab.ids.Slider_5.value += sign
            self.current_tab.ids.Variable_5.text = str(round(self.current_tab.ids.Slider_5.value))
            
        elif(id == "Variable_2"):
            self.current_tab.ids.Slider_2.value += sign
            self.current_tab.ids.Variable_2.text = str(round(self.current_tab.ids.Slider_2.value))
            
        elif(id == "extrude_speed"):
            self.current_tab.ids.extrude_speed_slider.value += sign
            self.current_tab.ids.extrude_speed.text = str(round(self.current_tab.ids.extrude_speed_slider.value))
            
        elif(id == "back_speed"):
            self.current_tab.ids.back_speed_slider.value += sign
            self.current_tab.ids.back_speed.text = str(round(self.current_tab.ids.back_speed_slider.value))
        elif(id == "Variable_8"):
            self.current_tab.ids.Slider_8.value += sign
            self.current_tab.ids.Variable_8.text = str(round(self.current_tab.ids.Slider_8.value))
           
    def on_stop(self):
        pass

    # def start_logi_usb_camera_recording(self):
    #     pass
    # def stop_logi_usb_camera_recording(self):
    #     self.recorder.stopRecording()
    #     if self.multi_camera:
    #         self.recorder_2.stopRecording()
    # def save_logi_usb_camera_recording(self):
    #     self.recorder.saveRecording()
    #     if self.multi_camera:
    #         self.recorder_2.saveRecording()

    def check_if_start(self):
        if(self.start_robot == True):
            print('end to sampling')
            self.start_robot = False
            self.start_flag = 0.0
            self.updateplotflag = False
    
            self.ros_node.update_plot()
            self.on_download_data()
            
        else:
            print('start to sampling')

            self.trial_start_time = str(time.asctime( time.localtime(time.time()) )).replace(" ", "_").replace(":","_")
            self.file_name = self.screen.ids.filename.text +"_"+ self.trial_start_time
            self.updateplotflag = True          
            self.start_time = time.time()
            self.frame = 0

            self.start_robot = True
            self.start_flag = 1
            self.ros_node.calibrate()
                  
            
        self.gui_message = Float64MultiArray()
        self.gui_message.data.append(self.start_flag)                                                   #[0]: if start the robot or stop the robot
        self.gui_message.data.append(float(self.drag_traj))                                             #[1]: add drag traj
        
        if(self.ros_node.id == "turtle"):
            if(self.drag_traj == 5):
                self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_3.value))/1000)           
                self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_4.value))/1000)              
                self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_5.value))/1000)               
                self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_6.value))/1000)  
                self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_2.value))/1000 )    
                self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_1.value)) * np.pi / 180  )   
                if(self.start_flag):
                    print("start preset gait without adaptation: ", self.gui_message.data)
                else:
                    print("end preset gait ")
                self.ros_node.start_preset_gait(self.gui_message)
                
            elif(self.drag_traj == 6):
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_3.value))/1000)           
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_4.value))/1000)              
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_5.value))/1000)               
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_6.value))/1000)  
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_2.value))/1000 )    
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_1.value)) * np.pi / 180  )     
                # MODIFIED: Append new start and end coordinate parameters (converted to radians)
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_7.value, 2)) * np.pi / 180)  # MODIFIED: start gamma
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_8.value, 2)) * np.pi / 180)  # MODIFIED: start theta
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_9.value, 2)) * np.pi / 180)  # MODIFIED: end gamma
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_10.value, 2)) * np.pi / 180) # MODIFIED: end theta


         
                        
                     
                if(self.start_flag):
                    print("start optimizing gait with initial: ", self.gui_message.data)
                else:
                    print("end optimizing gait ")    
                self.ros_node.start_gait_optimization(self.gui_message)
 

       
        


        
    def on_download_data(self):
        self.ros_node.download_data(self.file_name, self.ros_node.id, self.screen.ids.if_real_time_plot.active, self.gui_message.data)
    def save_configuration(self):
        self.gui_message = Float64MultiArray()
        self.start_flag = 0
        self.gui_message.data.append(self.start_flag)
        self.gui_message.data.append(float(self.drag_traj))                                             #[1]: add drag traj
        if(self.ros_node.id == "leg"):
            self.gui_message.data.append(float(round(self.extrude_tab.ids.extrude_speed_slider.value)) )           # cm/s
            self.gui_message.data.append(float(round(self.extrude_tab.ids.extrude_angle_slider.value)) )           # deg
            self.gui_message.data.append(float(round(self.extrude_tab.ids.extrude_length_slider.value)) )          # cm
            self.gui_message.data.append(float(round(self.shear_tab.ids.Slider_1.value)) )               # cm
            self.gui_message.data.append(float(round(self.shear_tab.ids.Slider_2.value)) )                # cm/s
            self.gui_message.data.append(float(round(self.shear_tab.ids.Slider_3.value)) )          # seconds
            self.gui_message.data.append(float(round(self.shear_tab.ids.Slider_4.value)) )              # cm
            self.gui_message.data.append(float(round(self.shear_tab.ids.Slider_5.value)) )               # cm/s
            self.gui_message.data.append(float(round(self.shear_tab.ids.Slider_6.value)) )         # seconds
            self.gui_message.data.append(float(round(self.shear_tab.ids.Slider_7.value)) )                # cm/s
            self.gui_message.data.append(float(round(self.workspace_tab.ids.moving_speed_slider.value)) )          # cm/s
            self.gui_message.data.append(float(round(self.workspace_tab.ids.moving_step_angle_slider.value)) )     # degrees
            self.gui_message.data.append(float(round(self.workspace_tab.ids.time_delay_slider.value)) )            # seconds
            self.gui_message.data.append(float(round(self.free_tab.ids.variable1_slider.value))/10 )         # cm
            self.gui_message.data.append(float(round(self.free_tab.ids.variable3_slider.value))   )               # degrees
            self.gui_message.data.append(float(round(self.ground_tab.ids.variable1_slider.value)/10)   )
            self.gui_message.data.append(float(round(self.ground_tab.ids.variable2_slider.value)/10)   )
            self.gui_message.data.append(float(round(self.ground_tab.ids.variable3_slider.value)/10))
            self.gui_message.data.append(float(round(self.extrude_tab.ids.back_speed_slider.value)) )          # cm

            print(self.gui_message.data)
        elif(self.ros_node.id == "turtle"):
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_1.value))  ) 
            print(float(round(self.turtle_tab.ids.Slider_2.value))/10)             # cm
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_2.value))/10 )                # cm/s
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_3.value)))           # seconds
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_4.value)))               # cm
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_5.value)))                # cm/s
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_6.value))   )       # seconds
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_7.value)))                 # cm/s
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_8.value)) )
            print(self.gui_message.data)
        elif(self.ros_node.id == "MiniRhex"):
            print(112312)
            self.gui_message.data.append(float(round(self.minirhex_tab.ids.Slider_1.value))  )              # cm
            self.gui_message.data.append(float(round(self.minirhex_tab.ids.Slider_2.value)) )                # cm/s
            self.gui_message.data.append(float(round(self.minirhex_tab.ids.Slider_3.value)))           # seconds
            self.gui_message.data.append(float(round(self.minirhex_tab.ids.Slider_4.value)))               # cm
            self.gui_message.data.append(float(round(self.minirhex_tab.ids.Slider_5.value)))                # cm/s
            self.gui_message.data.append(float(round(self.minirhex_tab.ids.Slider_6.value))   )       # seconds
            self.gui_message.data.append(float(round(self.minirhex_tab.ids.Slider_7.value)))                 # cm/s
            self.gui_message.data.append(float(round(self.minirhex_tab.ids.Slider_8.value)) )
            
        if not os.path.exists('./config'):
            os.makedirs('./config')

        path = "./config/"  +  "last_config.csv"
        with open(path, 'w', newline='') as f:
            writer=csv.writer(f)
            if(self.ros_node.id == "leg"):
                writer.writerow(["scenario","real_time_plot", "start_flag", "drag_traj" , "extrude_speed_slider","extrude_angle_slider", "extrude_length_slider", "down_length_slider", "down_speed_slider", "delay_after_down",
                                "shear_length", "shear_speed", "delay_after_shear", "back_speed",
                                "moving_speed", "moving_step_angle", "time_delay", "variable1", "variable3", "search_start", "search_end", "ground_height", "extrude_back_speed"])
                print(self.gui_message.data)
                writer.writerow([self.ros_node.id, self.screen.ids.if_real_time_plot.active,self.gui_message.data[0],self.screen.ids.drop_item.text, self.gui_message.data[2],  self.gui_message.data[3],self.gui_message.data[4],self.gui_message.data[5],
                                    self.gui_message.data[6],self.gui_message.data[7],self.gui_message.data[8],self.gui_message.data[9],
                                    self.gui_message.data[10],self.gui_message.data[11],self.gui_message.data[12],self.gui_message.data[13],self.gui_message.data[14],self.gui_message.data[15],self.gui_message.data[16]])
            if(self.ros_node.id == "turtle"):
                writer.writerow(["scenario","real_time_plot", "lateral_angle_range","drag_speed", "wiggle_time", "servo_speed", "extraction_angle", "wiggle frequency",
                                "insertion_angle", "wiggle_amplitude"])
                writer.writerow([self.ros_node.id, self.screen.ids.if_real_time_plot.active, self.gui_message.data[0], self.gui_message.data[1],self.gui_message.data[2],self.gui_message.data[3],self.gui_message.data[4],self.gui_message.data[5],
                                    self.gui_message.data[6],self.gui_message.data[7], self.gui_message.data[8],self.gui_message.data[9]])
            if(self.ros_node.id == "MiniRhex"):
                writer.writerow(["scenario","real_time_plot", "gait)shift_fr","gait_shift_br", "gait_shift_bl", "gait_period", "buehler_block", "wiggle frequency",
                                "insertion_angle", "wiggle_amplitude"])
                writer.writerow([self.ros_node.id, self.screen.ids.if_real_time_plot.active, self.gui_message.data[0], self.gui_message.data[1],self.gui_message.data[2],self.gui_message.data[3],self.gui_message.data[4],self.gui_message.data[5],
                                    self.gui_message.data[6],self.gui_message.data[7],self.gui_message.data[8],self.gui_message.data[9]])

                
    def on_start(self):
        
        try:
            path = "./config/"  +  "last_config.csv"
            with open(path) as f:
        
                data = csv.DictReader(f)
                config = next(data)
            self.change_configure_tab(config['drag_traj'])
            self.screen.ids.if_real_time_plot.active = config['real_time_plot'] == "True"
            self.extrude_tab.ids.extrude_speed_slider.value = float(config['extrude_speed_slider'])
            self.on_change_extrude_speed()
            
        except:
            pass
        if(inputargs.mode == 0):
            self.extrude_tab.ids.extrude_speed_slider.max = 10
            self.extrude_tab.ids.back_speed_slider.max = 10
            self.shear_tab.ids.Slider_2.max = 10
            self.shear_tab.ids.Slider_5.max = 10
            self.shear_tab.ids.Slider_7.max = 10
            

        # self.extrude_tab.ids.extrude_angle_slider.value
        # self.extrude_tab.ids.extrude_length_slider.value
        # self.shear_tab.ids.Slider_1.value
        # self.shear_tab.ids.Slider_2.value
        # self.shear_tab.ids.Slider_4.value
        # self.shear_tab.ids.Slider_5.value
        # self.shear_tab.ids.Slider_3.value
        # self.shear_tab.ids.Slider_6.value
        # self.shear_tab.ids.Slider_7.value
        # self.workspace_tab.ids.moving_speed_slider.value
        # self.workspace_tab.ids.moving_step_angle_slider.value
        # self.workspace_tab.ids.time_delay_slider.value
        # self.free_tab.ids.variable1_slider.value
        # self.free_tab.ids.variable3_slider.value

    def on_tab_switch(
        self, instance_tabs, instance_tab, instance_tab_label, tab_text
    ):
        pass




def main():

    rclpy.init()
    gait_optimizer = GaitOptimizer()
    terrain_sensor = TerrainSensor()
    node_turtle = ControlNodeTurtle(gait_optimizer, terrain_sensor)
    app = TravelerApp(node_turtle, multi_camera=True, scale_size=0.9, scale_size_2=0.6)
    
    # print("time spend: ", time.time()-test_time_start)
    app.run()
    app.save_configuration()
    



if __name__ == '__main__':
    main()

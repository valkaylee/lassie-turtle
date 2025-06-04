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
from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.button import MDFlatButton
from kivymd.uix.dialog import MDDialog
from kivymd.uix.textfield import MDTextField
start_time = time.time()
import numpy as np
import matplotlib.pyplot as plt
DEBUG = False


class Tab(MDFloatLayout, MDTabsBase):
    pass

class turtle_tab(MDCard):
    '''Implements a material design v3 card.'''
    text = StringProperty()
    
class turtle_optimize_tab(MDCard):
    '''Implements a material design v3 card.'''
    text = StringProperty()

class WaypointDialog(MDBoxLayout):
    '''Dialog content for adding waypoints'''
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.orientation = "vertical"
        self.spacing = "10dp"
        self.padding = "20dp"
        
        self.gamma_input = MDTextField(
            hint_text="Gamma (degrees)",
            helper_text="Enter the gamma angle",
            helper_text_mode="on_focus",
            input_filter="float"
        )
        self.theta_input = MDTextField(
            hint_text="Theta (degrees)",
            helper_text="Enter the theta angle",
            helper_text_mode="on_focus",
            input_filter="float"
        )
        
        self.add_widget(self.gamma_input)
        self.add_widget(self.theta_input)


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
        self.theme_cls.primary_palette = "BlueGray"
        self.ground_height = 0.17
        self.multi_camera = multi_camera
        self.scale_size = scale_size
        self.scale_size_2 = scale_size_2
        self.ros_node = node
        self.updateplotflag = False
        self.frame = 0
        self.title = 'Turtle'
        self.waypoints = []  # List to store waypoints
        
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
            
            # Initialize waypoints dialog
            self.waypoint_dialog = None
    
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
            
            # Make sure the waypoints list widget is updated
            self.update_waypoints_list()
    
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
        while(1):
            current_time = time.time()
            self.ros_node.update_force_data(self.updateplotflag)
            self.frame = self.frame + 1
            time.sleep(0.001)

    def write_errors(self):
        while(True):
            rclpy.spin_once(self.ros_node)

    # Waypoint management methods
    def show_add_waypoint_dialog(self):
        """Show dialog to add a new waypoint"""
        if not self.waypoint_dialog:
            self.waypoint_content = WaypointDialog()
            self.waypoint_dialog = MDDialog(
                title="Add Waypoint",
                type="custom",
                content_cls=self.waypoint_content,
                buttons=[
                    MDFlatButton(
                        text="CANCEL",
                        theme_text_color="Custom",
                        text_color=self.theme_cls.primary_color,
                        on_release=lambda x: self.waypoint_dialog.dismiss()
                    ),
                    MDFlatButton(
                        text="ADD",
                        theme_text_color="Custom",
                        text_color=self.theme_cls.primary_color,
                        on_release=lambda x: self.add_waypoint()
                    ),
                ],
            )
        self.waypoint_dialog.open()
        
    def add_waypoint(self):
        """Add a new waypoint from the dialog input"""
        try:
            gamma = float(self.waypoint_content.gamma_input.text)
            theta = float(self.waypoint_content.theta_input.text)
            self.waypoints.append((gamma, theta))
            self.waypoint_dialog.dismiss()
            self.update_waypoints_list()
        except ValueError:
            # Handle invalid input
            pass
            
    def remove_waypoint(self, index):
        """Remove a waypoint at the specified index"""
        if 0 <= index < len(self.waypoints):
            self.waypoints.pop(index)
            self.update_waypoints_list()
            
    def update_waypoints_list(self):
        """Update the waypoints list in the UI"""
        # Clear existing waypoints display
        if hasattr(self.turtle_optimize_tab.ids, 'waypoints_container'):
            self.turtle_optimize_tab.ids.waypoints_container.clear_widgets()
            
            # Add waypoints to the container
            for i, (gamma, theta) in enumerate(self.waypoints):
                waypoint_box = MDBoxLayout(orientation='horizontal', adaptive_height=True, spacing="10dp")
                
                # Add label with waypoint info
                waypoint_label = MDLabel(
                    text=f"Waypoint {i+1}: Gamma={gamma}°, Theta={theta}°",
                    halign="left"
                )
                
                # Add remove button
                remove_btn = MDIconButton(
                    icon="delete",
                    on_release=lambda x, idx=i: self.remove_waypoint(idx)
                )
                
                waypoint_box.add_widget(waypoint_label)
                waypoint_box.add_widget(remove_btn)
                self.turtle_optimize_tab.ids.waypoints_container.add_widget(waypoint_box)

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

            self.trial_start_time = str(time.asctime(time.localtime(time.time()))).replace(" ", "_").replace(":","_")
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
                self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_2.value))/1000)    
                self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_1.value)) * np.pi / 180)   
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
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_2.value))/1000)    
                self.gui_message.data.append(float(round(self.turtle_optimize_tab.ids.Slider_optimize_1.value)) * np.pi / 180)
                
                # Add all waypoints to the message
                self.gui_message.data.append(float(len(self.waypoints)))  # Send the number of waypoints
                
                # Add each waypoint coordinate (gamma, theta) converted to radians
                for gamma, theta in self.waypoints:
                    self.gui_message.data.append(float(gamma) * np.pi / 180)  # gamma in radians
                    self.gui_message.data.append(float(theta) * np.pi / 180)  # theta in radians
                        
                if(self.start_flag):
                    print("start optimizing gait with waypoints: ", self.gui_message.data)
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
        
        if(self.ros_node.id == "turtle"):
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_1.value)))              # cm
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_2.value))/10)           # cm/s
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_3.value)))              # seconds
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_4.value)))              # cm
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_5.value)))              # cm/s
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_6.value)))              # seconds
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_7.value)))              # cm/s
            self.gui_message.data.append(float(round(self.turtle_tab.ids.Slider_8.value)))
            
            # Save waypoints too
            self.gui_message.data.append(float(len(self.waypoints)))  # Number of waypoints
            for gamma, theta in self.waypoints:
                self.gui_message.data.append(float(gamma))
                self.gui_message.data.append(float(theta))
                
            print(self.gui_message.data)
                
        if not os.path.exists('./config'):
            os.makedirs('./config')

        path = "./config/last_config.csv"
        with open(path, 'w', newline='') as f:
            writer=csv.writer(f)
            if(self.ros_node.id == "turtle"):
                # Add waypoints to header
                waypoints_header = []
                for i in range(len(self.waypoints)):
                    waypoints_header.extend([f"waypoint_{i+1}_gamma", f"waypoint_{i+1}_theta"])
                    
                writer.writerow(["scenario","real_time_plot", "lateral_angle_range","drag_speed", "wiggle_time", 
                                "servo_speed", "extraction_angle", "wiggle frequency",
                                "insertion_angle", "wiggle_amplitude", "num_waypoints"] + waypoints_header)
                
                # Create row data with waypoints
                waypoints_data = []
                for gamma, theta in self.waypoints:
                    waypoints_data.extend([gamma, theta])
                    
                writer.writerow([self.ros_node.id, self.screen.ids.if_real_time_plot.active, 
                                self.gui_message.data[2], self.gui_message.data[3], self.gui_message.data[4], 
                                self.gui_message.data[5], self.gui_message.data[6], self.gui_message.data[7], 
                                self.gui_message.data[8], self.gui_message.data[9], len(self.waypoints)] + waypoints_data)

    def on_start(self):
        try:
            path = "./config/last_config.csv"
            with open(path) as f:
                data = csv.DictReader(f)
                config = next(data)
                
            self.change_configure_tab(config['drag_traj'])
            self.screen.ids.if_real_time_plot.active = config['real_time_plot'] == "True"
            
            # Load waypoints if they exist
            if 'num_waypoints' in config:
                num_waypoints = int(config['num_waypoints'])
                self.waypoints = []
                
                for i in range(num_waypoints):
                    gamma = float(config[f'waypoint_{i+1}_gamma'])
                    theta = float(config[f'waypoint_{i+1}_theta'])
                    self.waypoints.append((gamma, theta))
                
                self.update_waypoints_list()
                
        except:
            pass
            
    def on_tab_switch(self, instance_tabs, instance_tab, instance_tab_label, tab_text):
        pass


def main():
    rclpy.init()
    gait_optimizer = GaitOptimizer()
    terrain_sensor = TerrainSensor()
    node_turtle = ControlNodeTurtle(gait_optimizer, terrain_sensor)
    app = TravelerApp(node_turtle, multi_camera=True, scale_size=0.9, scale_size_2=0.6)
    
    app.run()
    app.save_configuration()


if __name__ == '__main__':
    main()
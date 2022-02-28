import PySimpleGUI as sg
import cv2
import numpy as np
import threading
import pyrealsense2 as rs
from TireSpraying_Interface_Functions import *



# Classify a Thread-Class with return Value, used to open Tasks in different Threads for better performance
class ThreadWithReturnValue(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, *, daemon=None):
        def function():
            self.result = target(*args, **kwargs)
        super().__init__(group=group, target=function, name=name, daemon=daemon)

#Define the Main Program  
def main():

    #Standart Option parameters
    v_cobot= 1
    a_cobot= 0.5

    Cobot_offset_x = 0.295 #Messung
    Cobot_offset_y = 0 #Ausrichtung
    Cobot_offset_z = 0.015 #Messung

    Table_height= 1.47
    Spray_bound_cw= 10 #Degree which is going to be not Sprayed clockwise from barcode
    Spray_bound_ccw= 10 #Degree which is going to be not Sprayed counterclockwise from barcode
    times_of_capturing=1 #migth delete

    Cobot_IP="169.254.95.199"

    #Initialize auxilary variables
    Thread_started = False
    circle_detected=False
    Points_calculated=False
    Barcode_detected=False
    Cobot_moved=False
    cap_open=False
    
    Case = 0
    pipeline= None


    #GUI-Theme
    sg.theme("LightGrey1")

    #Define the Laout of the 'Control-Tab'
    picture_col=[
        [sg.Image(filename="", key="-IMAGE-")],
        [sg.Text('',key='e_Status', size=(70,1))],  
        [sg.Text('')]  
    ]
    
    button_col= [
        [sg.Button("Exit", size=(15, 1))], 
        [sg.Button("Restart", size=(15, 1))],
        [sg.Button("Accept", size=(15, 1))]
    ]

    
    work_tab_layout = [
        [sg.Column(picture_col, element_justification='l'), sg.VSeperator(), sg.Column(button_col, element_justification='c',vertical_alignment='t')]
    ]

    # Define the layout of the 'options-tab'
    options_tab_layout = [
            [sg.Text('Normal Options:', size=(28,1))],
            [sg.Text('', size=(22,1))],
            [sg.Text('Cobot Velocity [%]:', size=(28,1)),sg.Input(v_cobot*100,key='e_v_cobot',size=(10,1))],
            [sg.Text('Cobot acceleration[%]', size=(28,1)),sg.Input(a_cobot*100,key='e_a_cobot',size=(10,1))],
            [sg.Text('Spraying-Gap CW from Barcode [°]:', size=(28,1)),sg.Input(Spray_bound_cw,key='e_Spray_bound_cw',size=(10,1))],
            [sg.Text('Spraying-Gap CCW from Barcode [°]:', size=(28,1)),sg.Input(Spray_bound_ccw,key='e_Spray_bound_ccw',size=(10,1))],
            [sg.Text('', size=(22,1))],
            [sg.Text('', size=(22,1))],
            [sg.Text('', size=(22,1))],
            [sg.Text('Deep Options:', size=(22,1))],
            [sg.Text('!Warning, change only if necessary. Changes can lead to wrong Cobot position!', size=(100,1))],
            [sg.Text('', size=(22,1))],
            [sg.Text('Cobot IP:', size=(20,1)),sg.Input(Cobot_IP,key='e_Cobot_IP',size=(19,1))],
            [sg.Text('offset_x [m]:', size=(20,1)),sg.Input(Cobot_offset_x,key='e_offset_x',size=(19,1))],
            [sg.Text('offset_y [m]:', size=(20,1)),sg.Input(Cobot_offset_y,key='e_offset_y',size=(19,1))],
            [sg.Text('offset_z [m]:', size=(20,1)),sg.Input(Cobot_offset_z,key='e_offset_z',size=(19,1))],
            [sg.Text('Distance Cam-Table[m]:', size=(20,1)),sg.Input(Table_height,key='e_Table_height',size=(19,1))],
            [sg.Text('', size=(22,1))],
            [sg.Button("Apply", size=(25, 1))]
    ]

    #Create Tabs
    work_tab= sg.Tab('Control',work_tab_layout)
    options_tab= sg.Tab('Options', options_tab_layout)
    
    
    #Add Tabs to a TabGroup
    tabgrp = [[sg.TabGroup([[work_tab,options_tab]], enable_events=True, size=(780,530), key='-TAB_SWITCH-')]]    

    #Create the window with the TabGroup inside
    window = sg.Window("Tire Spraying", tabgrp, location=(10,10)) 

    
    #This Loop consists the executable of the programm
    while True:
        
        #read if new inputs where made
        event, values = window.read(timeout=5)

        if event == "Exit" or event == sg.WIN_CLOSED: #Exit program when Exit is pressed or the window is closed
                break
            
        #Detect the progress of detecton and calculation (auxiliary variables)

        if event == '-CIRCLE_DETECTED-':
            circle_detected=True
        
        if event == '-POINTS_CALCULATED-':
            Points_calculated=True   

        if event == '-BARCODE_DETECTED-':
            Barcode_detected=True

        if event == '-COBOT_MOVED-':
            Cobot_moved=True
                      
        if(values['-TAB_SWITCH-']=='Options'): 
                
            if event == "Apply": #If the apply-button is pressed in the options-tab, save the new parameters
                
                #Apply Changes
                v_cobot= float(values['e_v_cobot'])/100
                a_cobot= float(values['e_a_cobot'])/100

                Cobot_offset_x = float(values['e_offset_x'])
                Cobot_offset_y = float(values['e_offset_y'])
                Cobot_offset_z = float(values['e_offset_z'])

                Table_height= float(values['e_Table_height'])
                Spray_bound_cw= float(values['e_Spray_bound_cw'])
                Spray_bound_ccw= float(values['e_Spray_bound_ccw'])
                Cobot_IP= values['e_Cobot_IP']
                
                event=1
        
        if event == '-TAB_SWITCH-': #when tabs are changed and options are not applied, dissmiss changes
                window['e_v_cobot'].update(v_cobot*100)
                window['e_a_cobot'].update(a_cobot*100)
                
                window['e_offset_x'].update(Cobot_offset_x)
                window['e_offset_y'].update(Cobot_offset_y)
                window['e_offset_z'].update(Cobot_offset_z)

                window['e_Table_height'].update(Table_height)
                window['e_Spray_bound_cw'].update(Spray_bound_cw)
                window['e_Spray_bound_ccw'].update(Spray_bound_ccw)
                window['e_Cobot_IP'].update(Cobot_IP)
               
        if(values['-TAB_SWITCH-']=='Control'): #Main Working Part // runs when Control-Tab is opened


            if event == "Accept": #When 'accept-button' is pressed, go one step further in Case-Structure
                Thread_started =False
                Case +=1
                if Case>=5: #prevent overflow
                    Case=0

            
            if event == "Restart": #reset the Parameters when 'restart-button' is pressed
                pipeline= None
                Case=0
                circle_detected=False
                Points_calculated=False
                Barcode_detected=False
                Cobot_moved==False
                depth_detected=False
                print("restart")
            
        
            ## Case-Structure to implement the Main Work ##
            
            if Case==0: # 1.Step: Get the Depth-Cam and show the Vdeo-feed
                
                if pipeline== None:
                    pipeline = rs.pipeline()
                    config = rs.config()
                   
                    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

                    # Start streaming
                    pipeline.start(config)

                    colorizer = rs.colorizer()
                    
                    #change the view pattern of the Picture
                    colorizer.set_option(rs.option.visual_preset, 1) # 0=Dynamic, 1=Fixed, 2=Near, 3=Far
                    colorizer.set_option(rs.option.min_distance, 1.2)
                    colorizer.set_option(rs.option.max_distance, 1.45)

                    colorizer.set_option(rs.option.color_scheme, 5)

                else:                   
                    #waiting for frames from the camera to come
                    frames = pipeline.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    
                    

                    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame, ).get_data())
                    #colorized_depth[0:125][:]=0
                    #colorized_depth[:][405:480]=0
                    #print(colorized_depth)
                    #colorized_depth[0:200][120:400]=0
                    #colorized_depth[440:640][120:400]=0
                    imgbytes = cv2.imencode(".png", colorized_depth)[1].tobytes()
                    window["-IMAGE-"].update(data=imgbytes)
                   
            if Case== 1: #2.Step: Detect the Tire
                                                 

                if Thread_started==False: #Starting the Tire Detection in different Thread
                    
                    window['e_Status'].update("Detecting tire...")
                    getCircle = ThreadWithReturnValue(target=circle_detection, args=(colorized_depth, window,), daemon=True)
                    getCircle.start()
                    Thread_started=True
                

                if(circle_detected == True) and (getCircle.is_alive()==False):
                                      
                    getCircle.join()
                    a, b, colorized_depth_circle = getCircle.result
                    imgbytes = cv2.imencode(".png", colorized_depth_circle)[1].tobytes()
                    window["-IMAGE-"].update(data=imgbytes)
                    window['e_Status'].update("Tire detected! Please accept detection.")
                
                            
            if Case == 2: #3.Step: Calculate the 3D Points from 2D Pixel (center of Tire)
                

                if Thread_started==False: #Starting the Calclation in different Thread
                    
                    window['e_Status'].update("Calculating Points...")            
                    getPoints = ThreadWithReturnValue(target=pixel_to_point, args=(a,b,times_of_capturing,Table_height,window,), daemon=True)
                    getPoints.start()
                    
                    Thread_started=True
                    
                if(getPoints.is_alive() == False) and (Points_calculated==True):
                                        
                    getPoints.join()
                    Camera_x, Camera_y, Camera_z = getPoints.result
                    Cobot_x = Cobot_offset_x - Camera_y 
                    Cobot_y = Camera_x + Cobot_offset_y
                    Cobot_z = Camera_z + Cobot_offset_z

                    Case+=1
                    Thread_started=False
                    Points_calculated=False

            if Case ==3: #3.Step detect the Barcode

                 if Thread_started==False: 
                    window['e_Status'].update("Detecting Barcode..")
                    #getBarcode = ThreadWithReturnValue(target=get_Alpha, args=(window,),daemon=True) 
                    getBarcode = ThreadWithReturnValue(target=get_barcode, args=(window,),daemon=True) 
                    getBarcode.start()
                    Thread_started=True

                    
                 if (Barcode_detected==True) and (getBarcode.is_alive()==False):
                    
                    getBarcode.join()
                    x_barcode,y_barcode, img= getBarcode.result
                    
                    
                    Circle_to_Barcode_x = x_barcode-Camera_x
                    Circle_to_Barcode_y = y_barcode-Camera_y

                    alpha= np.degrees(np.arctan2(Circle_to_Barcode_y,Circle_to_Barcode_x))
                    alpha= alpha + 90 # Tramsform camera angle in Cobot angle


                    alpha_bound_r=alpha+Spray_bound_cw
                    alpha_bound_l=alpha-Spray_bound_ccw

                    imgbytes = cv2.imencode(".png", img)[1].tobytes()
                    window["-IMAGE-"].update(data=imgbytes)
                    window['e_Status'].update("Calculation done & barcode detected! Please accept detection.")
                                
            
            if Case == 4:  #4.Step: Move the Cobot         
                
                if Thread_started==False:

                    window['e_Status'].update("Spraying Tire, please wait..")
                    moveCobot = ThreadWithReturnValue(target= move_cobot, args=(Cobot_IP, v_cobot, a_cobot, Cobot_x, Cobot_y, Cobot_z,alpha_bound_r, alpha_bound_l, Spray_bound_cw, Spray_bound_ccw,window,))
                    moveCobot.start()
                    Thread_started=True
                    
                    y1=25
                    y2=1080
                    x1=19
                    x2=1920


                if (Thread_started==True) and (moveCobot.is_alive()==True):
                    
                    if not cap_open: 
                        cap = cv2.VideoCapture(1)
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, (680))
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, (420))
                        cap_open=True

                    if cap.isOpened():
                        ret, img = cap.read()   

                    roi = img[y1:y2, x1:x2]


                    imgbytes = cv2.imencode(".png", roi)[1].tobytes()
                    window["-IMAGE-"].update(data=imgbytes)


                if(Cobot_moved==True)and (moveCobot.is_alive()==False):
                    cap.release()
                    cap_open=False
                    cv2.destroyAllWindows()
                    moveCobot.join
                    Thread_started=False
                    window.write_event_value("Restart", '** DONE **') #Restart the Program

    window.close()
    
main()


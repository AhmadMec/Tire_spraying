
import math

import cv2 as cv
import numpy as np
import pyrealsense2 as rs
import rtde_control
import rtde_io
import rtde_receive


#Cicrle Detection
def circle_detection(colorized_depth, window):
    #Grayscale the depth_picture
    gray = cv.cvtColor(colorized_depth, cv.COLOR_BGR2GRAY)
    # Gaussian Blur using 5 * 5 kernel. ##With Gray Picture,
    gray_blurred = cv.GaussianBlur(gray, (5, 5), 0)
    
    cv.imwrite("C:/Users/uic27710/Desktop/InnoLab/BilderProjekt1/unverzerrt.jpg", colorized_depth)
    # Apply Hough transformation on the blurred image.
    detected_circles = cv.HoughCircles(gray_blurred, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30)

    # Draw circles that are detected
    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))
        a, b, r = detected_circles[0][0]

        # Draw the circumference of the circle.
        
        cv.circle(colorized_depth, (a, b), r, (0, 200, 0), 3)
        
        # Draw a small circle (of radius 1) to show the center.
        cv.circle(colorized_depth, (a, b), 1, (0, 0, 255), 5)
       
        window.write_event_value('-CIRCLE_DETECTED-', '** DONE **') #Message for Main Process

        return a,b,colorized_depth


#Turn 2D Pixel in 3D-Camera-Coordinates
def pixel_to_point(a,b,times_of_capturing,Table_height, window):
    #this step could be better


    k = 0  # iterable to count the times when point has been =/= [0, 0, 0]
    n = 0  # iterable to count the number of measures (we want to get a certain point for "times_of_capturing")
    list_of_coordinates = [0, 0, 0]  # list where the coordinates get stored to get the average

    # GETTING THE CAMERA COORDINATE IN 3D (x, y, z) in meters from pixel (x, y)

    pipeline = rs.pipeline()  # make the camera run
    #pipeline = rs.pipeline()

    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Align Depth to color stream
    align_to = rs.stream.color
    align = rs.align(align_to)

    pipeline.start(config)  # start the camera with standard configurations
    Tire_camera_distance = 0  #initializey


    # capturing the 5 coordinates n-times, to calculate average
    while n < times_of_capturing:
        frames = pipeline.wait_for_frames()  # waiting for frames from the camera to come
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        aligned_frames = align.process(frames)
        depth_frame_align = aligned_frames.get_depth_frame()
        color_frame_align = aligned_frames.get_color_frame()

        
        #Detect Tire Hight with smaller aligned Picture
        pc2 = rs.pointcloud()
        points_hight = pc2.calculate(depth_frame_align)
        pc2.map_to(color_frame_align)

        v = points_hight.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

        #Tirme_camera_distance as average from all points between 0.5 and 1.40m away from camera
        list_of_points = abs(verts[:, 2])
        list_nonzero = [n for n in list_of_points if n > 0.5 and n < 1.37]
        Tire_camera_distance += np.average(list_nonzero)
     

        #Detect Koordinanates on Plane
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics  # getting the intrinsic parameters
        point = rs.rs2_deproject_pixel_to_point(intrinsics, (a, b), np.average(list_nonzero))  # pixel to point

        if point[2] > 0:
            for i in range(3):
                list_of_coordinates[i] += point[i]  # summing up the points
            k += 1
        n += 1
        
    if k > 0:
        for i in range(3):
            list_of_coordinates[i] /= k  # calculating the average for k times
    else:
        print("All measured values were invalid. It could be good"
                " to restart the camera and try it again. ")
        exit()

    Tire_camera_distance /= times_of_capturing

    # Calculate the target (middle of tire) in Camera Coordinates // extrating data from Vector to simplify
    Camera_x= list_of_coordinates[0]
    Camera_y= list_of_coordinates[1]
    Camera_z= Table_height-((Table_height-Tire_camera_distance)/2)
    pipeline.stop()

    window.write_event_value('-POINTS_CALCULATED-', '** DONE **') #Message for Main Process
    return Camera_x, Camera_y, Camera_z




def move_cobot(Cobot_IP, v_cobot, a_cobot, Cobot_x, Cobot_y, Cobot_z, alpha_bound_r, alpha_bound_l, Spray_bound_cw, Spray_bound_ccw, window):
    abort=False

    if Cobot_z > 1.4 and Cobot_z < 0.5 :
        print("critical Error")
        exit()

    #Initialize
    rtde_c = rtde_control.RTDEControlInterface(Cobot_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(Cobot_IP)
    rtde_io_ = rtde_io.RTDEIOInterface(Cobot_IP)
    
    #rtde_io.setStandardDigitalOut(0, False)

    tcp = rtde_r.getActualTCPPose()


    #Origin

    rtde_c.moveJ([math.radians(0),
                    math.radians(-15),
                    math.radians(-115),
                    math.radians(-115),
                    math.radians(-90),
                    math.radians(0)],
                    v_cobot, a_cobot)

    rtde_c.moveJ([math.radians(0),
                    math.radians(-40),
                    math.radians(-115),
                    math.radians(-115),
                    math.radians(-90),
                    math.radians(0)],
                    v_cobot, a_cobot)

    

    # moving to the center point
    tcp = rtde_r.getActualTCPPose()
    tcp = [Cobot_x, Cobot_y, tcp[2], tcp[3], tcp[4], tcp[5]]
    rtde_c.moveL(tcp, v_cobot, a_cobot)

    # moving to Z
    tcp = rtde_r.getActualTCPPose()
    tcp[2] = Cobot_z
    rtde_c.moveL(tcp, v_cobot, a_cobot)
    

    #Turning/Spraying
    Joints=rtde_r.getActualQ()
    Ref_zero=  Joints[0]

    #Different posibilities of Tireposition
    #if Ref_zero > alpha_bound_l and Ref_zero < alpha_bound_r or abs(Ref_zero - alpha_bound_l)>=abs(Ref_zero-alpha_bound_r):
    if abs(Ref_zero - math.radians(alpha_bound_l))>=abs(Ref_zero-math.radians(alpha_bound_r)):
        #turning clockwise
        Joints[5]= math.radians(alpha_bound_r) - Ref_zero
        rtde_c.moveJ(Joints, 2, 1)

        rtde_io_.setStandardDigitalOut(0, True) #Start Spraying
        
        Joints[5]=math.radians(alpha_bound_r + 360 - (Spray_bound_cw + Spray_bound_ccw)) - Ref_zero 
        rtde_c.moveJ(Joints,1, 1)

        rtde_io_.setStandardDigitalOut(0, False) #Stop Spraying

        Joints[5]=math.radians(0)
        rtde_c.moveJ(Joints,2, 1)
    elif abs(Ref_zero - math.radians(alpha_bound_l))<abs(Ref_zero-math.radians(alpha_bound_r)):
        #turning counterclockwise
        Joints[5]= math.radians(alpha_bound_l) - Ref_zero
        rtde_c.moveJ(Joints, 2, 1)

        rtde_io_.setStandardDigitalOut(0, True) #Start Spraying

        Joints[5]=math.radians(alpha_bound_l - 360 + Spray_bound_cw + Spray_bound_ccw) - Ref_zero 
        rtde_c.moveJ(Joints,1, 1)

        rtde_io_.setStandardDigitalOut(0, False) #Stop Spraying

        Joints[5]=math.radians(0)
        rtde_c.moveJ(Joints,2, 1)

    
    #Returning Home
    tcp = rtde_r.getActualTCPPose()
    tcp[2] = 1
    rtde_c.moveL(tcp, v_cobot, a_cobot)

    rtde_c.moveJ([math.radians(0),
                    math.radians(-15),
                    math.radians(-115),
                    math.radians(-115),
                    math.radians(-90),
                    math.radians(0)],
                    v_cobot, a_cobot)
    

    window.write_event_value('-COBOT_MOVED-', '** DONE **') #Message for Main Process
    

def get_barcode(window):
    
    cap = cv.VideoCapture(1)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, (1920/2))
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, (1080/2))

    if not cap.isOpened():
         print("Cannot open camera")
         exit()

    i=0 

    while i < 20:
        ret, img = cap.read()
        i+=1

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        exit


    # Apply Hough transformation on the blurred image.
    


    #cv.imwrite("C:/Users/uic27710/Desktop/InnoLab/BilderProjekt1/verzerrt.jpg", img)


    #limit the region of interest to only detect contours on the desk

    #y1= np.int0( y_circle-(r*1.5))
    #y2= np.int0( y_circle+(r*1.5))
    #x1= np.int0( x_circle-(r*1.5))
    #x2= np.int0( x_circle+(r*1.5))

    y1=25
    y2=1080
    x1=19
    x2=1920
    

    roi = img[y1:y2, x1:x2]

    # convert to grayscale
    gray = cv.cvtColor(roi,cv.COLOR_BGR2GRAY)

    edged = cv.Canny(gray, 170, 490)
    #cv.imwrite('C:/Users/uic27710/Desktop/InnoLab/Projekt1/roi.jpg', roi)

    # Apply adaptive threshold
    thresh = cv.adaptiveThreshold(edged, 255, 1, 1, 5, 2)
    #thresh_color = cv.cvtColor(thresh, cv.COLOR_GRAY2BGR)

    # apply some dilation and erosion to join the gaps - change iteration to detect more or less area's
    thresh = cv.dilate(thresh,None,iterations = 1)
    thresh = cv.erode(thresh,None,iterations = 1)
    #cv.imwrite('C:/Users/uic27710/Desktop/InnoLab/BilderProjekt1/thresh.jpg', thresh)
    # Find the contours
    contours,hierarchy = cv.findContours(thresh,
                                            cv.RETR_TREE,
                                            cv.CHAIN_APPROX_SIMPLE)
    #cv.imwrite('C:/Users/uic27710/Desktop/InnoLab/Projekt1/thresh.jpg', thresh)
    # For each contour, find the bounding rectangle and draw it
    init = cv.boundingRect(contours[0])
    x_Barcode=init[0]+x1
    y_Barcode=init[1]+y1
    w_Barcode=init[2]
    h_Barcode=init[3]

    for cnt in contours:
        x_temp,y_temp,w_temp,h_temp = cv.boundingRect(cnt)
        if w_temp+h_temp < w_Barcode+h_Barcode:
            x_Barcode=x_temp+x1
            y_Barcode=y_temp+y1
            w_Barcode=w_temp
            h_Barcode=h_temp

    #cv.rectangle(img, (x_Barcode,y_Barcode),(x_Barcode+w_Barcode,y_Barcode+h_Barcode),(0,255,0), 2) #Draw smallest Rectangle
    #cv.circle(img, (x_circle, y_circle), r, (0, 255, 0), 2)
    #cv.circle(img, (x_circle, y_circle), 1, (0, 0, 255), 3)

    #cv.imwrite('C:/Users/uic27710/Desktop/InnoLab/Projekt1/img_circle.jpg', img)
    #cv.imwrite('C:/Users/uic27710/Desktop/InnoLab/Projekt1/roi_circle.jpg', roi)
    
    x_Barcode=int(np.around(x_Barcode+(w_Barcode/2)))
    
    y_Barcode=int(np.around(y_Barcode+(h_Barcode/2)))
    cv.circle(img, (x_Barcode, y_Barcode), 11, (0, 255, 255), 2)

    cap.release()
    cv.destroyAllWindows()


    #Translate Pixels in point to combot visual distortion:

    # GETTING THE CAMERA COORDINATE IN 3D (x, y, z) in meters from pixel (x, y)

    pipeline = rs.pipeline()  # make the camera run
    #pipeline = rs.pipeline()

    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Align Depth to color stream
    align_to = rs.stream.color
    align = rs.align(align_to)

    pipeline.start(config)  # start the camera with standard configurations

    # capturing the 5 coordinates n-times, to calculate average
    
    frames = pipeline.wait_for_frames()  # waiting for frames from the camera to come
    
    aligned_frames = align.process(frames)
    depth_frame_align = aligned_frames.get_depth_frame()
    color_frame_align = aligned_frames.get_color_frame()

    
    #Detect Tire Hight with smaller aligned Picture
    #pc2 = rs.pointcloud()
    #pc2.map_to(color_frame_align)
   

    #Detect Koordinanates on Plane
    intrinsics = depth_frame_align.profile.as_video_stream_profile().intrinsics  # getting the intrinsic parameters
    point = rs.rs2_deproject_pixel_to_point(intrinsics, (x_Barcode, y_Barcode), depth_frame_align.get_distance(x_Barcode, y_Barcode))  # pixel to point


    # Calculate the target (middle of tire) in Camera Coordinates // extrating data from Vector to simplify
    Barcode_x_co= point[0]
    Barcode_y_co= point[1]

    

    pipeline.stop()
    window.write_event_value('-BARCODE_DETECTED-', '** DONE **') #Message for Main Process

    return Barcode_x_co,Barcode_y_co,roi
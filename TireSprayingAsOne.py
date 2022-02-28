
import pyrealsense2 as rs
import numpy as np
import cv2 as cv 
import rtde_control
import rtde_receive
import math


def main():
    ##Cobot parameters
    v_cobot= 1
    a_cobot= 0.5

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)



    # Start streaming
    pipeline.start(config)
    try:
        while not (cv.waitKey(1) & 0xff == 27):
                
            #waiting for frames from the camera to come
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            

            colorizer = rs.colorizer()
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            cv.imshow("Colorized depth cam; Press escape to close", colorized_depth)
            #cv.imwrite("D:/TireSpraying/colorized_depth_image.jpg", colorized_depth)
    finally:
        pipeline.stop()
        cv.destroyAllWindows()



    ##### Circle Detection ##### 
    ##Vielleicht löschen, durchvorherigen Schritt ersetzen, weitere Tests notwendig##
    gray = cv.cvtColor(colorized_depth, cv.COLOR_BGR2GRAY)
        # Gaussian Blur using 5 * 5 kernel. ##With Gray Picture,
    gray_blurred = cv.GaussianBlur(gray, (5, 5), 0)

    # Apply Hough transformation on the blurred image.
    detected_circles = cv.HoughCircles(gray_blurred, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30)

    # Draw circles that are detected
    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))
        a, b, r = detected_circles[0][0]

        # Draw the circumference of the circle.
        cv.circle(colorized_depth, (a, b), r, (0, 255, 0), 2)

        # Draw a small circle (of radius 1) to show the center.
        cv.circle(colorized_depth, (a, b), 1, (0, 0, 255), 3)
        cv.imshow("Detected Circle: Press 'Y' to Start, press esc to cancel", colorized_depth)
        
        
        while not (cv.waitKey(1) == ord('y')): #Wait to accept

            if (cv.waitKey(1) & 0xff == 27) :
                print("Abort by User")
                exit()
        
        # Print the center point and radius
        print(f"CP_u: {a}; CP_v: {b}; radius: {r}")
        print("Detection succesfull")
        cv.destroyAllWindows()


    #### Pixel in Punkt umrechnen ###    

    times_of_capturing=3


    #print("Starting the conversion... ")

    k = 0  # iterable to count the times when point has been =/= [0, 0, 0]
    n = 0  # iterable to count the number of measures (we want to get a certain point for "times_of_capturing")
    # standard case is 5 times

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
    
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        aligned_frames = align.process(frames)
        depth_frame_align = aligned_frames.get_depth_frame()
        color_frame_align = aligned_frames.get_color_frame()

        #depth_frame = frames.get_depth_frame()  # getting the depth frames
        
        #Detect Tire Hight with smaller aligned Picture
        pc2 = rs.pointcloud()
        points_hight = pc2.calculate(depth_frame_align)
        #pc2.map_to(color_frame_align)

        v = points_hight.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

        #Tirme_camera_distance as average from all points between 0.5 and 1.40m away from camera
        list_of_points = abs(verts[:, 2])
        list_nonzero = [n for n in list_of_points if n > 0.5 and n < 1.27]
        Tire_camera_distance += np.average(list_nonzero)
        
        # Debug
        #print(np.average(list_nonzero))

        #Detect Koordinanates on Plane
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics  # getting the intrinsic parameters
        point = rs.rs2_deproject_pixel_to_point(intrinsics, (a, b), np.average(list_nonzero))  # pixel to point

        if point[2] > 0:
            for i in range(3):
                list_of_coordinates[i] += point[i]  # summing up the points
            k += 1
        n += 1
        if n == times_of_capturing/2:
            print("Half of the frames are captured! ")

    if k > 0:
        for i in range(3):
            list_of_coordinates[i] /= k  # calculating the average for k times
    else:
        print("All measured values were invalid. It could be good"
                " to restart the camera and try it again. ")
        exit()

    ## ggf unnötig
    Tire_camera_distance /= times_of_capturing

    # Calculate the target (middle of tire) in Camera Coordinates // extrating data from Vector to simplify
    Camera_x= list_of_coordinates[0]
    Camera_y= list_of_coordinates[1]
    Camera_z= 1.47-((1.47-Tire_camera_distance)/2)

    #Debug Messages
    #print("Point on Plate: ")
    #print(f"\n({a}, {b}) --> ({list_of_coordinates[0]}, {list_of_coordinates[1]}, {list_of_coordinates[2]})\n")

    print("Tire Height")
    print((1.47-Tire_camera_distance))

    #print("Target for Cobot in Camera Coordinates")
    #print(f"\n({a}, {b}) --> ({list_of_coordinates[0]}, {list_of_coordinates[1]}, {list_of_coordinates[2]-((list_of_coordinates[2]-Tire_camera_distance)/2)})\n")

    #print(f"\n({a}, {b}) --> ({Camera_x}, {Camera_y}, {Camera_z})\n")


    pipeline.stop()
    cv.destroyAllWindows()

    ## Transform Camera in Robot Coordinates ##

    Cobot_offset_x = 0.295 #Messung
    Cobot_offset_y = 0 #Ausrichtung
    Cobot_offset_z = 0.015 #Messung

    Cobot_x = Cobot_offset_x - Camera_y 
    Cobot_y = Camera_x + Cobot_offset_y
    Cobot_z = Camera_z + Cobot_offset_z

    ## Move the Cobot ##
    Spray_bound=2 #Degree which is going to be not Sprayed
    alpha= get_Alpha()
    alpha_bound_r=alpha+(Spray_bound/2)
    alpha_bound_l=alpha-(Spray_bound/2)

    print(alpha)

    if Cobot_z < 1.4 and Cobot_z > 0.5 :
        print("Cobot Target")
        print(f"\n({a}, {b}) --> ({Cobot_x}, {Cobot_y}, {Cobot_z})\n")
        rtde_c = rtde_control.RTDEControlInterface("169.254.95.199")
        rtde_r = rtde_receive.RTDEReceiveInterface("169.254.95.199")

       


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
        if Ref_zero > alpha_bound_l and Ref_zero < alpha_bound_r or abs(Ref_zero - alpha_bound_l)>=abs(Ref_zero-alpha_bound_r):
            #turning clockwise
            Joints[5]= math.radians(alpha_bound_r) - Ref_zero
            rtde_c.moveJ(Joints, 1, 1)
            #Start Spraying
            
            print("start spraying // cw")
            Joints[5]=math.radians(alpha_bound_r + 360 - Spray_bound) - Ref_zero 
            rtde_c.moveJ(Joints,0.5, 1)
            #Stop Spraying
            
            print("stop spraying")
            Joints[5]=math.radians(0)
            rtde_c.moveJ(Joints,1, 1)
        elif abs(Ref_zero - alpha_bound_l)<abs(Ref_zero-alpha_bound_r):
            #turning counterclockwise
            Joints[5]= math.radians(alpha_bound_l) - Ref_zero
            rtde_c.moveJ(Joints, 1, 1)
            #Start Spraying
            
            print("start spraying // ccw")
            Joints[5]=math.radians(alpha_bound_l - 360 + Spray_bound) - Ref_zero 
            rtde_c.moveJ(Joints,0.5, 1)
            #Stop Spraying
            
            print("stop spraying")
            Joints[5]=math.radians(0)
            rtde_c.moveJ(Joints,1, 1)

        
        #Returning
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
        print("Done!")

    else:
        print("no realistic values")




#define function Alpha to get the Barcode


def get_Alpha():

    cap = cv.VideoCapture(2)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, (1920/2))
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, (1080/2))

    if not cap.isOpened():
         print("Cannot open camera")
         exit()

    i=0 

    while i < 5:
        ret, img = cap.read()
        i+=1

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        exit

    #img = cv.imread('D:/TireSpraying/CameraFrame1.jpg')

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Gaussian Blur using 5 * 5 kernel.
    gray_blurred = cv.GaussianBlur(gray, (5, 5), 0)

    # Apply Hough transformation on the blurred image.
    detected_circles = cv.HoughCircles(gray_blurred, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=92, maxRadius=160)

    # Draw circles that are detected
    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))
        x_circle,  y_circle, r = detected_circles[0][0]

    y1= np.int0( y_circle-(r*1.5))
    y2= np.int0( y_circle+(r*1.5))
    x1= np.int0( x_circle-(r*1.5))
    x2= np.int0( x_circle+(r*1.5))
    print(y1)

    roi = img[y1:y2, x1:x2]

    # convert to grayscale
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)

    edged = cv.Canny(img, 170, 490)
    # Apply adaptive threshold
    thresh = cv.adaptiveThreshold(edged, 255, 1, 1, 11, 2)
    thresh_color = cv.cvtColor(thresh, cv.COLOR_GRAY2BGR)

    # apply some dilation and erosion to join the gaps - change iteration to detect more or less area's
    thresh = cv.dilate(thresh,None,iterations = 1)
    thresh = cv.erode(thresh,None,iterations = 1)

    # Find the contours
    contours,hierarchy = cv.findContours(thresh,
                                            cv.RETR_TREE,
                                            cv.CHAIN_APPROX_SIMPLE)

    # For each contour, find the bounding rectangle and draw it
    init = cv.boundingRect(contours[0])
    x_Barcode=init[0]
    y_Barcode=init[1]
    w_Barcode=init[2]
    h_Barcode=init[3]

    for cnt in contours:
        x_temp,y_temp,w_temp,h_temp = cv.boundingRect(cnt)
        if w_temp+h_temp < w_Barcode+h_Barcode:
            x_Barcode=x_temp
            y_Barcode=y_temp
            w_Barcode=w_temp
            h_Barcode=h_temp

    cv.rectangle(img, (x_Barcode,y_Barcode),(x_Barcode+w_Barcode,y_Barcode+h_Barcode),(0,255,0), 2) #Draw smallest Rectangle
    cv.circle(img, (x_circle, y_circle), r, (0, 255, 0), 2)
    cv.circle(img, (x_circle, y_circle), 1, (0, 0, 255), 3)

    cv.imshow('img',img)
    print("x: "+ str(x_Barcode) + "// y: "+str(y_Barcode))

    Circle_to_Barcode_x = x_Barcode-x_circle
    Circle_to_Barcode_y = y_Barcode-y_circle

    Alpha= np.degrees(np.arctan2(Circle_to_Barcode_y,Circle_to_Barcode_x))
    Alpha= Alpha + 90 # Tramsform camera angle in CObot angle

    cv.waitKey(0)
    cv.destroyAllWindows()

    print("x:" + str(Circle_to_Barcode_x))
    print("y:" + str(Circle_to_Barcode_y))
    print("alpha:" + str(Alpha))

    return Alpha

if __name__ == '__main__':
    main()
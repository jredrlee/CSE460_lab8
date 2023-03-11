import socket
import sys
import time
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

IP_ADDRESS = "192.168.0.210" #'192.168.0.202'


positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

#################################


# Test Code
Goal = (-3.2,2.4,0.)




################################





if __name__ == "__main__":
    clientAddress = "192.168.0.48"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 210

    
    #robot_control_code 
    # Connect to the robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP_ADDRESS, 5000))
    print('Connected')


    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()


    ## initial parameters for robot for part 3 ##
    
    
    
    ######
    robot_data = []
    destination_data = []
    robot_time = []
    try:
        circle_time = 0
        c = 180/math.pi
        while is_running:
            if robot_id in positions:
                #Part 2 ##############################################################################

                ##### last position
                #print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                #res = tuple(map(lambda i, j: i - j, Goal, positions[robot_id]))
                
                #prints distance between goal and robot location
                #print('distance', res ) 

                ####arc-tangent equation
                
                #alpha = (math.atan2(res[1],res[0]) *180) / math.pi
                #print('alpha ', alpha)

                #omega = 200 *  (alpha - rotations[robot_id])
                
                #v = 1400 * math.sqrt((res[1] ** 2) + (res[0]**2))
                #u = np.array([v - omega, v + omega])

                #u[u > 1500] = 1500
                #u[u < -1500] = -1500

                #### Send control input to the motors
                #command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                #s.send(command.encode('utf-8'))

                #### Wait for 0.2 second
                #time.sleep(0.1)

                #######################################################################################


                #Part 3 ###########################################################################################

                #position of center is -1.5 and 0.25 for x and y respectively
                
                #circle_equation = (-1 *  math.sin( c * circle_time), math.cos(c * circle_time), 0.0)
                robot_time.append(circle_time)
                circle_equation = (  math.cos(circle_time) - 1.3, math.sin(circle_time) + 1.0, 0.0)
                destination_data.append(circle_equation)
                print('circle', circle_equation)
                print(circle_time)
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                robot_data.append(positions[robot_id])

                # new difference  of circle 
                # res = tuple(map(lambda i, j: i - j, circle_equation, positions[robot_id]))
                xer =circle_equation[0] - positions[robot_id][0]
                yer =circle_equation[1] - positions[robot_id][1]
                res = (xer,yer)
                print('res', res)

                #difference between new positions and original position
                #difference = tuple(map(lambda i, j: i - j, positions[robot_id] + circle_equation, positions[robot_id]))
                
                #prints distance between goal and robot location
                #print('distance', difference ) 

                #arc-tangent equation
                
                alpha = (math.atan2(res[1],res[0]) *180)/math.pi
                print('alpha ', alpha)

                #omega = 100 *  (alpha - rotations[robot_id])
                omega = 100 * ((math.atan2(math.sin((alpha - rotations[robot_id])*(math.pi / 180)), math.cos((alpha - rotations[robot_id])* (math.pi/180))) * 180) / math.pi)
                #v = 1400 * math.sqrt((res[1] ** 2) + (res[0]**2))
                v = 1200
                u = np.array([v - omega, v + omega])

                u[u > 1500] = 1500
                u[u < -1500] = -1500

                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))

                
                
                # Wait for 0.2 second
                circle_time += 0.1
                time.sleep(0.1)
                
                





                ######################################################################################################
        

    except KeyboardInterrupt:
        # STOP
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        streaming_client.shutdown()
        print('robot  x position')
        for i in robot_data:
            print(i[0])
        print()
        print('robot y position')
        for i in robot_data:
            print(i[1])
        print()
        print(' x destinations')
        for i in destination_data:
            print(i[0])
        print()
        print('y destinations')
        for i in destination_data:
            print(i[1])

        print()
        print('time')
        for i in robot_time:
            print(i)
        
        
    s.shutdown(2)
    s.close()


#####################################


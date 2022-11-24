# Python client example to get Lidar data from a drone
#
from array import array
from asyncio.windows_events import NULL
from pickle import TRUE
import pickle
from xml.sax.handler import property_interning_dict
import setup_path 
import airsim
import time
import sys
import math
import argparse
import pprint
import numpy

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.dijkstra import DijkstraFinder
from astar_algo import Astar3D

# Makes the drone fly and get Lidar data
class LidarTest:


    def __init__(self):

        # Connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        

    # Initialize the array containing the coordinates of all the obstacles
    def init_grid(self):
        x = numpy.zeros(15, dtype = int)
        y = numpy.zeros(15, dtype = int)
        z = numpy.zeros(15, dtype = int)
        point_cloud3d = numpy.c_[x.reshape(-1), y.reshape(-1), z.reshape(-1)]
        return point_cloud3d

        
    def run_a_star(self, start_grid_x, start_grid_y, start_grid_z, target_grid_x, target_grid_y, point_cloud3d):
        start_point = (start_grid_x, start_grid_y, start_grid_z)
        end_point = (target_grid_x, target_grid_y, 0)
        asa3d = Astar3D()

        # Pass search space over shape argument, to optimize results
        path3d = asa3d.generate_path(start_point = start_point, end_point = end_point, grid_map = point_cloud3d, shape = (15, 15, 15))
        return path3d
    

    # Investigate if the current found obstacle has already been discovered 
    def already_exists(self, obs_co, obs_x, obs_y, obs_z):
        j = 0
        for i in range(len(obs_co)):
            if obs_co[i][j] == obs_x and obs_co[i][j+1] == obs_y and obs_co[i][j+2] == obs_z:
                return True
        return False    
    # Check for obstacles that might be located below the drone
    def lidar_ground_check(self, obs_co, current_grid_x, current_grid_y, current_grid_z):
        if(current_grid_z > 0):
            lidar_data_ground = self.client.getLidarData("LidarSensor_Ground","Drone1");
            points_ground = self.parse_lidarData(lidar_data_ground)           
            pos_ground_x_max = numpy.max(points_ground[:,0])
            pos_ground_y_max = numpy.max(points_ground[:,1])
            pos_ground_z_max =  numpy.max(points_ground[:,2])
            pos_ground_x_min = numpy.min(points_ground[:,0])
            pos_ground_y_min = numpy.min(points_ground[:,1])
            pos_ground_z_min =  numpy.min(points_ground[:,2])
            if(current_grid_x == 7 and current_grid_y == 8 and current_grid_z == 3):
                print(pos_ground_x_max, pos_ground_x_min, pos_ground_y_max, pos_ground_y_min, pos_ground_z_max, pos_ground_z_min)
            if(pos_ground_x_min > 4.659 and pos_ground_x_max < 7):
                if(pos_ground_y_min > -1.8 and pos_ground_y_max < 1.8):
                    if(pos_ground_z_min > -2 and pos_ground_z_max < 1.9):
                        mask = self.already_exists(obs_co, current_grid_x, current_grid_y, current_grid_z - 1)
                        if(not mask):
                            return True
        return False

    # Check for obstacles that might be located above the drone 
    def lidar_sky_check(self, obs_co, current_grid_x, current_grid_y, current_grid_z):
        lidar_data_up = self.client.getLidarData("LidarSensor_Up","Drone1");
        points_up = self.parse_lidarData(lidar_data_up)
        if(points_up.size != 0):            
            pos_up_x_max = numpy.max(points_up[:,0])
            pos_up_y_max = numpy.max(points_up[:,1])
            pos_up_z_max =  numpy.max(points_up[:,2])
            pos_up_x_min = numpy.min(points_up[:,0])
            pos_up_y_min = numpy.min(points_up[:,1])
            pos_up_z_min =  numpy.min(points_up[:,2])
            if(pos_up_x_min > 3.8 and pos_up_x_max < 7):
                if(pos_up_y_min > -1.8 and pos_up_y_max < 1.8):
                    if(pos_up_z_min > -2 and pos_up_z_max < 1.7):
                        mask = self.already_exists(obs_co, current_grid_x, current_grid_y, current_grid_z + 1)
                        if(not mask):
                            return True
        return False

    # Scanning for obstacles
    # Calculate and set the coordinates for each obstacle found by LiDAR
    def lidarCheck(self, obs_co, current_grid_x, current_grid_y, current_grid_z, start_position_env):
        lidar_data_front = self.client.getLidarData("LidarSensor_Front","Drone1");
        points_front = self.parse_lidarData(lidar_data_front)
        maxsize = sys.maxsize
        pos_x = maxsize
        pos_y = maxsize
        pos_z = maxsize
        maxsize = sys.maxsize
        obstacle_coordinates = []
        found = False
        x = 0

        if len(points_front) == 0:
            point_x = 0
        else:
            minPoint_x = min(points_front[:,0])
            point_x = points_front[:,0]
            point_y = points_front[:,1]
            point_z = points_front[:,2]
        
        if((lidar_data_front.pose.position.z_val > 3 or lidar_data_front.pose.position.z_val < -3) and ((len(points_front) < 3 or minPoint_x == 0))):
            found = False
        else:
            for i in range(len(point_x)):
                x = point_x[i] / 5.8
                y = point_y[i] / 5.8
                z = point_z[i] 
                if(True):
                    if(x > 0.54 and x < 1.535):
                        pos_x = 1

                        if(y > -0.5125 and y < 0.472):
                            pos_y = 0
                        elif(y > 0.52 and y < 1.056):
                            pos_y = 1
                        elif(y > -1.0538 and y < -0.562):
                            pos_y = -1
                        else:
                            pos_y = maxsize

                        if(z > 0.5183 and z < 0.6):
                            if(current_grid_z > 0):
                                pos_z = current_grid_z - 1     
                            else:
                                pos_z = 0
                        elif(z > 0.3143 and z < 0.4373):
                                pos_z = current_grid_z
                        #elif(z > 0.76 and z < 0.77):
                        #elif(z > 2.5 and z < 2.8):
                        #    if(current_grid_z > 0):
                        #        pos_z = current_grid_z - 1
                        #        pos_y = 0
                            #else:
                            #    pos_z = maxsize

                        #elif(z > 0.2147 and z < 0.3111):
                        #    pos_z = 2
                        #elif(z > 0.1352 and z < 0.2021):
                        #    pos_z = 3
                        #elif(z > 0.0001 and z < 0.1302):
                        #    pos_z = 4
                        else:
                            pos_z = maxsize

                    elif(x > 1.562 and x < 2.566):
                        pos_x = 2

                        if(y > -0.484 and y < 0.477):
                            pos_y = 0
                        elif(y > 0.5258 and y < 1.5259):
                            pos_y = 1
                        elif(y > -1.5603 and y < -0.5603):
                            pos_y = -1
                        else:
                            pos_y = maxsize

                        if(z > 3.9 and z < 4.14):
                            if(current_grid_z > 0):
                                pos_z = current_grid_z - 1     
                            else:
                                pos_z = 0      
                        elif(z > 0.3143 and z < 0.54):
                            pos_z = current_grid_z
                        else:
                            pos_z = maxsize

                    elif(x > 2.58829 and x < 3.588291):
                        pos_x = 3

                        if(y > -0.48954 and y < 0.4627):
                            pos_y = 0
                        elif(y > 0.511558 and y < 1.47817):
                            pos_y = 1
                        elif(y > -1.556461 and y < -0.557405):
                            pos_y = -1
                        elif(y > 1.54603 and y < 1.791):
                            pos_y = 2
                        elif(y > -1.774318 and y < -1.591787):
                            pos_y = -2
                        else:
                            pos_y = maxsize

                        if(z > 4.1849 and z <  4.2661):
                            if(current_grid_z > 0):
                                pos_z = current_grid_z - 1     
                            else:
                                pos_z = 0      
                        elif(z > 0.3143 and z < 0.43):
                            pos_z = current_grid_z
                        else:
                            pos_z = maxsize

                    elif(x > 0.0104 and x < 0.45):
                        pos_x = 0

                        if(y > 0.5085 and y < 1.526):
                            pos_y = 1
                        elif(y > -1.5602 and y < -0.5604):
                            pos_y = -1
                        elif(y > 1.5602 and y < 2.5604):
                            pos_y = 2
                        elif(y > -2.593 and y < -1.5776):
                            pos_y = -2
                        else:
                            pos_y = maxsize

                        if(z > 0.5183 and z < 0.6):
                            if(current_grid_z > 0):
                                pos_z = current_grid_z - 1     
                            else:
                                pos_z = 0    
                        elif(z > 0.371 and z < 0.4373):
                            pos_z = current_grid_z
                        else:
                            pos_z = maxsize

                    else:
                        pos_x = maxsize

                if(pos_x != maxsize and pos_y != maxsize and pos_z != maxsize):

                    if(start_position_env == "a"):
                        if(current_grid_x + pos_x <15 and current_grid_y + pos_y < 15):
                            mask = self.already_exists(obs_co, current_grid_x + pos_x, current_grid_y + pos_y, pos_z)
                            if(not mask):
                                found = True
                                obstacle_coordinates.append([current_grid_x + pos_x, current_grid_y + pos_y, pos_z])

                    elif(start_position_env == "b"):
                        if(current_grid_x + pos_y <15 and current_grid_y + pos_x < 15):
                            mask = self.already_exists(obs_co, current_grid_x + pos_x, current_grid_y + pos_y, pos_z)
                            if(not mask):
                                found = True
                                obstacle_coordinates.append([current_grid_x + pos_y, current_grid_y + pos_x, pos_z])

        if(self.lidar_ground_check(obs_co, current_grid_x, current_grid_y, current_grid_z)):         
            obstacle_coordinates.append([current_grid_x, current_grid_y, current_grid_z - 1])
            found = True
        if(self.lidar_sky_check(obs_co, current_grid_x, current_grid_y, current_grid_z)):
            obstacle_coordinates.append([current_grid_x, current_grid_y, current_grid_z + 1])
            found = True
        obstacle_coordinates = list(map(list, set(map(tuple, obstacle_coordinates))))
        return obstacle_coordinates, found

    def set_movement(self, start_grid_x, start_grid_y, start_grid_z, target_grid_x, target_grid_y, point_cloud3d, start_position_env):
        path_list = []
        x = []
        y = []
        z = []
        obs_co = numpy.empty((0,3), dtype= int)
        scan_again = True
        found = False
        step = 5.8
        meters_traveled = 0
        current_grid_x = start_grid_x
        current_grid_y = start_grid_y
        current_grid_z = start_grid_z
        drone_x = self.client.simGetVehiclePose("Drone1").position.x_val
        drone_y = self.client.simGetVehiclePose("Drone1").position.y_val
        drone_z = self.client.simGetVehiclePose("Drone1").position.z_val
        path_list.append(numpy.array([drone_x, drone_y, drone_z]))
        path = self.run_a_star(current_grid_x, current_grid_y, current_grid_z, target_grid_x, target_grid_y, point_cloud3d)
        while(True):
            for i in range(len(path)):
                if(scan_again):
                    obstacle_coordinates, found = self.lidarCheck(obs_co, current_grid_x, current_grid_y, current_grid_z, start_position_env)
                # If an obstacle was found rerun the algorithm in order to update the path so it includes the new found obstacle 
                if(found):             
                    for j in range(len(obstacle_coordinates)):
                        x = numpy.append(x, [obstacle_coordinates[j][0]])
                        y = numpy.append(y, [obstacle_coordinates[j][1]])
                        z = numpy.append(z, [obstacle_coordinates[j][2]])
                    obs_co = numpy.concatenate((obs_co, obstacle_coordinates), axis = 0)
                    point_cloud3d = numpy.c_[x.reshape(-1), y.reshape(-1), z.reshape(-1)] 
                    path = self.run_a_star(current_grid_x, current_grid_y, current_grid_z, target_grid_x, target_grid_y, point_cloud3d)
                    found = False
                    scan_again = False
                    break
                else:
                    print("\n")
                    print(current_grid_x, current_grid_y, current_grid_z)
                    print("\n")
                    print(obs_co)
                    print("\n")
                    print(path)
                    next_grid_x = path[i][0]
                    next_grid_y = path[i][1]
                    next_grid_z = path[i][2]
                    if(start_position_env == "a"):            
                        # Set current grid position according to the path 
                        # Set current actual drone position according to the path
                        # Add step(5.8) to the total meters traveled by the drone
                        if(current_grid_x < next_grid_x):
                            current_grid_x += 1
                            drone_x += step
                            meters_traveled += step                             
                           
                        if(current_grid_z < next_grid_z):
                            current_grid_z += 1
                            drone_z -= step-0.35
                            meters_traveled += step 
                        elif(current_grid_z > next_grid_z):
                            current_grid_z -= 1
                            drone_z += step
                            meters_traveled += step                             

                        if(current_grid_y < next_grid_y):
                            current_grid_y += 1
                            drone_y += step
                            meters_traveled += step 
                            
                        elif(current_grid_y > next_grid_y):
                            current_grid_y -= 1
                            drone_y -= step
                            meters_traveled += step                             

                    elif(start_position_env == "b"):

                        if(current_grid_y < next_grid_y):
                            current_grid_y += 1
                            drone_y += step
                            meters_traveled += 5.8 

                        if(current_grid_z < next_grid_z):
                            current_grid_z += 1
                            drone_z -= step-0.35
                            meters_traveled += step 
                        elif(current_grid_z > next_grid_z):
                            current_grid_z -= 1
                            drone_z += step
                            meters_traveled += step  

                        if(current_grid_x < next_grid_x):
                            current_grid_x += 1
                            drone_x -= step
                            meters_traveled += 5.8 

                        elif(current_grid_x > next_grid_x):
                            current_grid_x -= 1
                            drone_x += step
                            meters_traveled += 5.8 

                    self.client.moveToPositionAsync(drone_x, drone_y, drone_z, 3).join()
                    self.client.hoverAsync()
                    drone_x = self.client.simGetVehiclePose("Drone1").position.x_val
                    drone_y = self.client.simGetVehiclePose("Drone1").position.y_val
                    drone_z = self.client.simGetVehiclePose("Drone1").position.z_val
                    path_list.append(numpy.array([drone_x, drone_y, drone_z]))         
                    scan_again = True
            if(current_grid_x == target_grid_x and current_grid_y == target_grid_y):
                break
            #print(path_list)
        return path_list, meters_traveled

    # Set parameters for the program to run.
    # Initializes the algorithms
    def move_on_path(self):
        while(True):
            start_position_env = input("Please select starting position...(a / b):")
            print("\n")
            if(start_position_env == "a" or start_position_env == "b"):
                break
            else:
                continue
        path_list = []
        start_time = time.perf_counter()

        while(True):          
            try:
                target_grid_x = int(input("Please type the ending goal coordinates on X Axis (0, 14): "))
                target_grid_y = int(input("Please type the ending goal coordinates on Y Axis (0, 14): "))
                if((target_grid_x >= 0 and target_grid_x < 15) and (target_grid_y >= 0 and target_grid_y < 15)):
                    break
                else:
                    continue
            except ValueError:
                
                ("Invalid Input. Try Again \n")
        
        # Drone moves to the designated starting position
        if(start_position_env == "a"):
            start_grid_x = 0
            start_grid_y = 7
            start_grid_z = 0

        elif(start_position_env == "b"):
            start_grid_x = 7
            start_grid_y = 0
            start_grid_z = 0

            y = self.client.simGetVehiclePose("Drone1").position.y_val + (7 * 5.8)
            self.client.moveToPositionAsync(0, -y, -5, 10).join()
            x = self.client.simGetVehiclePose("Drone1").position.x_val + (7 * 5.8)
            self.client.moveToPositionAsync(x, -y, -5, 10).join()
            self.client.rotateToYawAsync(90).join()

        point_cloud3d = self.init_grid()
        path_list, meters_traveled = self.set_movement(start_grid_x, start_grid_y, start_grid_z, target_grid_x, target_grid_y, point_cloud3d, start_position_env)
        self.client.hoverAsync()
        end_time = time.perf_counter()
        total_runtime = end_time - start_time
        print("\t\nTotal time needed to reach the target: %f\n" %total_runtime)        
        print("\tTotal distanced traveled to reach the target: %f\n" %meters_traveled)
        print("\tLanding...\n")
        self.client.hoverAsync()
        self.client.landAsync().join()
        return path_list

    # Executes the method responsible for moving the drone around   
    def execute(self):

        print("Arming the drone...")
        self.client.armDisarm(True)
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)

        airsim.wait_key('Press any key to takeoff\n')
        self.client.takeoffAsync().join()
        self.client.hoverAsync()

        state = self.client.getMultirotorState()
        path_list = self.move_on_path()
        self.moveToStart(path_list)

    # Going back to the starting position following the reversed path that we used to reach the target
    def moveToStart(self, path_list):
        print("\tGOING BACK TO START... \n")
        path_list.reverse()
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-5, 5).join()
        start_time = time.perf_counter()
        for i in range(len(path_list)):
            self.client.moveToZAsync(path_list[i][2], 5).join()
            self.client.moveToPositionAsync(path_list[i][0], path_list[i][1], path_list[i][2], 5).join()

        print("\tStarting position reached\n")
        end_time = time.perf_counter()
        total_runtime = end_time - start_time
        meters = 5.8 * (len(path_list) -1)
        print("\tTime Elapsed Moving From Destination Back to Start: %f seconds\n" % total_runtime)
        print("\tTotal Distance Traveled: %f meters\n" % meters)
        airsim.wait_key('Press Any Key to Land')
        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("\tAlready Landed...")
        else:
            print("\tLanding...\n")
            self.client.landAsync().join()
                    

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()
        self.client.enableApiControl(False)
        print("Done!\n")

    def parse_lidarData(self, data):

        # Reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points


# Main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()




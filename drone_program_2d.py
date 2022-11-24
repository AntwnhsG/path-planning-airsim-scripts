# Python client example to get Lidar data from a drone
#
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

# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def init_grid(self, start_grid_x, start_grid_y, start_position_env):
        matrix = numpy.empty((15,15), dtype= int)
        if(start_position_env == "b"):
            for i in range(0, start_grid_x):
                matrix[i][start_grid_y] = 35 - (i * 5)
            for i in range(start_grid_x, 15):
                matrix[i][start_grid_y] = (i * 5) - 35
            for i in range(start_grid_y + 1, 15):
                for j in range(15):
                    g_cost = (matrix[j][i-1] + 5)
                    matrix[j][i] = g_cost
        elif(start_position_env == "a"):
            for j in range(0, start_grid_y):
                matrix[start_grid_x][j] = (35 - (j * 5))   
            
            for j in range(start_grid_y, 15):
                matrix[start_grid_x][j] = ((j * 5) - 35)  

            for i in range(start_grid_x + 1, 15):
                for j in range(15):
                    g_cost = (matrix[i-1][j] + 5)
                    matrix[i][j] = g_cost
        #print(matrix)
        return matrix
            
    def run_a_star(self, start_grid_x, start_grid_y, target_grid_x, target_grid_y, matrix):
        grid = Grid(matrix = matrix)
        start = grid.node(start_grid_y, start_grid_x)
        end = grid.node(target_grid_y, target_grid_x)     
        #start A* to find the path
        #finder = AStarFinder(diagonal_movement = DiagonalMovement.only_when_no_obstacle)
        finder = DijkstraFinder(diagonal_movement = DiagonalMovement.only_when_no_obstacle)
        path, runs = finder.find_path(start, end, grid)
        print('operations:', runs, 'path length:', len(path))
        print(grid.grid_str(path=path, start=start, end=end))
        print(path, "\n")
        grid.cleanup()

        return path

    # calculate and set the coordinates for each obstacle found by LiDAR
    # Checking for obstacles
    def lidarCheck(self, matrix, current_grid_x, current_grid_y, start_position_env):
        lidarData = self.client.getLidarData("LidarSensor1","Drone1");
        points = self.parse_lidarData(lidarData)
        maxsize = sys.maxsize
        pos_x = maxsize
        pos_y = maxsize
        maxsize = sys.maxsize
        obstacle_coordinates = []
        found = False
        x = 0
        if len(points) == 0:
            point_x = 0
        else:
            minPoint_x = min(points[:,0])
            point_x = points[:,0]
            point_y = points[:,1]
            point_z = points[:,2]

        if((lidarData.pose.position.z_val > 3 or lidarData.pose.position.z_val < -3) and ((len(points) < 3 or minPoint_x == 0))):
            found = False
        else:
            for i in range(len(point_x)):
                x = point_x[i] / 5.8
                y = point_y[i] / 5.8
                z = point_z[i]
                #0.6
                if(z < 0.6):
                    #print(x, y)
                    if(x > 0.54 and x < 1.535):
                        pos_x = 1
                        if(y > -0.5125 and y < 0.472):
                            pos_y = 0
                        elif(y > 0.52 and y < 1.056):
                            pos_y = 1
                            #-1.0538
                        elif(y > -1.0538 and y < -0.562):
                            pos_y = -1
                        else:
                            pos_y = maxsize
                    elif(x > 1.562 and x < 2.566):
                        pos_x = 2
                        if(y > -0.484 and y < 0.477):
                            pos_y = 0
                        elif(y > 0.5258 and y < 1.5259):
                            pos_y = 1
                            #-1.5603
                        elif(y > -1.5603 and y < -0.5603):
                            pos_y = -1
                        else:
                            pos_y = maxsize
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
                            #-1.774318
                        elif(y > -1.774318 and y < -1.591787):
                            pos_y = -2
                        else:
                            pos_y = maxsize
                            #-0.087 #0.486
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
                    else:
                        pos_x = maxsize
                if(pos_x != maxsize and pos_y != maxsize):
                    if(start_position_env == "a"):
                        if(current_grid_x + pos_x <15 and current_grid_y + pos_y < 15):
                            if(matrix[current_grid_x + pos_x][current_grid_y + pos_y] != 0):
                                found = True
                                obstacle_coordinates.append([current_grid_x + pos_x, current_grid_y + pos_y])
                    elif(start_position_env == "b"):
                        if(current_grid_x + pos_y <15 and current_grid_y + pos_x < 15):
                            if(matrix[current_grid_x + pos_y][current_grid_y + pos_x] != 0):
                                found = True
                                obstacle_coordinates.append([current_grid_x + pos_y, current_grid_y + pos_x])
            obstacle_coordinates = list(map(list, set(map(tuple, obstacle_coordinates))))
        return obstacle_coordinates, found

    def set_movement(self, start_grid_x, start_grid_y, target_grid_x, target_grid_y, matrix, start_position_env):
        path_list = []
        scan_again = True
        found = False
        step = 5.8
        meters_traveled = 0
        current_grid_x = start_grid_x
        current_grid_y = start_grid_y
        drone_x = self.client.simGetVehiclePose("Drone1").position.x_val
        drone_y = self.client.simGetVehiclePose("Drone1").position.y_val
        drone_z = self.client.simGetVehiclePose("Drone1").position.z_val
        path_list.append(numpy.array([drone_x, drone_y, drone_z]))
        path = self.run_a_star(current_grid_x, current_grid_y, target_grid_x, target_grid_y, matrix)
        while(True):
            for i in range(len(path)):
                if(scan_again):
                    obstacle_coordinates, found = self.lidarCheck(matrix, current_grid_x, current_grid_y, start_position_env)
                if(found):             
                    for j in range(len(obstacle_coordinates)):
                        matrix[obstacle_coordinates[j][0]][obstacle_coordinates[j][1]] = 0
                    path = self.run_a_star(current_grid_x, current_grid_y, target_grid_x, target_grid_y, matrix)
                    found = False
                    scan_again = False
                    break
                else:
                    next_grid_x = path[i][1]
                    next_grid_y = path[i][0]
                    if(start_position_env == "a"):
                        if(current_grid_x < next_grid_x):
                            current_grid_x += 1
                            drone_x += step
                            meters_traveled += 5.8 
                        if(current_grid_y < next_grid_y):
                            current_grid_y += 1
                            drone_y += step 
                            meters_traveled += 5.8 
                        elif(current_grid_y > next_grid_y):
                            current_grid_y -= 1
                            drone_y -= step
                            meters_traveled += 5.8 
                    elif(start_position_env == "b"):
                        if(current_grid_y < next_grid_y):
                            current_grid_y += 1
                            drone_y += step
                            meters_traveled += 5.8 
                        if(current_grid_x < next_grid_x):
                            current_grid_x += 1
                            drone_x -= step
                            meters_traveled += 5.8 
                        elif(current_grid_x > next_grid_x):
                            current_grid_x -= 1
                            drone_x += step
                            meters_traveled += 5.8 
                    print(current_grid_x, current_grid_y)
                    self.client.moveToPositionAsync(drone_x, drone_y, drone_z, 3).join()
                    self.client.hoverAsync()
                    drone_x = self.client.simGetVehiclePose("Drone1").position.x_val
                    drone_y = self.client.simGetVehiclePose("Drone1").position.y_val
                    path_list.append(numpy.array([drone_x, drone_y, drone_z]))
                    scan_again = True
            if(current_grid_x == target_grid_x and current_grid_y == target_grid_y):
                break

        return path_list, meters_traveled

#set parameters for the program to run.
#initializes the algorithms
    def move_on_path(self):
        while(True):
            start_position_env = input("Please select starting position...(a / b):")
            print("\n")
            if(start_position_env == "a" or start_position_env == "b"):
                break
            else:
                #print("\tWrong value, try again.\n")
                continue
        path_list = []
        start_time = time.perf_counter()
        # x is equivalent of j
        # y is equivalent to i
        #start_position_env = ""
        while(True):          
            #target_grid_x = int(input("Please type the ending goal coordinates on Y Axis (0, 14): "))
            #target_grid_y = int(input("Please type the ending goal coordinates on Y Axis (0, 14): "))
            try:
                target_grid_x = int(input("Please type the ending goal coordinates on X Axis (0, 14): "))
                target_grid_y = int(input("Please type the ending goal coordinates on Y Axis (0, 14): "))
                if((target_grid_x >= 0 and target_grid_x < 15) and (target_grid_y >= 0 and target_grid_y < 15)):
                    break
                else:
                    continue
            except ValueError:
                print("Invalid Input. Try Again \n")

        if(start_position_env == "a"):
            #target_grid_x = 14
            #target_grid_y = 4
            start_grid_x = 0
            start_grid_y = 7
        elif(start_position_env == "b"):
            #target_grid_x = 5
            #target_grid_y = 14
            start_grid_x = 7
            start_grid_y = 0
            y = self.client.simGetVehiclePose("Drone1").position.y_val + (7 * 5.8)
            self.client.moveToPositionAsync(0, -y, -5, 10).join()
            x = self.client.simGetVehiclePose("Drone1").position.x_val + (7 * 5.8)
            self.client.moveToPositionAsync(x, -y, -5, 10).join()
            self.client.rotateToYawAsync(90).join()

        current_grid_x = start_grid_x
        current_grid_y = start_grid_y
        re_run = False
        matrix = self.init_grid(start_grid_x, start_grid_y, start_position_env)
        path_list, meters_traveled = self.set_movement(start_grid_x, start_grid_y, target_grid_x, target_grid_y, matrix, start_position_env)

        self.client.hoverAsync()
        end_time = time.perf_counter()
        total_runtime = end_time - start_time
        print("\t\nTotal time needed to reach the target: %f\n" %total_runtime)        
        print("\tTotal distanced traveled to reach the target: %f\n" %meters_traveled)
        print("\tLanding...\n")
        self.client.hoverAsync()
        self.client.landAsync().join()
        return path_list

       
    def execute(self):

        print("Arming the drone...")
        self.client.armDisarm(True)
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)

        airsim.wait_key('Press any key to takeoff\n')
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-5, 5).join()
        self.client.hoverAsync()

        state = self.client.getMultirotorState()
        pathList = self.move_on_path()
        self.moveToStart(pathList)

         #Going back to the starting position following the reversed path that we used to reach the target
    def moveToStart(self, pathList):
        print("\tGOING BACK TO START... \n")
        pathList.reverse()
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-5, 5).join()
        start_time = time.perf_counter()
        for i in range(len(pathList)):
            self.client.moveToZAsync(pathList[i][2], 5).join()
            self.client.moveToPositionAsync(pathList[i][0], pathList[i][1], pathList[i][2], 5).join()

        print("\tStarting position reached\n")
        end_time = time.perf_counter()
        total_runtime = end_time - start_time
        meters = 5.8 * (len(pathList) -1)
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

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points


# main
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



import setup_path 
import airsim
import time
import sys
import argparse
import numpy
import csv
import re
import os.path

from os import path
from astar_algo import Astar3D
from area_definer import limit_generator

# Makes the drone fly and get Lidar data
class LidarTest:


    def __init__(self):

        # Connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.simRunConsoleCommand("t.MaxFPS 60")
        self.client.enableApiControl(True)
        self.maxsize = sys.maxsize
        self.minsize = -sys.maxsize
        self.start_grid_x = 0
        self.start_grid_y = 0 
        self.start_grid_z = 0 
        self.target_grid_x = 0
        self.target_grid_y = 0
        self.target_grid_z = 0
        self.start_x = 0
        self.start_y = 0
        self.start_z = 0
        self.target_x = 0
        self.target_y = 0 
        self.meters_traveled = 0
        self.total_steps = 0
        self.total_runtime = 0
        self.crashed_on_building = 0
        self.crashed_on_ground = 0
        self.out_of_limits = 0
        self.step = 5.8
        self.center_grid_y = 77
        self.reach_finish_zone_time = 0
        self.target_area_gate = 1
        self.start_time = 0

    # Initialize the array containing the coordinates of all the obstacles
    def init_grid(self):
        x = numpy.zeros(15, dtype = int)
        y = numpy.zeros(15, dtype = int)
        z = numpy.zeros(15, dtype = int)
        point_cloud3d = numpy.c_[x.reshape(-1), y.reshape(-1), z.reshape(-1)]
        return point_cloud3d

        
    def run_a_star(self, start_grid_x, start_grid_y, start_grid_z, point_cloud3d):
        start_point = (start_grid_x, start_grid_y, start_grid_z)
        end_point = (self.target_grid_x, self.target_grid_y, self.target_grid_z)
        asa3d = Astar3D()

        # Pass search space over shape argument, to optimize results
        path3d = asa3d.generate_path(start_point = start_point, end_point = end_point, grid_map = point_cloud3d, shape = (74, 155, 5))
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
        pos_ground_x_max = self.maxsize
        pos_ground_y_max = self.maxsize
        pos_ground_z_max = self.maxsize
        pos_ground_x_min = self.minsize
        pos_ground_y_min = self.minsize
        pos_ground_z_min = self.minsize
        if(current_grid_z > 0):
            lidar_data_ground = self.client.getLidarData("LidarSensor_Ground","Drone1");
            points_ground = self.parse_lidarData(lidar_data_ground) 
            if(points_ground.size != 0):
                pos_ground_x_max = numpy.max(points_ground[:,0])
                pos_ground_y_max = numpy.max(points_ground[:,1])
                pos_ground_z_max =  numpy.max(points_ground[:,2])
                pos_ground_x_min = numpy.min(points_ground[:,0])
                pos_ground_y_min = numpy.min(points_ground[:,1])
                pos_ground_z_min =  numpy.min(points_ground[:,2])
            if(pos_ground_x_min > 2.8 and pos_ground_x_max < 7):
                if(pos_ground_y_min > -1.8 and pos_ground_y_max < 1.8):
                    if(pos_ground_z_min > -2 and pos_ground_z_max < 1.9):
                        mask = self.already_exists(obs_co, current_grid_x, current_grid_y, current_grid_z - 1)
                        if(not mask):
                            return True
        return False

    # Check for obstacles that might be located above the drone 
    def lidar_sky_check(self, obs_co, current_grid_x, current_grid_y, current_grid_z):
        pos_up_x_max = self.maxsize
        pos_up_y_max = self.maxsize
        pos_up_z_max = self.maxsize
        pos_up_x_min = self.minsize
        pos_up_y_min = self.minsize
        pos_up_z_min = self.minsize
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
    def lidarCheck(self, obs_co, current_grid_x, current_grid_y, current_grid_z):
        lidar_data_front = self.client.getLidarData("LidarSensor_Front","Drone1");
        points_front = self.parse_lidarData(lidar_data_front)
        obstacle_coordinates = []
        found = False

        if len(points_front) == 0:
            point_x = []
        else:
            minPoint_x = min(points_front[:,0])
            point_x = points_front[:,0]
            point_y = points_front[:,1]
            point_z = points_front[:,2]
        
            if((lidar_data_front.pose.position.z_val > 3 or lidar_data_front.pose.position.z_val < -3) and ((len(points_front) < 3 or minPoint_x == 0))):
                found = False
            else:
                for i in range(len(point_x)):
                    x = point_x[i] / self.step
                    y = point_y[i] / self.step
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
                                pos_y = self.maxsize

                            if(z > 0.5183 and z < 0.6):
                                if(current_grid_z > 0):
                                    pos_z = current_grid_z - 1     
                                else:
                                    pos_z = 0
                            elif(z > 0.3143 and z < 0.4373):
                                    pos_z = current_grid_z
                            else:
                                pos_z = self.maxsize

                        elif(x > 1.562 and x < 2.566):
                            pos_x = 2

                            if(y > -0.484 and y < 0.477):
                                pos_y = 0
                            elif(y > 0.5258 and y < 1.5259):
                                pos_y = 1
                            elif(y > -1.5603 and y < -0.5603):
                                pos_y = -1
                            else:
                                pos_y = self.maxsize

                            if(z > 3.9 and z < 4.14):
                                if(current_grid_z > 0):
                                    pos_z = current_grid_z - 1     
                                else:
                                    pos_z = 0      
                            elif(z > 0.3143 and z < 0.54):
                                pos_z = current_grid_z
                            else:
                                pos_z = self.maxsize

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
                                pos_y = self.maxsize

                            if(z > 4.1849 and z <  4.2661):
                                if(current_grid_z > 0):
                                    pos_z = current_grid_z - 1     
                                else:
                                    pos_z = 0      
                            elif(z > 0.3143 and z < 0.43):
                                pos_z = current_grid_z
                            else:
                                pos_z = self.maxsize

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
                                pos_y = self.maxsize

                            if(z > 0.5183 and z < 0.6):
                                if(current_grid_z > 0):
                                    pos_z = current_grid_z - 1     
                                else:
                                    pos_z = 0    
                            elif(z > 0.371 and z < 0.4373):
                                pos_z = current_grid_z
                            else:
                                pos_z = self.maxsize

                        else:
                            pos_x = self.maxsize

                    if(pos_x != self.maxsize and pos_y != self.maxsize and pos_z != self.maxsize):

                        if(current_grid_x + pos_x < 74 and current_grid_y + pos_y < 155):
                            mask = self.already_exists(obs_co, current_grid_x + pos_x, current_grid_y + pos_y, pos_z)
                            if(not mask):
                                found = True
                                obstacle_coordinates.append([current_grid_x + pos_x, current_grid_y + pos_y, pos_z])
        if(self.lidar_ground_check(obs_co, current_grid_x, current_grid_y, current_grid_z)):         
            obstacle_coordinates.append([current_grid_x, current_grid_y, current_grid_z - 1])
            found = True
        if(self.lidar_sky_check(obs_co, current_grid_x, current_grid_y, current_grid_z)):
            obstacle_coordinates.append([current_grid_x, current_grid_y, current_grid_z + 1])
            found = True
        obstacle_coordinates = list(map(list, set(map(tuple, obstacle_coordinates))))
        return obstacle_coordinates, found

    def set_movement(self, point_cloud3d, limits):
        path_list = []
        x = []
        y = []
        z = []
        obs_co = numpy.empty((0,3), dtype= int)
        scan_again = True
        found = False
        current_grid_x = self.start_grid_x
        current_grid_y = self.start_grid_y
        current_grid_z = self.start_grid_z
        drone_x = self.start_x
        drone_y = self.start_y
        drone_z = self.start_z

        # Append the starting position to the path
        path_list.append(numpy.array([self.start_x, self.start_y, self.start_z]))

        path = self.run_a_star(current_grid_x, current_grid_y, current_grid_z, point_cloud3d)
        while(True):
            for i in range(len(path)):
                if(scan_again):
                    obstacle_coordinates, found = self.lidarCheck(obs_co, current_grid_x, current_grid_y, current_grid_z)

                # If an obstacle was found rerun the algorithm in order to update the path so it includes the new found obstacle 
                if(found):             
                    for j in range(len(obstacle_coordinates)):
                        x = numpy.append(x, [obstacle_coordinates[j][0]])
                        y = numpy.append(y, [obstacle_coordinates[j][1]])
                        z = numpy.append(z, [obstacle_coordinates[j][2]])
                    obs_co = numpy.concatenate((obs_co, obstacle_coordinates), axis = 0)
                    point_cloud3d = numpy.c_[x.reshape(-1), y.reshape(-1), z.reshape(-1)] 
                    path = self.run_a_star(current_grid_x, current_grid_y, current_grid_z, point_cloud3d)
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
                    # Set current grid position according to the path 
                    # Set current actual drone position according to the path
                    # Add step(5.8) to the total meters traveled by the drone
                    if(current_grid_x < next_grid_x):
                        current_grid_x += 1
                        drone_x += self.step
                        self.meters_traveled += self.step                             
                           
                    if(current_grid_z < next_grid_z):
                        current_grid_z += 1
                        drone_z -= self.step-0.35
                        self.meters_traveled += self.step 
                    elif(current_grid_z > next_grid_z):
                        current_grid_z -= 1
                        drone_z += self.step
                        self.meters_traveled += self.step                             

                    if(current_grid_y < next_grid_y):
                        current_grid_y += 1
                        drone_y += self.step
                        self.meters_traveled += self.step 
                            
                    elif(current_grid_y > next_grid_y):
                        current_grid_y -= 1
                        drone_y -= self.step
                        self.meters_traveled += self.step                             

                    self.client.moveToPositionAsync(drone_x, drone_y, drone_z, 3, timeout_sec=3).join()

                    # Check for collisions
                    drone_state = self.client.getMultirotorState()

                    position = self.client.getMultirotorState().kinematics_estimated.position
                    if position.x_val >= 68 and position.x_val <= 77 and \
                        position.y_val >= -50 and position.y_val <= 50 and self.target_area_gate == 1:
                        self.reach_finish_zone_time = time.time() - self.start_time
                        self.target_area_gate = 0

                    if(drone_z > -1):
                            self.total_steps = int(self.meters_traveled / self.step)
                            self.crashed_on_ground = 1
                            return None

                    self.client.hoverAsync()

                    # Append the current position to the path
                    position = drone_state.kinematics_estimated.position                    
                    drone_x = position.x_val
                    drone_y = position.y_val
                    drone_z = position.z_val
                    path_list.append(numpy.array([drone_x, drone_y, drone_z]))         
                    scan_again = True

            if(current_grid_x == self.target_grid_x and current_grid_y == self.target_grid_y):
                break
            x_dis = abs(self.target_x - drone_x)
            y_dis = abs(self.target_y - drone_y)
            if(limits[0] < x_dis and limits[1] < y_dis):
                self.out_of_limits = 1
                return None
        if self.reach_finish_zone_time == 0:
            self.crashed_on_building = 1

        self.total_steps = int(self.meters_traveled / self.step)
        return None

    # Set parameters for the program to run.
    # Initializes the algorithms
    def move_on_path(self, limits):
        self.start_time = time.time()
        point_cloud3d = self.init_grid()
        self.set_movement(point_cloud3d, limits)
        self.client.hoverAsync()
        end_time = time.time()
        self.total_runtime = end_time - self.start_time
        print("\t\nTotal time needed to reach the target: %f\n" %self.total_runtime)        
        print("\tTotal distanced traveled to reach the target: %f\n" %self.meters_traveled)
        print("\tTotal steps: %d\n" %int(self.total_steps))

    # Executes the method responsible for moving the drone around   
    def execute(self):
        print("Arming the drone...")
        with open('eval_points_whole.csv') as file_obj:
            heading = next(file_obj)
            reader_obj = csv.reader(file_obj)
            for row in reader_obj:
                self.start_x = int(row[0])
                self.start_y = int(row[1])
                self.start_z = int(row[2])
                self.target_x = int(row[3])
                self.target_y = int(row[4])
                target_z = int(row[5])

                # Establish the correct positions on the grid depending on start/ target actual positions
                self.target_grid_x = int((self.target_x / self.step) + abs((self.start_x / self.step)))

                if(self.start_y != 0):
                    self.start_grid_y = int(self.center_grid_y + (self.start_y / self.step))
                    self.target_grid_y = int(self.center_grid_y + (self.target_y / self.step))
                else:
                    self.start_grid_y = 77

                self.start_grid_z = abs(int(self.start_z / self.step))
                self.target_grid_z = abs(int(target_z / self.step))

                self.client.reset()
                self.client.enableApiControl(True)
                self.client.armDisarm(True)

                # Set teleport position info
                start_vector = airsim.Vector3r(self.start_x, self.start_y, 0)
                teleport_pose = airsim.Pose(start_vector)      

                # Teleport drone to requested position
                self.client.simSetVehiclePose(pose=teleport_pose, ignore_collision=True)

                self.client.moveToZAsync(self.start_z, 1).join()
                limits = limit_generator(self.start_x, self.start_y, self.target_x, self.target_y)

                # Reinitialize variables required for each episode (flight)
                self.meters_traveled = 0
                self.total_steps = 0
                self.crashed_on_building = 0
                self.crashed_on_ground = 0
                self.out_of_limits = 0

                self.move_on_path(limits)
                self.client.hoverAsync()
                data = [self.total_steps, self.total_runtime, 1, self.crashed_on_ground, self.crashed_on_building, self.out_of_limits, 1]
                self.log_to_file(data)

    def log_to_file(self, data, path="metrics.csv"):
        if(not os.path.exists("metrics.csv")):
            header = ['Total Steps', 'Total Runtime', 'Reached Target', 'Crashed on Floor', 'Crashed on Building', 'Out of Limits', 'Reached Finish Zone']
            with open("metrics.csv", 'w') as f:
                f.write(f'{header}\n')
        with open("metrics.csv", "a") as f:
            f.write(f'{data}\n')                    

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





# Documentation: CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


The points addressed below are towards fulfilment of the project requirements as per the rubric.

[//]: # (Image References)

[image1]: ./writeup_images/Left_Switch.jpg "Successful Run"
[image2]: ./writeup_images/Right_Switch.jpg "Right Lane Switch"
[image3]: ./writeup_images/Left_Switch.jpf "Left Lane Switch"


## Compilation
### Code Compiles Correctly

The code compiles without any issues using `cmake` and `make`. The generic `CMakeLists.txt` provided was used as is. 
Used the following set of commands to build

`cd CarND-Path-Planning-Project`
`mkdir build && cd build`
`cmake ..`
`make`

Ran the build on the simulator using 

`./path_planning`



## Valid Trajectories

### Car Drives 4.32 Miles Without Incident

The car was run of simulator to verify it runs without incidents. Tested multiple time to verify it runs beyond the minimum 4.32 miles.
A screenshot is provided below. No incidents, vis-a-vis exceeding acceleration/jerk/speed, collision, and lane departure occured during during finals trials.

![alt text][image1]


### Speed Limit
The speed limit of 50mph was followed thorughout. To prevent any uninteded violation, the car is set to drive at 48-49 mph most of the times.


### Acceleration and Jerk Not Exceeded

Acceleration and jerk stay way under the definded limits of 10m/s^2 and 10m/s^3, respectively. The path was smooth on curves and lane changes as well.


### Does Not Have Collisions

No collisions occurred during the final trials. This includes lane changes and sudden slow-downs.

### Car Stays in Lane

The car stays in it's lane for the entirety of time, expect for when changing lanes.

### Able to Change Lanes 

When needed, car is able to change lanes following a smooth trajectory. No jerks, high acceleration occur during these events. Details of the lane change process are described in the next section. Snapshots of couple lane changes are below.

Right Lane Change
![alt text][image2]

Left Lane Change
![alt text][image3]


## Reflection

### Path Generation

The program works in two parts. First section reads the data to make trajectory decisions. The next section then calculates the trajectory points based on decisions made by the previous one.

Section 1 of the code in `main.cpp` begins at line 166. Trajectory decisions like lane speed-up, slow-down, lane change etc are made here.
If the vehicle in front is driving slow or brakes, the 's' distance between the vehicles is checked to determine if the distance is unsafe. If it's unsafe, the vehicle is slowed down, by reducing the target speed. For the distance calculations, the predicted position of the vehicle after 1 second is used, i.e. after 50 points at intervals of 0.2 second.

To determine if a lane change is needed, two conditions are checked. If there is a vehicle ahead within a defined distance, and if the ego vehicle speed is below a limit, other lane conditions are evaluated to check if a lane change maneuver is safe. For this, first ego vehicle's lane is used to find what options are available, like for leftmost lane there is only one option, to go to the center lane, whiel the center lane has two options. The available lane(s) are checked for any traffic. Positions of all vehicles from the sensor fusion data are check for the potential lane(s). If there is any vehicle positioned in a defined vicinity currently, or predicted to be next one second, the maneurver is considered unsafe. Other wise, the target lane is chosen, and lane change trajectory is planned bases on that in section 2. A sample section of the code is given below.
Ref: `main.cpp` line 184

```
    // Check if the vehicle is in the potential lane
    if((d<(((pot_lane*lane_width)+ 2) + 2)) && (d>(((pot_lane*lane_width)+ 2) - 2))){

      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double)path_size*0.02*check_speed);  // For vehicle position one second later

      if(fabs(check_car_s-car_s) < safe_distance_lc){       // check magnitude of difference for safe distance         
        lc_safe = false;
        break;
      }              
    }
```

If the vehicle had to slow down to avoid any collisions, and the there is no vehicle in the safe distance ahead, the vehicle speeds back up to it's original target speed of close to 50 mph.



Section 2 of the code computes the trajectory based on the decisions made above. First the data is gathered for generating a spline. The first two points are chosen from the previous path, and the angle between them is calculated for continuity between the consecutive updates. 3 new points are chosen at an appropriate 's' distance (30 in this case), and their x and y coordinates are obtain using the function `getXY`. 

Ref: `main.cpp` line 261
```
    double range_inc = 30.0;   // Incremental distance for points to be used in spline

    // Get x an y coordinates for three points in incremental range
    vector<double> next_wp0 = getXY(car_s+(1.0*range_inc), (lane_num+0.5)*lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s+(2.0*range_inc), (lane_num+0.5)*lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s+(3.0*range_inc), (lane_num+0.5)*lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

The x and y coordinates are appended to the vectors, and then transformation is applied to each for vehicle position and angle. Teh corrected arrays are then used to create spline using the following code in line 287:
```
    tk::spline sp;          
    sp.set_points(range_x, range_y);     
```

The generated spline is used for the generating the new points. A target distance is chosen in 'x' and then 'y' values are then calculated using the spline. The coordinates are used to calculate the distance, which is used to calculate the smaller incremental steps for iteration, considering the vehicle speed and time steps. These coordinates are converted back to global before appending to the path vector.
Ref: `main.cpp` line 315
```
    x_point = (x_ref*cos(pos_yaw)) - (y_ref*sin(pos_yaw));
    y_point = (x_ref*sin(pos_yaw)) + (y_ref*cos(pos_yaw));
```
Values like starting lane, speed, and lane width are defined outside the lamda function.
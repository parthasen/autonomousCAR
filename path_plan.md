# Path-Planning-Project

###  Introduction
1. Safe navigation of my car around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
2. Car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway are provided.
3. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.
4. **Simulator** which contains the Path Planning Project are provided and can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).
5. Helper functions are provided like 'hasData()', 'distance()' to calculate euclidean distance,'ClosestWaypoint()' takes input of x and y value to find way points surrounded, 'NextWaypoint()'helps to know distance at front and back, 'getFrenet()' for finding Frenet Coordinate,  'getXY()'   

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

# Implementation

1. highway_map.csv file provides waypoints data, spread about 30 meters

		844.6398 1134.911 60.0463714599609 -0.002048373 -0.9999979
		875.0436 1134.808 90.4504146575928 -0.001847863 -0.9999983

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

2. Engineering points like Localization of car and other cars are estimated  by sensor fusion of 'telemetry' data

		// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];


		// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];//vector<vector<double>> sensor_fusion=j[1]["sensor_fusion"];

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 


3. First simply tried to move the car forward in a straight line at a constant 50 MPH velocity. Use the car's localization information and its heading direction to create a simple, straight path made up of (x,y) points that the car will visit sequentially every .02 seconds
 		
		https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/845a4549-6858-439e-a223-f387f15d477d


 		double dist_inc = 0.5;
		for(int i = 0; i < 50; i++)
    		{
          	next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          	next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    		}

Lots of jerk and cars go out of lane very soon !!

4. Staying in lane with spine.h. Three lanes are defined like 0,1,2. My car is Starting from lane 1.

		vector<double> next_wp0 = getXY(car_s+50,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                vector<double> next_wp1 = getXY(car_s+100,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                vector<double> next_wp2 = getXY(car_s+150,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

'ptsx.push_back()' and 'ptsy.push_back()' are used to get the spline

		tk::spline s;
		s.set_points(ptsx,ptsy);

5. Generating future paths

		for(int i = 1; i <= 50-previous_path_x.size(); i++)
                {
                ....

		  next_x_vals.push_back(x_point);
                  next_y_vals.push_back(y_point);
               	}

A maximum 50 points path will be generated. But few lesser points will be generted as '50-previous_path_x.size()' is used.

                  double x_point = x_add_on+(target_x)/N;
                  double y_point = s(x_point);

's(x_point)' is used to get waypoints to be.

Braking spline in to 30 points that used to get x,y points and target distance. 

		double target_x = 30.0;
                double target_y = s(target_x);
                double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

6. Changing from local to global coordinates

 		x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

7. Cold start and velocity control

		   if(too_close)
		   {
			ref_vel-=0.4;
		   }
		   else if(ref_vel<49.5 && dist_closest_front>50)
		   {
		   	ref_vel+=0.224;
		   }

If distance between cars is lower than 30 then velocity will be decreased but maximum velocity is 49.5

8. Checking cars at front and other lanes.

Lane of my car is estimated through 'LaneFrenet(car_d)'. Sensor fusion helps to get details of the other cars.

Sensor Fusion:

	        if(other_car_lane >= 0 && other_car_lane <=2)
            	{
            	int id = sensor_fusion[i][0];
		double x= sensor_fusion[i][1];
		double y= sensor_fusion[i][2];
                double vx = sensor_fusion[i][3];
                double vy= sensor_fusion[i][4];
                double check_speed= sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
		double dist = distance(car_x,car_y,x,y);
		...

Details of car at left lane is calculated using 'lane -1', right lane by 'lane+1'. 'check_car_s'  and 'car_s' are gives the distance between two cars. 

		if (other_car_lane==lane-1)
		{
			cars_myleft_s.push_back(check_car_s);
			cars_myleft_vel.push_back(check_speed);
		}


9. Changing lanes based on the distance of the other cars in same and other lanes.

		if(dist_closest_front<30)
                {
         	   too_close=true;
	   	   if(lane==1)//and cars_myright_s.size()==0
		   {
			if(dist_closest_leftfront>50 && dist_closest_leftback>20 && dist_closest_leftfront>dist_closest_rightfront)
				lane=lane-1;
			if(dist_closest_rightfront>50 && dist_closest_rightback>20 && dist_closest_leftfront<dist_closest_rightfront )
				lane=lane+1;
		 ...
	

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).



Note:  Basic Build `cmake .. && make` and running `./path_planning`. Here is the data provided from the Simulator to the C++ Program

https://classroom.udacity.com/courses/ud1301/lessons/b2c20010-95ed-49a6-ac85-49fefe8c470b/concepts/d89141bb-3f20-4a24-93e9-4854551fd231
#### Instruction
The goal in this project is to create a path planner that is able to create smooth, safe paths for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.
The car will be transmitting its localization information, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.
Use the map, which consists of waypoints along the middle of the highway, to figure out where the lanes are, and what the curvature of the road is.
#### Point Paths
The input to the path planner is a list of x and y global map coordinates. You can use any number of points that you want, but the x list should be the same length as the y list.
Every 20 ms the car will move exactly to the next point on it's list and remove the previous one. The car's new rotation matches that of the line between its previous point and its new point.
The car moves from point to point perfectly, so you don't have to worry about building a controller for this project.
The animation above shows how the car moves and rotates through a given list of points.
#### Velocity
The velocity of the car depends on the spacing of the points; the further points are spaced apart the faster the car will be traveling. The speed goal is to have the car traveling at the 50 MPH speed limit as often as possible. But there will be times when traffic gets in the way.
#### Simple Test
For a first test with the car, try having it simply move forward in a straight line at a constant 50 MPH velocity. Use the car's (x, y) localization information and its heading direction to create a simple, straight path that is drawn directly in front of the car.
In main.cpp, instead of setting the speed directly, we pass next_x_vals, and next_y_vals to the simulator. We will set the points 0.5 m apart. Since the car moves 50 times a second, a distance of 0.5m per move will create a velocity of 25 m/s. 25 m/s is close to 50 MPH.

      double dist_inc = 0.5;
          for(int i = 0; i < 50; i++)
          {
              next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
              next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          }

Great! With just a couple of lines of code we were able to get the car to drive forward at constant velocity.

However there is a big problem because we went from 0 MPH to 56 MPH in a single 20 ms frame, causing a spike in acceleration.

Acceleration is calculated by comparing the rate of change of average speed over .2 second intervals. In this case total acceleration at one point was as high as 75 m/s^2. Jerk was also very high. The jerk is calculated as the average acceleration over 1 second intervals. In order for the passenger to have an enjoyable ride both the jerk and total acceleration should not reach values over 10.

Part of the total acceleration is the normal component, AccN which measures the centripetal acceleration from turning. The tighter and faster a turn is made, the higher the AccN value will be, but in our simple test we were not turning at all so the value of AccN is always zero. Something to think about going forward is how we can minimize total acceleration and jerk, by gradually increasing and decreasing point path spacing based on the car_speed variable.

To get a better idea of how movement affects the acceleration of the car, click the check box in the top left Manual Mode to drive the car around your self.

#### More Complex Paths
          double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();

          for(int i = 0; i < path_size; i++)
          {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          if(path_size == 0)
          {
              pos_x = car_x;
              pos_y = car_y;
              angle = deg2rad(car_yaw);
          }
          else
          {
              pos_x = previous_path_x[path_size-1];
              pos_y = previous_path_y[path_size-1];

              double pos_x2 = previous_path_x[path_size-2];
              double pos_y2 = previous_path_y[path_size-2];
              angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
          }

          double dist_inc = 0.5;
          for(int i = 0; i < 50-path_size; i++)
          {    
              next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
              next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
              pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
              pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


#### Using Previous Path Points

We still use 50 points as in our last experiment, but first we add the previous path points that we used during the last cycle. Then we add new points in reference to the previous path until we have 50 points. Using information from the previous path can help ensure that there is a smooth transition from cycle to cycle but the longer the previous path the less consideration the car is taking for dynamic changes in its environment. Ideally maybe only a small starting portion of the previous path should be used and the rest of the path is then generated based on new data from the car's sensor fusion information.
#### Timing
Here is another important consideration, the simulator will be running cycles every 20 ms (50 Frames Per Second), but the C++ program at the very least will provide new path information a 20 ms cycle behind. The simulator will simply keep progressing down its last given path while it waits for a new generated path. This means that using previous path data becomes even more important when higher latency is involved, for instance using a sufficient amount of previous path point means that even if there is 1-2 second delay between paths being generated for the simulator, the path transitions are smooth. The one concern then, as talked about above is it becomes difficult to fully predict how other traffic will act 1-2 seconds out, a car might all of a sudden slam on its brakes or change lanes in front of you during that amount of time. Newly generated paths take into account the most up to date state of other traffic.
#### Setting Point Paths with Latency
As a mentioned, the C++ program will at the very least be one cycle behind the simulator because the C++ program can't receive and send data on the same cycle. As a result the path that the simulator ends up receiving will be from the perspective of a previous cycle compared to the current state the simulator is now in. Luckily you don't have to worry about this too much, as the simulator has built in tools to deal with this timing difference. The simulator actually expects the received path to be a little out of date compared to where the car is and it will consider which point on the received path is actually closest to the car and adjust from there. This built in path setting method works well for paths that increasingly get further away from the car over time, which is the case in a highway path planning environment.

#### Highway Map
Now that we understand more about creating and setting path's for the car to follow, lets go over getting the car to stay in its lanes.
Inside data/highway_map.csv there is a list of way points that go all the way around the highway track. The track is a total of 181 way points and the last way point maps back around to the first. The track is a distance of 6945.554 meters around (about 4.32 miles). If the car is mostly going 50 MPH then it should take a little more than 5 minutes for it to go all the way around the highway.
The highway is a total of 6 lanes, where there are 3 lanes on each side. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right hand side. The car should always be inside a lane unless doing a lane change.
#### Way point Data
Each way point has an (x,y) global map position, and frenet s value and frenet d unit normal vector ( split up into the x component, and the y component. The s value is the distance along the direction of the road, the first way point has an s value of 0 because its the starting point. The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right hand side of the road. The d vector can be used to reference lane positions, for instance if you want to be in the middle lane at some way point just add the way points (x,y) with the d vector multiplied by 6 = (2+4), since each lane is 4 m.
#### More Tools
The method talked about above for finding lane reference positions only works at way points which are not close together, and the rest of the path will need to be generated by interpolating points. There are tools included like the function getXY which takes in a Frenet (s,d) coordinate and transforms it to an (x,y) coordinate.
#### Interpolating Points
At some point you may want to do some point interpolation, in previous lessons we looked at fitting to polynomials which we can do here too, but there are certainly other methods to consider as well such as Bezier curve fitting with control points and spline fitting, which guarantees that the generated function passes through every point. Here is a great and easy to setup and use spline tool for C++, contained in just a single header file.

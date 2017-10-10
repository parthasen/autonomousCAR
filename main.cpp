#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

double mylane(double car_d)
{
	double lane_no;
	if ((car_d >=0) and (car_d <4.0))
	{
		lane_no = 1;
	}
	
	else if ((car_d >=4.0) and (car_d < 8.0))
	{
		lane_no = 2;
	}
	
	else if((car_d >=8.0) and (car_d < 12.0))
	{
		lane_no = 3;
	}
	
	else
	{
		lane_no = 0; // out of bound
	}
	
	return lane_no;
	
}

// Cost Functions
double cost_keep_lane(double dist_closest_front, double num_cars_mylane, double buffer_my, double cost_collision)

{	double cost;
	
	if (num_cars_mylane  == 0) // If my lane is empty
	 {
		  cost = 0;
	 }
	 
	else 
	{
		if (dist_closest_front <= buffer_my) // Too close to car in front
		{
			cost = cost_collision;
		}
		
		else
		{
			cost = 0;
		}
	}
	
	return cost;
	
}


double cost_change_left(double dist_closest_leftfront,double dist_closest_leftback, double buffer_lc, double cost_collision, double cost_left_turn, double violate_left)

{	double cost;
	
	if (violate_left  == 1) // Left turn not allowed - collision likely
	 {
		  cost = cost_collision;
	 }
	 
	else 
	{
		if ((dist_closest_leftfront >= 2.5*buffer_lc)  and (dist_closest_leftback >= 0.6*buffer_lc))// No car is close by on either front or back
		{
			if (dist_closest_leftfront >=200)
			{
			cost = cost_left_turn*0.90; // Slight preference if left lane is wide open
			}
			
			else
			{
				cost = cost_left_turn;  // Cost is that of turning left
			}
		
		}
		
		else
		{
			cost = cost_collision;  // Collision likely
		}
	}
	
	return cost;
	
}



double cost_change_right(double dist_closest_rightfront,double dist_closest_rightback, double buffer_lc, double cost_collision, double cost_right_turn, double violate_right)

{ 	double cost;
	
	if (violate_right  == 1) // Right turn not allowed - collision likely
	 {
		 cost = cost_collision;
	 }
	 
	else 
	{
		if ((dist_closest_rightfront >= 2.5*buffer_lc)  and (dist_closest_rightback >= 0.6*buffer_lc))// No car is close by on either front or back
		{
			if (dist_closest_rightfront >=200)
			{
			cost = cost_right_turn*0.90; // Slight preference if right lane is wide open
			}
			
			else
			{
				cost = cost_right_turn;  // Cost is that of turning left
			}
		
		}
		
		else
		{
			cost = cost_collision;  // Collision likely
		}
	}
	
	return cost;
	
}

double cost_brake(double cost_slow_down)
{
	double cost;
	cost = cost_slow_down + 0.0;
	return cost;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  //start in lane 1
  int lane=1;

  //reference vlocity to target
  double ref_vel=0;//mph		

  h.onMessage([&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];//vector<vector<double>> sensor_fusion=j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
		
//simply trying to move the car forward in a straight line at a constant 50 MPH velocity. Use the car's (x, y) localization information and its heading direction to create a simple, straight path 
//https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/845a4549-6858-439e-a223-f387f15d477d

/*
 		double dist_inc = 0.5;
		for(int i = 0; i < 50; i++)
    		{
          	next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          	next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    		}
*/

//###Using Previous Path Points
//https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/66b03ee0-8e28-4a20-a1b2-690801d57d06
/*
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

*/

// Use of spline.h
		int prev_size=previous_path_x.size();
	
		vector<double> ptsx;
                vector<double> ptsy;

                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);
                //car_speed = car_speed*2.237;

//## USE of SENSOR FUSION data

//Start looking at sensor fusion data,cout<<"Sensor Fusion data"<< sensor_fusion;
		
			  
            	vector<double> cars_mylane_s;
            	vector<double> cars_myleft_s;
            	vector<double> cars_myright_s;
            
            	vector<double> cars_mylane_vel;
            	vector<double> cars_myleft_vel;
            	vector<double> cars_myright_vel;
            
              
             	double target_d;
             	double target_s_inc;
            
			 
		double dist_closest_front = 999;
		double closest_front_vel = 0;
		 
		double dist_closest_leftfront = 999;
		double dist_closest_leftback = 999;
		double closest_leftfront_vel = 0;
			 
		double dist_closest_rightfront = 999;
		double dist_closest_rightback = 999;
		double closest_rightfront_vel = 0;
		double violate_left = 0;
		double violate_right = 0;




//##Use of sensor fusion

	if(prev_size>0)
	{
	car_s=end_path_s;
	}
	
	bool too_close=false;	

//STEP 1: Analyze the senor fusion data and categorize it meaningfully
            // finding closest cars in all 3 lanes, one in front and one in back

        for(int i=0; i <sensor_fusion.size();i++)
        {

            float d = sensor_fusion[i][6];

            if(d < (2+4*lane+2) && d > (2+4*lane-2))
            {
                int id = sensor_fusion[i][0];
		double x= sensor_fusion[i][1];
		double y= sensor_fusion[i][2];
                double vx = sensor_fusion[i][3];
                double vy= sensor_fusion[i][4];
                double check_speed= sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
		double d = sensor_fusion[i][6];
		double dist = distance(car_x,car_y,x,y);
		

                check_car_s+= ((double)prev_size *.02*check_speed);


		//Categorize traffic as being in one of the 3 lanes

		//My lane
              	if (mylane(car_d) == mylane(d))
              	{
		  cars_mylane_s.push_back(check_car_s);
		  cars_mylane_vel.push_back(check_speed);
	      	}
			  
		// My left lane
		else if (mylane(d) > 0 and(mylane(d) == mylane(car_d)-1))
			  
		{
		 cars_myleft_s.push_back(check_car_s);
		 cars_myleft_vel.push_back(check_speed);
		}
			  
		//My Right lane
		else if (mylane(d) == mylane(car_d)+1)
		{
		  cars_myright_s.push_back(check_car_s);
		  cars_myright_vel.push_back(check_speed);
		}
			 
             
             cout<<endl;
             cout<<"Traffic_mylane:"<< cars_mylane_s.size()<<endl;
             cout<<"Traffic_myleft:"<< cars_myleft_s.size()<<endl;
             cout<<"Traffic_myright:"<< cars_myright_s.size()<<endl;;
             cout<<endl;

	     //Find closest car in my lane
             
             if (cars_mylane_s.size() > 0)
             {
             for (int i = 0; i< cars_mylane_s.size() ; ++i)
             {	  double mylane_s = cars_mylane_s[i];
				 
				 double dist_bw = mylane_s - car_s;
				 if ((dist_bw > 0) and (dist_bw < abs(dist_closest_front)))
				 {
					 dist_closest_front  = dist_bw;
					 closest_front_vel = cars_mylane_vel[i];
					
				 }
			 }
		   }
		   cout<<endl;
		   cout<<"dist_closest_front:"<< dist_closest_front<<endl;
		   
		   
	     //Find closest cars in left lane
		   
	     if (((car_d -4.0) > 0) and  ((car_d -4.0) < 8)) // Left lane is practical
	     {
	      if (cars_myleft_s.size() == 0) //Empty left lane
	      {
		 dist_closest_leftfront = 999;
		 dist_closest_leftback = 999;
		 closest_leftfront_vel = 0;
	      }
			 
	      else
              {
               for (int i = 0; i< cars_myleft_s.size() ; ++i)
               {
		  double myleft_s = cars_myleft_s[i];
   		  double dist_front = myleft_s - car_s;
		  double dist_back = car_s - myleft_s;
		  if ((dist_front > 0) and (dist_front < abs(dist_closest_leftfront)))
		  {
		   dist_closest_leftfront  = dist_front;
		   closest_leftfront_vel = cars_myleft_vel[i];
		  }
				 
		  if ((dist_back > 0)  and (dist_back< abs(dist_closest_leftback)))
		  {
		   dist_closest_leftback  = dist_back;
		  }
				 
	       }
	      }
	    }

	   else // Never go in left lane
	   {
		   violate_left = 1;
	   }
		   
	//Find closest cars in right lane
        if (((car_d +4.0) > 4) and  ((car_d + 4.0) < 12)) // Right lane is practical
	{
	  if (cars_myright_s.size() == 0) //Empty right lane
	  {
	 	dist_closest_rightfront = 999;
		dist_closest_rightback = 999;
		closest_rightfront_vel = 0;
	  }
			 
	  else
          {
             for (int i = 0; i< cars_myright_s.size() ; ++i)
             {	  double myright_s = cars_myright_s[i];
				 
				 double dist_front = myright_s - car_s;
				 double dist_back = car_s - myright_s;
				 if ((dist_front > 0) and (dist_front < abs(dist_closest_rightfront)))
				 {
					 dist_closest_rightfront  = dist_front;
					  closest_rightfront_vel = cars_myright_vel[i];
				 }
				 
				  if ((dist_back > 0)  and (dist_back< abs(dist_closest_rightback)))
				 {
					 dist_closest_rightback  = dist_back;
				 }
				 
	     }
	  }
	}
	else// Never go in right lane
	{
		   violate_right = 1;
	}
	  
		    
	cout<<"Dist closest left_front:"<< dist_closest_leftfront<<endl;
	cout<<"Dist closest left_back:"<< dist_closest_leftback<<endl;
	cout<<"Dist closest right_front:"<< dist_closest_rightfront<<endl;
	cout<<"Dist closest right_back:"<< dist_closest_rightback<<endl;
		      
	//Checks is lane changes into that lane are allowed
	//cout<<"Violate_left: "<<violate_left << endl;  
	//cout<<"Violate Right :"<< violate_right<<endl;


//STEP 2: Calculate cost of actions and choose the one with minimum cost
		    
	double buffer_my = 50;
	double buffer_lc = 20;
	double cost_collision = 500;
	double cost_left_turn = 150;
	double cost_right_turn = 150;
	double cost_slow_down = 200;
		    
        double continue_lane_tc;
        double brake_tc;
        double left_turn_tc;
        double right_turn_tc;
            
        int decision = 999;
        double min_cost = 800;
        vector<double> costs;
             
        continue_lane_tc = cost_keep_lane(dist_closest_front, cars_mylane_s.size() , buffer_my, cost_collision);
        costs.push_back(continue_lane_tc);
            
        brake_tc = cost_brake(cost_slow_down);
        costs.push_back(brake_tc);
            
        left_turn_tc =  cost_change_left(dist_closest_leftfront,dist_closest_leftback, buffer_lc, cost_collision, cost_left_turn, violate_left);
        costs.push_back(left_turn_tc);
             
             
        right_turn_tc = cost_change_right(dist_closest_rightfront, dist_closest_rightback, buffer_lc, cost_collision, cost_right_turn, violate_right);
        costs.push_back(right_turn_tc);
             
	cout<<"Cost continue lane" <<continue_lane_tc<<endl;
	cout<<"Cost slow down" <<brake_tc<<endl;
        cout<<"Cost left turn" <<left_turn_tc<<endl;
        cout<<"Cost Right turn" <<right_turn_tc<<endl;
             
             
        // Find the action with minimum cost
             
        for (int i = 0; i< costs.size() ; ++i)
             
        {
		double cost1 = costs[i];
				
		if (cost1 < min_cost)
		 {
			 min_cost = cost1;
			 decision = i;
		 }
			 
	}
			 
	if (decision == 0)
	{
		 cout<< "Decision: Stay in lane with max vel" <<endl;
	}
			 
	else if (decision == 1)
	{
		 cout<< "Decision: Stay in lane but slow down" <<endl;
	}
			 
	else if (decision == 2)
	{
		 cout<< "Decision: Change to left lane" <<endl;
        }
	else
	{
	 	cout<< "Decision: Change to right lane" <<endl;
				  
	}
			 
           
        // decision = 0 ; stay in current lane at max speed
        //decision = 1; stay in current lane but slow down
        // decision = 2; change to left lane
        // decision = 3 ; change to right lane
             
        cout<<"Min cost: "<< min_cost<<endl;
	cout <<"Decision is:" <<decision<<endl;
	cout<<"Velocity of closest front: "<<closest_front_vel<<endl;
	




        if((check_car_s>car_s) && ((check_car_s-car_s)<30))
        {
        	//ref_vel=29.5;
		too_close=true;

		//Lane change	
 		float d = sensor_fusion[i][6];
			
		if (d >= 0 and d < 4)
    		{
		   lane = 0;
    		}
    		else if (d>=4 and d < 8)
    		{
        	   lane  = 1;
    		}
    		else 
		{
	       	   lane = 2;
 		}


		    /*if(lane>0)
		    {
			lane=lane-1;
		    }
		    else if(lane==0)
		    {
			lane=1;
		    }
		    if(lane==1 and cars_myright_s.size()==0)
		    {
			lane=lane+1;
		    }
		    if(lane==1 and cars_myleft_s.size()==0)
		    {
			lane=lane-1;
		    }
		    if(lane==0 and cars_myright_s.size()==0)
		    {
			lane=lane+1;
		    }
	
		    if(lane==2 and cars_myleft_s.size()==0)
		    {
			lane=lane-1;
		    }*/
		    if(lane==1 and decision==3)
		    {
			lane=lane+1;
		    }
		    if(lane==1 and decision==2)
		    {
			lane=lane-1;
		    }
		    if(lane==0 and decision==3)
		    {
			lane=lane+1;
		    }
	
		    if(lane==2 and decision==1)
		    {
			lane=lane-1;
		    }
					
 	
        }
        }
   }

//velocity control
	if(too_close)
	{
	ref_vel-=0.4;
	}
	else if(ref_vel<49.5)
	{
	ref_vel+=0.4;
	}


                    if(prev_size < 2)
                    {
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);

                    }
                    else {
                        ref_x = previous_path_x[prev_size-1];
                        ref_y = previous_path_y[prev_size-1];

                        double  ref_x_prev = previous_path_x[prev_size-2];
                        double ref_y_prev = previous_path_y[prev_size-2];
                        ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);

                    }
                    vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for(int i=0; i<ptsx.size(); i++){
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
                        ptsy[i]= (shift_x*sin(0-ref_yaw)+ shift_y*cos(0-ref_yaw));
                    }
                tk::spline s;
		s.set_points(ptsx,ptsy);
     
		for(int i = 0; i < previous_path_x.size(); i++)
                {
                   next_x_vals.push_back(previous_path_x[i]);
                   next_y_vals.push_back(previous_path_y[i]);
                }
                
		// breaking the spline in 30 points   
		double target_x = 30.0;
                double target_y = s(target_x);
                double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

                double  x_add_on = 0;
                double cur_velocity;

                for(int i = 1; i <= 50-previous_path_x.size(); i++)
                {
                  /*if(cur_velocity > ref_vel)
                    {
                       cur_velocity+=0.224;
                    }*/

                  double N = (target_dist/(.02*ref_vel/2.24));
                  double x_point = x_add_on+(target_x)/N;
                  double y_point = s(x_point);

                  x_add_on = x_point;

                  double x_ref = x_point;
                  double y_ref = y_point;

                  x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
                  y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

                  x_point += ref_x;
                  y_point += ref_y;

		  next_x_vals.push_back(x_point);
                  next_y_vals.push_back(y_point);
               }




          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;      // Lane #
    this->s = s;            // s, distance along the road (Frenet's coords)
    this->v = v;            // v, velocity
    this->a = a;            // a, acceleration
    state = "CS";           // "Current Speed", possibly??
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}

// --- Implement Finite State Machine -- 
// Get Next Valid States
// Returns a list of valid states, for any state
vector<string> Vehicle::next_valid_states() {

  vector<string> next_states;  // our return value

  if (state.compare("KL") == 0) {   // from Keep Lane, can do Perform Lane Change Right/Left only
    next_states.push_back("KL");
    next_states.push_back("PLCL");
    next_states.push_back("PLCR");
  } 
  else if (state.compare("PLCL") == 0) { // from Perform Lane Change Left --> KL, LCL, PLCL
    next_states.push_back("KL");
    // next_states.push_back("PLCL");
    next_states.push_back("LCL");
  }
  else if (state.compare("PLCR") == 0) { // from Perform Lane Change Right --> KL, LCR, PLCR
    next_states.push_back("KL");
    // next_states.push_back("PLCR");
    next_states.push_back("LCR");
  }
  else if (state.compare("LCR") == 0) { // from Lane Change Right --> Keep Lane
    next_states.push_back("KL");
  }
  else if (state.compare("LCL") == 0) { // from Lane Change Left --> Keep Lane
    next_states.push_back("KL");
  }

  // remove edge cases
  if (this->lane == 0) {                // cannot move Right from right-most lane
    next_states.erase(std::remove(next_states.begin(), 
                                  next_states.end(),
                                  "PLCR"),
                      next_states.end());
  }
  if (this->lane == 3) {              // cannot move left from left-most lane
    next_states.erase(std::remove(next_states.begin(), 
                                  next_states.end(),
                                  "PLCL"),
                      next_states.end());
  }

  return next_states;
}

// NOTE: TODO: - Implement this method.
void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
  
    // Define Cost Functions - as Lambdas
    // 1 - Keep Lane Cost: cost = 0 if we are in the goal lane; else difference b/w current lane & goal lane
    auto keep_lane_cost = [this]() {
      auto c = 1.0 * (lane - goal_lane);
      return c * c; // bcos we want a positive cost
    };

    // 2 -- Perform Lane Change Cost
    auto perform_lane_change_cost = [this](int delta_lane) {
      // delta_lane is either +1 or -1, depending on L or R 
      double c = (lane + 0.5 * delta_lane - goal_lane);  // FIXME: try changing the factor
      cout << " c: " << c * c << ", lane: " << lane << ", delta_lane: " << delta_lane << endl;
      return c * c; 
    };

    // 3 -- Change Lane Cost 
    // Take predictions / collisions into account
    auto change_lane_cost = [this](int delta_lane, const map<int, vector<vector<int> > >& predictions) {

      bool collision = false;
      // see if there are any collisions, if so, add a HUGE cost
      for (auto it = predictions.begin(); it != predictions.end(); it++) {
          int now = 0;
          int vid = it->first;  // vehicle id
          vector<vector<int> > v = it->second;

          if (vid != -1 ) { // ego_key, which is our car
              collision = (abs(lane - v[now][0]) == 1) && (abs(s - v[now][1]) <= L);
              if (collision) {
                  return 100000.0;  // HUGE PENALTY for collision
              }
          }
      }

      auto cst = 1.0 * (lane + delta_lane - goal_lane);
      return cst * cst;
    };

    // 4 - Cost of Speed
    auto speed_cost = [this]() {
       auto cst =  0.5 * (target_speed - v); 
       return cst * cst;
    };

    // get next valid states
    vector<string> valid_states = next_valid_states();

    // store the potential costs
    map<string, double> costs;

    // calc the costs for each state change
    costs["KL"]   = keep_lane_cost() + speed_cost();
    costs["PLCL"] = perform_lane_change_cost(1); //  + speed_cost();  // delta is +1
    costs["PLCR"] = perform_lane_change_cost(-1); //  + speed_cost(); // delta is -1
    costs["LCL"]  = change_lane_cost(1, predictions); // + speed_cost(); 
    costs["LCR"]  = change_lane_cost(-1, predictions); //   + speed_cost();

    // TODO: Add Cost for going SLOW / FAST !!

    // find the state with min cost
    double max_cost = 1E12;

    // got thru each valid state and find the one with min cost
    for (auto test_state: valid_states) {
        double test_cost = costs[test_state];

        cout << " checking next state: " << test_state << ", cost: " << test_cost << endl;
        if (test_cost < max_cost) {
            max_cost = test_cost;
            state    = test_state;

            cout << " --> NEXT state: " << state << " <-- " << endl;
        }
    }

    // DONE --   
} // end update_state

// --- Configure the Ego vehicle with Goal info ---
void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed      = road_data[0];   // SPEED LIMIT
    lanes_available   = road_data[1];   // NUM OF LANES 
    goal_s            = road_data[2];   // GOAL_s,      NOTE: Fixed this (was incorrect)
    goal_lane         = road_data[3];   // GOAL_Lane
    max_acceleration  = road_data[4];   // a, NOTE: fixed this (was incorrectly assigned to goal_s)
}

string Vehicle::display() {

	ostringstream oss;
	
	  oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

// --- Run time step for given state ---
void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
  this->v += this->a * dt;
}

// --- Predict Vehicle State at given time t ---
vector<int> Vehicle::state_at(int t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

// --- Check if Collision Occurs with another Vehicle ---
bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

// --- Will Collide With Other Vehicle ---
Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t; 
        	return collider_temp;
    	}
	}

	return collider_temp;
}

// --- Execute: Change to a given state, given its predictions ---
void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

// --- Internal --- Max Acceleration for Lane
int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }
    
    return max_acc;

}

// ---- Execute -- Keep Lane ----
void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

// ----- Execute Lane Change -----
void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

// ------- Execute - Prepare for Lane Change ------
void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<int> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

// --- Generate Predictions for vehicle, over horizon number of time steps ---
vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      vector<int> check1 = state_at(i);
      vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
  	}
    return predictions;

}
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include "hybrid_breadth_first.h"

using namespace std;


/**
 * Initializes HBF
 */
HBF::HBF() {

}

HBF::~HBF() {}


int HBF::theta_to_stack_number(double theta){
  /*
  Takes an angle (in radians) and returns which "stack" in the 3D configuration space
  this angle corresponds to. Angles near 0 go in the lower stacks while angles near 
  2 * pi go in the higher stacks.
  */

  double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI))) % NUM_THETA_CELLS;
  return stack_number;
}


int HBF::idx(double float_num) {
  /*
  Returns the index into the grid for continuous position. So if x is 3.621, then this
  would return 3 to indicate that 3.621 corresponds to array index 3.
  */

  return int(floor(float_num));
}


// --- Expand ---
/**
    Expand the given node, return list of possible next nodes
*/
vector<HBF::maze_s> HBF::expand(HBF::maze_s state, 
                                vector<int> goal, 
                                vector<vector<int> > grid) 
{
  int g     = state.g;
  double x  = state.x;
  double y  = state.y;
  double theta = state.theta;

  int g2 = g+1;  // incr iteration
  vector<HBF::maze_s> next_states;

  // for every steering angle from -35 to +35 (for each 5-degree increment)...
  // ...generate the x,y,theta transform equations
  for(double delta_i = -35; delta_i < 40; delta_i+=5)   // Steering angle [-35, +35]
  {

    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;
    if(theta2 > 0)
    {
    	theta2 += 2*M_PI;
    }
    double x2 = x + SPEED * cos(theta2);
    double y2 = y + SPEED * sin(theta2);

    // --- NEW -- Adding a Cost Heuristic
    double deltax = goal[0] - x2;   // distance from goal, x
    double deltay = goal[1] - y2;   // distance from goal, y
    double W = 1.0;                 // weight factor

    int ix = idx(x2);
    int iy = idx(y2);

    double cost_obstacle = 1E3;
    // if within the grid, and no obstacles, zero out the cost of obstacles
    if ((iy >=0 && iy < (int)grid.size()) && 
        (ix >=0 && ix < (int)grid.size()) ) {
      if(grid[iy][ix] == 0) {
        cost_obstacle = 0.0;
      }
    }

    // heuristic cost
    double h2 = W * (abs(deltax) + abs(deltay)); //  + cost_obstacle; 
    // double h2 = pow( deltax * deltax + deltay * deltay, 0.5);

    HBF::maze_s state2;
    state2.g = g2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    state2.f = g2 + h2;         // new total heuristic cost
    next_states.push_back(state2);

  }

  // Now sort by the lowest f cost
  sort(next_states.begin(),
      next_states.end(), [](const maze_s& left, const maze_s& right) {
          return left.f < right.f;
      } 
  );

  return next_states;
}

// --- Reconstruct Path ---
vector< HBF::maze_s> HBF::reconstruct_path(vector< vector< vector<HBF::maze_s> > > came_from, vector<double> start, HBF::maze_s final){

	vector<maze_s> path = {final};
	
	int stack = theta_to_stack_number(final.theta);

	maze_s current = came_from[stack][idx(final.x)][idx(final.y)];
	
	stack = theta_to_stack_number(current.theta);
	
	double x = current.x;
	double y = current.y;
	while( x != start[0] && y != start[1] )
	{
		path.push_back(current);
		current = came_from[stack][idx(x)][idx(y)];
		x = current.x;
		y = current.y;
		stack = theta_to_stack_number(current.theta);
	}
	
	return path;
}

// --- SEARCH ---
HBF::maze_path HBF::search(vector< vector<int> > grid, vector<double> start, vector<int> goal) {
  /*
  Working Implementation of breadth first search. Does NOT use a heuristic
  and as a result this is pretty inefficient. Try modifying this algorithm 
  into hybrid A* by adding heuristics appropriately.

    :param grid is the Grid (vector<vector<int>>): 16x16
    :param start is vector(3, double), with (x, y, theta)
    :param goal is vector(2, int), with (x, y)

  */

  // closed is our set of Explored cells
  vector< vector< vector<maze_s> > > closed(NUM_THETA_CELLS, vector<vector<maze_s>>(grid[0].size(),   vector<maze_s>(grid.size())) );
  vector< vector< vector<int> > >    closed_value(NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())) );
  vector< vector< vector<maze_s> > > came_from(NUM_THETA_CELLS, vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())) );
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  int g = 0;

  maze_s state;                     // initial state
  state.g = g;
  state.x = start[0];
  state.y = start[1];
  state.f = 0;        // total cost for heuristic

  closed[stack][idx(state.x)][idx(state.y)]       = state;
  closed_value[stack][idx(state.x)][idx(state.y)] = 1;
  came_from[stack][idx(state.x)][idx(state.y)]    = state;

  int total_closed = 1;

  vector<maze_s> opened = {state};  // our Frontier queue
  bool finished = false;

  while(!opened.empty())
  {
    // Pop 1st element in Frontier
    maze_s next = opened[0];      //grab first elment
    opened.erase(opened.begin()); //pop first element

    int x = next.x;
    int y = next.y;

    // GOAL TEST
    if(idx(x) == goal[0] && idx(y) == goal[1])
    {
      cout << "found path to goal in " << total_closed << " expansions" << endl;
      maze_path         path;
      path.closed     = closed;
      path.came_from  = came_from;
      path.final      = next;
      return path;

    }
    // Else, expand next node
    vector<maze_s> next_state = expand(next, goal, grid);

    // Search each child node in the expand list
    for(int i = 0; i < next_state.size(); i++)
    {
      int g2    = next_state[i].g;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size()))
      {
        //invalid cell
        continue;
      }
      int stack2 = theta_to_stack_number(theta2);

      if(closed_value[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0)
      {

        maze_s state2;
        state2.g = g2;
        state2.x = x2;
        state2.y = y2;
        state2.theta = theta2;

      // add this state2 to our Frontier list
        opened.push_back(state2);   

        // add this node/state to our Explored list
        closed[stack2][idx(x2)][idx(y2)]        = next_state[i];
        closed_value[stack2][idx(x2)][idx(y2)]  = 1;
        came_from[stack2][idx(x2)][idx(y2)]     = next;
        total_closed += 1;
      }
    }
  }

  cout << "no valid path." << endl;
  HBF::maze_path path;
  path.closed     = closed;
  path.came_from  = came_from;
  path.final      = state;
  return path;

} // end search()

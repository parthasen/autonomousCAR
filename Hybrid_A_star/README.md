# Implementing Hybrid A*
In this exercise, you will be provided a working implementation of a *breadth first* search algorithm which _does not_ use any heuristics to improve its efficiency. 

Your goal is to try to make the appropriate modifications to the algorithm so that it takes advantage of **heuristic functions** (possibly the ones mentioned in the previous paper) to reduce the number of grid cell expansions required.

### Instructions:
Modify the code in 'hybrid_breadth_first.cpp' and hit Test Run to check your results.
Note the number of expansions required to solve an empty 15x15 grid (it should be about 18,000!). Modify the code to try to reduce that number. How small can you get it?

## Results

#### Using Heuristics

Using a simple heuristic function **absolute distance from goal**, we can reduce teh number of expansions down to 9069.

```
Finding path through grid:
  GOAL is: (15, 15)
0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0
0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0
0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0
0,1,1,0,0,0,0,1,1,0,0,0,1,1,1,0
0,1,1,0,0,0,1,1,0,0,0,1,1,1,0,0
0,1,1,0,0,1,1,0,0,0,1,1,1,0,0,0
0,1,1,0,1,1,0,0,0,1,1,1,0,0,0,0
0,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0
0,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0
0,1,1,0,0,0,1,1,1,0,0,1,1,1,1,1
0,1,0,0,0,1,1,1,0,0,1,1,1,1,1,1
0,0,0,0,1,1,1,0,0,1,1,1,1,1,1,1
0,0,0,1,1,1,0,0,1,1,1,1,1,1,1,1
0,0,1,1,1,0,0,1,1,1,1,1,1,1,1,1
0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0
1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0
found path to goal in 9069 expansions
```
Full results are in ![output](output.txt)

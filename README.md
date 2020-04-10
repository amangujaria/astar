# astar
a*(a-star) usage for efficient movement on a grid layout

Usage
-----

- Generate map using a custom grid (an array of rows and columns :: list(tuple())). eg. `astar:create_map([{0,0,0,0,0},{0,0,1,1,0},{0,0,0,1,0},{0,0,0,1,0}])`. 
- Movement is allowed on the tile with value 0 and blocked for value 1.
- Search for a path between two points in the map if one is available. eg. `astar:path({3,0}, {3,4}, Map)`

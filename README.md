# Galton-board

Simulation implemented in C++ and OpenGL to simulate the physics of a Galton-board.  
It was implemented a QuadTree algorithm to enhance the performance of the collisions.  

Geometry collisions implemented:

- Sphere: balls
- AABB: walls and bins
- OBB: funnel and pegs

## Controls

- ESC: quit
- W: move camera up
- S: move camera down
- D: move camera right
- A: move camera left
- UP: move camera forward
- DOWN: move camera backward
- 1: add ball
- R: reset the board
- P: pause
- U: increase time scale
- J: decrease time scale
- I: increase friction
- K: decrease friction
- T: increase ball size
- B: decrease ball size
- O: increase restitution (bouncing)
- L: decrease restitution (bouncing)
- F1: show the quad tree areas and collision boxes

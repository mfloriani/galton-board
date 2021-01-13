# galton-board

Simulation implemented in C++ and OpenGL to simulate physics of a galton-board.

Geometry collisions implemented:

- Sphere: balls
- AABB: walls and bins
- OBB: funnel and pegs

I implemented a Quad Tree algorithm to enhance the performance of the collisions.

Access `Constant.h` to alter parameters that define how the simulation works, e.g. `BALL_TOTAL` to change the initial amount of balls.

## Keys

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


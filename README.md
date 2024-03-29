The goal of this assignment will be to implement a working solver for the puzzle game Sokoban. Sokoban is a puzzle game in which a warehouse robot must push boxes into storage spaces. The rules hold  that onlyone box can be moved at a time, that boxes can only be pushed by robots and not pulled, and that neither robots nor boxes can pass through obstacles. In addition, robots cannot push more than one box, i.e., if there are two boxes in a row, they cannot push them. The game is over when all the boxes are in their storage spots.

In our version of Sookban the rules are slightly more complicated, as there may be more than one warehouse robot available to push boxes. These  robots cannot pass through one another nor can they move simultaneously, however.

Sokoban has the following formal description. Note that our version differs from the standard one. Read the description carefully:
• The puzzle is played on a board that is a grid board with N squares in the x-dimension and M squares in the y-dimension.
• Each state contains the x and y coordinates for each robot, the boxes, the storage spots, and the obstacles.
• From each state, each robot can move North, South, East, or West. No two robots can move simultaneously, however. If a robot moves to the location of  a box, the box will move one square in the same direction. Boxes and robots cannot pass through walls or obstacles, however. If a robot moves to the location of a box, the box will move one square in the same direction. Boxes and robots cannot pass through walls or obstacles, however. Robots cannot push more than one box at a time; if two boxes are in succession the robot will not be able to move them. Movements that cause a box to move more than one unit of the grid are also illegal. Whether or not a robot is pushing an object does not change the cost.
• Each movement is of equal cost. Whether or not the robot is pushing an object does not change the cost.
• The goal is achieved when each box is located in a storage area on the grid.

Ideally, we will want our robots to organize everything before the supervisor arrives. This means that with each problem instance, you will be given a computation time constraint. You must attempt to provide some legal solution to the problem within this constraint. Better plans will be plans that are shorter, i.e. that require fewer operators to complete.


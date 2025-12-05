/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*
* Project: Arduino Maze Solver                                  *
* File: NavigationController.c                                  *
* Version: 1.2                                                  *
* Last Modified: Dec. 3rd, 2025                                 *
*                                                               *
* Author: Chloe Beal                                            *
*                                                               *
* School: University of Washington - Bothell                    *
* Course: CSS427 - Introduction to Embedded Systems             *
* Section: A                                                    *
* Instructor: Dr. Tyler Folsom                                  *
*                                                               *
* Description: Using 3 distance measurements (left, front, and  *
*   right), decide if the maze is complete, or if a direction   *
*   should be moved in.                                         *
*                                                               *
* Change Log:                                                   *
*   1.0: Inital release.                                        *
*   1.1: Updated maze complete checking to allow for when a     *
*       sensor fails to find a wall, it counts that as being    *
*       outside the maze.                                       *
*   1.2: Adjusted wall constsnts to account for not being       *
*       parrallel to a wall.                                    *
*                                                               *
*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/

/*===============================================================
||                      Private Constants                      ||
===============================================================*/

#define LEFT_WALL 0x04  // Value to represent a left wall.
#define FRONT_WALL 0x02 // Value to represent a front wall.
#define RIGHT_WALL 0x01 // Value to represent a right wall.

/*===============================================================
||                       Public Constants                      ||
===============================================================*/

#define MAX_VAL_FRONT_WALL 30   // Max value that could still be 
                                // a front wall
#define MAX_VAL_SIDE_WALL 30    // Max value that could still be 
                                // a side wall
#define WALL_VAL_ERROR 0.5      // Possible error measured value
#define MAX_VAL_MAZE_WALL 140   // Max value that could still be
                                // a wall in the maze.

/*===============================================================
||                      Public Data Types                      ||
===============================================================*/

// The 4 possible movement states and the maze complete state.
enum Direction {FORWARD, RIGHT, BACK, LEFT, COMPLETE};

/*===============================================================
||                      Private Functions                      ||
===============================================================*/

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*
*                            GetWalls                           *
*------------------------- Description -------------------------*
* Given the measured distance to the left, front, and right,    *
* decide of there are walls or not in each direction.           *
*                                                               *
-------------------------- Parameters --------------------------*
* const float leftDist: The distance measured to the left side  *
*                                                               *
* const float frontDist: The distance measured to the front side*
*                                                               *
* const float rightDist: The distance measured to the right side*
*                                                               *
------------------------- Return Value -------------------------*
* Returns a char representing the walls in each direction. Uses *
* the last 3 bits of a char as follows:                         *
*   - Left:     0x04                                            *
*   - Front:    0x02                                            *
*   - Right:    0x01 (LSB)                                      *
*                                                               *
*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
char GetWalls(const double leftDist, const double frontDist, const double rightDist)
{
    char walls = 0x00;
    //check for left gap
    if(leftDist < (MAX_VAL_SIDE_WALL - WALL_VAL_ERROR))
    {
        walls = walls | LEFT_WALL;
    }
    //check for front gap
    if(frontDist < (MAX_VAL_FRONT_WALL - WALL_VAL_ERROR))
    {
        walls = walls | FRONT_WALL;
    }
    //check for right gap
    if (rightDist < (MAX_VAL_SIDE_WALL - WALL_VAL_ERROR))
    {
        walls = walls | RIGHT_WALL;
    }
    return walls;
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*
*                           IsComplete                          *
*------------------------- Description -------------------------*
* Given the distance measured in each direction, decide if the  *
* maze is complete.                                             *
*                                                               *
-------------------------- Parameters --------------------------*
* const float leftDist: The distance measured to the left side  *
*                                                               *
* const float frontDist: The distance measured to the front side*
*                                                               *
* const float rightDist: The distance measured to the right side*
*                                                               *
------------------------- Return Value -------------------------*
* Returns a char representing if the maze is complete.          *
*   - Complete:     0x01                                        *
*   - Not Complete: 0x00                                        *
*                                                               *
*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
char IsComplete(const double leftDist, const double frontDist, const double rightDist)
{
    char walls = 0x00;
    //check for left wall
    if ((leftDist < (MAX_VAL_MAZE_WALL - WALL_VAL_ERROR)) && (leftDist > WALL_VAL_ERROR))
    {
        walls = walls | LEFT_WALL;
    }
    //check for front wall
    if ((frontDist < (MAX_VAL_MAZE_WALL - WALL_VAL_ERROR)) && (frontDist > WALL_VAL_ERROR))
    {
        walls = walls | FRONT_WALL;
    }
    //check for right wall
    if ((rightDist < (MAX_VAL_MAZE_WALL - WALL_VAL_ERROR)) && (rightDist > WALL_VAL_ERROR))
    {
        walls = walls | RIGHT_WALL;
    }

    // no walls detected, maze complete
    if ((walls & 0xFF) == 0x00)
    {
        return 0x01;
    }
    // walls detected, maze not complete
    else
    {
        return 0x00;
    }
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*
*                        ChooseDirection                        *
*------------------------- Description -------------------------*
* Given a set of walls and if the maze is complete, decide if   * 
* to move or not.                                               *
*                                                               *
-------------------------- Parameters --------------------------*
* const char walls: A char representing if there is a wall in   *
*       each direction. Uses the last 3 bits of a char as       *
*       follows:                                                *
*           - Left:     0x04                                    *
*           - Front:    0x02                                    *
*           - Right:    0x01 (LSB)                              *
*                                                               *
* const char isComplete: A char representing if the maze is     *
*       complete.                                               *
*           - Complete:     0x01                                *
*           - Not Complete: 0x00                                *
*                                                               *
------------------------- Return Value -------------------------*
* Returns a direction to move or that the maze is complete.     *
*                                                               *
*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
enum Direction ChooseDirection(const char walls, const char isComplete)
{
    // check for commplete first
    if (isComplete == 0x01)
    {
        return COMPLETE;
    }
    else
    {
        //clockwise priority starting with left
        if((walls & LEFT_WALL) != LEFT_WALL)
        {
            return LEFT;
        }
        else if ((walls & FRONT_WALL) != FRONT_WALL)
        {
            return FORWARD;
        }
        else if ((walls & RIGHT_WALL) != RIGHT_WALL)
        {
            return RIGHT;
        }
        else
        {
            return BACK;
        }
    }
}

/*===============================================================
||                       Public Functions                      ||
===============================================================*/

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*
*                           GetNextMov                          *
*------------------------- Description -------------------------*
* Given the distance measured in each direction, decide if to   *
* move or not.                                                  *
*                                                               *
-------------------------- Parameters --------------------------*
* const float leftDist: The distance measured to the left side  *
*                                                               *
* const float frontDist: The distance measured to the front side*
*                                                               *
* const float rightDist: The distance measured to the right side*
*                                                               *
------------------------- Return Value -------------------------*
* Returns a direction to move or that the maze is complete.     *
*                                                               *
*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
enum Direction GetNextMov(const double leftDist, const double frontDist, const double rightDist)
{
    // get walls from sensor distances
    char walls = GetWalls(leftDist, frontDist, rightDist);

    //check for complete maze
    char isComplete = IsComplete(leftDist, frontDist, rightDist);

    //choose a movement direction
    return ChooseDirection(walls, isComplete);
}


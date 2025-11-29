#ifndef NAVIGATIONCONTROLLER_H
#define NAVIGATIONCONTROLLER_H

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*
* Project: Arduino Maze Solver                                  *
* File: NavigationController.h                                  *
* Version: 1.0                                                  *
* Last Modified: Nov. 20th, 2025                                *
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
*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/

/*===============================================================
||                       Public Constants                      ||
===============================================================*/

#define MAX_VAL_FRONT_WALL 0.5  // Max value that could still be 
                                // a front wall
#define MAX_VAL_SIDE_WALL 1     // Max value that could still be 
                                // a side wall
#define WALL_VAL_ERROR 0.1      // Possible error measured value

#define MAX_VAL_MAZE_WALL 5     // Max value that could still be
                                // a wall in the maze.

/*===============================================================
||                      Public Data Types                      ||
===============================================================*/

// The 4 possible movement states and the maze complete state.
enum Direction {FORWARD, RIGHT, BACK, LEFT, COMPLETE};

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
* const int leftDist: The distance measured to the left side.   *
*                                                               *
* const int frontDist: The distance measured to the front side. *
*                                                               *
* const int rightDist: The distance measured to the right side. *
*                                                               *
------------------------- Return Value -------------------------*
* Returns a direction to move or that the maze is complete.     *
*                                                               *
*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
enum Direction GetNextMov(const float leftDist, const float frontDist, const float rightDist);

#endif 

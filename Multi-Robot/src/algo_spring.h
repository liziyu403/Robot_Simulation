/* "global" variables for the spring algo
 */

#ifndef ALGO_SPRING_H
#define ALGO_SPRING_H

#include "utilities/arenas.h"
#include "utilities/interactions_project.h"
#include "utilities/obstacles.h"
/* Arenas structure */
arenas_t Arenas;

/* Obstacles structure */
obstacles_t obstacles;

/* Parameters of the basic SPP terms */
double V_Flock;
double V_Max;

/* Parameters of repulsion & attraction */
double R_0;
double Slope_Rep;
double Slope_Att;

/* Parameter of the alignement */
double Slope_Align;
double Alpha;

/* Parameters of the wall and its shill interaction */
double V_Shill;
double Acc_Shill;
double Slope_Shill;
double R_0_Shill;


/* Hyper parameters of the flocking */
double Size_Neighbourhood;

/* 2 or 3 dimensions? */
double Dim;

/* Parameters of the arena */
double ArenaRadius;
double ArenaShape;
double ArenaCenterX;
double ArenaCenterY;

#endif

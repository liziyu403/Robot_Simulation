//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* "global" variables for the evolution-compatible SPP algo
 */

#ifndef ALGO_SPP_EVOL_H
#define ALGO_SPP_EVOL_H

#include "utilities/arenas.h"
#include "utilities/interactions.h"
#include "utilities/obstacles.h"
/* Arenas structure */
arenas_t Arenas;

/* Obstacles structure */
obstacles_t obstacles;

/* Parameters of the basic SPP terms */
double V_Flock;
double V_Max;

/* Parameters of repulsion & attraction */
double V_Rep;
double R_0;
double Slope_Rep;
double Slope_Att;
double K_Press;

/* Parameters of friction */
double C_Frict;
double V_Frict;
double R_0_Offset_Frict;
double Slope_Frict;
double Acc_Frict;

/* Parameters of the wall and its shill interaction */
double V_Shill;
double Acc_Shill;
double Slope_Shill;
double R_0_Shill;

/* Parameters of the gradient term from Olfati */
double H_Bump;
double Epsilon;
double A_Action_Function;
double B_Action_Function;

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

//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * This file contains unversal interaction terms.
 */

#ifndef INTERACTIONS_PROJECT_H
#define INTERACTIONS_PROJECT_H

#include "math_utils.h"
#include "dynamics_utils.h"
#include "arenas.h"

/* Repulsion */

void RepulsionSpring(double *OutputVelocity,
        phase_t * Phase, const double p_l, const double R_0_l, 
        const int WhichAgent, const int Dim_l,
        const bool normalize);
        
/* Attraction */

void AttractionSpring(double *OutputVelocity,
        phase_t * Phase, const double p_l, const double R_0_l, 
        const int WhichAgent, const int Dim_l,
        const bool normalize);

/* Alignment */

void Alignment(double *OutputVelocity,
        phase_t * Phase, const double h, 
        const double alpha,
        const int WhichAgent, 
        const int Dim_l);


/* Target tracking function */
void TargetTracking(double *OutputVelocity, double *TargetPosition,
        phase_t * Phase, const double R_CoM, const double d_CoM,
        const double R_trg, const double d_trg, 
        const int SizeOfNeighbourhood, const int WhichAgent, 
        const int Dim_l);
        
#endif

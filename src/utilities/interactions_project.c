//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * This file contains unversal interaction terms.
 */

#include <limits.h>
#include "interactions_project.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/* 


**************  EXERCICE 1  ************* 
        
GOAL: Fill in the next function 'RepulsionSpring' to create a repulsive force.

You'll need different functions such as:
- UnitVect()
- MultiplicateWithScalar()
- VectSum()

All these functions are defined in the utilities/math_utils.c file. 

INPUTS: 
- Phase: This object gathers all the information at a given time about all the agents (velocities, positions, ...).
- R_0_l: It is the equilibrium distance under which we want a repulsive action between the agents (x0 in the course)
- p_l: It is the linear coefficient used for the formula (k in the course)
- Dim_l: 2D or 3D simulation, only useful for the MultiplicateWithScalar() function

OUTPUT:
-OutputVelocity: The output vector of the repulsion (u_rep in the course)

Hints:
-The distance between the agents as well as the difference vector between them as already been computed for you and are 
respectively 'DistanceFromNeighbour' and 'DifferenceVector'. Be careful of the orientation of the 'DifferenceVector'.

-Note that most of the functions are of the 'void' type. The output vector is usually set as an input pointer of a function.

For example, to get the difference between the 3D vector v1[3] and v2[3] and ouput it in u[3], use the following command:
            
            VectDifference(u, v1, v2) ---> u = v1 - v2

-You should sum the repulsion over all the agents, so at each iteration, don't forger to sum the calculated vector to the 'OutputVelocity'

*/

void RepulsionSpring(double *OutputVelocity,
        phase_t * Phase, const double p_l,
        const double R_0_l, const int WhichAgent, const int Dim_l,
        const bool normalize) {

    NullVect(OutputVelocity, 3);

    int i;
    int n = 0;

    double *AgentsCoordinates;
    double *NeighboursCoordinates;

    AgentsCoordinates = Phase->Coordinates[WhichAgent];

    static double DifferenceVector[3];
    static double DistanceFromNeighbour;

    /* Repulsive interaction term */
    for (i = 0; i < Phase->NumberOfAgents; i++) {
        if (i == WhichAgent)
            continue;
        NeighboursCoordinates = Phase->Coordinates[i];
        VectDifference(DifferenceVector, AgentsCoordinates,
                NeighboursCoordinates);
        if (2 == Dim_l) {
            DifferenceVector[2] = 0.0;
        }
        DistanceFromNeighbour = VectAbs(DifferenceVector);
        /* Check if we interact at all */
        if (DistanceFromNeighbour >= R_0_l)
            continue;
        n += 1;

        /*

        ************* YOUR CODE HERE ***************

        */
    }

    /* divide result by number of interacting units */
    if (normalize && n > 1) {
        double length = VectAbs(OutputVelocity) / n;
        UnitVect(OutputVelocity, OutputVelocity);
        MultiplicateWithScalar(OutputVelocity, OutputVelocity, length, Dim_l);
    }
}


/*


**************  EXERCICE 2  ************* 

GOAL: Fill in the next function 'AttractionSpring' to create a attractive force.

Hints: It is the same as the repulsion.


*/

void AttractionSpring(double *OutputVelocity,
        phase_t * Phase, const double p_l, const double R_0_l, 
        const int WhichAgent, const int Dim_l,
        const bool normalize) {

    NullVect(OutputVelocity, 3);

    int i;
    int n = 0;

    double *AgentsCoordinates;
    double *NeighboursCoordinates;
    // printf("nb agents = %d\n", Phase->NumberOfAgents);
    AgentsCoordinates = Phase->Coordinates[WhichAgent];
    
    static double DifferenceVector[3];
    static double DistanceFromNeighbour;
    /* Attractive interaction term */
    for (i = 0; i < Phase->NumberOfAgents; i++) {
        if (i == WhichAgent)
            continue;
        
        NeighboursCoordinates = Phase->Coordinates[i];
        VectDifference(DifferenceVector, AgentsCoordinates,
                NeighboursCoordinates);
        if (2 == Dim_l) {
            DifferenceVector[2] = 0.0;
        }
        DistanceFromNeighbour = VectAbs(DifferenceVector);
        /* Check if we interact at all */
        if (DistanceFromNeighbour <= R_0_l)
            continue;
        n += 1;

        /* *********************** YOUR CODE HERE ************************* */
    }

    /* divide result by number of interacting units */
    if (normalize && n > 1) {
        double length = VectAbs(OutputVelocity) / n;
        UnitVect(OutputVelocity, OutputVelocity);
        MultiplicateWithScalar(OutputVelocity, OutputVelocity, length, Dim_l);
    }
    //printf("Number of Attractive neighbours: %d Norm of attractive term relative to max repulsion velocity: %f\n", n, VectAbs (OutputVelocity)/V_Rep_l);
}

/*


**************  EXERCICE 3  ************* 

GOAL: Fill in the next function 'Alignment' to create alignment force.

You'll need different functions such as:
- VectPow()
- MultiplicateWithScalar()
- VectSum()

All these functions are defined in the utilities/math_utils.c file. 

INPUTS: 
- Phase: This object gathers all the information at a given time about all the agents (velocities, positions, ...).
- h: It is the linear coefficient used for the formula (mu in the course)
- alpha: It is the order of the power of the velocity difference (alpha in the course)
- Dim_l: 2D or 3D simulation, only useful for the MultiplicateWithScalar() function

OUTPUT:
-OutputVelocity: The output vector of the alignment (u_align in the course)


*/

void Alignment(double *OutputVelocity,
        phase_t * Phase, const double h, 
        const double alpha,
        const int WhichAgent, 
        const int Dim_l) {

        NullVect(OutputVelocity, 3);

        int i;

        double *AgentsVelocity;
        double *NeighboursVelocity;

        AgentsVelocity = Phase->Velocities[WhichAgent];

        static double DifferenceVelocities[3];

        for (i = 1; i < Phase->NumberOfAgents; i++) {   // i = 0 is the WhichAgent

            NeighboursVelocity = Phase->Velocities[i];

            VectDifference(DifferenceVelocities, NeighboursVelocity, AgentsVelocity);
            if (2 == Dim_l) {
                DifferenceVelocities[2] = 0.0;
            }

            /*

            ************* YOUR CODE HERE ***************

            */
            
        }
}

/* Olfati tracking */
void TrackingOlfati(double *OutputVelocity, double *TargetPosition,
        double *TargetVelocity, phase_t * Phase, const int WhichAgent, 
        const int Dim_l) {

        NullVect(OutputVelocity, 3);

        double *AgentsCoordinates;
        double *AgentsVelocity;

        double PositionComponent[3];
        double VelocityComponent[3];
        double PositionDiff[3];
        double VelocityDiff[3];



        AgentsCoordinates = Phase->Coordinates[WhichAgent];
        AgentsVelocity = Phase->Velocities[WhichAgent];

        VectDifference(PositionDiff, AgentsCoordinates, TargetPosition);
        // SigmaGrad(PositionComponent, PositionDiff, 1, Dim_l);
        SigmaGrad(OutputVelocity, PositionDiff, 1, Dim_l);

        // VectDifference(VelocityDiff, AgentsVelocity, TargetVelocity);

        // VectSum(OutputVelocity, VelocityDiff, PositionComponent);
        MultiplicateWithScalar(OutputVelocity, OutputVelocity, -3, Dim_l);

        }


/* Target tracking function */
void TargetTracking(double *OutputVelocity, double *TargetPosition,
        phase_t * Phase, const double R_CoM, const double d_CoM,
        const double R_trg, const double d_trg, 
        const int SizeOfNeighbourhood, const int WhichAgent, 
        const int Dim_l) {


        NullVect(OutputVelocity, 3);

        double *AgentsCoordinates;
        double TargetComponent[3];

        AgentsCoordinates = Phase->Coordinates[WhichAgent];

        /* CoM component */
        static double CoMDifferenceVector[3];
        double CoMCoef;
        double CoMCoords[3];
        double CoMComponent[3];

        GetNeighbourhoodSpecificCoM(CoMCoords, Phase, SizeOfNeighbourhood);
        VectDifference(CoMDifferenceVector, CoMCoords, AgentsCoordinates);
        UnitVect(CoMComponent, CoMDifferenceVector);

        CoMCoef = SigmoidLike(VectAbs(CoMDifferenceVector), R_CoM, d_CoM);

        MultiplicateWithScalar(CoMComponent, CoMComponent, CoMCoef, Dim_l);

        /* Trg component */
        static double TrgDifferenceVector[3];
        double TrgCoef;
        double TrgComponent[3];

        VectDifference(TrgDifferenceVector, TargetPosition, CoMCoords);
        UnitVect(TrgComponent, TrgDifferenceVector);

        TrgCoef = SigmoidLike(VectAbs(TrgDifferenceVector), R_trg, d_trg);

        MultiplicateWithScalar(TrgComponent, TrgComponent, TrgCoef, Dim_l);

        /* Add and normalize */
        VectSum(OutputVelocity, CoMComponent, TrgComponent);

        }

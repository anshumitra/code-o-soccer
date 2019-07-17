//	Warning! Do not modify this file

/*	It is through the skill layer that the MyStrategy layer talks to the Simulator/Actual robots
	1)namespace MyStrategy implements 2 important classes (LocalAvoidance and BeliefState) along
	with several other important functions namely: GoToPoint, GoToBall, TurnToAngle, Spin etc.
	2)Important functions are explained in brief below
	-> GoToPoint uses obstacle avoidance to reach a destination point while GoToPointStraight
	uses direct path to reach destination without any obstacle avoidance
	-> void maingotopoint(int botID,BeliefState *state, Vector2D<int> dpoint, float finalvel, 
	float finalslope, float clearance = CLEARANCE_PATH_PLANNER,bool increaseSpeed=0,bool 
	avoid_obstacle = true);
	It implememts gotopoint with greater flexibility of customizing path.
	Also clearance, increaseSpeed and avoid_obstacle are optional arguments with default values 
	set to CLEARANCE_PATH_PLANNER, 0 , 1 respectively
	3)bool pointyInField(Vector2D<int> final)
*/
#pragma once
#ifndef SKILLS_H
#define SKILLS_H


#include <cassert>
#include <string>
#include <vector>

#include "Utils/comdef.h"
#include "Utils/pathPlanners.h"
#include "Core/beliefState.h"
#include "winDebugger/Client.h"
#include "common/include/config.h"
#include "HAL/comm.h"
#include "common/include/geometry.hpp"

// Forward Declarations
namespace MyStrategy
{
  class LocalAvoidance;
  class BeliefState;
}

template <class T> class Vector2D;
typedef Vector2D<int> Vec2D;
// Use Vec2D to define a point with x and y as coordinate


namespace MyStrategy
{  
    /****************************************************List of Skills******************************************************/
    // Go to a point with obstacle avoidance.
    void GoToPoint(int botID,BeliefState *state,Vector2D<int> dpoint, float finalslope, bool increaseSpeed, bool shouldAlign);
    
    // Go to point without obstacle avoidance.
    void GoToPointStraight(int botID,BeliefState *state,Vector2D<int>dpoint,float finalslope, bool increaseSpeed, bool shouldAlign);
    
    // Go to ball: If shouldAlign is true, then bot will align with the line joining the ball and the goal else will go straightaway.
    void GoToBall(int botID,BeliefState *state,bool shouldAlign);
    
    // Angle in radian
    void TurnToAngle(int botID,BeliefState *state,float angle);
    
    // Control the bot directly.
    void Velocity(int botID,int vl,int vr);

    // Spin the bot. Positive angular speed for clockwise rotation
    void Spin(int botID,float angularSpeed);// spped in radians
    
    // Stop the bot.
    void Stop(int botID);
	void goToBall_copy(BeliefState *state,int botID, bool shouldAlign);
	void vibrate(BeliefState *state,int botID,int c);
	void oscillate(BeliefState *state,int botID,int c);
	void rotate(BeliefState *state,int botID);
    /*
     * For those who want to know more may checkout the function definiton in skills.cpp
     * And you can also add new skills depending upon your need here.
    */
    /*********************************************************************************************************************/
    

    bool pointyInField(Vector2D<int> final);

	void kick(BeliefState *state, int botID);
	void kickBall(BeliefState *state, int botID);
	void GoToPoint_copy(int botID,BeliefState *state,Vector2D<int> dpoint, float finalslope,bool increaseSpeed, bool shouldAlign);
    void maingotopoint(int botID,BeliefState *state, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance = CLEARANCE_PATH_PLANNER,bool increaseSpeed=0,bool avoid_obstacle = true);
    static void clearDebugData()
    {
      #ifdef FIRASSL_COMM
        comm->clearDebugData();
      #endif
    }



} // namespace MyStrategy
#endif // SKILLS_H

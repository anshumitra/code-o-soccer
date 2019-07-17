/*#pragma once
#include "skills.h"
*/
/*
	Vec2D can be used to make variables that can store points as (x,y) and provide utility functions.
	Vec2D point1;
	Vec2D point2;
	point1.x = HALF_FIELD_MAXX;
	point1.y = 0;
	similarly for point2 can be specified
	Some utility functions:
	These will appear automatically as you type Vec2D::
	float angle = Vec2D::angle(point1,point2);   
	int distance = Vec2D::distSq(point1,point2);
*/

/*
namespace MyStrategy
{
  void goalkeeper(BeliefState *state,int botID)
  {
	Vec2D ballPoint(state->ballPos.x, state->ballPos.y);
	Vec2D dpoint(OUR_GOAL_X+DBOX_WIDTH/2,state->ballPos.y);

	//if(state->pr_ball_in_our_dbox && state->pr_looseBall)  //probably not working at all
	//if(state->pr_ball_in_our_dbox)// && state->pr_looseBall)
	//{
		//GoToBall(botID, state, true);
	//}
	
		if(dpoint.y > OUR_GOAL_MAXY)
		  dpoint.y = OUR_GOAL_MAXY;
		if(dpoint.y < OUR_GOAL_MINY)
		  dpoint.y = OUR_GOAL_MINY;
		GoToPoint(botID,state,dpoint,PI/2,false,false);
  }

  void goalkeeper_throw(BeliefState *state, int botID) //NOT WORKING
  {
	  	Vec2D ballPoint(state->ballPos.x, state->ballPos.y);

	  //if( Vec2D::distSq(state->homePos[botID],state->ballPos) < 2*BOT_BALL_THRESH){ //ball with bot
			if(ballPoint.y>0){
				Vec2D dest(-1*HALF_FIELD_MAXX, DBOX_HEIGHT*1.5);
				GoToPoint(botID, state, dest, 0, false, false);
			}
			else{
				Vec2D dest(-1*HALF_FIELD_MAXX, -1*DBOX_HEIGHT*1.5);
				GoToPoint(botID, state, dest, 0, false, false);
			}
		}
}*/

#pragma once
#include "skills.h"

/*
	Vec2D can be used to make variables that can store points as (x,y) and provide utility functions.
	Vec2D point1;
	Vec2D point2;
	point1.x = HALF_FIELD_MAXX;
	point1.y = 0;
	similarly for point2 can be specified
	Some utility functions:
	These will appear automatically as you type Vec2D::
	float angle = Vec2D::angle(point1,point2);   
	int distance = Vec2D::distSq(point1,point2);
*/

namespace MyStrategy
{
  void goalkeeper(BeliefState *state,int botID)
  {
	Vec2D ballPoint(state->ballPos.x, state->ballPos.y);
	Vec2D dpoint(OUR_GOAL_X+DBOX_WIDTH/2,state->ballPos.y);


	if(!(state->ballPos.y < OUR_GOAL_MAXY && state->ballPos.y > OUR_GOAL_MINY)) {
		oscillate(state,botID,state->ballPos.y);
	}
	else
	{
		if(dpoint.y > OUR_GOAL_MAXY)
				dpoint.y = OUR_GOAL_MAXY;
			if(dpoint.y < OUR_GOAL_MINY)
				dpoint.y = OUR_GOAL_MINY; 
			GoToPoint(botID,state,dpoint,PI/2,false,false);
			//vibrate(state, botID, 0);
				
	}
	if(state->homePos[botID].x < OUR_GOAL_X) {
		Vec2D fpoint(OUR_GOAL_X+DBOX_WIDTH/2,state->ballPos.y);
		GoToPoint(botID,state,fpoint,PI/2,false,false);
	}
  }
}
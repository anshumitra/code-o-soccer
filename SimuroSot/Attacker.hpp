#pragma once
#include "skills.h"
#include <time.h>

namespace MyStrategy
{
  
	/* define the default action of the attacker */
	void attacker_default(BeliefState *state, int botID, double align_angle)
	{
		if(Vec2D::dist(state->homePos[botID], state->ballPos.x, state->ballPos.y) < 0.5*BOT_BALL_THRESH){
			Vec2D shootPoint;
			shootPoint.x= OPP_GOAL_X - DBOX_DEPTH/2;
			shootPoint.y= OPP_GOAL_Y;
			GoToPoint(botID, state, shootPoint, align_angle, true, false);
		}
		else
			GoToPoint(botID, state, state->ballPos, align_angle, true, true);
	}
	 void drag_mod(BeliefState *state, int botID) {
		 Vec2D dpoint;
		 dpoint.x = OPP_GOAL_X;
		 dpoint.y = state->ballPos.y>0?OPP_GOAL_MINY:OPP_GOAL_MAXY;
		 GoToPointStraight(botID,state,dpoint,0,true,false);
  }
	/*void attacker_compulsory(BeliefState *state, int botID)
	{
        Vec2D ballPoint(state->ballPos.x, state->ballPos.y);
		if((Vec2D::dist(state->homePos[botID], state->ballPos.x, state->ballPos.y) < 0.5*BOT_BALL_THRESH) &&
			ballPoint.x > HALF_FIELD_MAXX*0.6){
				TurnToAngle(botID, state, 5*PI);
		}
	}*/

	/* define different strategies of the attacker based on the ball's position on the field */
	void attacker(BeliefState *state, int botID)
	{
		print("Attacker\n");
		if(state->ballPos.x > OPP_GOAL_X - DBOX_WIDTH && state->homePos[botID].x > OPP_GOAL_X - DBOX_WIDTH) {
			Spin(0,20*PI);
			drag_mod(state, botID);
		}
		if(state->ballPos.x > 0){ //ball in opponent side
			double align_angle;
			if(state->pr_ballInSideStrip && state->ballPos.y<0)
				align_angle= PI/2;
			else if(state->pr_ballInSideStrip && state->ballPos.y>0)
				align_angle= -PI/2; //fix
			else
				align_angle= 0.0;
			attacker_default(state, botID, align_angle);
		}
		else{ //ball our side
			//attacker_default(state, botID, 0.0);
			if(Vec2D::dist(state->homePos[botID], state->ballPos.x, state->ballPos.y) < 0.5*BOT_BALL_THRESH){
				Vec2D shootPoint;
				shootPoint.x= OPP_GOAL_X - DBOX_DEPTH/2;
				shootPoint.y= OPP_GOAL_Y;
				GoToPoint(botID, state, shootPoint, 0, true, false);
			}
			else
				GoToBall(botID, state, false);
		}
		
		/*
        Vec2D ballPoint(state->ballPos.x, state->ballPos.y);
		Vec2D dpoint;

        //Region 1
		if((ballPoint.x < OUR_GOAL_X + DBOX_WIDTH*1.5) && 
			(ballPoint.y > DBOX_HEIGHT*1.5 || ballPoint.y < -1*DBOX_HEIGHT*1.5)){
			attacker_default(state, botID);
		}
        
        //Region 2
		else if(state->pr_ball_in_our_dbox){
			attacker_default(state, botID);
		}
        
		//Region 3
		else if(state->pr_ballInSideStrip && state->pr_ballOppSide){
			srand(time(NULL));
			if( Vec2D::distSq(state->homePos[botID],state->ballPos) < 2*BOT_BALL_THRESH)
				GoToPoint(botID,state,state->opp_goalpoints[rand() % 5],0,true,true);
			else 
				GoToPoint(botID,state,ballPoint,0,true, true);
			//attacker_compulsory(state, botID);
		}

		//Region 4
		else if(state->pr_ball_in_opp_dbox){
			srand(time(NULL));
			if( Vec2D::distSq(state->homePos[botID],state->ballPos) < 2*BOT_BALL_THRESH) //ball with bot
				//TurnToAngle(botID, state, 5*PI);
				GoToPoint(botID,state,state->opp_goalpoints[rand() % 5],0,true,false);
			else 
				GoToPoint(botID,state,ballPoint,0,true, true);
		}

		//Region 5
        else{
			attacker_default(state, botID);
		}
		//TurnToAngle(botID, state, 5*PI);
		*/

		//attacker_default(state, botID);
		//attacker_compulsory(state, botID);
    }
}
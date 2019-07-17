#pragma once
#include "skills.h"

namespace MyStrategy
{
	/* by default, the defender stays at (-HALF_FIELD_MAXX/2, ballPos.y) */ 
	void defender_default(BeliefState *state,int botID)
	{
		Vec2D dpoint;
		dpoint.x = -1*HALF_FIELD_MAXX/2;
		dpoint.y = state->ballPos.y;
		GoToPoint(botID,state,dpoint,PI/2,false,false);
	}

	/* define different strategies of the defender based on the ball's 
	   position in the field*/
    void defender(BeliefState *state, int botID)
    {
        Vec2D ballPoint(state->ballPos.x, state->ballPos.y);
		Vec2D dpoint;

        /*Region 1*/
		if((ballPoint.x < OUR_GOAL_X + DBOX_WIDTH*2) && 
			(ballPoint.y > OUR_GOAL_MAXY || ballPoint.y < OUR_GOAL_MINY)){
			dpoint.x= ballPoint.x;
			if(ballPoint.y > OUR_GOAL_MAXY)
				dpoint.y= DBOX_HEIGHT*3 
				;
			else if(ballPoint.y < OUR_GOAL_MINY)
				dpoint.y= -1*DBOX_HEIGHT*3;

			GoToPoint(botID, state, dpoint, PI, false, false);
		}
        
        /*Region 2*/               /*NEED TO IMPROVE!*/
		else if(ballPoint.x < OUR_GOAL_X + DBOX_WIDTH*3 && ballPoint.x > OUR_GOAL_X + DBOX_WIDTH*1.5 && (ballPoint.y < OUR_GOAL_MAXY*3 || ballPoint.y > OUR_GOAL_MINY*3) ){ //&& state->pr_oppBall
			/*
			Spin(0,20*PI);
			dpoint.y= ballPoint.y - DBOX_WIDTH/4;
			dpoint.x= ballPoint.x - DBOX_WIDTH/4; 
			double defend_radius = -50.0;
			float ball_angle = atan2(state->ballPos.y-dpoint.y,state->ballPos.x-dpoint.x);
			Vector2D<int> fpoint;
			printf("Ball : %d, %d\n", state->ballPos.x, state->ballPos.y);
			fpoint.y=dpoint.y + defend_radius*sin(ball_angle);
			fpoint.x=dpoint.x + defend_radius*cos(ball_angle);
			GoToPoint(botID,state,fpoint,ball_angle,true,false);
			*/
			float angle;
			if(state->ballPos.y < 0)
				angle = -1 * PI/2;
			else
				angle = PI/2;
			GoToPointStraight(botID,state,ballPoint, angle,true,true);
			//GoToPoint(botID, state, dpoint, PI, false, true);
			
		}
        
        /*Region 3*/
		else if(ballPoint.x > HALF_FIELD_MAXX/2){
			dpoint.x= HALF_FIELD_MAXX/2;
			dpoint.y= ballPoint.y;

			GoToPoint(botID, state, dpoint, PI, true, true);
		}
        
        /*Region 4*/
        else{
			defender_default(state, botID);
		}

    }
}

/* IMPROVE:
 * use vibrate()
 * improve Region #2
 */
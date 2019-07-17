//	Warning! Do not modify this file
#include "stdafx.h"
#include "skills.h"
#include<time.h>
#define ANGLE_THRESH 0.1

using namespace HAL;
using namespace std;
namespace MyStrategy
{
  

  /***************************GoToPoint with pathplanner***************************************************/
  void GoToPoint(int botID,BeliefState *state,Vector2D<int> dpoint, float finalslope,bool increaseSpeed, bool shouldAlign)
  {

	if(shouldAlign)
		maingotopoint(botID,state,dpoint,0,finalslope,CLEARANCE_PATH_PLANNER, increaseSpeed,true);
	else
		maingotopoint(botID,state,dpoint,0,finalslope,0,increaseSpeed,true);
  }
  void GoToPointStraight(int botID,BeliefState *state,Vector2D<int> dpoint, float finalslope,bool increaseSpeed, bool shouldAlign)
  {
	if(shouldAlign)
		maingotopoint(botID,state,dpoint,0,finalslope,CLEARANCE_PATH_PLANNER, increaseSpeed, false);
    else
		maingotopoint(botID,state,dpoint,0,finalslope,0,increaseSpeed, false);

  }
  void GoToPoint_copy(int botID,BeliefState *state,Vector2D<int> dpoint, float finalslope,bool increaseSpeed, bool shouldAlign)
  {

	if(shouldAlign)
		maingotopoint(botID,state,dpoint,5000,finalslope,CLEARANCE_PATH_PLANNER, increaseSpeed,true);
	else
		maingotopoint(botID,state,dpoint,5000,finalslope,0,increaseSpeed,true);
  }
 
  void maingotopoint(int botID,BeliefState *state, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance,bool increaseSpeed,bool avoid_obstacle)
  {
    int prevVel = 0;
    static LocalAvoidance*    pathPlanner;
    pathPlanner = new LocalAvoidance();
    Comm* comm  = Comm::getInstance();
    std::vector<obstacle> obs;
    obstacle obsTemp;
    for (int i = 0; i < HomeTeam::SIZE ; ++i)
    {
      /// Adding Condition to REMOVE all obstacles that are sufficiently CLOSE to BALL
      if( i != botID && Vector2D<int>::distSq(state->homePos[botID], state->homePos[i]) < COLLISION_DIST * COLLISION_DIST && Vector2D<int>::distSq(state->ballPos, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 9)
      {
        /// If destination is close to bot, don't add it as obstacle
        if(Vector2D<int>::distSq(dpoint, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 1)
        {
          obsTemp.x = state->homePos[i].x;
          obsTemp.y = state->homePos[i].y;
          obsTemp.radius =  BOT_RADIUS;
          obs.push_back(obsTemp);
        }
      }
    }

    for (int i = 0; i < AwayTeam::SIZE; ++i)
    {
      if(Vector2D<int>::distSq(state->homePos[botID], state->awayPos[i]) < COLLISION_DIST * COLLISION_DIST && Vector2D<int>::distSq(state->ballPos, state->awayPos[i]) > BOT_RADIUS * BOT_RADIUS * 9)
      {
        /// If destination is close to bot, don't add it as obstacle
        if(Vector2D<int>::distSq(dpoint, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 1)
        {
          obsTemp.x = state->awayPos[i].x;
          obsTemp.y = state->awayPos[i].y;
          obsTemp.radius =  BOT_RADIUS;
          obs.push_back(obsTemp);
        }
      }
    }
    Vector2D<int> point, nextWP, nextNWP;
    float r = 0, t = 0, dist = 0;
    dist = Vector2D<int>::dist(dpoint, state->homePos[botID]);  // Distance of next waypoint from the bot
    if(dist < BOT_POINT_THRESH )
    {
      float angle = fabs((float)firaNormalizeAngle(state->homeAngle[botID] - finalslope));
      if(angle > DRIBBLER_BALL_ANGLE_RANGE)
      {
        TurnToAngle(botID,state,finalslope);
        return;
      }
     
      comm->sendCommand(botID, finalvel, finalvel);
      return;
      // This part of the function is just for safety.
      // The tactic should actually prevent this call.
      // The bot should be aligned properly before this condition is reached.
    }

    pathPlanner->plan(state->homePos[botID],
                      state->homeVel[botID],
                      dpoint,
                      obs,
                      botID,
                      true,
                      state->homeAngle[botID],
                      finalslope,
                      t,
                      r,
                      comm,
                      clearance,
                      avoid_obstacle);
	
    float fTheta = asin(sqrt(fabs((float)r)));
    fTheta = 1 - fTheta/(PI/2);
    fTheta = pow(fTheta,2.2) ;
    float fDistance = (dist > BOT_POINT_THRESH*3) ? 1 : dist / ((float) BOT_POINT_THRESH *3);
    float fTot = fDistance * fTheta;
    fTot = 0.2 + fTot*(1-0.2);

    float profileFactor = MAX_BOT_SPEED*fTot;
    
    r *= 0.2*profileFactor;
    t *= profileFactor;
    #if FIRA_COMM || FIRASSL_COMM
    comm->sendCommand(botID, (t - r), (t + r));
    #else
    comm->sendCommand(botID, (t - r), (t + r));
    #endif
  }
  /*******************************************************************************/



  /************************************TurnToAngle*********************************/
  void TurnToAngle(int botID,BeliefState *state, float angle)
  {
    Comm *comm = Comm::getInstance();
    float vl,vr;
    float finalSlope = angle;
    float turnAngleLeft = normalizeAngle(finalSlope - state->homeAngle[botID]); // Angle left to turn
    
    if(turnAngleLeft>PI/2||turnAngleLeft<-PI/2)
    {
      if(turnAngleLeft>PI/2)
        turnAngleLeft=turnAngleLeft-PI;
      else
        turnAngleLeft=turnAngleLeft+PI;
    }

    float factor = (turnAngleLeft+(turnAngleLeft))/(PI/2);
    vr = -0.4*MAX_BOT_OMEGA*(factor)/(PI/2);
	vl = -vr;
    
    if(fabs((float)turnAngleLeft) > DRIBBLER_BALL_ANGLE_RANGE/2)
    {
      #if FIRA_COMM || FIRASSL_COMM 
      comm->sendCommand(botID, vl, vr);        
      #else
      comm->sendCommand(botID, vr, vl); 
      #endif
    }
    else 
    {
      comm->sendCommand(botID, 0, 0);
    }
  }
  /*********************************************************************************/

  /*************************************GoToBall************************************/
  void GoToBall(int botID,BeliefState *state,bool align)
  {
		Comm *comm = Comm::getInstance();
		Vector2D<int> ballInitialpos,ballFinalpos;
		ballInitialpos = state->ballPos;
		ballFinalpos.x = state->ballPos.x+(0.7*state->ballVel.x);
		ballFinalpos.y = state->ballPos.y+(0.7*state->ballVel.y);
		
		float botballdist = Vector2D<int>::dist(ballInitialpos,state->homePos[botID]);
		float botballVeldist = Vector2D<int>::dist(ballFinalpos,state->homePos[botID]);
		
		Vec2D goalPoint(HALF_FIELD_MAXX,0);
		float theta=Vector2D<int>::angle(ballInitialpos,goalPoint);
		
		if (align==true)
		{
			if((Vector2D<int>::dist(ballInitialpos,state->homePos[botID]))>=BOT_BALL_THRESH)
			{
				if(fabs((float)tan(Vector2D<int>::angle(ballInitialpos,ballFinalpos))-tan(Vector2D<int>::angle(state->homePos[botID],ballFinalpos)))<1)
					maingotopoint(botID,state,ballInitialpos,0,theta,CLEARANCE_PATH_PLANNER,false);
				else
					maingotopoint(botID,state,ballFinalpos,0,theta,CLEARANCE_PATH_PLANNER,false);
			}
			else 
			{
				comm->sendCommand(botID,0,0);
			}
		}
		else
		{
			if((Vector2D<int>::dist(ballInitialpos,state->homePos[botID]))>=BOT_BALL_THRESH)
			{
					maingotopoint(botID,state,ballInitialpos,0,theta,00.0,false);
			}
			else 
			{
				comm->sendCommand(botID,0,0);
			}
		}
  }
  /**********************************************************************************************/

  void Stop(int botID)
  {
    Comm *comm = Comm::getInstance();
    comm->sendCommand(botID,0,0);
  } // stop

  void Velocity(int botID,int vl,int vr)
  {
    Comm *comm = Comm::getInstance();
    comm->sendCommand(botID,vl,vr);
  }

  void Spin(int botID,float angularSpeed)// speed in radians
  {
    Comm *comm = Comm::getInstance();
    float omega,vl,vr ; 
    vr = (angularSpeed * (MAX_BOT_SPEED))/MAX_BOT_OMEGA ;
    vl = -vr;
    printf("%f %f\n", vl, vr);
    
    #if FIRA_COMM || FIRASSL_COMM
      comm->sendCommand(botID, vl, vr);        
    #else
      comm->sendCommand(botID, vr, vl);
    #endif
  }

  bool pointyInField(Vector2D<int> final)
  {
    if(final.y < -HALF_FIELD_MAXY + BALL_AT_CORNER_THRESH || final.y > HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH)
      return false;
    else return true;
  }
  void oscillate(BeliefState *state,int botID,int c)
	{
		
		Vector2D<int> s1,s2;
		s1.x = s2.x = OUR_GOAL_X + DBOX_WIDTH/2;
		s1.y= c + HALF_FIELD_MAXY*.1;
		s2.y= c - HALF_FIELD_MAXY*.1;
		if(state->homeAngle[botID] == 0.0) //==0.0
		{
			TurnToAngle(botID,state,0.0);
		}
		if(abs(state->homePos[botID].y-s1.y)<=BOT_BALL_THRESH)
		{
		
				Velocity(botID,10,10);
				GoToPoint(botID,state,s2, 90, false, false); //both false
		}
		else if(abs(state->homePos[botID].y-s2.y)<=BOT_BALL_THRESH)
		{
			Velocity(botID,10,10);
			GoToPoint(botID,state,s1, 90, false, false); //both false
			
		}
		/*
		if(state->homeVel[botID].y>0)
				GoToPoint(botID,state,s1, 90,false, true);
			else if(state->homeVel[botID].y<0)
				GoToPoint(botID,state,s2, 90,false, true);
		*/
				
			//	GoToPoint(botID,state,s2, 90,false, false);
		//	else if(abs(state->homePos[botID].y-s2.y)==BOT_BALL_THRESH)
			//	GoToPoint(botID,state,s1, 90,false, false);
		    
            
  }
 void rotate(BeliefState *state,int botID)
	{
		
		Vector2D<int> s1,s2;
		//s1.y=c+HALF_FIELD_MAXY*.3;
		s1.y = s2.y = state->ballPos.y;
		s1.x = s2.x = OUR_GOAL_X + DBOX_WIDTH/2;
		//s2.y=c-HALF_FIELD_MAXY*.3;
		
		if(abs(state->homePos[botID].y-s1.y)<=BOT_BALL_THRESH)
			{
					
				Velocity(botID,0,0);
				GoToPoint(botID,state,s2, 90,false, false);
			
		}
		else if(abs(state->homePos[botID].y-s2.y)<=BOT_BALL_THRESH)
		{
			Velocity(botID,0,0);
			GoToPoint(botID,state,s1, 90,false, false);
			
		}
		if(state->homeVel[botID].y>0)
				GoToPoint(botID,state,s1, 90,false, true);
			else if(state->homeVel[botID].y<0)
				GoToPoint(botID,state,s2, 90,false, true);		
			//	GoToPoint(botID,state,s2, 90,false, false);
		//	else if(abs(state->homePos[botID].y-s2.y)==BOT_BALL_THRESH)
			//	GoToPoint(botID,state,s1, 90,false, false);
		    
            
  }
				
 void kick(BeliefState *state, int botID)
 {
	srand(time(NULL));
	Vec2D ballPoint(state->ballPos.x, state->ballPos.y);
	if( Vec2D::distSq(state->homePos[botID],state->ballPos) < 2*BOT_BALL_THRESH)
	{
			GoToPoint(botID,state,state->opp_goalpoints[rand() % 5],0,true,false);
			if(state->ballPos.y < HALF_FIELD_MAXY) {
				TurnToAngle(botID, state, PI/4);
			}
			else {
				//if we are attacking towards the left
				TurnToAngle(botID, state, PI/4);
			}
	}
	else { 
				GoToPoint(botID,state,ballPoint,0,true, true);
	}
 }
 void vibrate(BeliefState *state,int botID,int c)
	{
		
		Vector2D<int> s1,s2;
		s1.y=c+HALF_FIELD_MAXY*.1;
		s1.x = s2.x = OUR_GOAL_X + DBOX_WIDTH;
		s2.y=c-HALF_FIELD_MAXY*.1;
		
		if(abs(state->homePos[botID].y-s1.y)<=BOT_BALL_THRESH)
			{
					
				Velocity(botID,0,0);
				GoToPoint(botID,state,s2, 90,false, false);
			
		}
		else if(abs(state->homePos[botID].y-s2.y)<=BOT_BALL_THRESH)
		{
			Velocity(botID,0,0);
			GoToPoint(botID,state,s1, 90,false, false);
			
		}
		
		if(state->homeVel[botID].y>0)
				GoToPoint(botID,state,s1, 90,false, true);
			else if(state->homeVel[botID].y<0)
				GoToPoint(botID,state,s2, 90,false, true);
			else if(abs(state->homePos[botID].y-s2.y)==BOT_BALL_THRESH)
				GoToPoint(botID,state,s1, 90,false, false);
		    
            
  }

 void kickBall(BeliefState *state, int botID)
  {
	// is called only when bot is behind the ball  
	  char debug[250];
	  Comm* comm  = Comm::getInstance();
	if (abs(state->ballPos.x)>HALF_FIELD_MAXX ||abs(state->ballPos.y)>HALF_FIELD_MAXY || abs(state->ballVel.x)>10000 || abs(state->ballVel.y)>10000) return;
	 // written for safe condition 
	   if(state->ballPos.x <= state->homePos[botID].x) 
	   {
		   comm->sendCommand(botID,0,0) ;
		   return ;
	   }
	   // if there is kicking situation
	   float d = (HALF_FIELD_MAXX-GOAL_DEPTH)/10 ;
	   float m ;
	   if(abs(state->ballVel.x)<10)
		   m = INF ;
	   else
		   m = state->ballVel.y/state->ballVel.x;
	   
	   float xpos[11]={0},ypos[11]={0};
	   float time_error[11]={0};
	   int minimum = INF , vbot=100;     // vbot is the average bot speed .. can be set to something else
	   Vector2D<int> dest;
	   float tx1=0,tx2=0,ty1=0,ty2=0;
	   if(abs(state->ballVel.x)>30)
	  {
			  for(int i=1;i<10;i++)
			  {
				  xpos[i]=d*i;
				  if(m==INF)
					  ypos[i] = INF ;
				  else
	    			  ypos[i]=state->ballPos.y+ m*(xpos[i]-state->ballPos.x);
				 
				  if(abs(state->ballVel.x)>10)
				    tx1=abs((xpos[i]-state->ballPos.x)/state->ballVel.x);
				  else
			        tx1=INF;
				 
				  if((ypos[i]==INF)||abs(state->ballVel.y)<10)
					  ty1 = INF ;
				  else
					  ty1=abs((ypos[i]-state->ballPos.y)/state->ballVel.y);
				 
				  dest.x = xpos[i] ; dest.y = ypos[i] ;
			//	no use of this condition ::  if(abs(state->homeVel[botID].x)>10)
				  tx2 = (Vector2D<int>::dist(state->homePos[botID],dest))/vbot; 
				  ty2 = (Vector2D<int>::dist(state->homePos[botID],dest))/vbot; 
			 		  // earlier equation was wrong 

// edited till here ........................................................................................................
// continue further from here...............................................................................................
				  if(abs(ypos[i])>OUR_GOAL_MAXY+3*BOT_RADIUS)
					  time_error[i] = INF ;
				  else
					  if(tx1 == INF || ty1 == INF )
						  time_error[i] = INF ;
					  else  
				          time_error[i]=abs((tx1-tx2)*(ty1-ty2));
		  
			//   outfile<<"i : "<<i<<" xpos[i] : "<<xpos[i]<<" ypos[i] : "<<ypos[i]<<" time_error"<<time_error[i]<<endl ;
			 //  outfile.close();
 			  }
	   
			  time_error[0] = INF;
			  float min=time_error[0];
			  for(int i=1;i<10;i++)
			  {
				  if(time_error[i]<min) {
					  minimum=i;
					  min = time_error[i] ;
				  }
			  }

			  dest.x=xpos[minimum];
			  dest.y=ypos[minimum];
	    
			  sprintf(debug,"%f :: %d :: %d :: %d ",time_error[minimum],minimum,dest.x,dest.y);
		      Client::debugClient->SendMessages(debug);

		}
	 
	   else
	  {
		  
		  d=HALF_FIELD_MAXY/100;
		  dest.x=state->ballPos.x;
		  for(int i=1;i<100;i++)
		  {
			  ypos[i]=d*i*SGN(state->ballVel.y);
			  if(abs(state->ballVel.y)>10) 
			  ty1=abs((state->ballPos.y-ypos[i])/state->ballVel.y);
			  else
			  ty1=0;
			  ty2=abs((state->homePos[botID].y-ypos[i]/vbot));
			  time_error[i]=abs(ty1-ty2);
		  
		  }
		  float min=time_error[0];
		   for(int i=1;i<100;i++)
			  {
				  if(min>time_error[i]) minimum=i;
			  }
		   dest.y=ypos[minimum];
		   
	    //sprintf(debug,"goign in else %f :: %d :: %d :: %d ",time_error[minimum],minimum,dest.x,dest.y);
		//Client::debugClient->SendMessages(debug);
	  }
	  
	  float finalangle=Vector2D<int>::angle(dest,Vector2D<int> (OPP_GOAL_X,0));
	  int finalvel=0;
	  //sprintf(debug,"%f  %f  %f  %d  %d \n",state->ballVel.x,state->ballVel.y,time_error[minimum],dest.x,dest.y);
	//	Client::debugClient->SendMessages(debug);
	  maingotopoint(botID,state, dest, finalvel,finalangle, CLEARANCE_PATH_PLANNER,1,1);
 
}
void goToBall_copy(BeliefState *state,int botID, bool shouldAlign)
  {
		Comm* comm  = Comm::getInstance();
		Vector2D<int> ballInitialpos,ballFinalpos;
		ballInitialpos = state->ballPos;
		ballFinalpos.x = state->ballPos.x+(0.7*state->ballVel.x);
		ballFinalpos.y = state->ballPos.y+(0.7*state->ballVel.y);
		
		float botballdist = Vector2D<int>::dist(ballInitialpos,state->homePos[botID]);
		float botballVeldist = Vector2D<int>::dist(ballFinalpos,state->homePos[botID]);
		
		float theta=Vector2D<int>::angle(ballInitialpos,ballFinalpos);
		
		if (shouldAlign==true)
		{
			if((Vector2D<int>::dist(ballInitialpos,state->homePos[botID]))>=BOT_POINT_THRESH)
			{
					if(fabs((float)tan(Vector2D<int>::angle(ballInitialpos,ballFinalpos))-tan(Vector2D<int>::angle(state->homePos[botID],ballFinalpos)))<ANGLE_THRESH)
						GoToPoint(botID,state,ballInitialpos,theta, true, true);
          else
			  GoToPoint(botID,state,ballFinalpos,theta,true, true);
			}
			else 
			{
		        float vl,vr;
				float finalSlope = Vector2D<int>::angle(ballInitialpos,ballFinalpos);
		        float turnAngleLeft = normalizeAngle(theta - state->homeAngle[botID]); // Angle left to turn
		        if(turnAngleLeft>PI/2||turnAngleLeft<-PI/2)
		        {
			if(turnAngleLeft>PI/2)
				turnAngleLeft=turnAngleLeft-PI;
			else
			 turnAngleLeft=turnAngleLeft+PI;
			}
			float omega = turnAngleLeft * MAX_BOT_OMEGA/(16*PI); // Speedup turn
			vr = -MAX_BOT_SPEED*turnAngleLeft/(2*PI);
			vl = -vr;
			printf("vl=%f, \nvr=%f \n,omega=%f \n",vl,vr,omega);
			printf("State home angle: %f",state->homeAngle[botID]);
			printf("State Home pos: %d %d", state->homePos[botID].x,state->homePos[botID].y);

        if(fabs((float)turnAngleLeft) < 0.3)
          comm->sendCommand(botID,0,0);
        else
          comm->sendCommand(botID,0,0);
			}
		}
		else
		{
			if((Vector2D<int>::dist(ballInitialpos,state->homePos[botID]))>=BOT_POINT_THRESH)
			{
				GoToPoint(botID,state,ballInitialpos,0,theta,00.0);
      }
			else 
			{
				comm->sendCommand(botID,0,0);
			}
		}
  } // goToBall
} // namespace MyStrategy
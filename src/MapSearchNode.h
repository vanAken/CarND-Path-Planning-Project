#ifndef PATH_PLANNING_MAPSEARCHNODE_H
#define PATH_PLANNING_MAPSEARCHNODE_H

#include "stlastar.h" // See header for copyright and usage information
#include "prediction.h"
#include <iostream>
#include <stdio.h>
#include <math.h>


       

class MapSearchNode
{
public:
	int d;	 // the (d,s,t,v) state of the node
	int s;	
        int t;
        int v;   // stat variable form speed
   
        int v_max =+2;
        int v_min =-2;
	std::shared_ptr<Prediction> pre;// = &trajectory ; // pre->GetMap mit instance trajectory in main.cpp bekannt machen forward declaration???

	MapSearchNode(std::shared_ptr<Prediction> preditcion) : pre(preditcion) { d = s = t = v = 0; }
	MapSearchNode( int pd, int ps, int pt , int pv ) { d=pd; s=ps; t=pt; v=pv; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
        float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo();
        int GetNode_s();  
        int GetNode_d();  
        int GetNode_v();  
};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
	// same state in a maze search is simply when (d,s,t) are the same not v!
	if( (d == rhs.d) &&
            (s == rhs.s) &&
  	    (t == rhs.t) )
	        return true;
	return false;
}

void MapSearchNode::PrintNodeInfo(){
	char str[100];
	sprintf( str, "Node position : (%d,%d,%d,%d)\n", d,s,t,v );
	cout << str;
}

int MapSearchNode::GetNode_s(){
    return s;
}
int MapSearchNode::GetNode_d(){
    return d;
}
int MapSearchNode::GetNode_v(){
    return v;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	return abs(d - nodeGoal.d) + abs(s - nodeGoal.s) + abs(t - nodeGoal.t); // t is importand!!!
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{
	if( (d == nodeGoal.d) && // only S
            (s == nodeGoal.s) ){
	    return true;
	}
	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{
	if(!pre) return false;

	MapSearchNode NewNode;

        
	if (-2<=v && v<=2){ // traverse move possible only with v=0
            if (pre->GetMap( d, s, t+1) < 9){    // no move with const. v just wait
		NewNode = MapSearchNode( d, s, t+1, 0 );
		astarsearch->AddSuccessor( NewNode );
	    }	            
            if (pre->GetMap( d+1, s, t+1) < 9){  // lanechange: left and right           
		NewNode = MapSearchNode( d+1, s, t+1, 0 );
		astarsearch->AddSuccessor( NewNode );
            }
            if (pre->GetMap( d-1, s, t+1) < 9){            
		NewNode = MapSearchNode( d-1, s, t+1, 0 );
		astarsearch->AddSuccessor( NewNode );
            }
	}	
	
        // go futher in s
        if (pre->GetMap( d, s+1, t) < 9){
	    if ( v<=v_max && -1<=v && v<=3 ){ 
                NewNode = MapSearchNode( d, s+1, t+1, +1 );
		astarsearch->AddSuccessor( NewNode );
            }
            if (pre->GetMap( d, s+2, t) < 9 ){
                if( v<=v_max && 0<=v && v<=4){
		    NewNode = MapSearchNode( d, s+2, t+1, +2 );
		    astarsearch->AddSuccessor( NewNode );
                }
                if (pre->GetMap( d, s+3, t) < 9 &&
                    pre->GetMap( d, s+4, t) < 9 ){
	            if ( v<=v_max && 1<=v && v<=5){ 
                        NewNode = MapSearchNode( d, s+3, t+1, +3 );
		        astarsearch->AddSuccessor( NewNode );
                    }
                    if (pre->GetMap( d, s+5, t) < 9 && 
                        pre->GetMap( d, s+6, t) < 9 ){
                        if( v<=v_max && 2<=v && v<=6){
		            NewNode = MapSearchNode( d, s+4, t+1, +4 );
 		            astarsearch->AddSuccessor( NewNode );
                        }
                        if (pre->GetMap( d, s+7, t) < 9 && 
                            pre->GetMap( d, s+8, t) < 9 && 
                            pre->GetMap( d, s+9, t) < 9){
                            if( v<=v_max && 3<=v && v<=7){
		                NewNode = MapSearchNode( d, s+5, t+1, +5 );
		                astarsearch->AddSuccessor( NewNode );
                            }
                        } 
                    }
                }
	    }	
	}	

        // fall back in s
	if (pre->GetMap( d, s-1, t) < 9){
	    if ( v_min<=v && -3<v && v<=1 ){ 
                NewNode = MapSearchNode( d, s-1, t+1, -1 );
		astarsearch->AddSuccessor( NewNode );
            }
            if (pre->GetMap( d, s-2, t) < 9){
                if( v_min<=v && -4<=v && v<=0){
		    NewNode = MapSearchNode( d, s-2, t+1, -2 );
		    astarsearch->AddSuccessor( NewNode );
                }
                if (pre->GetMap( d, s-3, t) < 9 &&
                    pre->GetMap( d, s-4, t) < 9 ){
	            if ( v_min<=v && -5<=v && v<=-1){ 
                        NewNode = MapSearchNode( d, s-3, t+1, -3 );
		        astarsearch->AddSuccessor( NewNode );
                    }
                    if (pre->GetMap( d, s-5, t) < 9 &&
                        pre->GetMap( d, s-6, t) < 9 ){
                        if( v_min<=v && -6<=v && v<=-2){
		            NewNode = MapSearchNode( d, s-4, t+1, -4 );
 		            astarsearch->AddSuccessor( NewNode );
                        }
                        if (pre->GetMap( d, s-7, t) < 9 && 
                            pre->GetMap( d, s-8, t) < 9 && 
                            pre->GetMap( d, s-9, t) < 9){
                            if( v_min<=v && -7<=v && v<=-3){
		                NewNode = MapSearchNode( d, s-5, t+1, -5 );
		                astarsearch->AddSuccessor( NewNode );
                            }
                        } 
                    }
                }
	    }	
	}	
	
	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
       return pre->GetMap( d, s, t );
}
#endif

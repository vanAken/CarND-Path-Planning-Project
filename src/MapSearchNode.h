#ifndef PATH_PLANNING_MAPSEARCHNODE_H
#define PATH_PLANNING_MAPSEARCHNODE_H

#include "stlastar.h" // See header for copyright and usage information
#include <iostream>
#include <stdio.h>
#include <math.h>

class MapSearchNode
{
public:
	double s;   // s,d,v,t - state of the node
	int d;	
        double v;   
        int t;
        int c;      // cost stat: can be increased for lanchange for example
 
        //std::shared_ptr<Prediction> _pre; // = trajectory to use GetMap
	//MapSearchNode(std::shared_ptr<Prediction> preditcion) : _pre(preditcion) { d = s = t = v = 0; }
	MapSearchNode() { s = d = v = t = c= 0; }
	MapSearchNode( double ps, int pd, double pv,  int pt,  int pc ) 
                     {      s=ps;   d=pd;      v=pv;    t=pt;    c=pc;}

	double GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool   IsGoal( MapSearchNode &nodeGoal );
	bool   GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
        double GetCost( MapSearchNode &successor );
	bool   IsSameState( MapSearchNode &rhs );

	int    GetNode_s();
	int    GetNode_d();
	double GetNode_v();
	int    GetNode_t();
};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
	// same state in a maze search is simply when (s,d,v,t)
	if( ((int)s == (int)rhs.s) 
            && (d == rhs.d) 
            && (t == rhs.t) 
//  	    && ((int)100*v == (int)100*rhs.v)
                                             )
	        return true;
	return false;
}

int MapSearchNode::GetNode_s()
{
        return s;
}
int MapSearchNode::GetNode_d()
{
        return d;
}
double MapSearchNode::GetNode_v()
{
        return v;
}
int MapSearchNode::GetNode_t()
{
        return t;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

double MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	return abs(s - nodeGoal.s) + 
               abs(d - nodeGoal.d) + 
               abs(t - nodeGoal.t); // t is importand
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{
	if( //( d == nodeGoal.d) && // only S as goleline?
            ( s >= nodeGoal.s) ){ // s is double
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
    MapSearchNode NewNode;
   
    double v_add = min(::d_v_max, v + ::d_a_max); // d_v_max is 1.1     
    double v_sub = max(  0.0    , v - ::d_a_max); // a_max is 0.1        
    double s_add = s + v + ::d_a_max/2 ;
    double s_sub = s + v - ::d_a_max/2 ;
   
    // accelarating nodes for 0-2 areas
    if ( GetMap( d, s+0, t+1) < 9){   // if this is free
        if (s_add - s < 1 ){         // same area
            NewNode = MapSearchNode( s_add, d, v_add, t+1, -1);
            astarsearch->AddSuccessor( NewNode );
            NewNode = MapSearchNode( s_sub, d, v_sub, t+1, -1); 
            astarsearch->AddSuccessor( NewNode );
        }
        if ( GetMap( d, s+1, t+1) < 9){   // if this is free
            if (s_add - s < 2 ){         // next area
                NewNode = MapSearchNode( s_add, d, v_add, t+1, 0);
                astarsearch->AddSuccessor( NewNode );
                if ( GetMap( d+1, s+0, t+1) < 9
                  && GetMap( d+1, s+1, t+1) < 9
                  && GetMap( d+1, s+2, t+1) < 9){ // left fields free
                    NewNode = MapSearchNode( s_add, d+1, v_add, t+1, 1); 
                    astarsearch->AddSuccessor( NewNode );
                    NewNode = MapSearchNode( s_sub, d+1, v_sub, t+1, 1); 
                    astarsearch->AddSuccessor( NewNode );
                }
                if ( GetMap( d-1, s+0, t+1) < 9
                  && GetMap( d-1, s+1, t+1) < 9
                  && GetMap( d-1, s+2, t+1) < 9){ // right fields free
                    NewNode = MapSearchNode( s_add, d-1, v_add, t+1, 1); 
                    astarsearch->AddSuccessor( NewNode );
                    NewNode = MapSearchNode( s_sub, d-1, v_sub, t+1, 1); 
                    astarsearch->AddSuccessor( NewNode );
                }
            }
            if ( GetMap( d, s+2, t  ) < 9
              && GetMap( d, s+2, t+1) < 9){ // if this is free
                if (s_add - s < 3 ){         // OVER next area
                    NewNode = MapSearchNode( s_add, d, v_add, t+1, GetMap( d, s+1, t+1));
                    astarsearch->AddSuccessor( NewNode );
                    NewNode = MapSearchNode( s_sub, d, v_sub, t+1, GetMap( d, s+1, t+1)); 
                    astarsearch->AddSuccessor( NewNode );
                }
            }
        }
    }    
    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

double MapSearchNode::GetCost( MapSearchNode &successor )
{                   // Map cost + nodecost from lanchange
       return GetMap( d, s, t ) + c ;
}
#endif

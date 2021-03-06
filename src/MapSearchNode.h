#ifndef PATH_PLANNING_MAPSEARCHNODE_H
#define PATH_PLANNING_MAPSEARCHNODE_H

#include "stlastar.h" // See header for copyright and usage information

class MapSearchNode
{
public:
	int s;   // s,d,v,t - state of the node
	int d;	
        int v;   
        int t;
        int c;      // cost stat: can be increased for lanchange for example
        int l;   // counter lanchange
 
	MapSearchNode() { s = d = v = t = c = l = 0;}
	MapSearchNode( int ps, int pd, int pv,  int pt,  int pc ,  int pl ) 
                     {   s=ps;   d=pd;   v=pv;    t=pt;    c=pc;    l=pl;}

	double GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool   IsGoal( MapSearchNode &nodeGoal );
	bool   GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
        double GetCost( MapSearchNode &successor );
	bool   IsSameState( MapSearchNode &rhs );

	int    GetNode_s();
	int    GetNode_d();
	int    GetNode_v();
	int    GetNode_t();
};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
	// same state in a maze search is simply when (s,d,v,t,c)
	if((    s == rhs.s) 
            && (d == rhs.d) 
   	    && (v == rhs.v)
            && (t == rhs.t)
            && (c == rhs.c)
            && (l == rhs.l))
	        return true;
	return false;
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
int MapSearchNode::GetNode_t(){
        return t;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

double MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	return abs(s - nodeGoal.s) +  
//             abs(d - nodeGoal.d) + 
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
    
    // store lanechange cost
    int l1 = GetMap(d+1,s+2,t+1);
    int l2 = GetMap(d+1,s+2,t+1);
    int l3 = GetMap(d+1,s+3,t+1);
    int l4 = GetMap(d+1,s+4,t+1);
    int l5 = GetMap(d+1,s+5,t+1);
    int l6 = GetMap(d+1,s+6,t+1);

    int r1 = GetMap(d-1,s+2,t+1);
    int r2 = GetMap(d-1,s+2,t+1);
    int r3 = GetMap(d-1,s+3,t+1);
    int r4 = GetMap(d-1,s+4,t+1);
    int r5 = GetMap(d-1,s+5,t+1);
    int r6 = GetMap(d-1,s+6,t+1);

    const int l_cost = 4;
    if (l>0) l -= 1;
       

    int cost_so_far = 0;          // store cost  
    int m = GetMap( d, s+0, t+1); // cost from map - car = 8 or 9  
    if (m < 8){                                          // is free ?
        if (v <= 2 ){                                    // can be reachted by braking from v=2
            NewNode = MapSearchNode( s+0, d, 0, t+1, m*2, 0); // and can be same area
            astarsearch->AddSuccessor( NewNode );
        }   // no move => no lanechange no increase cost_so_far
    }
    m =         GetMap( d ,s+1,t+1);
    if (m < 8){
        cost_so_far = m;          // the valu from the map are the cost 
        if (v <= 3){               
            NewNode     = MapSearchNode( s+1, d  , 1, t+1, cost_so_far+1, 0); // next area (s,d,v,t,c,l)
            astarsearch->AddSuccessor( NewNode );
        }
        m = GetMap( d ,s+2,t+1);
        if (m < 8){ 
            cost_so_far += m;     // add the cost from previous area to the new one 
            if (0 <= v && v <= 4){       
                NewNode     = MapSearchNode( s+2, d  , 2, t+1, cost_so_far, 0);
                astarsearch->AddSuccessor( NewNode );
                if ( l1 < 8 && l2 < 8 && l3 < 8){                                         // lanechange left 
                    NewNode = MapSearchNode( s+2, d+1, 2, t+1, cost_so_far+r2+l*l,l_cost); //cost extra 
                    astarsearch->AddSuccessor( NewNode );
                }
                if ( r1 < 8 && r2 < 8 && r3 < 8){                                         // lanechange right
                    NewNode = MapSearchNode( s+2, d-1, 2, t+1, cost_so_far+r2+l*l,l_cost); //cost extra 
                    astarsearch->AddSuccessor( NewNode );
                }
            }
            m = GetMap( d ,s+3,t+1);
            if (m < 8){ 
                cost_so_far += m;
                if (1 <= v && v <= 5){        
                    NewNode     = MapSearchNode( s+3, d  , 3, t+1, cost_so_far, 0);
                    astarsearch->AddSuccessor( NewNode );
                    if ( l2 < 8 && l3 < 8 && l4 < 8){                                         // lanechange left 
                        NewNode = MapSearchNode( s+3, d+1, 3, t+1, cost_so_far+l3+l*l,l_cost); //cost extra 
                        astarsearch->AddSuccessor( NewNode );
                    }
                    if ( r2 < 8 && r3 < 8 && r4 < 8){                                         // lanechange right
                        NewNode = MapSearchNode( s+3, d-1, 3, t+1, cost_so_far+r3+l*l,l_cost); //cost extra 
                        astarsearch->AddSuccessor( NewNode );
                    }
                }
                m = GetMap( d ,s+4,t+1);
                if (m < 8){ 
                    cost_so_far += m;
                    if (2 <= v){        
                        NewNode     = MapSearchNode( s+4, d  , 4, t+1, cost_so_far, 0);
                        astarsearch->AddSuccessor( NewNode );
                        if ( l3 < 8 && l4 < 8 && l5 < 8){                                         // lanechange left 
                            NewNode = MapSearchNode( s+4, d+1, 4, t+1, cost_so_far+l4+l*l,l_cost); //cost extra 
                            astarsearch->AddSuccessor( NewNode );
                        }  
                        if ( r3 < 8 && r4 < 8 && r5 < 8){                                         // lanechange right
                            NewNode = MapSearchNode( s+4, d-1, 4, t+1, cost_so_far+r4+l*l,l_cost); //cost extra 
                            astarsearch->AddSuccessor( NewNode );
                        }
                    }
                    m = GetMap( d ,s+5,t+1);// position 5  
                    if (m < 8){ 
                        cost_so_far += m;
                        if (3 <= v){    
                            NewNode     = MapSearchNode( s+5, d  , 5, t+1, cost_so_far, 0);
                            astarsearch->AddSuccessor( NewNode );
                            if ( l4 < 8 && l5 < 8 && l6 < 8){                                         // lanechange left 
                                NewNode = MapSearchNode( s+5, d+1, 5, t+1, cost_so_far+l5+l*l,l_cost); //cost extra 
                                astarsearch->AddSuccessor( NewNode );
                            }
                            if ( r4 < 8 && r5 < 8 && r6 < 8){                                         // lanechange right
                                NewNode = MapSearchNode( s+5, d-1, 5, t+1, cost_so_far+r5+l*l,l_cost); //cost extra 
                                astarsearch->AddSuccessor( NewNode );
                            }
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

double MapSearchNode::GetCost( MapSearchNode &successor )
{                   // cost for map, lanchange and velocity
       // cout << "c: " << c  endl;
       return c+5-2*v;
}
#endif

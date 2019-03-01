////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information
#include "Map3D.h"

#include <iostream>
#include <stdio.h>
#include <math.h>


#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 1
 

using namespace std;


// map helper functions

int GetMap( int d, int s, int t )
{
	if( d < 0 ||
            d >= WIDTH_D ||
                 s < 0 || 
                 s >= HORIZONT  || 
                    t >= TIME )
	{
		return 9;	 
	}

	return world_map[(d + s*WIDTH_D + t * WIDTH_D * HORIZONT)];
}



// Definitions

class MapSearchNode
{
public:
	int d;	 // the (d,s,t,v) state of the node
	int s;	
        int t;
        int v;   // stat variable form speed

        int v_max = +5;
        int v_min = -5;
	
	MapSearchNode() { d = s = t = v = 0; }
	MapSearchNode( int pd, int ps, int pt , int pv ) { d=pd; s=ps; t=pt; v=pv; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo(); 
};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
	// same state in a maze search is simply when (d,s,t) are the same not v!
	if( (d == rhs.d) &&
            (s == rhs.s) &&
  	    (t == rhs.t) )
	        return true;
	else    return false;
}

void MapSearchNode::PrintNodeInfo()
{
	char str[100];
	sprintf( str, "Node position : (%d,%d,%d,%d)\n", d,s,t,v );
	cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	return abs(d - nodeGoal.d) + abs(s - nodeGoal.s) + abs(t - nodeGoal.t); // t is importand!!!
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( (d == nodeGoal.d) &&
		(s == nodeGoal.s) )
	{
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

        
	if (-2<=v && v<=2){ // traverse move possible only with v=0
            if (GetMap( d, s, t+1) < 9){    // no move with const. v just wait
		NewNode = MapSearchNode( d, s, t+1, 0 );
		astarsearch->AddSuccessor( NewNode );
	    }	            
            if (GetMap( d+1, s, t+1) < 9){  // lanechange: left and right           
		NewNode = MapSearchNode( d+1, s, t+1, 0 );
		astarsearch->AddSuccessor( NewNode );
            }
            if (GetMap( d-1, s, t+1) < 9){            
		NewNode = MapSearchNode( d-1, s, t+1, 0 );
		astarsearch->AddSuccessor( NewNode );
            }
	}	
	
        // go futher in s
        if (GetMap( d, s+1, t) < 9){
	    if ( v<=v_max && -1<=v && v<=3 ){ 
                NewNode = MapSearchNode( d, s+1, t+1, +1 );
		astarsearch->AddSuccessor( NewNode );
            }
            if (GetMap( d, s+2, t) < 9 ){
                if( v<=v_max && 0<=v && v<=4){
		    NewNode = MapSearchNode( d, s+2, t+1, +2 );
		    astarsearch->AddSuccessor( NewNode );
                }
                if (GetMap( d, s+3, t) < 9 &&
                    GetMap( d, s+4, t) < 9 ){
	            if ( v<=v_max && 1<=v && v<=5){ 
                        NewNode = MapSearchNode( d, s+3, t+1, +3 );
		        astarsearch->AddSuccessor( NewNode );
                    }
                    if (GetMap( d, s+5, t) < 9 && 
                        GetMap( d, s+6, t) < 9 ){
                        if( v<=v_max && 2<=v && v<=6){
		            NewNode = MapSearchNode( d, s+4, t+1, +4 );
 		            astarsearch->AddSuccessor( NewNode );
                        }
                        if (GetMap( d, s+7, t) < 9 && 
                            GetMap( d, s+8, t) < 9 && 
                            GetMap( d, s+9, t) < 9){
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
	if (GetMap( d, s-1, t) < 9){
	    if ( v_min<=v && -3<v && v<=1 ){ 
                NewNode = MapSearchNode( d, s-1, t+1, -1 );
		astarsearch->AddSuccessor( NewNode );
            }
            if (GetMap( d, s-2, t) < 9){
                if( v_min<=v && -4<=v && v<=0){
		    NewNode = MapSearchNode( d, s-2, t+1, -2 );
		    astarsearch->AddSuccessor( NewNode );
                }
                if (GetMap( d, s-3, t) < 9 &&
                    GetMap( d, s-4, t) < 9 ){
	            if ( v_min<=v && -5<=v && v<=-1){ 
                        NewNode = MapSearchNode( d, s-3, t+1, -3 );
		        astarsearch->AddSuccessor( NewNode );
                    }
                    if (GetMap( d, s-5, t) < 9 &&
                        GetMap( d, s-6, t) < 9 ){
                        if( v_min<=v && -6<=v && v<=-2){
		            NewNode = MapSearchNode( d, s-4, t+1, -4 );
 		            astarsearch->AddSuccessor( NewNode );
                        }
                        if (GetMap( d, s-7, t) < 9 && 
                            GetMap( d, s-8, t) < 9 && 
                            GetMap( d, s-9, t) < 9){
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
       float cost = GetMap( d, s, t )  ;
        
       return cost;

}


// Main

int main( int argc, char *argv[] )
{

	cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

	// Our sample problem defines the world as a 2d array representing a terrain
	// Each element contains an integer from 0 to 5 which indicates the cost 
	// of travel across the terrain. Zero means the least possible difficulty 
	// in travelling (think ice rink if you can skate) whilst 5 represents the 
	// most difficult. 9 indicates that we cannot pass.

	// Create an instance of the search class...

	AStarSearch<MapSearchNode> astarsearch;

	unsigned int SearchCount = 0;

	const unsigned int NumSearches = 1;

	while(SearchCount < NumSearches)
	{

		// Create a start state
		MapSearchNode nodeStart;
		nodeStart.d = 1;
		nodeStart.s = 0; 

		// Define the goal state
		MapSearchNode nodeEnd;
		nodeEnd.d = 1;						
		nodeEnd.s = HORIZONT-1; 
		
		// Set Start and goal states
		
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		do
		{
			SearchState = astarsearch.SearchStep();

			SearchSteps++;

	#if DEBUG_LISTS

			cout << "Steps:" << SearchSteps << "\n";

			int len = 0;

			cout << "Open:\n";
			MapSearchNode *p = astarsearch.GetOpenListStart();
			while( p )
			{
				len++;
	#if !DEBUG_LIST_LENGTHS_ONLY			
				((MapSearchNode *)p)->PrintNodeInfo();
	#endif
				p = astarsearch.GetOpenListNext();
				
			}

			cout << "Open list has " << len << " nodes\n";

			len = 0;

			cout << "Closed:\n";
			p = astarsearch.GetClosedListStart();
			while( p )
			{
				len++;
	#if !DEBUG_LIST_LENGTHS_ONLY			
				p->PrintNodeInfo();
	#endif			
				p = astarsearch.GetClosedListNext();
			}

			cout << "Closed list has " << len << " nodes\n";
	#endif

		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			cout << "Search found goal state\n";

				MapSearchNode *node = astarsearch.GetSolutionStart();

	#if DISPLAY_SOLUTION
				cout << "Displaying solution\n";
	#endif
				int steps = 0;

				node->PrintNodeInfo();
				for( ;; )
				{
					node = astarsearch.GetSolutionNext();

					if( !node )
					{
						break;
					}

					node->PrintNodeInfo();
					steps ++;
				
				};

				cout << "Solution steps " << steps << endl;

				// Once you're done with the solution you can free the nodes up
				astarsearch.FreeSolutionNodes();

	
		}
		else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
		{
			cout << "Search terminated. Did not find goal state\n";
		
		}

		// Display the number of loops the search went through
		cout << "SearchSteps : " << SearchSteps << "\n";

		SearchCount ++;

		astarsearch.EnsureMemoryFreed();
	}
	
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

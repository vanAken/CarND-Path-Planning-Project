#include <iostream>
#include <cmath>
#include "prediction.h"
#include "MapSearchNode.h"

// the inilizing of of Perdition discrete the ego and sensorfusion data
Prediction::Prediction(double ego_s, double d_dt, vector<vector<double>> sensor_fusion)
{   
    // clear the old pushed back values
    ::next_s.clear();
    ::next_d.clear();
    ::next_v.clear();    
   
    // convert the other cars into discrete areas
    vector<int> d_other_s, d_other_d, d_o_d_pos, d_o_d_neg;
    vector<double> d_other_v; // continiuos velocity 
    for(int i = 0; i < sensor_fusion.size(); i++) {      
        d_other_s.push_back( discrete2s( sensor_fusion[i][5], ego_s ));
        d_other_d.push_back( discrete2d( sensor_fusion[i][6]));
        d_o_d_pos.push_back( discrete2d( sensor_fusion[i][6]+1)); // possiblel left  lanechange
        d_o_d_neg.push_back( discrete2d( sensor_fusion[i][6]-1)); // possiblel right lanechange
        //    v == vs!                v= distance(0,0,    vx             ,    vy             ) 
        d_other_v.push_back( discrete2v( distance(0,0,sensor_fusion[i][3],sensor_fusion[i][4]), d_dt) ); 
    }

    // discrete 2D(t) road(t) - filled with 1
    for (int i = 0; i < ::num_of_lanes * ::d_horizont_s * ::d_horizont_t; i++) {  
        ::time_road[i]= 1;
    } 

    //place the other cars in the road for each timestep
    for (int t = 0; t < ::d_horizont_t; t++){
    // Predict the relative position of the other cars, for each time layer of the road
        for(int i = 0; i < sensor_fusion.size(); i++) {
            int d_pre_o_s = d_other_s[i] + d_other_v[i] * t; // discrete prediction in s + t * v 
            if (0 < d_pre_o_s && d_pre_o_s < ::d_horizont_s){  // is d_pre is within the horizont? 
                int pointer_pre_s = d_pre_o_s * ::num_of_lanes + t * ::num_of_lanes * ::d_horizont_s;
               // Potentialfield around each other car befor car
                if (d_pre_o_s+2 < ::d_horizont_s){ // d_pre +2 also on the map ? inc area infront
                    ::time_road [ d_other_d[i] + pointer_pre_s + ::num_of_lanes*3] += 1;
                    ::time_road [ d_other_d[i] + pointer_pre_s + ::num_of_lanes*2] += 3;
                    ::time_road [ d_other_d[i] + pointer_pre_s + ::num_of_lanes*1] += 4;
                }
                if (0 < d_pre_o_s-2){ // d_pre -2 also on the map ? inc area backwards
                    ::time_road [ d_other_d[i] + pointer_pre_s - ::num_of_lanes*1] += 4;
                    ::time_road [ d_other_d[i] + pointer_pre_s - ::num_of_lanes*2] += 3;
                    ::time_road [ d_other_d[i] + pointer_pre_s - ::num_of_lanes*3] += 1;
                }  
                 if (d_other_d[i]>0) // add cost on the right side if not lane 0 
                    ::time_road [ d_other_d[i]-1 + pointer_pre_s] += 1;
            } 
        }
        for(int i = 0; i < sensor_fusion.size(); i++) {
            int d_pre_o_s = d_other_s[i] + d_other_v[i] * t; // discrete prediction in s + t * v 
            if (0 < d_pre_o_s && d_pre_o_s < ::d_horizont_s){  // is d_pre is within the horizont? 
                int pointer_pre_s = d_pre_o_s * ::num_of_lanes + t * ::num_of_lanes * ::d_horizont_s;
             //   ::time_road [ d_other_d[i] + pointer_pre_s + ::num_of_lanes] = 9;
                ::time_road [ d_other_d[i] + pointer_pre_s                 ] = 9;
             //   ::time_road [ d_other_d[i] + pointer_pre_s - ::num_of_lanes] = 9;
                // maybe d_other_d plus or minus 1 is different, so set it again or left - no if
                ::time_road [ d_o_d_pos[i] + pointer_pre_s] = 9;
                ::time_road [ d_o_d_neg[i] + pointer_pre_s] = 9;
            } 
        }
    } 
}

Prediction::~Prediction(){}

void Prediction::search(double ego_s, double ego_d, double ego_v, double v_max, double a_max, double d_dt) {
   
    // Create an instance of the search class A*
    AStarSearch<MapSearchNode> astarsearch;

    unsigned int       SearchCount = 0;
    const unsigned int NumSearches = 1;
    while(SearchCount < NumSearches){
        // Create a start state
        MapSearchNode nodeStart;
        nodeStart.s = ::offset_s; // start in the midle of the first area
        nodeStart.d = discrete2d( ego_d);
        nodeStart.v = discrete2v( ego_v, d_dt);// double! discrete velocity with repect of d_dt and discrete

        d_v_max = discrete2v( v_max, d_dt );        // convert discrete velocity
        d_a_max = discrete2v( a_max, d_dt ) * d_dt; // sam as v, but one more d_dt [m/s^2]
        
        // Define the goal state
        MapSearchNode nodeEnd;
        nodeEnd.d = nodeStart.d;						
        nodeEnd.s = ::d_horizont_s - 1; 
	
        // Set Start and goal states
        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
        unsigned int SearchState;
        unsigned int SearchSteps = 0;
          do  {   SearchState = astarsearch.SearchStep();
                SearchSteps++;
            }
            while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
            if   ( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED ){

                cout << "Search found goal state\n";
                MapSearchNode *node = astarsearch.GetSolutionStart();
                int steps    = 0;
                ::time_road[node->GetNode_d()       // and store Start to the result map
                          + node->GetNode_s() * ::num_of_lanes
                          + node->GetNode_t() * ::num_of_lanes * ::d_horizont_s] = 0;                 

                for(; ; ){ // endless
                    node = astarsearch.GetSolutionNext();
                    if( !node ) break;
 
                       ::time_road[node->GetNode_d()       // and store Start to the result map
                                 + node->GetNode_s() * ::num_of_lanes
                                 + node->GetNode_t() * ::num_of_lanes * ::d_horizont_s] = 0;                 
                       if(steps == 5){//% 3 == 0 && 6 <= steps  ){
                       //     ::next_s.push_back( continuous2s (node->GetNode_s(), ego_s) );
                       //     ::next_d.push_back( continuous2d (node->GetNode_d()       ) );
                       //     ::next_v.push_back( continuous2v (node->GetNode_v(), d_dt ) );
                       double car_d = continuous2d( node->GetNode_d() );
                       double car_v = continuous2v( node->GetNode_v(), d_dt );
                       ::next_s = { ego_s+32,ego_s+64,ego_s+96 ,ego_s+128,ego_s+160,ego_s+192,ego_s+224,ego_s+256 };
                       ::next_d = { car_d   ,car_d   ,car_d    ,car_d   ,car_d    ,car_d     ,car_d    ,car_d     };
                       ::next_v = { car_v   ,car_v   ,car_v    ,car_v   ,car_v    ,car_v     ,car_v    ,car_v     };     
                       }
                    steps ++;
                }
                cout << "Solution steps " << steps << endl;
                cout << "Solution cost " << astarsearch.GetSolutionCost() << endl;
                cout << "next_s[0]: "<< next_s[0] - ego_s << "next_s[1]" << next_s[1] - ego_s << std::endl;
                // Once you're done with the solution you can free the nodes up
                astarsearch.FreeSolutionNodes();

	
            }
            else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) {
                cout << "Search terminated. Did not find goal state\n";
                cout << "############################################# "<< endl;
		cout << "############################################# "<< endl;
                double car_d = 2 ;
                double car_v = 15 ;
                ::next_s = { ego_s+32,ego_s+64,ego_s+96 ,ego_s+128,ego_s+160,ego_s+192,ego_s+224,ego_s+256 };
                ::next_d = { car_d   ,car_d   ,car_d    ,car_d   ,car_d    ,car_d     ,car_d    ,car_d     };
                ::next_v = { car_v   ,car_v   ,car_v    ,car_v   ,car_v    ,car_v     ,car_v    ,car_v     };  
                
	    }
		// Display the number of loops the search went through
              cout << "SearchSteps : " << SearchSteps << "\n";
		SearchCount ++;
		astarsearch.EnsureMemoryFreed();


             
	} 	
} 

// 6 Funtion to discrete sdv and bring back sdv "2" continous action space
 
int Prediction::discrete2s(double s, double ego_s){ // set ego_s-rearview to zero, every discrete meter an area
    int result = int( (s-ego_s + ::discrete/2 ) / ::discrete )+offset_s;
    if (result > 1000) result -= 1732; // howerver 6945,554÷4 = 1736,3885  
    if (result <-1000) result += 1732; // but value jumps from 1732-1731 
    return result;
}
double Prediction::continuous2s(int s, double ego_s) {          // back to continous track s
    return (double) (s-offset_s) * ::discrete + ego_s;
}

int Prediction::discrete2d(double d) {  
    const int road_width = 12;          // left lane is 2
    const int lane_width =  4;         // right lane is 0  
    int result = int(( road_width - d) / lane_width );
    result = max(result, 0);                      // cut to the left
    result = min(result, ::num_of_lanes-1);       // cut to the right
    return result;
}
double Prediction::continuous2d(int d) {          // back to continous track d 
    const int road_offset = 10.;                   // distance to the outside right midlane
    const int lane_width = 4.;  
    return (road_offset - d * lane_width); 
}

double Prediction::discrete2v(double v, double d_dt) { // every discrete m/s one area                      
    return ( (int) v * d_dt/::discrete *10 +.5)/10. ;    
}
double Prediction::continuous2v(double v, double d_dt) { // back to continous track v  
    return ( v * ::discrete/d_dt);
}


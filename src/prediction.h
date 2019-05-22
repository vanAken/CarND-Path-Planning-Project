#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include "MapSearchNode.h"

class Prediction {

public:
    Prediction(double ego_s, double pre_t, vector<vector<double>> sensor_fusion,  
               vector<double>previous_path_x,vector<double>previous_path_y, Frenet frenet);
    ~Prediction();

    void search (double ego_s, double ego_d, double ego_v, double v_max, double a_max, vector<vector<double>> sensor_fusion, double d_dt);

    vector<double> next_s, next_d, next_v;

};

// ---------------------------------------------------------------------
// implementation part, which could be separated into a cpp file
// ---------------------------------------------------------------------

Prediction::Prediction(double ego_s, double d_dt, vector<vector<double>> sensor_fusion,  
               vector<double>previous_path_x,vector<double>previous_path_y, Frenet frenet)
{   
    // place the position of the other cars into the discrete time road
    vector<int> d_other_s, d_other_d, d_o_d_pos, d_o_d_neg;
    vector<double> d_other_v; // continuous velocity 
    for(int i = 0; i < sensor_fusion.size(); i++) {      
        d_other_s.push_back( ::discrete_to_s( sensor_fusion[i][5], ego_s ));
        d_other_d.push_back( ::discrete_to_d( sensor_fusion[i][6]));
        d_o_d_pos.push_back( ::discrete_to_d( sensor_fusion[i][6]+1)); // possible left  lane change
        d_o_d_neg.push_back( ::discrete_to_d( sensor_fusion[i][6]-1)); // possible right lane change
        //         v == vs!                v= distance(0,0,    vx             ,    vy             ) 
        d_other_v.push_back( ::discrete_to_v( distance(0,0,sensor_fusion[i][3],sensor_fusion[i][4])) ); 
    }

    // discrete 2D(t) road(t) - filled with 1
    for (int i = 0; i < ::num_of_lanes * ::d_horizont_s * ::d_horizont_t; i++) {  
        ::time_road[i]= 1;
    } 

    // write the pre path as 0 to reduce A* flicker
    for (int i = 0; i < previous_path_x.size(); i++) { 
         vector<double> previous_sd;
         previous_sd = frenet.xy_to_sd(previous_path_x[i],previous_path_y[i]);
         int previous_s = ::discrete_to_s( previous_sd[0],ego_s);
         int previous_d = ::discrete_to_d( previous_sd[1]);
         if (previous_s < ::d_horizont_s){
             int BMW=0;
             if  (previous_d==0 ){ 
                 BMW=1;
                 if  (previous_s > 15) BMW=2;
             } // BMW adds costs for keeping lane 0.
             ::time_road[previous_d +
                         previous_s * ::num_of_lanes +
                         (i+49)/49  * ::num_of_lanes * ::d_horizont_s]= BMW;
         }
    } 

    
    //place the other cars etc. in the time road for each future secound
    for (int t = 0; t < ::d_horizont_t; t++){
    // Predict the relative position of the other cars, for each time layer of the road
        for(int i = 0; i < sensor_fusion.size(); i++) {
            int d_pre_o_s = d_other_s[i] + d_other_v[i] * t;   // discrete prediction in s + t * v 
            if (0 < d_pre_o_s && d_pre_o_s < ::d_horizont_s){  // is d_pre is within the horizont? 
                int pointer_pre_s = d_pre_o_s * ::num_of_lanes + t * ::num_of_lanes * ::d_horizont_s;
               // Potential field around each other car 
                if (d_pre_o_s+2 < ::d_horizont_s){
                    ::time_road [ d_other_d[i] + pointer_pre_s + ::num_of_lanes*2] += 3;
                    if (d_pre_o_s+3 < ::d_horizont_s){
                        ::time_road [ d_other_d[i] + pointer_pre_s + ::num_of_lanes*3] += 3;
                        if (d_pre_o_s+4 < ::d_horizont_s){
                            ::time_road [ d_other_d[i] + pointer_pre_s + ::num_of_lanes*4] += 2;
                            if (d_pre_o_s+5 < ::d_horizont_s){
                                ::time_road [ d_other_d[i] + pointer_pre_s + ::num_of_lanes*5] += 1;
                            }   
                        }   
                    }   
                }
                if (0 <= d_pre_o_s-2){
                    ::time_road [ d_other_d[i] + pointer_pre_s - ::num_of_lanes*2] += 3;
                    if (0 <= d_pre_o_s-3){
                        ::time_road [ d_other_d[i] + pointer_pre_s - ::num_of_lanes*3] += 3;
                        if (0 <= d_pre_o_s-4){
                            ::time_road [ d_other_d[i] + pointer_pre_s - ::num_of_lanes*4] += 2;
                            if (0 <= d_pre_o_s-5){
                                ::time_road [ d_other_d[i] + pointer_pre_s - ::num_of_lanes*5] += 1;
                            }   
                        }   
                    }   
                }
                if (d_other_d[i]==1){  // add extra cost on the right side of other cars if lane ==1
                    ::time_road [ pointer_pre_s] += 2;
                }  
            }  
        }
        for(int i = 0; i < sensor_fusion.size(); i++) {       // place the other cars
            int d_pre_o_s = d_other_s[i] + d_other_v[i] * t;  // discrete prediction in s + t * v 
            if (0 < d_pre_o_s && d_pre_o_s < ::d_horizont_s){ // is d_pre is within the horizont? 
                int pointer_pre_s = d_pre_o_s * ::num_of_lanes + t * ::num_of_lanes * ::d_horizont_s;
                if (d_pre_o_s+1 < ::d_horizont_s){
                    ::time_road [ d_other_d[i] + pointer_pre_s +  ::num_of_lanes] = 8;
                    ::time_road [ d_o_d_pos[i] + pointer_pre_s +  ::num_of_lanes] = 8;
                    ::time_road [ d_o_d_neg[i] + pointer_pre_s +  ::num_of_lanes] = 8;
                }
                ::time_road [ d_other_d[i] + pointer_pre_s] = 9;
                ::time_road [ d_o_d_pos[i] + pointer_pre_s] = 9;
                ::time_road [ d_o_d_neg[i] + pointer_pre_s] = 9;
                if (0 <= d_pre_o_s-1){
                    ::time_road [ d_other_d[i] + pointer_pre_s -  ::num_of_lanes] = 8;
                    ::time_road [ d_o_d_pos[i] + pointer_pre_s -  ::num_of_lanes] = 8;
                    ::time_road [ d_o_d_neg[i] + pointer_pre_s -  ::num_of_lanes] = 8;
                }
            } 
        }
    } 
}

Prediction::~Prediction(){}

void Prediction::search(double ego_s, double ego_d, double ego_v, double v_max, double a_max, vector<vector<double>> sensor_fusion, double d_dt) {

    // Create an instance of the search class A*
    AStarSearch<MapSearchNode> astarsearch;

    unsigned int       SearchCount = 0;
    const unsigned int NumSearches = 1;
    while(SearchCount < NumSearches){
        // Create a start state
        MapSearchNode nodeStart;
        nodeStart.s = 0; // start in the midle of the first area
        nodeStart.d = ::discrete_to_d( ego_d);      // discrete lane 
        nodeStart.v = ::discrete_to_v( ego_v);// discrete velocity 
        
        // Define the goal state
        MapSearchNode nodeEnd;
        nodeEnd.d = nodeStart.d;						
        nodeEnd.s = ::d_horizont_s - 1; 
	
        // Set Start and goal states
        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
        unsigned int SearchState;
        unsigned int SearchSteps = 0;
        do{ SearchState = astarsearch.SearchStep();
            SearchSteps++;
        }
        while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
        if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ){
            // Define the new goal state
            nodeEnd.s = ::d_horizont_s / 2; 
            cout << "Search terminated. Try a shorter horizont: " << ::d_horizont_s-1 << " ==> " <<  nodeEnd.s << endl;
	    do{ SearchState = astarsearch.SearchStep();
                SearchSteps++;
            }
            while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
	}
        if  ( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED ){
            cout << "Search found goal state\n";
            MapSearchNode *node = astarsearch.GetSolutionStart();
            int steps   = 0;
            int pre_d_s = 0; 
            int pre_d_d = 1; 
            ::time_road[node->GetNode_d()       // and store Start in the result map
                      + node->GetNode_s() * ::num_of_lanes
                      + node->GetNode_t() * ::num_of_lanes * ::d_horizont_s] = 99;                 
            for(; ; ){ // endless
                node = astarsearch.GetSolutionNext();
                if( !node ) break;
                int d_s = node->GetNode_s(); // discrete results of A*
                int d_d = node->GetNode_d();
                int d_v = node->GetNode_v();
                int d_t = node->GetNode_t();
                ::time_road[ d_d       // and store next node in the result map
                           + d_s * ::num_of_lanes
                           + d_t * ::num_of_lanes * ::d_horizont_s] = 99;                 
                if( pre_d_s < d_s ){   // filter same d_s
                    if ( 2 < (pre_d_d-d_d)*(pre_d_d-d_d) ){
                        d_d = 1;
                    }
                    double s = continuous_to_s (d_s, ego_s ); // continuous results of A*
                    double d = continuous_to_d (d_d        );
                    double v = continuous_to_v (d_v, v_max );
                    next_s.push_back( s + v ); // transfer to result
                    next_d.push_back( d );
                    next_v.push_back( v );
                }
                pre_d_s = d_s;
                steps ++;
            }
            cout << "Solution steps " << steps << endl;
            cout << "Solution cost " << astarsearch.GetSolutionCost() << endl;
            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();	
        }	
        else{ // follow mode: speed will slow down by the contoller
            cout << "Search terminated. Follower mode " << endl;;
            double c_next_d  = ::continuous_to_d( nodeStart.d );
            double c_next_v = v_max;
            for(int i=2; i<::d_horizont_s; i+=2){
                next_s.push_back( ego_s + i*discrete ); // transfer to result
                next_d.push_back( c_next_d );
                next_v.push_back( c_next_v );
            }
            cout << "c_next_v "<< c_next_v << "c_next_d " << c_next_d <<endl;
        }
        // Display the number of loops the search went through
        cout << "SearchSteps : " << SearchSteps << "\n";
	SearchCount ++;
	astarsearch.EnsureMemoryFreed();               
    } // end while	
}

#endif


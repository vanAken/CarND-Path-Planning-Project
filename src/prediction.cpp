#include <iostream>
#include <cmath>
#include "prediction.h"
#include "MapSearchNode.h"

// the inilizing of of Perdition discrete the ego and sensorfusion data
Prediction::Prediction(double ego_s,double ego_d,double ego_v,vector<vector<double>> sensor_fusion)
        :_ego_s(ego_s), _ego_d(ego_d), _ego_v(ego_v){

    
    // convert the ego car into discrete areas
    d_ego_s = discrete2s (ego_s);     
    d_ego_d = discrete2d (ego_d);
    d_ego_v = discrete2v (ego_v);

    // convert the other cars into discrete areas
    vector<int> d_other_s, d_other_d, d_o_d_pos, d_o_d_neg;
    vector<double> c_other_v; // continiuos_spped / discrete 
    for(int i = 0; i < sensor_fusion.size(); i++) {      
        d_other_s.push_back( discrete2s( sensor_fusion[i][5]));
        d_other_d.push_back( discrete2d( sensor_fusion[i][6]));
        d_o_d_pos.push_back( discrete2d( sensor_fusion[i][6]+1)); // possiblel left  lanechange
        d_o_d_neg.push_back( discrete2d( sensor_fusion[i][6]-1)); // possiblel right lanechange
        //   v == vs!     v= distance(0,0,vx                 ,vy                 ) 
        c_other_v.push_back((distance(0,0,sensor_fusion[i][3],sensor_fusion[i][4])-ego_v)/4); 
    }


    // discrete 2D(t) road(t) - filled with 1
    int map = num_of_lanes*d_horizont_s;
    for(int i = 0; i < map * d_horizont_t; i++) {  
       time_road[i]= 1;
    } 

    //place the other cars in the road for each timestep
    for (int t = 0; t < d_horizont_t; t++){
    // Predict the relative position of the other cars, for each time layer of the road
        for(int i = 0; i < sensor_fusion.size(); i++) {
            int d_pre_o_s = d_other_s[i] + t * c_other_v[i]; // discrete prediction in s + t * v 
            if (0 < d_pre_o_s && d_pre_o_s < d_horizont_s){  // if d_pre is within the horizont put it on the map
                int pointer_pre_s = d_pre_o_s * num_of_lanes + t * map;
                time_road [ d_other_d[i] + pointer_pre_s] = 9;
                // maybe d_other_d plus or minus 1 is different, so set it again or left - no if
                time_road [ d_o_d_pos[i] + pointer_pre_s] = 9;
                time_road [ d_o_d_neg[i] + pointer_pre_s] = 9;
                //cout << t << " " << d_other_d[i] << " "<< d_pre_o_s << endl;
            }

        // TODO safety area around cars d_other_s[i]+1 = 5 ... 3     1 and on the right to avoid right passing
        }
    } 
    for (int i=0 ;i<=20;i++) time_road [1+3*10+i*map] = 0;  
    for (int s = d_horizont_s; s > 0; s--){
        int s_nol = s*num_of_lanes;
        cout<< time_road[2+s_nol]        << time_road[1+s_nol]        << time_road[0+s_nol]       << " "<<
               time_road[2+s_nol+ 1*map] << time_road[1+s_nol+ 1*map] << time_road[0+s_nol+ 1*map]<< " "<<
               time_road[2+s_nol+ 2*map] << time_road[1+s_nol+ 2*map] << time_road[0+s_nol+ 2*map]<< " "<< 
               time_road[2+s_nol+ 3*map] << time_road[1+s_nol+ 3*map] << time_road[0+s_nol+ 3*map]<< " "<< 
               time_road[2+s_nol+ 4*map] << time_road[1+s_nol+ 4*map] << time_road[0+s_nol+ 4*map]<< " "<< 
               time_road[2+s_nol+ 5*map] << time_road[1+s_nol+ 5*map] << time_road[0+s_nol+ 5*map]<< " "<< 
               time_road[2+s_nol+ 6*map] << time_road[1+s_nol+ 6*map] << time_road[0+s_nol+ 6*map]<< " "<<
               time_road[2+s_nol+ 7*map] << time_road[1+s_nol+ 7*map] << time_road[0+s_nol+ 7*map]<< " "<< 
               time_road[2+s_nol+ 8*map] << time_road[1+s_nol+ 8*map] << time_road[0+s_nol+ 8*map]<< " "<< 
               time_road[2+s_nol+ 9*map] << time_road[1+s_nol+ 9*map] << time_road[0+s_nol+ 9*map]<< " "<< 
               time_road[2+s_nol+10*map] << time_road[1+s_nol+10*map] << time_road[0+s_nol+10*map]<< " "<< 
               time_road[2+s_nol+11*map] << time_road[1+s_nol+11*map] << time_road[0+s_nol+11*map]<< " "<< 
               time_road[2+s_nol+12*map] << time_road[1+s_nol+12*map] << time_road[0+s_nol+12*map]<< " "<< 
               time_road[2+s_nol+13*map] << time_road[1+s_nol+13*map] << time_road[0+s_nol+13*map]<< " "<< 
               time_road[2+s_nol+14*map] << time_road[1+s_nol+14*map] << time_road[0+s_nol+14*map]<< " "<< 
               time_road[2+s_nol+15*map] << time_road[1+s_nol+15*map] << time_road[0+s_nol+15*map]<< " "<< 
               time_road[2+s_nol+16*map] << time_road[1+s_nol+16*map] << time_road[0+s_nol+16*map]<< " "<< 
               time_road[2+s_nol+17*map] << time_road[1+s_nol+17*map] << time_road[0+s_nol+17*map]<< " "<< 
               time_road[2+s_nol+18*map] << time_road[1+s_nol+18*map] << time_road[0+s_nol+18*map]<< " "<< 
               time_road[2+s_nol+19*map] << time_road[1+s_nol+19*map] << time_road[0+s_nol+19*map]<< " "<< 
               time_road[2+s_nol+20*map] << time_road[1+s_nol+20*map] << time_road[0+s_nol+20*map]<<endl;
    } 
    cout << "===================================================================================" << endl;
}

Prediction::~Prediction(){}

// Function to get acces to the time_road vector in a 3D space, outside is always 9 - inside is the time_road
int Prediction::GetMap( int d, int s, int t ) {
    if( d < 0 || d >= Prediction::num_of_lanes || 
        s < 0 || s >= Prediction::d_horizont_s || 
        t < 0 || t >= Prediction::d_horizont_t )
         return 9;	
    else return time_road[(d
                         + s * Prediction::num_of_lanes
                         + t * Prediction::num_of_lanes * Prediction::d_horizont_s)];
}


void Prediction::search() {

    // Create an instance of the search class A*
    AStarSearch<MapSearchNode> astarsearch;

  
    unsigned int       SearchCount = 0;
    const unsigned int NumSearches = 1;
    while(SearchCount < NumSearches){
        // Create a start state
        MapSearchNode nodeStart;
        nodeStart.d = Prediction::d_ego_d;
        nodeStart.s = Prediction::d_ego_s; 

        // Define the goal state
        MapSearchNode nodeEnd;
        nodeEnd.d = 2;						
        nodeEnd.s = d_horizont_s-1; 
	
        // Set Start and goal states
        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
        unsigned int SearchState;
        unsigned int SearchSteps = 0;
  
        do  {
            SearchState = astarsearch.SearchStep();
            SearchSteps++;
            cout << "Volker was here" << endl;   
	  	/*	cout << "Steps:" << SearchSteps << "\n";
			int len = 0;
			cout << "Open:\n";
			MapSearchNode *p = astarsearch.GetOpenListStart();
			while( p )			{
				len++;
				//((MapSearchNode *)p)->PrintNodeInfo();
				p = astarsearch.GetOpenListNext();
			}
			cout << "Open list has " << len << " nodes\n";
			len = 0;
			cout << "Closed:\n";
			p = astarsearch.GetClosedListStart();
			while( p ){
				len++;
				//p->PrintNodeInfo();
				p = astarsearch.GetClosedListNext();
			}
			cout << "Closed list has " << len << " nodes\n";  
		*/}
  
            while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
            if   ( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED ){

                cout << "Search found goal state\n";
                MapSearchNode *node = astarsearch.GetSolutionStart();
                cout << "Displaying solution\n";
                int steps = 0;
                node->PrintNodeInfo();
                for( ;; ){ //infinite loop 
                    node = astarsearch.GetSolutionNext();
                    if( !node )
                        break;
                    node->PrintNodeInfo();            
                    //path_s.push_back( continuous2s (node->GetNode_s) );
                    //path_d.push_back( continuous2d (node->GetNode_d) );
                    //path_v.push_back( continuous2v (node->GetNode_v) );
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
} 

// 6 Funtion to discrete sdv and bring back sdv "2" continous action space
 
int Prediction::discrete2s(double s){ // set ego_s-rearview to zero, every discrete meter an area
    int result = int( (s-_ego_s+rearview+discrete/2)/discrete );
    if (result > 1000) result -= 1732; // howerver 6945,554÷4 = 1736,3885  
    if (result <-1000) result += 1732; // but value jumps from 1732-1731 
    return result;
}

int Prediction::discrete2d(double d) {   
    const int road_width = 12;  
    const int lane_width =  4;  
    int result = int((road_width-d) / lane_width);
    result = max(result, 0);                      // cut to the left
    result = min(result, num_of_lanes);           // cut to the right
    return result;
}

int Prediction::discrete2v(double v) {            // for every 4 m/s one discrete area 
    const int discrete_v = 4;                     // velocity area every 4(m/s) 
    int result = int((v-_ego_v)/discrete_v );   //
    return result;
}

double Prediction::continuous2s(int s) {          // back to continous track s
    return (double) s * discrete + _ego_s - rearview;
}

double Prediction::continuous2d(int d) {          // back to continous track d 
    const int road_offset = 10;                   // distance to the outside right midlane
    const int lane_width = 4;  
    return (road_offset-d*lane_width); 
}

double Prediction::continuous2v(int v) {          // back to continous track v
   double discrete_v = 4.;  
   return (v * discrete_v + _ego_v);
}


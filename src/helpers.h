#ifndef HELPERS_H
#define HELPERS_H

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  double dx = x2-x1; 
  double dy = y2-y1;
  return sqrt(dx*dx+dy*dy);
}

///////////////////////////////////////////////////////////////////////////////////
// global variabls
///////////////////////////////////////////////////////////////////////////////////

const int discrete =  4; // in s and v: 4m one area 4m/s one speed case

const int num_of_lanes =  3;
const long d_horizont_s = 300/discrete ; // m look ahead distance
const int  d_horizont_t = 60;

int time_road[num_of_lanes * d_horizont_s * d_horizont_t];  //size of 3D time_road = 2D(t) as 1D array

// Function to get acces to the time_road in a 3D space, outside is always 9 
int GetMap( int d, int s, int t ) {
    if( d < 0 || d >= num_of_lanes || 
        s < 0 || s >= d_horizont_s || 
        t < 0 || t >= d_horizont_t )
         return 9;	
    else return time_road[(d
                         + s * num_of_lanes
                         + t * num_of_lanes * d_horizont_s)];
}

void print_time_raod(){
    for (int s = ::d_horizont_s-1; s >= 0; --s){  // reverse order
        int s_nol = s * ::num_of_lanes;
        int map =  ::num_of_lanes * ::d_horizont_s;
        for (int column=0; column < 30;column++){   
            for (int lane=2; 0 <= lane; lane--){
                if (time_road[lane+s_nol+ column*map] == 99){ 
                    if (lane==2)std::cout << "\033[35m\033[1m" << "0" << "\033[0m";
                    if (lane==1)std::cout << "\033[33m\033[1m" << "0" << "\033[0m";
                    if (lane==0)std::cout << "\033[31m\033[1m" << "0" << "\033[0m";
                }
                else if (time_road[lane+s_nol+ column*map] == 9)
                     std::cout << "\033[36m" << "O" << "\033[0m"; 
                else if (time_road[lane+s_nol+ column*map] == 8)
                     std::cout << "\033[36m" << "|" << "\033[0m"; 
                else std::cout << "\033[30m"<< time_road[lane+s_nol+ column*map]<< "\033[0m"; 
            } 
            std::cout<< " ";     
        }
    std::cout << s <<  std::endl;
    } 
    std::cout << "=0===1===2===3===4===5===6===7===8===9==10==11==12==13==15==16==17==18==19==20==21 t(d_dt)" << std::endl;
}

// 6 Function for converting s, d and v from the continuous action space into discrete values and vice versa
 
int discrete_to_s(double s, double ego_s){ // ego_s converts to zero
    int result = int( (s-ego_s + discrete/2 ) / discrete );
    if (result > 1000) result -= 1732; // howerver 6945,554÷4 = 1736,3885  
    if (result <-1000) result += 1732; // but value jumps from 1732-1731 
    return result;                     // results are between 0 and max 1000
}
double continuous_to_s(int s, double ego_s) {  // back to continous s
    return s * discrete  + ego_s;
}

int discrete_to_d(double d) {  
    const int road_width = 12;           // left lane is 2
    const int lane_width =  4;           // right lane is 0  
    int result = int(( road_width - d) / lane_width );
    result = std::max(result, 0);              // cut to the left
    result = std::min(result, num_of_lanes-1); // cut to the right
    return result;
}
double continuous_to_d(int d) {          // back to continous  d 
    const double road_offset = 9.99;      // distance to the outside right midlane
    const double lane_width  = 3.99;  
    return (road_offset - d * lane_width); 
}

double discrete_to_v(double v) {         // every discrete*m/s one area                      
    return  v / discrete ;    
}
double continuous_to_v(double v, double v_max) { // back to continous v  
    return v * discrete * v_max/20 ;
}

#endif  // HELPERS_H

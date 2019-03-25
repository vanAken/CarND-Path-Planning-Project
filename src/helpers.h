#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>


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

const int discrete =  4; // in s and v direction 4m one area
const int offset_s =  0; // shift of startposition in s

const int num_of_lanes =  3;
const long d_horizont_s = 240/discrete + offset_s; // m frontview
const int  d_horizont_t = 60;

int time_road[num_of_lanes * d_horizont_s * d_horizont_t];  //size of 3D time_road = 2D(t) as 1D array

double d_v_max; // discrete velocity
double d_a_max; // discrete acceleration

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

vector<double> next_s;
vector<double> next_d;
vector<double> next_v;       

void print_time_raod(){
    for (int s = ::d_horizont_s-1; s >= 0; --s){  // reverse order
        int s_nol = s * ::num_of_lanes;
        int map =  ::num_of_lanes * ::d_horizont_s;
        for (int column=0; column < 50;column++){   
            for (int lane=2; 0 <= lane; lane--){
                if (time_road[lane+s_nol+ column*map] == 0){ 
                    if (lane==2)std::cout << "\033[35m\033[1m" << "0" << "\033[0m";
                    if (lane==1)std::cout << "\033[33m\033[1m" << "0" << "\033[0m";
                    if (lane==0)std::cout << "\033[31m\033[1m" << "0" << "\033[0m";
                }
                else if (time_road[lane+s_nol+ column*map] == 9)
                     std::cout << "\033[36m" << "O" << "\033[0m"; 
                //else if (time_road[lane+s_nol+ column*map] == 8)
                  //   std::cout << "\033[36m" << "|" << "\033[0m"; 
                else std::cout << "\033[30m"<< time_road[lane+s_nol+ column*map]<< "\033[0m"; 
            } 
            std::cout<< " ";     
        }
    std::cout << s - offset_s <<  std::endl;
    } 
    std::cout << "=0===1===2===3===4===5===6===7===8===9==10==11==12==13==15==16==17==18==19==20==21 t(d_dt)" << std::endl;
}

#endif  // HELPERS_H

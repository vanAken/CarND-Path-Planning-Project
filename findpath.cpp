////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;

// Global data

// The world map

const int WIDTH_D  = 6;
const int HORIZONT = 50;
const int TIME = 10;

//vector<vector<int> > world_map

int world_map [ WIDTH_D * HORIZONT * TIME ] =
{ 
// 000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//1000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//2000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//3000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//4000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//5000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//6000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//7000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//8000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1,   // 49
//9000102 030405
    1,1,1 ,1,1,1,   // 00
    1,9,9 ,9,9,9,   // 01
    1,9,9 ,1,1,9,   // 02
    1,9,9 ,1,1,9,   // 03
    1,9,1 ,1,1,1,   // 04
    1,9,1 ,1,9,1,   // 05
    1,9,9 ,9,9,1,   // 06
    1,9,9 ,9,9,9,   // 07
    1,9,1 ,1,1,1,   // 08
    1,1,1 ,9,9,1,   // 09
    9,9,9 ,9,1,1,   // 10
    1,1,1 ,1,9,1,   // 11
    1,9,1 ,1,9,1,   // 12
    1,1,9 ,1,9,1,   // 13
    9,1,9 ,1,9,1,   // 14
    1,1,9 ,1,9,1,   // 15
    1,9,9 ,1,9,1,   // 16
    1,1,9 ,1,9,1,   // 17
    9,1,9 ,1,1,1,   // 18
    1,1,9 ,9,9,9,   // 19
    1,1,1 ,1,1,1,   // 20
    1,9,1 ,1,9,1,   // 21
    1,9,9 ,1,9,1,   // 22
    1,9,9 ,1,9,1,   // 23
    1,9,1 ,1,9,1,   // 24
    1,9,1 ,1,9,1,   // 25
    1,9,9 ,9,9,1,   // 26
    1,9,9 ,9,9,9,   // 27
    1,9,1 ,1,1,1,   // 28
    1,9,1 ,9,9,9,   // 29
    1,1,1 ,1,1,1,   // 30
    1,9,9 ,9,9,9,   // 31
    1,9,9 ,1,1,9,   // 32
    1,9,9 ,1,1,9,   // 33
    1,9,1 ,1,1,1,   // 34
    1,9,1 ,1,9,1,   // 35
    1,9,9 ,9,9,1,   // 36
    1,9,9 ,9,9,9,   // 37
    1,9,1 ,1,1,1,   // 38
    1,9,1 ,9,9,9,   // 39
    1,1,1 ,1,1,1,   // 40
    1,9,9 ,9,9,9,   // 41
    1,9,9 ,1,1,9,   // 42
    1,9,9 ,1,1,9,   // 43
    1,9,1 ,1,1,1,   // 44
    1,9,1 ,1,9,1,   // 45
    1,9,9 ,9,9,1,   // 46
    1,9,9 ,9,9,9,   // 47
    1,9,1 ,1,1,1,   // 48
    1,1,1 ,1,1,1    // 49
};

//int world_map = world_maps[0];
// map helper functions

int GetMap( int d, int s, int t = 1 )
{
	if( d < 0 ||
            d >= WIDTH_D ||
                 s < 0 || 
                 s >= HORIZONT )
	{
		return 9;	 
	}

	return world_map[(d + s*WIDTH_D + t * WIDTH_D * HORIZONT)];
}



// Definitions

class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;	
	
	MapSearchNode() { x = y = 0; }
	MapSearchNode( int px, int py ) { x=px; y=py; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo(); 


};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

	// same state in a maze search is simply when (x,y) are the same
	if( (x == rhs.x) &&
		(y == rhs.y) )
	{
		return true;
	}
	else
	{
		return false;
	}

}

void MapSearchNode::PrintNodeInfo()
{
	char str[100];
	sprintf( str, "Node position : (%d,%d)\n", x,y );

	cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( (x == nodeGoal.x) &&
		(y == nodeGoal.y) )
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

	int parent_x = -1; 
	int parent_y = -1; 

	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}
	

	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if( (GetMap( x-1, y ) < 9) 
		&& !((parent_x == x-1) && (parent_y == y))
	  ) 
	{
		NewNode = MapSearchNode( x-1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x, y-1 ) < 9) 
		&& !((parent_x == x) && (parent_y == y-1))
	  ) 
	{
		NewNode = MapSearchNode( x, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x+1, y ) < 9)
		&& !((parent_x == x+1) && (parent_y == y))
	  ) 
	{
		NewNode = MapSearchNode( x+1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

		
	if( (GetMap( x, y+1 ) < 9) 
		&& !((parent_x == x) && (parent_y == y+1))
		)
	{
		NewNode = MapSearchNode( x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
	return (float) GetMap( x, y );

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
		nodeStart.x = 1;
		nodeStart.y = 0; 

		// Define the goal state
		MapSearchNode nodeEnd;
		nodeEnd.x = 1;						
		nodeEnd.y = HORIZONT-1; 
		
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
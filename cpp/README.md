astar-algorithm-cpp
===================

[![Build Status](https://travis-ci.org/justinhj/astar-algorithm-cpp.svg?branch=master)](https://travis-ci.org/justinhj/astar-algorithm-cpp)

Summary
-------

This code is an efficient implementation in C++ and C# of the A* algorithm, designed to be used from high performance realtime applications (video games) and with an optional fast memory allocation scheme.   

It accompanies this A* tutorial: http://www.heyes-jones.com/astar.html

Contributions: 

* @justinhj Original code and tutorial
* @ScaryG C# port
* @Rasoul for submitting the path to Bucharest. Sample from Artificial Intelligence: A Modern Approach 
* @sergiosota For fixing a number of issues related to memory management

License
=======

This software is released under the MIT License, see license.txt

Commercial Use
==============

This software has been used in AAA video games and is well tested in the wild. Please let me know if you use this code in you games, studies or hobby projects. 

If you feel the need to pay money for this code, it is not required by the license, but you could contribute to Unicef, a charity which helps children worldwide,  http://www.unicef.org/ that would be awesome.

If you wish to be added to the list of known products using the code please contact me.


Compilation
===========

Enter the cpp folder and run make

Introduction
============

This implementation is intended to be simple to read yet fairly efficient. 
To build it you can compile, with any recent C++ compiler, the following files :


For path finder 
* findpath.cpp
* stlastar.h
* optionally fsa.h

pathfind has no arguments. You can edit the simple map in pathfind.cpp and the start 
and goal co-ordinates to experiement with the pathfinder.

Fixed size allocator notes: As mentioned briefly in the tutorial you can enable and disable the
faster memory allocation. This allocates a fixed size block of memory, so you have to specify this size
with the astar constructor. You need to enlarge it if you hit an out of memory assert during the
search.

Please let me know if it doesn't work for you and I will try to help. I cannot help if you are using
an old compiler such as Turbo C++, since I update the code to meet Ansi Standard C++ as required.


Cheers!

Justin

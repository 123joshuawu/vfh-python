# vfh-python

This repository contains the Python implementations for the Vector Field Histogram (VFH) path planning algorithm I had researched and developed during the Summer of 2018.

My original goal was to find an online local LIDAR-based path planning algorithm. After a bit of research, I discovered VFH and decided to implement it.

This repository's initial commit contains my original semi-haphazard implementations for VFH and two variants VFH+ and VFH`* as they were at the end of the Summer of 2018.

At the moment, the VFH\* implementation is incomplete however both VFH and VFH* are operational.

*Additionally, my VFH and VFH+ implementations are in Python 2 (R.I.P. Python 2)*

The papers that I used as reference for each algorithm are linked below:
- VFH  http://www-personal.umich.edu/~johannb/Papers/paper16.pdf
- VFH+ http://www-personal.umich.edu/~johannb/Papers/paper73.pdf
- VFH\* http://www.cs.cmu.edu/~iwan/papers/vfhstar.pdf

The goal for this project is to develop a web interface that allows users to interact with the algorithms.

The next steps for this project are:
- [ ] Clean up the code for each algorithm
- [ ] Port VFH and VFH+ to Python 3
- [ ] Make an attempt to fix the VFH\*
- [ ] Develop a live web interface for interacting with the above algorithms

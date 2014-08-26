/** Class to calculate marginals and several other stuff from a Factor Graph.
 *  Copyright (c) A.Albore 2014 - derived from libDAI.                      
 *  Copyright (c) 2006-2011, the libDAI authors.  All rights reserved.
 *  Use of this source code is governed by a BSD-style license as follows:
 @author Alexandre Albore
 @version 1.0
 * Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
#ifndef PATH_FOLLOWER_APPROX_MARGINALS_H
#define PATH_FOLLOWER_APPROX_MARGINALS_H

//#include <dai/alldai.h>
#include <dai/evidence.h>
#include <dai/daialg.h>
#include <dai/factorgraph.h>
#include <dai/treeep.h>
#include <dai/properties.h>
#include <dai/bp.h>

#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

using namespace std;
using namespace dai;


const Real tol = 1e-9;
const size_t max_iterations = 10000;

/** 
    Struct to hold the coordinates of the map.
 */
typedef struct MapCoordinates {
        const int XOFF;
        const int YOFF;
        const int COL;

        MapCoordinates(int a, int b, int c) : XOFF(a), YOFF(b), COL(c) {}

        size_t pose2var(geometry_msgs::Pose p) 
        {
                geometry_msgs::Point coord;
        
                coord.x = p.position.x+XOFF;
                coord.y = p.position.y+YOFF;
                coord.z = p.position.z;

                return (coord.x/3 + coord.y*COL/3);
        }

        geometry_msgs::Pose var2pose(int var)
        {
                geometry_msgs::Pose p;
                p.position.x = var%COL*3-XOFF;
                p.position.y = var/COL*3-YOFF;
                p.position.z = 4.9; // Safe value. 9m2 at f:50mm

                return p;
        }
} MapCoordinates;

class ApproxMarginals {

        size_t _num_classes;
        BP _bp;
public:
        struct MapCoordinates coord;
        ApproxMarginals(FactorGraph &network);
        ApproxMarginals(FactorGraph &network, int nc);

        void evidence(geometry_msgs::Pose p, int o);
        geometry_msgs::Pose next_waypoint();
        Real quality();
        void set_num_classes(const int c) { _num_classes = c; }
        int num_classes() { return _num_classes; }
        void print_max_state();
        void print_quality_map();
        void print_Gibbs_sample(size_t n);
        FactorGraph& fg() { return _bp.fg();}
        BP& bp() { return _bp; } //NB: to change when using different algos

protected:
        std::vector<Real> quality_map();
        BP loopy_belief_propagation(FactorGraph &fg);
        size_t most_uncertain_var();                
        template<typename T>
                void print_state(std::vector<T> &s,  std::ofstream &outfile);
};
#endif // PATH_FOLLOWER_APPROX_MARGINALS_H

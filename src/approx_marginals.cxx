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

// #define MANY_ALGORITHMS
// #define MAXPROD

#include "path_follower/approx_marginals.h"
#include <dai/gibbs.h>

/// Constructor
ApproxMarginals::ApproxMarginals(FactorGraph &network) : 
        _bp(loopy_belief_propagation(network)), coord(25, 47, 17)
{
        ApproxMarginals( network, network.var(1).states() );
}

/// Constructor
ApproxMarginals::ApproxMarginals(FactorGraph &network, int nc):
        _num_classes(nc), _bp(loopy_belief_propagation(network)), coord(25, 47, 17)
{  
}

/**
   Receives the Waypoint and the observation, and updates the factor graph.
   @param p Pose of the observation.
   @param o The observation result.
*/
void ApproxMarginals::evidence(geometry_msgs::Pose p, int o)
{
        size_t i = coord.pose2var(p);

        std::cout << "Observing variable:" << i << ", value: " << (o > num_classes() ? num_classes() : o) << std::endl;

        // Adds evidence to FG
        fg().clamp(i, (o > num_classes() ? num_classes() : o));
        bp().init();
        bp().run();
}


/**
   Provides the next sampling site based on the adaptive sampling algorithm.
   @return return The Pose corresponding to the most uncertain variable following the given marginals algorithm.
   @see ApproxMarginals::most_uncertain_var()
*/
geometry_msgs::Pose ApproxMarginals::next_waypoint()
{

        size_t var = most_uncertain_var();
        geometry_msgs::Pose p = coord.var2pose(var);

        return p;
}


/**
   Calculates the variable with most uncertainty on its value, i.e. with a probability closer to \f$\frac{1}{C}\f$, where \f$C\f$ is the number of possible classes ("states").
   @return The variable with more uncertainty on its value.
*/
size_t ApproxMarginals::most_uncertain_var()
{
        float avg_dist= 1.0 / num_classes();
        Real min_dist = 1.0;
        size_t min_var = 0;

        for( size_t i = 0; i < fg().nrVars(); i++ ) // iterate over all variables in the factor graph
        {
                Real dist = 0.0;
                Factor f = bp().belief(fg().var(i));
                for (size_t j = 0; j < f.nrStates(); j++) 
                        dist += abs( f[j] - avg_dist );

                if (dist < min_dist) {
                        min_dist = dist;
                        min_var = i;
                }
                // Report variable marginals for factor graph, calculated by the Gibbs sampler algorithm
                // display the "belief" of gibbsSample for that variable
                // std::cout << gibbsSampler.belief(fg().var(i)) << " - " << dist << std::endl; 
        } 
        std::cout << "most uncertain var is {" << min_var << "}: " << min_dist << std::endl;
        return min_var;
}


/**
   Calculates the quality of the graph.
   @return The sum of the higher probability by state. 
*/
Real ApproxMarginals::quality()
{
        Real sum = 0.0;
        for( size_t i = 0; i < fg().nrVars(); ++i ) // iterate over all variables in network
        {
                Real higher_value = 0.0;
                Factor f = bp().belief( fg().var(i));
                for (size_t j = 0; j < f.nrStates(); j++) 
                        if  (higher_value < f[j])
                                higher_value = f[j];
                sum +=  higher_value;
        }
        return sum;
}

/**
   Makes a newly created inference algorithm.
   @param fg The Factor Graph upon which is built the loopy belief propagation algorithm.
   @return The algorithm to approximate marginal probability distributions ("beliefs") for the subset of variables depending on the factor.
*/
BP ApproxMarginals::loopy_belief_propagation(FactorGraph &fg)
{
// Store the constants in a PropertySet object
        PropertySet opts;
        opts.set("maxiter",max_iterations); // Maximum number of iterations
        opts.set("tol",tol); // Tolerance for convergence
        opts.set("verbose", size_t(1)); // Verbosity (amount of output generated)


// Construct a BP (belief propagation) object from the FactorGraph fg
// using the parameters specified by opts and two additional properties,
// specifying the type of updates the BP algorithm should perform and
// whether they should be done in the real or in the logdomain
        BP bp( fg, opts("updates",string("SEQRND"))("logdomain",false));
// Initialize belief propagation algorithm
        bp.init();
// Run belief propagation algorithm
        bp.run();
        
        // Output some information 
        cout << bp.maxDiff() << " max diff" << endl;
        cout << bp.name() << " name" << endl;
        

#ifdef MAXPROD
// Note that inference is set to MAXPROD, which means that the object
// will perform the max-product algorithm instead of the sum-product algorithm
        BP mp( fg, opts("updates",string("SEQRND"))("logdomain",false)("inference",string("MAXPROD"))("damping",string("0.1")));
// Initialize max-product algorithm
        mp.init();
// Run max-product algorithm
        mp.run();
// Calculate joint state of all variables that has maximum probability
// based on the max-product result
        vector<size_t> mpstate = mp.findMaximum();

// Report max-product variable marginals
        cout << "Approximate (max-product) MAP variable marginals:" << endl;
        for( size_t i = 0; i < network.nrVars(); i++ )
                cout << mp.belief(network.var(i)) << endl;

// Report max-product factor marginals
        cout << "Approximate (max-product) MAP factor marginals:" << endl;
        for( size_t j = 0; j < network.nrFactors(); ++j )
                cout << mp.beliefF(j) << endl;
#endif


        /** Code to perform updates **/
        //  bp.fg().clamp(1,1);
        //  bp.init();
        // // Run belief propagation algorithm
        // bp.run();
#ifdef DEBUG
// Report variable marginals for network, calculated by the belief propagation algorithm
        cout << "Approximate (loopy belief propagation) variable marginals:" << endl;
        for( size_t i = 0; i < bp.fg().nrVars(); i++ ) // iterate over all variables in network
                cout << bp.belief( bp.fg().var(i)) << endl; // display the belief of bp for that variable

// Report factor marginals for network, calculated by the belief propagation algorithm
        cout << "Approximate (loopy belief propagation) factor marginals:" << endl;
        for( size_t i = 0; i < bp.fg().nrFactors(); ++i ) // iterate over all factors in network
                cout << bp.belief(bp.fg().factor(i).vars()) << endl; // display the belief of bp for the variables in that factor
#endif

        return bp;
}


/**
   Prints out a single state (the probability distribution for a variable)
   @param s The state in the Markov field.
   @param outfile The output stream.
*/
void ApproxMarginals::print_Gibbs_state(std::vector<size_t> &s,  std::ofstream &outfile)
{
        for( size_t i = 0; i < s.size(); i++ ) {
                outfile << ((i == 0 || !((i)% coord.col) ) ? "" : "\t") << s[i];
                
                if ((i > 0) && !((i+1)%coord.col)) 
                        outfile << std::endl;
        }
        outfile << std::endl;
}

/** Runs Gibbs sampling over the factor graph. Prints out the resulting sample.
    The number of samples is decided by the 'nrSamples' constant.
    @param nrIterations The number of iterations for the sampler.
*/
void ApproxMarginals::print_Gibbs_sample(size_t nrIterations)
{
        const size_t nrSamples = 1; // number of Gibbs samples to produce
        // const size_t nrIterations = 1000; // number of Gibbs sampler iterations

// Prepare a Gibbs sampler
        PropertySet gibbsProps;
        gibbsProps.set("maxiter", size_t(nrIterations)); 
        gibbsProps.set("burnin", size_t(0));
        gibbsProps.set("verbose", size_t(0));
        Gibbs gibbsSampler( fg(), gibbsProps );

// Open a .tab file for writing
        std::ofstream outfile;
        outfile.open( "map_results.tab" );
        if( !outfile.is_open() )
                throw "Cannot write to file!";
// Write header (consisting of variable labels)
        for( size_t i = 0; i < fg().nrVars(); i++ ) {
                outfile <<  "\t" << fg().var(i).label();
                if ((i > 0) && !((i+1)%coord.col)) 
                        outfile << std::endl;
        }
        outfile << std::endl << std::endl;

// Draw samples from joint distribution using Gibbs sampling
// and write them to the .tab file
        gibbsSampler.randomizeState();
        std::vector<size_t> state;
        for( size_t t = 0; t < nrSamples; t++ ) {
                gibbsSampler.init(); 
                gibbsSampler.run();
                print_Gibbs_state( gibbsSampler.state(), outfile );
        } 
        outfile << std::endl;
        outfile.close();
        std::cout << nrSamples << " samples written to map_results.tab" << std::endl;
}




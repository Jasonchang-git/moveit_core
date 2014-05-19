/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Southwest Research Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Southwest Research Institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Chris Lewis */

#include <moveit/trajectory_processing/fir_trajectory_filter.h>
#include <console_bridge/console.h>
#include <moveit/robot_state/conversions.h>
#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>

namespace trajectory_processing
{
  namespace{
void  print_tragectory_to_matlab_format(std::string filename, std::string mat_name, robot_trajectory::RobotTrajectory& rob_trajectory)
    {
      FILE *fp = fopen(filename.c_str(),"w");
      const int num_points = rob_trajectory.getWayPointCount(); 
      const int num_states = rob_trajectory.getWayPoint(0).getVariableCount();
      fprintf(fp, "%s = [\n",mat_name.c_str());
      for(int i=0; i<num_points; i++){
	for(int j=0; j<num_states; j++){
	  fprintf(fp,"%lf ", rob_trajectory.getWayPoint(i).getVariablePosition(j)); // j'th state of i'th waypoint
	}
	fprintf(fp, ";\n");
      }
      fprintf(fp, "];\n");
      fclose(fp);
    }// end of print function
  } // end of un named namespace


  FIRTrajectoryFilter::FIRTrajectoryFilter(std::vector<double> coef)
  {
    num_coef_ = coef.size();
    double sum =0;
    for(int i=0; i<num_coef_; i++){
      coef_.push_back(coef[i]);
      sum += coef[i];
    }
    gain_ = sum;  // set gain to be the sum of the coefficients because we need unity gain
  }

  FIRTrajectoryFilter::~FIRTrajectoryFilter()
  {
    coef_.clear();
  }

 bool FIRTrajectoryFilter::applyFilter(robot_trajectory::RobotTrajectory& rob_trajectory)
  {
    print_tragectory_to_matlab_format("/home/clewis/test1.m","A1",  rob_trajectory);
    const int num_points = rob_trajectory.getWayPointCount(); 
    if(num_points <=2) return(false); // nothing to do here, can't change either first or last point
    const int num_states = rob_trajectory.getWayPoint(0).getVariableCount();
    std::vector<double> xv;
    
    // filter each variable independently
    for(int i=0; i<num_states; i++){ 
      
      // initialize the filter values to the first point
      double start_value = rob_trajectory.getWayPoint(0).getVariablePosition(i);
      double second_value = rob_trajectory.getWayPoint(1).getVariablePosition(i);
      double delta = start_value - second_value;
      xv.clear();
      for(int j=0; j<num_coef_; j++) { 
	double value = start_value+(num_coef_ - j )*delta;
	xv.push_back(value);	// set filter as if linear with current trend
      }
      
      // cycle through every waypoint, and apply the filter, NOTE, 1st and last waypoints should not be changed
      for(int j=1; j<num_points-1; j++){
	// shift backwards
	for(int k=0; k<num_coef_-1; k++){
	  xv[k] = xv[k+1];  
	}
	
	// get next input to filter
	xv[num_coef_ - 1] = rob_trajectory.getWayPoint(j).getVariablePosition(i); // i'th state of j'th waypoint

	// apply the filter
	double sum = 0.0;
	for(int k=0; k<num_coef_; k++){ 
	  sum += xv[k]*coef_[k];
	}

	// save the results
	rob_trajectory.getWayPointPtr(j)->setVariablePosition(i,sum/gain_); // j'th waypoint, i'th variable set to output value

      }// end for every waypoint

    } // end for every state

    print_tragectory_to_matlab_format("/home/clewis/test2.m","A2",  rob_trajectory);

    // TODO don't be lazy, do some error checking!!
    return(true);

}// end FIRTrajectoryFilter::applyfilter()

}  // end namespace 

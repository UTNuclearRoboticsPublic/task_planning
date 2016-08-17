///////////////////////////////////////////////////////////////////////////////
//      Title     : goap_test.cpp
//      Project   : task_planning
//      Created   : 6/6/2016
//      Author    : Blake Anderson
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

/*
 * Tests the goap::ActionPlanner class and its subsidiary classes.
 * The test simulates a waypoint survey task for a mobile robot.
 * Uses the GOAP 2.0 algorithm utilizing Action Description Language (ADL)
 * https://en.wikipedia.org/wiki/Action_description_language
 * 
 * Usage: rosrun task_planning goap_test [num_waypoints]
 */

#include "task_planning/action_planner.h"

#include <string.h>
#include <cstdio>
#include <sys/stat.h>

#include <boost/chrono/thread_clock.hpp>

#include <stdlib.h>
#include <iostream>


int main( int argc, char* argv[] )
{
  // Get number of waypoints from command line
  int num_waypoints = 5; // Default
  if( argc == 2 ) {
    num_waypoints = atoi(argv[1]);
  }
  
  // Declare ActionPlanner
  goap::ActionPlanner ap;
  
  boost::chrono::thread_clock clock;
  
  /*
   * Define actions using ADL: https://en.wikipedia.org/wiki/Action_description_language
   * 
   * Action parameters are defined by (name:type).
   * This defines the action MoveRobot which takes three parameters.
   * Preconditions are propositions which must be met in order to perform the action.
   * Effects are propositions that will be set by the action.
   * Propositions can have parameters as well. Using a param named in the base definition will link the proposition to it.
   * 
   * Requirements:
   * 	Definitions must use correct syntax.
   * 	Overloaded actions are not allowed; the root names must be unique.
   * 	An action must have at least one effect.
   * 	An action param must have both a name and type.
   * 	Proposition params only have a name.
   * 	Actions and propositions are allowed to take no parameters, ie MoveRobot().
   * 	Negate a proposition by putting ~ in front.
   */
  
  ap.addAction("Survey(r:robot, a:location)",	// Base definition
	       {"At(r,a)", "~Is_Surveyed(a)"},	// Precondition list
	       {"Is_Surveyed(a)"});		// Effect list
  
  ap.addAction("MoveRobot(r:robot, destination:location)",
	       {"~Docked(r)", "~At(r,destination)"},
	       {"At(r,destination)", "~At(r,~destination:location)"}); // negated parameter
  
  // Survey location_3 twice
  ap.addAction("DoubleCheck(r:robot)",
	       {"At(r, $location_3:location)", "Is_Surveyed($location_3:location)"},	// literal parameters
	       {"Double_Checked_3()"});							// void parameter
  
  
  ap.addAction("Dock(r:robot)",
	       {"~Docked(r)"},
	       {"Docked(r)", "~At(r,ALL:location)"});	// this is an ALL parameter
  
  ap.addAction("Undock(r:robot)",
	       {"Docked(r)"},
	       {"~Docked(r)"});
  
  /*
   * Declare the entities in the worldspace
   * 
   * addEntitySet( set_name, list_of_members )
   * The sets are linked to the param types named in the action definitions.
   * EntitySets can be declared before the actions or vice versa; order doesn't matter.
   */
  ap.addEntitySet("robot", {"Pioneer"});
  
  /*
   * You can specify a range of entities.
   * This example will produce location_1, location_2... location_[num_waypoints}
   */
  ap.addEntitySet("location", num_waypoints);
  
  /*
   * buildActionSpace() takes the declared actions and entities and compiles them.
   * 
   * It creates a version of each actions for each valid permutations of their parameters.
   */
  boost::chrono::thread_clock::time_point build_start = clock.now();
  if (!ap.buildActionSpace()) {
    printf("Error: Failed to build action space.");
    return 1;
  }
  boost::chrono::duration<double> build_duration = clock.now() - build_start;

  // Print out the created actions and propositions.
  std::cout << ap.description() << std::endl;
  
  /*
   * Define the initial worldstate.
   * Propositions in a worldspace are initialized to false.
   * Here we set the proposition At(Pioneer, location_1) = true
   */
  goap::Worldstate start;
  ap.setProposition(&start, "Docked(Pioneer)", true);

  /*
   * Define the goal worldstate.
   * 
   * You can also set a whole group of propositions using ADL.
   * Below we set Is_Surveyed(location) for all values of location. 
   */
  goap::Worldstate goal;
  ap.setProposition(&goal, "Is_Surveyed(ALL:location)", true);
  ap.setProposition(&goal, "Double_Checked_3()", true);
  ap.setProposition(&goal, "Docked(Pioneer)", true);

  // Print the starting worldstate
  std::string desc;
  desc = ap.worldstateDescription(&start);
  printf("Start: %s\n", desc.c_str());
  
  // Print the goal worldstate
  desc = ap.worldstateDescription(&goal);
  printf("GOAL: %s", desc.c_str());

  std::vector<goap::Worldstate> states;
  goap::action_plan_t plan;

  // Create the action plan
  boost::chrono::thread_clock::time_point planning_start = clock.now();
  const int cost = ap.createPlan(start, goal, &plan, &states);
  boost::chrono::duration<double> planning_duration = clock.now() - planning_start;
  
  // Print the action plan
  printf("ACTION PLAN\nPlan cost = %d\n", cost);
  for (int i = 0; i < plan.size(); ++i)
  {
    desc = ap.worldstateDescription(&states.at(i));
    printf("%d: %-20s\n", i, plan.at(i).c_str());
  }
  
  printf("Build time: %f secs\n",  build_duration.count());
  printf("Plan time: %f secs\n",  planning_duration.count());

  return 0;
}
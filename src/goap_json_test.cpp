/*-------------------------------------------------------------------------------
 goap_test.cpp
 Version: In Development
 Commit Date: Never

 Authors: Blake Anderson
 The University of Texas at Austin
 Department of Mechanical Engineering
 Nuclear and Applied Robotics Group
 
 Modified from code by Abraham Stolk.

 Description: Test script for Goal Oriented Action Planning

 Command Line Arguments: None
-------------------------------------------------------------------------------*/

/*
Copyright 2012 Abraham T. Stolk

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
*/

#include "task_planning/goap.h"	// for planner interface.

#include <string.h>
#include <cstdio>
#include <sys/stat.h>

#include <ros/ros.h>
#include <ros/package.h>

int main( int argc, char* argv[] )
{
  goap::GOAP goap;
  goap::actionplanner_t ap;
  
  std::string file_path = ros::package::getPath("task_planning") + "/json_test";
  std::cout << file_path << std::endl;
  
  if (!goap.load_json(file_path, &ap)) {
    return 1;
  }
  
  goap::worldstate_t fr;
  goap.goap_worldstate_set( &ap, &fr, "loaded", false );
  goap.goap_worldstate_set( &ap, &fr, "enemy_alive", true );

  goap.goap_set_cost( &ap, "shoot", 1 );
  goap.goap_set_cost( &ap, "load", 1 );
  
  std::string desc;
  goap.goap_description(&ap, &desc);
  printf("Planner:\n%s", desc.c_str());

  goap::worldstate_t goal;
  goap.goap_worldstate_set(&ap, &goal, "enemy_alive", false);

  goap.goap_worldstate_description(&ap, &goal, &desc);
  printf("GOAL: %s", desc.c_str());
  //goap_worldstate_set( &ap, &goal, "alive", true ); // add this to avoid suicide actions in plan.

  std::vector<goap::worldstate_t> states;
  std::vector<std::string> plan;

  const int plancost = goap.astar_plan(&ap, fr, goal, &plan, &states);
  printf("plancost = %d\n", plancost);

  goap.goap_worldstate_description(&ap, &fr, &desc);
  printf("Start: %s\n", desc.c_str());
  for (int i = 0; i < plan.size(); ++i)
  {
    goap.goap_worldstate_description(&ap, &states.at(i), &desc);
    printf("%d: %-20s%s\n", i, plan.at(i).c_str(), desc.c_str());
  }

  return plancost;
}
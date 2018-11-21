
// A simple example of moving from location 1 to location 2

#include "task_planning/task_manager.h"

////////////////////////////
// Define callback functions
////////////////////////////
goap::TaskManager::RETURN_TYPE navigateRobotCallback(std::string robot_name, std::string label)
{
	// TODO: put this in a library, and send the navigation command from there
	printf("Navigating\n");
	return goap::TaskManager::RETURN_TYPE::CONTINUE;
}

int main( int argc, char* argv[] )
{
	////////////////////////
  // Declare ActionPlanner
  ////////////////////////
  goap::ActionPlanner action_planner;

  /////////////////
  // Define actions
  /////////////////
  action_planner.addAction("NavigateRobot(r:robot, destination:location)",	// (name:type) parameters
	  {"~At(r,destination)"},	// Precondition list
	  {"At(r,destination)"});		// Effect list

  //////////////////
  // Define entities
  //////////////////
  action_planner.addEntitySet("robot", {"Husky"});
  action_planner.addEntitySet("location", 2);

  /////////////////////////
  // Build the action space
  /////////////////////////
  if (!action_planner.buildActionSpace())
  {
    printf("Error: Failed to build action space.");
    return 1;
  }

  std::cout << action_planner.description() << std::endl;

  /////////////////////////////////////////
  // Apply the action plan via task manager
  /////////////////////////////////////////
  goap::TaskManager task_manager(action_planner);

  ////////////////////////////
  // Set start and goal states
  ////////////////////////////
  task_manager.setStartStateProposition("At(Husky, location_1)", true);

  task_manager.setGoalStateProposition("At(Husky, location_2)", true);

  ////////////////////////////////////////////////////
  // Associate the callback functions with the actions
  ////////////////////////////////////////////////////
  // This callback takes 2 arguments, hence the _2
  // Put 2 placeholders for those arguments
  goap::TaskManager::ActionCallback_2<std::string, std::string> navigate_robot_func =
    std::bind(&navigateRobotCallback, std::placeholders::_1, std::placeholders::_2);
  std::vector<std::string> robot_name;
  robot_name.push_back("Husky");
  // Need a vector of arguments for each location entity, e.g. location_1 and location_2
  std::vector<std::string> destination_label;
  destination_label.push_back("location_1");
  destination_label.push_back("location_2");
  task_manager.setActionCallback("NavigateRobot(r:robot, destination:location)", navigate_robot_func, robot_name, destination_label );

  ////////////////////////////
  // Create and print the plan
  ////////////////////////////
  // By default, costs for each action are 0
  int cost = 0;
  std::vector<goap::Worldstate> states;
  task_manager.buildPlan(&cost, &states);

  std::string desc;
  printf("ACTION PLAN\nPlan cost = %d\n", cost);
  printf("States:\n");
  for (int i = 0; i < states.size(); ++i)
  {
    desc = action_planner.worldstateDescription(&states.at(i));
    printf(desc.c_str());
  }
  printf("Actions:\n");
  goap::action_plan_t actions = task_manager.getPlan();
  for (int i = 0; i < states.size(); ++i)
  {
    printf(actions.at(i).c_str());
  }
  printf("\n");

  //////////////////////
  // Run the action plan
  //////////////////////
  task_manager.runPlan();
}
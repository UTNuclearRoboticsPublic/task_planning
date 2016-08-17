///////////////////////////////////////////////////////////////////////////////
//      Title     : task_manager.cpp
//      Project   : task_planning
//      Created   : 5/25/2016
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

#include "task_planning/task_manager.h"

#include <iostream>

namespace goap {

TaskManager::TaskManager(ActionPlanner planner) :
  planner_(planner)
{
  action_counter_ = plan_.end();
  
  // Initialize the worldstates
  planner_.updateWorldstateSize(&start_state_);
  planner_.updateWorldstateSize(&current_state_);
  planner_.updateWorldstateSize(&goal_state_);
}

bool TaskManager::buildPlan(int* cost, std::vector<Worldstate>* worldstates)
{
  int plan_cost = planner_.createPlan(start_state_, goal_state_, &plan_, worldstates);
  
  if (plan_cost != -1) {
    action_counter_ = plan_.begin();
  }
  
  *cost = plan_cost;
  
  return plan_cost != -1;
}


bool TaskManager::runPlan()
{
  action_counter_ = plan_.begin();
  current_state_ = start_state_;
  
  // Iterate until end of action stack
  while (action_counter_ != plan_.end()) {
    
    // Verify that preconditions are met. If not, skip this action
    bool is_possible;
    planner_.isActionPossible(&current_state_, *action_counter_, &is_possible);
    if (!is_possible) {
      std::cout << "Error in TaskManager::runPlan. Preconditions not met for action " << *action_counter_ << std::endl;
      action_counter_++;
    }
    else {
      preActionFunction();
      RETURN_TYPE ret = executeAction(*action_counter_);
      
      // Respond to the RETURN_TYPE of the action.
      switch (ret) {
	case CONTINUE:
	  // Update worldstate
	  planner_.doAction(*action_counter_, &current_state_, &current_state_);
	  
	  // Move to next action
	  action_counter_++;
	  break;
	case CONTINUE_FAILED:
	  action_counter_++;
	  break;
	case REPEAT:
	  // Do not increment the action stack counter
	  break;
	case BAD_CALL:
	  std::cout << "In TaskManager::runPlan. Action " << *action_counter_ << " returned BAD_CALL. Exiting plan.\n";
	  // We do not break, so we continue to case FAIL_TERMINATE.
	case TERMINATE:
	  action_counter_ = plan_.end();
	  return false;
	case REPLAN:
	{
	  int cost;
	  std::vector<Worldstate> worldstates;
	  start_state_ = current_state_;
	  
	  // Try to build new plan
	  if (buildPlan(&cost, &worldstates)) {
	    printf("New Action Plan:\n");
	    for (int i = 0; i < plan_.size(); ++i) {
	      printf("%d: %s\n", i, plan_.at(i).c_str());
	    }
	    
	    // Start at beginning of new plan
	    action_counter_ = plan_.begin();
	    break;
	  }
	  else {
	    // No valid plan
	    std::cout << "Error in TaskManager::runPlan. Failed to replan. Valid plan not found. Exiting plan.\n";
	    action_counter_ = plan_.end();
	    return false;
	  }
	}
	default:
	  // Invalid return type
	  std::cout << "Error in TaskManager::runPlan. Unhandled return type " << ret << ". Exiting plan.\n";
	  action_counter_ = plan_.end();
	  return false;
      }
    }
  }
  
  // We reached the end of the action stack
  return true;
}


bool TaskManager::setActionCallback(std::string action_name, ActionCallback func)
{
  // Check that action exists in the planner
  if (planner_.findAction(action_name) == NULL) {
    std::cout << "Error in TaskManager::setActionCallback. Action " << action_name << " not found.\n";
    return false;
  }
  
  callback_funcs_[action_name] = func;
  return true;
}


bool TaskManager::setStartStateProposition(std::string atom, bool value)
{
  return planner_.setProposition(&start_state_, atom, value);
}

bool TaskManager::setCurrentStateProposition(std::string atom, bool value)
{
  return planner_.setProposition(&current_state_, atom, value);
}

bool TaskManager::setGoalStateProposition(std::string atom, bool value)
{
  return planner_.setProposition(&goal_state_, atom, value);
}

void TaskManager::clearStartState()
{
  start_state_.clear();
}

void TaskManager::clearCurrentState()
{
  current_state_.clear();
}

void TaskManager::clearGoalState()
{
  goal_state_.clear();
}

action_plan_t TaskManager::getPlan() const
{
  return plan_;
}

const goap::ActionPlanner& TaskManager::getPlanner() const
{
  return planner_;
}

bool TaskManager::getCurrentStateProposition(std::string atom, bool* result) const
{
  return planner_.getPropositionValue(&current_state_, atom, result);
}

void TaskManager::setStartStatetoCurrent()
{
  start_state_ = current_state_;
}

std::string TaskManager::getCurrentAction() const
{
  if (action_counter_ != plan_.end()) {
    return *action_counter_;
  }
  
  return "";
}

TaskManager::RETURN_TYPE TaskManager::executeAction(std::string action) const
{
  ActionCallback function;
  
  // Check that the function exists
  try {
    function = callback_funcs_.at(action);
  } catch(std::out_of_range) {
    return BAD_CALL;
  }
  
  // Execute callback function and return the result
  return function();
}

void TaskManager::preActionFunction() const
{
  std::cout << "Running action " << *action_counter_ << ".\n";
}

} // namespace goap
///////////////////////////////////////////////////////////////////////////////
//      Title     : task_manager.tpp
//      Project   : task_planning
//      Created   : 6/12/2016
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

// Contains the template definitions for TaskManager

template <typename A>
bool TaskManager::setActionCallback(std::string action_name, ActionCallback_1<A> func, std::vector<A> first)
{
  const std::size_t arrity = 0;
  
  ActionPlanner::action_vector permuted_actions;
  
  if (!actionPermuteHelper(action_name, &permuted_actions)) {
    return false;
  }
  
  BOOST_FOREACH (ActionPlanner::Action action, permuted_actions) {
    const ActionPlanner::Action::parameter* param;
    try {
      param = &action.params.at(arrity);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Expected action with " << arrity+1 << " parameter(s)." << std::endl;
      return false;
    }
    
    // Get index of parameter in entity list
    std::vector<std::string>::const_iterator it = std::find(param->set_members->begin(), param->set_members->end(), param->label);
    std::size_t index = it - param->set_members->begin();

    A* func_arg;
    try {
      func_arg = &first.at(index);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Received insufficient arguments in vector." << std::endl;
      return false;
    }

    ActionCallback new_func = std::bind(func, *func_arg);
    if (!setActionCallback(action.name, new_func)) {
      return false;
    }
  }
  return true;
}


template <typename A, typename B>
bool TaskManager::setActionCallback(std::string action_name, ActionCallback_2<A,B> func, std::vector<A> first, std::vector<B> second)
{
  const std::size_t arrity = 1;
  
  ActionPlanner::action_vector permuted_actions;
  
  if (!actionPermuteHelper(action_name, &permuted_actions)) {
    return false;
  }
  
  BOOST_FOREACH (ActionPlanner::Action action, permuted_actions) {
    const ActionPlanner::Action::parameter* param;
    try {
      param = &action.params.at(arrity);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Expected action with " << arrity+1 << " parameter(s)." << std::endl;
      return false;
    }
    
    // Get index of parameter in entity list
    std::vector<std::string>::const_iterator it = std::find(param->set_members->begin(), param->set_members->end(), param->label);
    std::size_t index = it - param->set_members->begin();

    B* func_arg;
    try {
      func_arg = &second.at(index);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Received insufficient arguments in vector." << std::endl;
      return false;
    }

    ActionCallback_1<A> new_func = std::bind(func, std::placeholders::_1, *func_arg);
    if (!setActionCallback(action.name, new_func, first)) {
      return false;
    }
  }
  return true;
}


template <typename A, typename B, typename C>
bool TaskManager::setActionCallback(std::string action_name, ActionCallback_3<A,B,C> func, std::vector<A> first, std::vector<B> second, std::vector<C> third)
{
  const std::size_t arrity = 2;
  
  ActionPlanner::action_vector permuted_actions;
  
  if (!actionPermuteHelper(action_name, &permuted_actions)) {
    return false;
  }
  
  BOOST_FOREACH (ActionPlanner::Action action, permuted_actions) {
    const ActionPlanner::Action::parameter* param;
    try {
      param = &action.params.at(arrity);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Expected action with " << arrity+1 << " parameter(s)." << std::endl;
      return false;
    }
    
    // Get index of parameter in entity list
    std::vector<std::string>::const_iterator it = std::find(param->set_members->begin(), param->set_members->end(), param->label);
    std::size_t index = it - param->set_members->begin();

    C* func_arg;
    try {
      func_arg = &third.at(index);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Received insufficient arguments in vector." << std::endl;
      return false;
    }

    ActionCallback_2<A,B> new_func = std::bind(func, std::placeholders::_1, std::placeholders::_2, *func_arg);
    if (!setActionCallback(action.name, new_func, first, second)) {
      return false;
    }
  }
  return true;
}


template <typename A, typename B, typename C, typename D>
bool TaskManager::setActionCallback(std::string action_name, ActionCallback_4 <A,B,C,D> func, std::vector<A> first, std::vector<B> second, std::vector<C> third, std::vector<D> fourth)
{
  const std::size_t arrity = 3;
  
  ActionPlanner::action_vector permuted_actions;
  
  if (!actionPermuteHelper(action_name, &permuted_actions)) {
    return false;
  }
  
  BOOST_FOREACH (ActionPlanner::Action action, permuted_actions) {
    const ActionPlanner::Action::parameter* param;
    try {
      param = &action.params.at(arrity);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Expected action with " << arrity+1 << " parameter(s)." << std::endl;
      return false;
    }
    
    // Get index of parameter in entity list
    std::vector<std::string>::const_iterator it = std::find(param->set_members->begin(), param->set_members->end(), param->label);
    std::size_t index = it - param->set_members->begin();

    D* func_arg;
    try {
      func_arg = &fourth.at(index);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Received insufficient arguments in vector." << std::endl;
      return false;
    }

    ActionCallback_3<A,B,C> new_func = std::bind(func, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, *func_arg);
    if (!setActionCallback(action.name, new_func, first, second, third)) {
      return false;
    }
  }
  return true;
}

template <typename A, typename B, typename C, typename D, typename E>
bool TaskManager::setActionCallback(std::string action_name, ActionCallback_5<A,B,C,D,E> func, std::vector<A> first, std::vector<B> second, std::vector<C> third, std::vector<D> fourth, std::vector<E> fifth)
{
  const std::size_t arrity = 4;
  
  ActionPlanner::action_vector permuted_actions;
  
  if (!actionPermuteHelper(action_name, &permuted_actions)) {
    return false;
  }
  
  BOOST_FOREACH (ActionPlanner::Action action, permuted_actions) {
    const ActionPlanner::Action::parameter* param;
    try {
      param = &action.params.at(arrity);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Expected action with " << arrity+1 << " parameter(s)." << std::endl;
      return false;
    }
    
    // Get index of parameter in entity list
    std::vector<std::string>::const_iterator it = std::find(param->set_members->begin(), param->set_members->end(), param->label);
    std::size_t index = it - param->set_members->begin();

    E* func_arg;
    try {
      func_arg = &fifth.at(index);
    } catch (std::out_of_range) {
      std::cout << "TaskManager::setActionCallback failed. Received insufficient arguments in vector." << std::endl;
      return false;
    }

    ActionCallback_4<A,B,C,D> new_func = std::bind(func, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, *func_arg);
    if (!setActionCallback(action.name, new_func, first, second, third, fourth)) {
      return false;
    }
  }
  return true;
}

bool TaskManager::actionPermuteHelper(std::string action_name, ActionPlanner::action_vector* permuted_actions)
{
  
  permuted_actions->clear();
  
  // Look in built actions
  const ActionPlanner::Action* action_ptr = planner_.findAction(action_name);
  
  // If found
  if (action_ptr != NULL) {
    permuted_actions->push_back(*action_ptr);
  }
  else {
    // Look in unbuilt actions
    action_ptr = planner_.findUnbuiltAction(action_name);
    // If found
    if (action_ptr != NULL) {
      ActionPlanner::Action action = *action_ptr;
      
      if (!planner_.linkEntities(action)) {
	return false;
      }
      
      // Permute
      if (!planner_.permuteAction(action, permuted_actions)) {
	return false;
      }
      
    }
    else {
      std::cout << "setActionCallback() - Action " << action_name << " not found." << std::endl;
      return false;
    }
  }
  
  return true;
}
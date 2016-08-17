///////////////////////////////////////////////////////////////////////////////
//      Title     : action.cpp
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

#include <task_planning/action_planner.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <sstream>

namespace goap {

ActionPlanner::Action::Action() {
  cost = 0;
  cost_func = NULL;
}

ActionPlanner::Action::Action(int num_atoms) :
  Action()
{
  pre.initialize(num_atoms);
  pst.initialize(num_atoms);
}

bool ActionPlanner::Action::parse(std::string definition, string_vec preconditions, string_vec effects)
{
  /** Parse the definition **/
  boost::trim(definition);
  
  // Find the start of the parameter list
  std::size_t param_start = definition.find('(');
  if (param_start == std::string::npos) {
    printf("Malformed action %s. '(' expected.\n", definition.c_str());
    return false;
  }
  
  if (param_start == 0) {
    printf("Malformed action %s. Missing root.\n", definition.c_str());
    return false;
  }
  
  std::size_t param_end = definition.find(')', param_start+1);
  if (param_end == std::string::npos) {
    printf("Malformed action %s. ')' expected.\n", definition.c_str());
    return false;
  }
  
  if (param_end < definition.size()-1) {
    printf("Malformed action %s. Characters found after param list.\n", definition.c_str());
    return false;
  }
  
  root = definition.substr(0, param_start);
  name = definition;
  
  // Tokenize the parameters
  std::string param_list = definition.substr(param_start+1);
  param_list.pop_back();
  
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

  boost::char_separator<char> sep(",");
  tokenizer tokens(param_list, sep);

   for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
    std::string param = *tok_iter;
    
    // Separate the label and type
    std::size_t param_separator = param.find(':');
    std::string label = param.substr(0, param_separator);
    std::string type = param.substr(param_separator+1);
    
    boost::trim(label);
    boost::trim(type);
    
    // check if this parameter already exists
    parameter* new_param = getParameter(label);
    if (new_param != NULL) {
      printf("Malformed action %s. Multiple params with label %s.\n", definition.c_str(), label.c_str());
      return false;
    }
    
    // build parameter struct and add to list
    new_param = new parameter;
    new_param->label = label;
    new_param->type = type;
    new_param->set_members = NULL;
    params.push_back(*new_param);
    delete new_param;
  }
  
  /** Add preconditions **/  
  BOOST_FOREACH(std::string precondition, preconditions) {
    if (!buildPrecondition(precondition)) {
      return false;
    }
  }
  
  /** Add effects **/
  if (effects.empty()) {
    printf("Malformed action, no effects found.\n");
    return false;
  }
  BOOST_FOREACH(std::string effect, effects) {
    if (!buildEffect(effect)) {
      return false;
    }
  }
  
  return true;
}

bool ActionPlanner::Action::buildPrecondition(std::string definition)
{
  boost::trim(definition);
  
  bool req_value;
  
  // Check if first char is '~' indicating NOT
  if (definition.at(0) == '~') {
    definition.erase(0,1); // erase first character
    req_value = false;
  }
  else {
    req_value = true;
  }
  
  // Parse the definition
  Proposition new_prop;
  if (!new_prop.parse(definition)) {
    return false;
  }
  
  preconditions.push_back(condition_t(new_prop, req_value));
  
  return true;
}

bool ActionPlanner::Action::buildEffect(std::string definition)
{
  boost::trim(definition);
  
  bool set_value;
  
  // Check if first char is '~' indicating NOT
  if (definition.at(0) == '~') {
    definition.erase(0,1); // erase first character
    set_value = false;
  }
  else {
    set_value = true;
  }
  
  // Parse the definition
  Proposition new_prop;
  if (!new_prop.parse(definition)) {
    return false;
  }
  
  effects.push_back(condition_t(new_prop, set_value));
  
  return true;
}

void ActionPlanner::Action::buildName()
{
  std::stringstream output;
  
  output << root << "(";
  
  for (Action::param_vec::iterator param = params.begin(); param != params.end(); param++) {
    if (param != params.begin()) {
      output << ", ";
    }
    
    output << param->label;
  }
  
  output << ")";
  
  name = output.str();
}

void ActionPlanner::Action::set_cost(int val)
{
  cost = val;
  cost_func = NULL;
}

void ActionPlanner::Action::set_cost(cost_function func)
{
  cost_func = func;
}

int ActionPlanner::Action::get_cost(const ActionPlanner* ap, const Worldstate* ws) const
{
  if (cost_func != NULL) {
    return cost_func(ap, ws);
  }
  return cost;
}

ActionPlanner::Action::parameter* ActionPlanner::Action::getParameter(param_label label)
{
  parameter* param = NULL;
  param_vec::iterator it = std::find(params.begin(), params.end(), label);
  
  if (it != params.end()) {
    param = &(*it);
  }
  
  return param;
}

} // namespace goap
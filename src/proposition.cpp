///////////////////////////////////////////////////////////////////////////////
//      Title     : proposition.cpp
//      Project   : task_planning
//      Created   : 6/3/2016
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
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <sstream>

namespace goap {

bool ActionPlanner::Proposition::parse(std::string definition)
{
  boost::trim(definition);
  
  // Find the start of the parameter list
  std::size_t param_start = definition.find('(');
  if (param_start == std::string::npos) {
    printf("Malformed Proposition, '(' expected: %s\n", definition.c_str());
    return false;
  }
  
  if (param_start == 0) {
    printf("Malformed Proposition, missing root: %s\n", definition.c_str());
    return false;
  } 
  
  std::size_t param_end = definition.find(')', param_start+1);
  if (param_end == std::string::npos) {
    printf("Malformed Proposition, ')' expected: %s\n", definition.c_str());
    return false;
  }
  
  if (param_end < definition.size()-1) {
    printf("Malformed Proposition, characters found after param: %s\n", definition.c_str());
    return false;
  }
  
  // Tokenize the parameters
  std::string param_list = definition.substr(param_start+1);
  param_list.pop_back(); // Remove ')'
  
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

  boost::char_separator<char> sep(",");
  tokenizer tokens(param_list, sep);

   for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
    std::string param = *tok_iter;
    
    parameter new_param;
    if (!parseParam(param, &new_param)) {
      printf("Malformed Proposition, failed to parse param: %s\n", param.c_str());
      return false;
    }
    
    params.push_back(new_param);
  }
  
  root = definition.substr(0, param_start);
  name = definition;
  
  return true;
}

void ActionPlanner::Proposition::buildName()
{
  std::stringstream output;
  
  output << root << "(";
  
  for (size_t i = 0, size = params.size(); i < size; ++i) {
    if (i != 0) {
      output << ", ";
    }
    
    output << params.at(i).label;
  }
  
  output << ")";
  name = output.str();
}

bool ActionPlanner::Proposition::parseParam(std::string text, parameter* parameter)
{
  boost::trim(text);
  
  parameter->negated = false;
  parameter->all = false;
  parameter->literal = false;
  parameter->set_members = NULL;
  
  // Check if empty
  if (text.empty()) {
    return true;
  }
  
  // Look for ':' separator and parse the two parts
  std::size_t separator = text.find(':');
  bool separated = (separator != std::string::npos);
  
  if (separated) {
    parameter->entity_set_name = text.substr(separator+1);
    boost::trim(parameter->entity_set_name);
    
    if (parameter->entity_set_name.empty()) {
      printf("Malformed Proposition, empty string following ':'. Expected parameter type.\n.");
      return false;
    }
  }
  
  parameter->label = text.substr(0, separator); // is entire string if no separator
  boost::trim(parameter->label);
  if (parameter->label.empty()) {
    printf("Malformed Proposition, empty parameter label.\n.");
    return false;
  }
  
  // Parse for negated parameter
  if (parameter->label.front() == '~') {
    
    parameter->negated = true;
    parameter->label.erase(0,1); // Remove the '~'
    
    if (parameter->label.empty()) {
      printf("Malformed Proposition, negated parameter must reference an entity set.\n.");
      return false;
    }
  } // end negated parsing
  
  // Parse for literal
  if (parameter->label.front() == '$') {
    parameter->literal = true;
    parameter->label.erase(0,1); // remove the '$'

    // Must have entity set
    if (!separated) {
      printf("Malformed Proposition, literal parameter must reference an entity set.\n");
      return false;
    }
  } // end literal parsing
  
  
  // Parse for all parameter
  if (parameter->label == "ALL") {
    
    // Cannot combine all with negated
    if (parameter->negated) {
      printf("Malformed Proposition, can't combine ""ALL"" with ""~""\n");
      return false;
    }
    
    // Cannot combine all with literal
    if (parameter->literal) {
      printf("Malformed Proposition, can't combine ""ALL"" with ""$"" in parameter.\n");
      return false;
    }
    
    // Must have entity set
    if (!separated) {
      printf("Malformed Proposition, ""ALL"" parameter must reference an entity set.\n");
      return false;
    }
    
    parameter->all = true;
  } // end ALL parsing
  
  
  return true;
}

ActionPlanner::Proposition::parameter* ActionPlanner::Proposition::getParameter(std::string value)
{
  parameter* param = NULL;
  std::vector<parameter>::iterator it = std::find(params.begin(), params.end(), value);
  
  if (it != params.end()) {
    param = &(*it);
  }
  
  return param;
}

} // namespace goap
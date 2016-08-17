///////////////////////////////////////////////////////////////////////////////
//      Title     : action_planner.cpp
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
#include <jsoncpp/json/json.h>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <algorithm>
#include <sstream>
#include <fstream>
#include <iostream>
#include <unordered_set>

namespace goap {
  
/**
 * @breif Produces permutation and then increments the counter. Run this in a do...while loop to run the counter forward.
 * @param v Vector of vector references containing the elements to be permuted.
 * @param it Counter vector. It is incremented by one.
 * @return True if we have not reached the end of the permutations.
 */
template <typename T>
bool next_permutation(const std::vector<const std::vector<T>*>& v, std::vector<std::size_t>& it)
{  
  // Increment the counter
  for (std::size_t i = 0, size = it.size(); i != size; ++i) {
    const std::size_t index = size - 1 - i;
    ++it[index];
    if (it[index] == v[index]->size()) {
      it[index] = 0;
    } else {
      return true;
    }
  }
  return false;
}



/**
 * @breif Determine if all elements in a container are unique
 * @param first Front of range
 * @param last End of range
 * @return True if all elements in the range are unique
 */
template <class FwdIterator>
bool check_unique(FwdIterator first, FwdIterator last)
{
    FwdIterator result = first;
    std::unordered_set<typename FwdIterator::value_type> seen;

    for (; first != last; ++first) {
        if (!seen.insert(*first).second) {
            return false;
	}
    }
    return true;
}




int ActionPlanner::createPlan
  (
    const Worldstate& start,
    const Worldstate& goal,
    action_plan_t* plan,
    std::vector<Worldstate>* worldstates
  ) const
{
  NodeMap opened;	// The set of nodes we should consider.
  NodeMap closed;	// The set of nodes we already visited.

  // create start node
  astarnode_t* n0 = new astarnode_t;
  n0->ws = start;
  n0->parentws = new Worldstate;
  *n0->parentws = start;
  n0->g = 0;
  n0->f = n0->g + calcH(&start, &goal);
  n0->actionname = "start";
  opened().insert(n0);
  
  while (!opened().empty()) {

    const NodeMap::sequence_iter lowest_rank = getLowestRank(&opened);
    
    // remove the node with the lowest rank
    astarnode_t* cur;
    cur = *lowest_rank;
    
    opened.getSequence().erase(lowest_rank);
    
    // if it matches the goal, we are done!
    if ( (cur->ws.values & goal.care) == (goal.values & goal.care) ) {
      reconstructPlan(cur, &closed, plan, worldstates);
      return cur->f;
    }
    // add it to closed
    closed().insert(cur);
    
    // iterate over neighbours
    std::vector<const std::string*> actionnames;
    std::vector<int> actioncosts;
    std::vector<Worldstate> to;

    const int numtransitions = getPossibleStateTransitions(&cur->ws, &to, &actionnames, &actioncosts);

    for (int i=0; i<numtransitions; ++i) {
      const float cost = cur->g + actioncosts[i];
      
      // if neighbor in CLOSED and cost less than g(neighbor):
      NodeMap::hash_iter closed_it = closed.getHashed().find(to[i]);
      if (closed_it != closed().end()) {
	if (cost < (**closed_it).g) {
	  // remove neighbor from CLOSED
	  if (!closed().empty()) {
	    (**closed_it).g = cost;
	    (**closed_it).f = cost + calcH(&to[i], &goal);
	    (**closed_it).parentws = &cur->ws;
	    opened().insert(*closed_it);
	    closed().erase(closed_it);
	  }
	}
	else continue;
      }
      else {
	// if neighbor in OPEN and cost less than g(neighbor):
	NodeMap::hash_iter opened_it = opened.getHashed().find(to[i]);
	if (opened_it != opened().end()) {
	  if (cost < (**opened_it).g) {
	    // remove neighbor from OPEN, because new path is better
	    if (!opened().empty()) {
	      (**opened_it).g = cost;
	      (**opened_it).f = cost + calcH(&to[i], &goal);
	      (**opened_it).parentws = &cur->ws;
	    }
	  }
	  else continue;
	}
	else {
	  // if neighbor not in OPEN and neighbor not in CLOSED:
	  astarnode_t* new_node = new astarnode_t;
	  new_node->ws = to[i];
	  new_node->g = cost;
	  new_node->f = cost + calcH(&to[i], &goal);
	  new_node->actionname = *(actionnames[i]);
	  new_node->parentws = &cur->ws;
	  opened().insert(new_node);
	}
      }
      
      // if neighbor not in OPEN and neighbor not in CLOSED:
//       astarnode_t new_node;
//       new_node.ws = to[i];
//       new_node.g = cost;
//       new_node.f = cost + calcH(&to[i], &goal);
//       new_node.actionname = *(actionnames[i]);
//       new_node.parentws = cur.ws;
//       opened.insert(new_node);
    }
  } // end while
  
  printf("Error in ActionPlanner::createPlan - Path not found.\n");
  return -1;
}



bool ActionPlanner::buildActionSpace()
{
  // Clear built action list
  actions_.clear();
  
  // For each Action
  BOOST_FOREACH (Action& action, unbuilt_actions_) {
    
    if (!linkEntities(action)) {
      return false;
    }
    
    // handle the "all" propositions
    if (!expandAllParams(action)) {
      return false;
    }
    
    // Build permutations of Action
    action_vector permuted_actions;
    if (!permuteAction(action, &permuted_actions)) {
      printf("ActionPlanner::buildActionSpace - Permutation failed for action %s.", action.name.c_str());
      return false;
    }
    
    // handle the negated propositions
    BOOST_FOREACH (Action& action, permuted_actions) {
      if (!expandNegatedParams(action)) {
	return false;
      }
    }

    // Add permuted Actions to list (addAction guarantees that actions will be unique)
    actions_.splice(actions_.end(), permuted_actions);
  }
  
  // Set pre and pst of built actions
  BOOST_FOREACH (Action& action, actions_) {
    
    // Set pre
    BOOST_FOREACH (Action::condition_t precondition, action.preconditions) {
      
      if (!setPrecondition(action.name, precondition.first.name, precondition.second)) {
	printf("ActionPlanner::buildActionSpace - Setting pre failed for action %s.", action.name.c_str());
	return false;
      }
    }
    
    // Set effects
    BOOST_FOREACH (Action::condition_t effect, action.effects) {
      if (!setEffect(action.name, effect.first.name, effect.second)) {
	printf("ActionPlanner::buildActionSpace - Setting pst failed for action %s.", action.name.c_str());
	return false;
      }
    }
  }
  
  return true;
}



void ActionPlanner::clear()
{
  proposition_index_.clear();
  entities_.clear();
  actions_.clear();
  unbuilt_actions_.clear();
}



bool ActionPlanner::addAction(std::string definition, std::vector<std::string> preconditions, std::vector<std::string> effects)
{
  Action new_action;
  if (!new_action.parse(definition, preconditions, effects)) {
    printf("ActionPlanner::addAction - Failed to add action %s.", definition.c_str());
    return false;
  }
  
  unbuilt_actions_.push_back(new_action);
  
  return true;
}



bool ActionPlanner::addEntitySet(std::string name, std::vector<std::string> members)
{
  // Validate
  if (members.empty()) {
    printf("Error in ActionPlanner::addEntitySet: No members received.");
    return false;
  }
  
  if (name.empty()) {
    printf("Error in ActionPlanner::addEntitySet: Empty name received.");
    return false;
  }
  
  // Verify that all member names are unique
  if (!check_unique(members.begin(), members.end())) {
    printf("Error in ActionPlanner::addEntitySet: Member list contains duplicate elements.");
    return false;
  }
  
  // Check if a set with this name exists
  EntitySet* new_set = findEntitySet(name);
  if (new_set == NULL) {
    // Set does not already exist. Create new one
    new_set = new EntitySet;
    new_set->name = name;
    new_set->members = members;
    entities_.push_front(*new_set);
    delete new_set;
  }
  else {
    // Set already exists. Update members.
    new_set->members = members;
  }
  
  return true;
}



bool ActionPlanner::addEntitySet(std::string name, int num)
{
  // Validate
  if (num < 1) {
    printf("Error in ActionPlanner::addEntitySet: num must be positive. Received %d.\n", num);
    return false;
  }
  
  // Build member list with generic names
  string_vec members;
  members.reserve(num);
  for (int i = 1; i <= num; ++i) {
    members.push_back(name + "_" + std::to_string(i));
  }
  
  return addEntitySet(name, members);
}



bool ActionPlanner::setPrecondition(std::string actionname, std::string name, bool value)
{
  Action* action = findAction(actionname);
  if (!action) {
    return false;
  }
  
  int proposition_index;
  addProposition(name);

  return setProposition(&action->pre, name, value);
}



bool ActionPlanner::setEffect(std::string actionname, std::string name, bool value)
{
  Action* action = findAction(actionname);
  if (!action) {
    return false;
  }
  
  int proposition_index;
  addProposition(name);
  
  return setProposition(&action->pst, name, value);
}



bool ActionPlanner::setActionCost(std::string actionname, int cost)
{
  Action* action = findAction(actionname);
  if (!action) {
    printf("Error in ActionPlanner::setActionCost: Action %s not found in action planner.\n", actionname.c_str());
    return false;
  }
  
  action->set_cost(cost);
  return true;
}



bool ActionPlanner::setActionCost(std::string actionname, int (*func)(const ActionPlanner* ap, const Worldstate* ws))
{
  Action* action = findAction(actionname);
  if (!action) {
    printf("Error in ActionPlanner::setActionCost: Action %s not found in action planner.\n", actionname.c_str());
    return false;
  }
  
  action->set_cost(func);
  return true;
}



bool ActionPlanner::setProposition(Worldstate* ws, std::string name, bool value) const
{
  // See if name matches any existing atoms
  int index;
  if(!findPropositionIndex(name, &index)) {
    
    // See if name can be interpreted via ADL
    if (setPropositionADL(ws, name, value)) {
      return true;
    }
    
    printf("Error in ActionPlanner::setProposition: Proposition %s not found in action planner.\n", name.c_str());
    return false;
  }
  
  updateWorldstateSize(ws);
  
  ws->values[index] = value;
  ws->care[index] = true;

  return true;
}



bool ActionPlanner::setPropositionADL(Worldstate* ws, std::string name, bool value) const
{
  Worldstate new_ws = *ws;
  
  Proposition prop;
  if (!prop.parse(name)) {
    return false;
  }
  
  Action temp_action;
  temp_action.preconditions.push_back(Action::condition_t(prop, value));
  
  if (!linkEntities(temp_action)) {
    return false;
  }
  
  // handle the "all" propositions
  if (!expandAllParams(temp_action)) {
    return false;
  }
  
  // handle the negated propositions
  if (!expandNegatedParams(temp_action)) {
    return false;
  }
  
  std::vector<Action::condition_t> created_conditions = temp_action.preconditions;
  
  BOOST_FOREACH (Action::condition_t& condition, created_conditions) {
    condition.first.buildName();
    
    int index;
    if(!findPropositionIndex(condition.first.name, &index)) {
      return false;
    }
    
    updateWorldstateSize(&new_ws);
    
    new_ws.values[index] = condition.second;
    new_ws.care[index] = true;
  }

  *ws = new_ws;
  return true;
}



bool ActionPlanner::getPropositionValue(const Worldstate* ws, std::string name, bool* result) const
{
  int index;
  if(!findPropositionIndex(name, &index)) {
    printf("Error in GOAP::getPropositionValue: Proposition %s not found in action planner.\n", name.c_str());
    return false;
  }
  
  *result = ws->values[index];
  return true;
}



bool ActionPlanner::isActionPossible(const Worldstate* ws, std::string action_name, bool* result) const
{
  const Action* action = findAction(action_name);
  if (!action) {
    printf("ActionPlanner::isActionPossible: Action %s not found in action planner.\n", action_name.c_str());
    return false;
  }
  
  const Worldstate* pre = &action->pre;
  *result = (pre->values & pre->care) == (ws->values & pre->care);
  return true;
}



bool ActionPlanner::doAction(std::string actionname, const Worldstate* fr, Worldstate* to) const
{
  const Action* action = findAction(actionname);
  if (action) {
    doActionInternal(action, fr, to);
    return true;
  }
  
  return false;
}



std::string ActionPlanner::description() const
{
  std::stringstream sstream;
  
  // Print the proposition list
  sstream << "Propositions:";
  //for (prop_index_map_t::right_const_iterator it = proposition_index_.right.begin(); it != proposition_index_.right.end(); it++) {
  for (std::size_t i = 0, size = proposition_index_.size(); i < size; ++i) {
    sstream << " " << proposition_index_.left.at(i);
  }
  sstream << "\n\n";
  
  // Print the actions
  BOOST_FOREACH (const Action& action, actions_) {
    
    const Worldstate* pre = &(action.pre);
    const Worldstate* pst = &(action.pst);
    
    // Print preconditions
    sstream << "Action: " << action.name << "\nPreconditions:\n";
    BOOST_FOREACH (Action::condition_t precondition, action.preconditions) {
      sstream << "\t" << precondition.first.name << ": " << precondition.second << "\n";
    }
    
    // Print effects
    sstream << "Effects:\n";
    BOOST_FOREACH (Action::condition_t effect, action.effects) {
      sstream << "\t" << effect.first.name << ": " << effect.second << "\n";
    }
    
    sstream << "\n";
  }
  return sstream.str();
}



std::string ActionPlanner::worldstateDescription(const Worldstate* ws) const
{
  std::stringstream sstream;
  // Iterate through all propositions in planner
  for (uint i = 0; i < proposition_index_.size(); i++) {
    
    // Ignore if care == false
    if (ws->care[i]) {
      
      std::string proposition_name = proposition_index_.left.at(i);
      
      // Add ~ prefix for false
      if (!ws->values[i]) {
	proposition_name = "~" + proposition_name;
      }
      
      sstream << proposition_name << ", ";
    }
  }
  sstream << std::endl;
  
  return sstream.str();
}



void ActionPlanner::updateWorldstateSize(Worldstate* ws) const
{
  ws->initialize(proposition_index_.size());
}



ActionPlanner::Action* ActionPlanner::addAction(std::string actionname)
{
  Action* new_action = findAction(actionname);
  if (!new_action) {
    new_action = new Action(proposition_index_.size());
    new_action->name = actionname;
    actions_.push_back(*new_action);
    
    delete new_action;
    new_action = &actions_.back();
  }
  
  return new_action;
}


  
bool ActionPlanner::addProposition(std::string name)
{  
  // See if proposition already exists
  try {
    proposition_index_.right.at(name);
  } catch (std::out_of_range) { // Does not exist
    proposition_index_.insert( prop_index_t(proposition_index_.size(), name) );
    
    // Update size of all action preconditions and postconditions
    for (action_vector::iterator it = actions_.begin(); it != actions_.end(); ++it) {
      updateWorldstateSize(&it->pre);
      updateWorldstateSize(&it->pst);
    }
    
    return true;
  }
    
  return false;
}



ActionPlanner::NodeMap::sequence_iter ActionPlanner::getLowestRank(NodeMap* nodes) const
{
  NodeMap::sequence_iter lowest_rank = nodes->getSequence().begin();
  for (NodeMap::sequence_iter it = nodes->getSequence().begin(), end = nodes->getSequence().end(); it != end; ++it) {
    if ((**lowest_rank).f > (**it).f)
      lowest_rank = it;
  }
  return lowest_rank;
}



void ActionPlanner::reconstructPlan(astarnode_t* goalnode, NodeMap* nodes, action_plan_t* plan, std::vector<Worldstate>* worldstates) const
{
  plan->clear();
  worldstates->clear();
  
  const astarnode_t* curnode = goalnode;
  while (curnode != NULL && curnode->actionname != "start") {
    plan->insert(plan->begin(), curnode->actionname);
    worldstates->insert(worldstates->begin(), curnode->ws);
    
    const NodeMap::hash_iter it = nodes->getHashed().find(*curnode->parentws);
    if (it != nodes->getHashed().end())
      curnode = *it;
    else
      curnode = NULL;
  }
}



int ActionPlanner::getPossibleStateTransitions(const Worldstate* fr, std::vector<Worldstate>* to, std::vector<const std::string*>* actionnames, std::vector<int>* actioncosts) const
{
  Worldstate to_ws;
  updateWorldstateSize(&to_ws);
  
  for (action_vector::const_iterator it = actions_.begin(); it != actions_.end(); ++it) {
    const Worldstate* pre = &(it->pre);
    
    if ((pre->values & pre->care) == (fr->values & pre->care)) {
      actionnames->push_back(&(it->name));
      actioncosts->push_back(it->get_cost(this, fr));
      
      doActionInternal(&(*it), fr, &to_ws);
      to->push_back(to_ws);
    }
  }
  
  return actionnames->size();
}



bool ActionPlanner::doActionInternal(const Action* action, const Worldstate* fr, Worldstate* to) const
{
  const Worldstate* pst = &action->pst;
  boost::dynamic_bitset<> notcare = pst->care;
  notcare.flip();
  
  to->values = (fr->values & notcare) | (pst->values & pst->care);
  to->care = fr->care | pst->care;
  return true;
}



bool ActionPlanner::findPropositionIndex(std::string name, int* index) const
{  
  try {
    *index = proposition_index_.right.at(name);
    return true;
  } catch (std::out_of_range) { // Does not exist
    return false;
  }
}



ActionPlanner::Action* ActionPlanner::findAction(std::string actionname)
{
  Action* action = NULL;
  action_vector::iterator it = std::find(actions_.begin(), actions_.end(), actionname);
  
  if (it != actions_.end()) {
    action = &(*it);
  }
  
  return action;
}



const ActionPlanner::Action* ActionPlanner::findAction(std::string actionname) const
{
  const Action* action = NULL;
  action_vector::const_iterator it = std::find(actions_.begin(), actions_.end(), actionname);
  
  if (it != actions_.end()) {
    action = &(*it);
  }
  
  return action;
}


ActionPlanner::Action* ActionPlanner::findUnbuiltAction(std::string actionname)
{
  Action* action = NULL;

  action_vector::iterator it = std::find(unbuilt_actions_.begin(), unbuilt_actions_.end(), actionname);
  
  if (it != unbuilt_actions_.end()) {
    action = &(*it);
  }
  
  return action;
}



const ActionPlanner::Action* ActionPlanner::findUnbuiltAction(std::string actionname) const
{
  const Action* action = NULL;

  action_vector::const_iterator it = std::find(unbuilt_actions_.begin(), unbuilt_actions_.end(), actionname);
  
  if (it != unbuilt_actions_.end()) {
    action = &(*it);
  }
  
  return action;
}



ActionPlanner::EntitySet* ActionPlanner::findEntitySet(std::string name)
{
  EntitySet* entity_set = NULL;
  entity_list::iterator it = std::find(entities_.begin(), entities_.end(), name);
  
  if (it != entities_.end()) {
    entity_set = &(*it);
  }
  
  return entity_set;
}



const ActionPlanner::EntitySet* ActionPlanner::findEntitySet(std::string name) const
{
  const EntitySet* entity_set = NULL;
  entity_list::const_iterator it = std::find(entities_.begin(), entities_.end(), name);
  
  if (it != entities_.end()) {
    entity_set = &(*it);
  }
  
  return entity_set;
}



int ActionPlanner::calcH(const Worldstate* fr, const Worldstate* to ) const
{
  const boost::dynamic_bitset<> diff = ( (fr->values ^ to->values) & to->care );
  return (int)diff.count();
}



bool ActionPlanner::permuteAction(const Action &action, action_vector* result) const
{
  action_vector permutations;
  
  // Collect this action's entities
  std::vector<param_label> original_labels;
  std::vector<const EntitySet::member_list*> sets;
  sets.reserve(action.params.size());
  BOOST_FOREACH (Action::parameter param, action.params) {
    sets.push_back(param.set_members);
    original_labels.push_back(param.label);
  }
  
  Action new_action = action;
  std::vector<std::size_t> counters(sets.size());
  std::vector<param_label> permutation(sets.size());

  // Produce the action permutations
  do {
    // Build this permutation
    for (std::size_t i = 0, size = counters.size(); i != size; ++i) {
      permutation[i] = sets[i]->at(counters[i]);
    }
    
    // Verify that we obey the mutual excusion rule for the param labels
    if (elementUniqueness(original_labels, permutation)) {

      new_action.params = action.params;
      new_action.preconditions = action.preconditions;
      new_action.effects = action.effects;
      
      // Insert permuted params into the action
      int i = 0;
      BOOST_FOREACH (param_label original, original_labels) {
	if (!substituteParamLabel(new_action, original, permutation[i])) {
	  printf("ActionPlanner::permuteAction - Param substitution failed for action %s.", action.name.c_str());
	  return false;
	}
	++i;
      }
      
      new_action.buildName();
      
      // Add to output
      permutations.push_back(new_action);
    }
  } while (next_permutation(sets, counters)); // Increments the permutation counter
  
  *result = permutations;
  return true;
}



bool ActionPlanner::substituteParamLabel(Action &action, param_label original, param_label replacement) const
{
  // Verify that the action contains the original parameter
  Action::parameter* action_param = action.getParameter(original);
  if (action_param == NULL) {
    printf("ActionPlanner::substituteParamLabel() - Action %s does not contain param %s.\n", action.name.c_str(), original.c_str());
    return false;
  }
  // Insert replacement label
  action_param->label = replacement;
  
  // Iterate over action preconditions
  BOOST_FOREACH(Action::condition_t &condition, action.preconditions) {

    // Iterate over the parameters of this condition
    BOOST_FOREACH(Proposition::parameter &param, condition.first.params) {
	
      // if this parameter references an action parameter (is neither a literal nor all)
      if (!param.literal && !param.all) {
	  
	// check for match
	if (param.label == original) {
	  // Insert replacement label into parameter
	  param.label = replacement;
	}
      }
    }
    
    // Update name of this Proposition to use substituted params
    condition.first.buildName();
  }
  
  // Do the same thing for the effects
  BOOST_FOREACH(Action::condition_t &condition, action.effects) {

    // Iterate over the parameters of this condition
    BOOST_FOREACH(Proposition::parameter &param, condition.first.params) {
	
      // if this parameter references an action parameter (is neither a literal nor all)
      if (!param.literal && !param.all) {
	  
	// check for match
	if (param.label == original) {
	  // Insert replacement label into parameter
	  param.label = replacement;
	}
      }
    }
    
    // Update name of this Proposition to use substituted params
    condition.first.buildName();
  }
  
  // Update the action name to use substituted params
  action.buildName();
  
  return true;
}



template <typename T>
bool ActionPlanner::elementUniqueness(const string_vec& labels, const std::vector<T>& v) const
{
  for (size_t i = 0, size = labels.size(); i < size; ++i) {
    for (size_t j = i+1, size = labels.size(); j < size; ++j) {
      if (labels[i] != labels[j]) { // if the labels are different...
	if (v[i] == v[j]) {         // then the values must be different
	  return false;
	}
      }
    }
  }
  
  return true;
}



bool ActionPlanner::linkEntities(Action& action) const
{  
  // For each action parameter
  BOOST_FOREACH(Action::parameter& param, action.params) {
    
    // Confirm that the needed EntitySet exists
    const EntitySet* set = findEntitySet(param.type);
    if (set == NULL) {
      printf("ActionPlanner::associateEntities() - EntitySet %s not found for action %s.\n", param.type.c_str(), action.name.c_str());
      return false;
    }
    
    // Associate the set with the parameter
      param.set_members = &(set->members);
  }
  
  // For each precondition
  BOOST_FOREACH(Action::condition_t& precondition, action.preconditions) {
    
    // For each parameter of the proposition
    BOOST_FOREACH (Proposition::parameter& param, precondition.first.params) {
      
      // If this parameter references an entity set
      if (!param.entity_set_name.empty()) {
	
	// Confirm that the needed EntitySet exists
	const EntitySet* set = findEntitySet(param.entity_set_name);
	if (set == NULL) {
	  printf("ActionPlanner::associateEntities() - Failed to link EntitySet %s in action %s.\n", param.entity_set_name.c_str(), action.name.c_str());
	  return false;
	}
	
	// Associate the set with parameter
	param.set_members = &(set->members);
	
	// For a literal parameter, verify that the set contains the literal
	if (param.literal) {
	  // search the set members
	  EntitySet::member_list::const_iterator it = std::find(param.set_members->begin(), param.set_members->end(), param.label);
	  
	  // not found
	  if (it == param.set_members->end()) {
	    printf("ActionPlanner::associateEntities() - Failed to link literal parameter %s in action %s to EntitySet %s.\n", param.label.c_str(), action.name.c_str(), param.entity_set_name.c_str());
	    return false;
	  }
	  
	  precondition.first.buildName();
	}
      }
    }
  }
    
  // For each effect
  BOOST_FOREACH(Action::condition_t& effect, action.effects) {
    
    // For each parameter of the proposition
    BOOST_FOREACH (Proposition::parameter& param, effect.first.params) {
      
      // If this parameter references an entity set
      if (!param.entity_set_name.empty()) {
	
	// Confirm that the needed EntitySet exists
	const EntitySet* set = findEntitySet(param.entity_set_name);
	if (set == NULL) {
	  printf("ActionPlanner::associateEntities() - Failed to link EntitySet %s in action %s.\n", param.entity_set_name.c_str(), action.name.c_str());
	  return false;
	}
	
	// Associate the set with parameter
	param.set_members = &(set->members);
	
	// For a literal parameter, verify that the set contains the literal
	if (param.literal) {
	  // search the set members
	  EntitySet::member_list::const_iterator it = std::find(param.set_members->begin(), param.set_members->end(), param.label);
	  
	  // not found
	  if (it == param.set_members->end()) {
	    printf("ActionPlanner::associateEntities() - Failed to link literal parameter %s in action %s to EntitySet %s.\n", param.label.c_str(), action.name.c_str(), param.entity_set_name.c_str());
	    return false;
	  }
	  
	  effect.first.buildName();
	}
      }
    }
  }
  
  return true;
}



bool ActionPlanner::expandAllParams(Action &action) const
{
  std::vector<ActionPlanner::Action::condition_t> expanded_conditions;
  
  // Iterate over action preconditions
  for(std::vector<Action::condition_t>::iterator condition = action.preconditions.begin(); condition != action.preconditions.end(); ) {

    // Build expanded conditions
    expanded_conditions = expandAllCondition(*condition);
    
    // We found a condition to expand
    if (!expanded_conditions.empty()) {
      // Delete the condition that we expanded
      action.preconditions.erase(condition);
      
      // Append expanded conditions to action
      action.preconditions.insert(action.preconditions.end(), expanded_conditions.begin(), expanded_conditions.end());
      
      // Reset the iteration to the beginning
      condition = action.preconditions.begin();
      
      expanded_conditions.clear();
    }
    else {
      ++condition;
    }
  }
  
  // Iterate over action preconditions
  for(std::vector<Action::condition_t>::iterator condition = action.effects.begin(); condition != action.effects.end(); ) {

    // Build expanded conditions
    expanded_conditions = expandAllCondition(*condition);
    
    // We found a condition to expand
    if (!expanded_conditions.empty()) {
      // Delete the condition that we expanded
      action.effects.erase(condition);
      
      // Append expanded conditions to action
      action.effects.insert(action.effects.end(), expanded_conditions.begin(), expanded_conditions.end());
      
      // Reset the iteration to the beginning
      condition = action.effects.begin();
      
      expanded_conditions.clear();
    }
    else {
      ++condition;
    }
  }
  
  return true;
}



std::vector<ActionPlanner::Action::condition_t> ActionPlanner::expandAllCondition(Action::condition_t condition) const
{
  std::vector<Action::condition_t> output;

  // Search for an ALL parameter
  BOOST_FOREACH(Proposition::parameter& param, condition.first.params) {
    if (param.all) {
      param.all = false;
      
      // Expand this parameter
      BOOST_FOREACH (param_label member, *param.set_members) {
	param.label = member;
	condition.first.buildName();
	output.push_back(condition);
      }
      break;
    }
  }
  
  return output;
}




bool ActionPlanner::expandNegatedParams(Action &action) const
{
  std::vector<ActionPlanner::Action::condition_t> expanded_conditions;
  
  // Iterate over action preconditions
  for(std::vector<Action::condition_t>::iterator condition = action.preconditions.begin(); condition != action.preconditions.end(); ) {

    // Build expanded conditions
    expanded_conditions = expandNegatedCondition(*condition);
    
    // We found a condition to expand
    if (!expanded_conditions.empty()) {
      // Delete the condition that we expanded
      action.preconditions.erase(condition);
      
      // Append expanded conditions to action
      action.preconditions.insert(action.preconditions.end(), expanded_conditions.begin(), expanded_conditions.end());
      
      // Reset the iteration to the beginning
      condition = action.preconditions.begin();
      
      expanded_conditions.clear();
    }
    else {
      ++condition;
    }
  }
  
  // Iterate over action preconditions
  for(std::vector<Action::condition_t>::iterator condition = action.effects.begin(); condition != action.effects.end(); ) {

    // Build expanded conditions
    expanded_conditions = expandNegatedCondition(*condition);

    // We found a condition to expand
    if (!expanded_conditions.empty()) {
      // Delete the condition that we expanded
      action.effects.erase(condition);
      
      // Append expanded conditions to action
      action.effects.insert(action.effects.end(), expanded_conditions.begin(), expanded_conditions.end());
      
      // Reset the iteration to the beginning
      condition = action.effects.begin();
      
      expanded_conditions.clear();
    }
    else {
      ++condition;
    }
  }
  
  return true;
}



std::vector<ActionPlanner::Action::condition_t> ActionPlanner::expandNegatedCondition(Action::condition_t condition) const
{
  std::vector<Action::condition_t> output;

  // Search for an ALL parameter
  BOOST_FOREACH(Proposition::parameter& param, condition.first.params) {
    if (param.negated) {
      std::string original = param.label;
      param.negated = false;
      
      // Expand this parameter
      BOOST_FOREACH (param_label member, *param.set_members) {
	if (member != original) {
	  param.label = member;
	  condition.first.buildName();
	  output.push_back(condition);
	}
      }
      break;
    }
  }
  
  return output;
}


} // namespace goap
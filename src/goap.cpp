/*-------------------------------------------------------------------------------
 goap.cpp
 Version: In Development
 Commit Date: Never

 Authors: Blake Anderson
 The University of Texas at Austin
 Department of Mechanical Engineering
 Nuclear and Applied Robotics Group
 
 Modified from code by Abraham Stolk.

 Description: The GOAP class provides an interface for defining a Goal Oriented Action Planner.

 Command Line Arguments: None
-------------------------------------------------------------------------------*/

/*
Copyright 2012 Abraham T. Stolk

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
*/

#include "task_planning/goap.h"

namespace goap
{

GOAP::GOAP() {}

bool GOAP::load_json(std::string filename, actionplanner_t* ap)
{
  Json::Value root;
  Json::Reader reader;
  std::ifstream json_stream(filename.c_str(), std::ifstream::in);
  bool parse_success = reader.parse(json_stream, root);
  if ( !parse_success )
  {
    std::stringstream error_ss;
    std::cout << "Failed to parse JSON file\n" << reader.getFormattedErrorMessages();
    
    return false;
  }
  
  // Clear old planner data
  goap_actionplanner_clear(ap);
  
  // Iterate over the actions
  for (Json::Value::iterator action = root.begin(); action != root.end(); action++) {
    std::string action_name = action.memberName();
    
    Json::Value pre = (*action)["pre"];
    Json::Value pst = (*action)["pst"];
    
    // Iterate over preconditions
    for (Json::Value::iterator precondition = pre.begin(); precondition != pre.end(); precondition++) {
      
      std::string atom = (*precondition)["atom"].asString();
      bool value = (*precondition)["value"].asBool();
      goap_set_pre(ap, action_name, atom, value);
    }
    
    // Iterate over postconditions
    for (Json::Value::iterator postconditions = pst.begin(); postconditions != pst.end(); postconditions++) {
      
      std::string atom = (*postconditions)["atom"].asString();
      bool value = (*postconditions)["value"].asBool();
      goap_set_pst(ap, action_name, atom, value);
    }
  }
  
  return true;
}

bool GOAP::find_atom_index(actionplanner_t* ap, std::string atomname, int* index)
{
  std::vector<std::string> *vec_ptr = &(ap->atm_names);
  if (vec_ptr->empty())
    return false;

  for (int i = 0; i < vec_ptr->size(); i++) {
    if (vec_ptr->at(i) == atomname) {
      *index = i;
      return true;
    }
  }

  return false;
}

bool GOAP::find_action_index(actionplanner_t* ap, std::string actionname, int* index)
{
  std::vector<action_t> *vec_ptr = &(ap->actions);
  if (vec_ptr->empty())
    return false;
  
  for (int i = 0; i < vec_ptr->size(); i++) {
    if (vec_ptr->at(i).name == actionname) {
      *index = i;
      return true;
    }
  }

  return false;
}

bool GOAP::add_atom_to_planner(actionplanner_t* ap, std::string atomname, int* index)
{
  if (!find_atom_index(ap, atomname, index)) {
    ap->atm_names.push_back(atomname);
    *index = ap->atm_names.size() - 1;
    
    std::size_t num_atms = ap->atm_names.size();

    for (std::vector<action_t>::iterator it = ap->actions.begin(); it != ap->actions.end(); it++) {
      goap_worldstate_clear(&it->pre, num_atms);
      goap_worldstate_clear(&it->pst, num_atms);
    }
    
    return true;
  }

  return false;
}

bool GOAP::add_action_to_planner(actionplanner_t* ap, std::string actionname, int* index)
{
  if (!find_action_index(ap, actionname, index)) {
    action_t new_act;
    new_act.name = actionname;
 
    std::size_t num_atms = ap->atm_names.size();
    goap_worldstate_clear(&new_act.pre, num_atms);
    goap_worldstate_clear(&new_act.pst, num_atms);
    
    ap->actions.push_back(new_act);
    
    int f = ap->actions.size();
    *index = ap->actions.size() - 1;
    return true;
  }
  
  return false;
}

void GOAP::goap_actionplanner_clear(actionplanner_t* ap)
{
  ap->atm_names.clear();
  ap->actions.clear();
}

void GOAP::goap_worldstate_clear(worldstate_t* ws, std::size_t new_size)
{
  ws->values.resize(new_size, false);
  ws->care.resize(new_size, false);
}

bool GOAP::goap_worldstate_set(actionplanner_t* ap, worldstate_t* ws, std::string atomname, bool value)
{
  int index;
  if(!find_atom_index(ap, atomname, &index)) {
    printf("Error in GOAP::goap_worldstate_set: Atom %s not found in action planner.\n", atomname.c_str());
    return false;
  }
  
  std::size_t num_atms = ap->atm_names.size();	
  goap_worldstate_clear(ws, num_atms);
  
  ws->values[index] = value;
  ws->care[index] = true;

  return true;
}

bool GOAP::goap_get_atom_value(actionplanner_t* ap, worldstate_t* ws, std::string atomname, bool* result) {
  int index;
  if(!find_atom_index(ap, atomname, &index)) {
    printf("Error in GOAP::goap_get_atom_value: Atom %s not found in action planner.\n", atomname.c_str());
    return false;
  }
  
  *result = ws->values[index];
  return true;
}

void GOAP::goap_set_pre(actionplanner_t* ap, std::string actionname, std::string atomname, bool value)
{
  int action_index;
  add_action_to_planner(ap, actionname, &action_index);
  
  int atom_index;
  add_atom_to_planner(ap, atomname, &atom_index);

  worldstate_t* preconditions = &ap->actions.at(action_index).pre;
  goap_worldstate_set(ap, preconditions, atomname, value);
}

void GOAP::goap_set_pst(actionplanner_t* ap, std::string actionname, std::string atomname, bool value)
{
  int action_index;
  add_action_to_planner(ap, actionname, &action_index);
  
  int atom_index;
  add_atom_to_planner(ap, atomname, &atom_index);

  worldstate_t* postconditions = &ap->actions.at(action_index).pst;
  goap_worldstate_set(ap, postconditions, atomname, value);
}

bool GOAP::goap_set_cost(actionplanner_t* ap, std::string actionname, float cost)
{
  int index;
  if (!find_action_index(ap, actionname, &index)) {
    printf("Error in GOAP::goap_set_cost: Action %s not found in action planner.\n", actionname.c_str());
    return false;
  }
  
  ap->actions.at(index).set_cost(cost);
  return true;
}

bool GOAP::goap_set_cost(actionplanner_t* ap, std::string actionname, float (*func)(actionplanner_t* ap, worldstate_t* ws))
{
  int index;
  if (!find_action_index(ap, actionname, &index)) {
    printf("Error in GOAP::goap_set_cost: Action %s not found in action planner.\n", actionname.c_str());
    return false;
  }
  
  ap->actions.at(index).set_cost(func);
  return true;
}

void GOAP::goap_worldstate_description(actionplanner_t* ap, worldstate_t* ws, std::string* output)
{
  goap_worldstate_clear(ws, ap->atm_names.size());
  
  std::stringstream sstream;
  for (uint i = 0; i < ap->atm_names.size(); i++) {
    
    if (ws->care[i]) {
      std::string atom_name = ap->atm_names.at(i);
      
      if (ws->values[i]) {
	boost::to_upper(atom_name);
      }
      
      sstream << atom_name << ", ";
    }
  }
  *output = sstream.str();
}

void GOAP::goap_description(actionplanner_t* ap, std::string* output)
{
  std::stringstream sstream;
  for (std::vector<action_t>::iterator it = ap->actions.begin(); it != ap->actions.end(); it++) {
    sstream << *output << it->name << "\n" << "Preconditions\n";
    
    worldstate_t* pre = &(it->pre);
    worldstate_t* pst = &(it->pst);
    
    for (std::size_t i = 0; i < pre->values.size(); i++) {
      if (pre->care[i])
	sstream << *output << ap->atm_names.at(i) << ": " << pre->values[i] << "\n";
    }
    sstream << "Postconditions\n";
    
    for (std::size_t i = 0; i < pst->values.size(); i++) {
      if (pst->care[i])
	sstream << ap->atm_names.at(i) << ": " << pst->values[i] << "\n";
    }
  }
  *output = sstream.str();
}


bool GOAP::do_action(actionplanner_t* ap, std::string actionname, worldstate_t fr, worldstate_t* to)
{
  int index;
  if (find_action_index(ap, actionname, &index)) {
  
    do_action(ap, index, fr, to);
    return true;
  }
  
  return false;
}

int GOAP::calc_h(const worldstate_t* fr, const worldstate_t* to )
{
  const boost::dynamic_bitset<> diff = ( (fr->values ^ to->values) & to->care );
  return (int)diff.count();
}

bool GOAP::idx_in_opened(const worldstate_t* ws, int* index)
{
  for (int i = 0; i < opened.size(); i++) {
    if (opened.at(i).ws.values == ws->values) {
      *index = i;
      return true;
    }
  }
  return false;
}

bool GOAP::idx_in_closed(const worldstate_t* ws, int* index)
{
  for (int i = 0; i < closed.size(); i++) {
    if (closed.at(i).ws.values == ws->values) {
      *index = i;
      return true;
    }
  }
  return false;
}

void GOAP::reconstruct_plan(actionplanner_t* ap, astarnode_t* goalnode, ActionPlan_t* plan, std::vector<worldstate_t>* worldstates)
{
  astarnode_t* curnode = goalnode;
  while (curnode != NULL && curnode->actionname != "start")
  {
    plan->insert(plan->begin(), curnode->actionname);
    worldstates->insert(worldstates->begin(), curnode->ws);
    
    int index;
    if (idx_in_closed(&curnode->parentws, &index))
      curnode = &closed.at(index);
    else
      curnode = NULL;
  }
}

std::vector<astarnode_t>::iterator GOAP::get_lowest_rank()
{
  std::vector<astarnode_t>::iterator lowest_rank = opened.begin();
  for (std::vector<astarnode_t>::iterator it = opened.begin(); it != opened.end(); it++) {
    if (lowest_rank->f > it->f)
      lowest_rank = it;
  }
  return lowest_rank;
}

int GOAP::astar_plan
( 
  actionplanner_t* ap,
  worldstate_t start,
  worldstate_t goal,
  ActionPlan_t* plan,
  std::vector<worldstate_t>* worldstates
)
{
  // put start in opened list
  astarnode_t n0;
  n0.ws = start;
  n0.parentws = start;
  n0.g = 0;
  n0.h = calc_h(&start, &goal);
  n0.f = n0.g + n0.h;
  n0.actionname = "start";
  opened.clear();
  closed.clear();
  opened.push_back(n0);

  do
  {    
    if (opened.empty()) {
      printf("Error in astar::astar_plan: Path not found.\n");
      return -1;
    }
    std::vector<astarnode_t>::iterator lowest_rank = get_lowest_rank();

    // remove the node with the lowest rank
    astarnode_t cur = *lowest_rank;
    opened.erase(lowest_rank);

    // if it matches the goal, we are done!
    if ( (cur.ws.values & goal.care) == (goal.values & goal.care) )
    {
      reconstruct_plan(ap, &cur, plan, worldstates);
      return cur.f;
    }
    // add it to closed
    closed.push_back(cur);

    // iterate over neighbours
    std::vector<std::string> actionnames;
    std::vector<int> actioncosts;
    std::vector<worldstate_t> to;
    const int numtransitions = goap_get_possible_state_transitions(ap, cur.ws, &to, &actionnames, &actioncosts);

    for (int i=0; i<numtransitions; ++i)
    {
      astarnode_t nb;
      const int cost = cur.g + actioncosts[i];
      int idx_o = -1;
      int idx_c = -1;
      idx_in_opened(&to[i], &idx_o);
      idx_in_closed(&to[i], &idx_c);

      // if neighbor in OPEN and cost less than g(neighbor):
      if (idx_o != -1 && cost < opened.at(idx_o).g)
      {
	// remove neighbor from OPEN, because new path is better
	if (!opened.empty())
	  opened.erase(opened.begin() + idx_o);
	idx_o = -1; // BUGFIX: neighbor is no longer in OPEN, signal this so that we can re-add it.
      }
      
      // if neighbor in CLOSED and cost less than g(neighbor):
      if (idx_c != -1 && cost < closed.at(idx_c).g)
      {
	// remove neighbor from CLOSED
	if (!closed.empty())
	  closed.erase(closed.begin() + idx_c);
	idx_c = -1;
      }
      
      // if neighbor not in OPEN and neighbor not in CLOSED:
      if (idx_c == -1 && idx_o == -1)
      {
	nb.ws = to[i];
	nb.g = cost;
	nb.h = calc_h(&nb.ws, &goal);
	nb.f = nb.g + nb.h;
	nb.actionname = actionnames[i];
	nb.parentws = cur.ws;
	opened.push_back(nb);
      }
    }
  } while(true);

  return -1;
}

int GOAP::goap_get_possible_state_transitions(actionplanner_t* ap, worldstate_t fr, std::vector<worldstate_t>* to, std::vector<std::string>* actionnames, std::vector<int>* actioncosts)
{
  for (std::vector<action_t>::iterator it = ap->actions.begin(); it != ap->actions.end(); it++) {
    const worldstate_t* pre = &(it->pre);
    
    const bool met = (pre->values & pre->care) == (fr.values & pre->care);
    
    if (met) {
      actionnames->push_back(it->name);
      actioncosts->push_back(it->get_cost(ap, &fr));
      worldstate_t to_ws;
      do_action(ap, it-ap->actions.begin(), fr, &to_ws);
      to->push_back(to_ws);
    }
  }
  
  return actionnames->size();
}

bool GOAP::is_action_possible(actionplanner_t* ap, worldstate_t* ws, std::string action_name, bool* result)
{
  int index;
  if (!find_action_index(ap, action_name, &index)) {
    printf("Error in GOAP::is_action_possible: Action %s not found in action planner.\n", action_name.c_str());
    return false;
  }
  
  worldstate_t* pre = &ap->actions.at(index).pre;
  *result = (pre->values & pre->care) == (ws->values & pre->care);
  return true;
}

bool GOAP::do_action(actionplanner_t* ap, int actionnr, worldstate_t fr, worldstate_t* to)
{
  worldstate_t* pst = &ap->actions[actionnr].pst;
  boost::dynamic_bitset<> notcare = pst->care;
  notcare.flip();
  
  to->values = (fr.values & notcare) | (pst->values & pst->care);
  to->care = fr.care | pst->care;
  return true;
}

} // namespace goap
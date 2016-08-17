///////////////////////////////////////////////////////////////////////////////
//      Title     : task_manager.h
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

#ifndef GOAP_TASK_MANAGER_H
#define GOAP_TASK_MANAGER_H

#include "action_planner.h"

#include <functional>
#include <unordered_map>

#include <boost/foreach.hpp>

namespace goap {

/*
 * Class TaskManager - Encapsulate the planning and execution of complex tasks using GOAP.
 * 
 * Actions are associated with callback functions (typedef ActionCallback). The action stack
 * is run in a loop which reacts to the return codes (enum RETURN_TYPE) of the callback functions.
 */
class TaskManager
{
public:
  
  /*
   * Callback functions must return this type.
   * CONTINUE        - Action executed successfully. Modify internal worldstate and continue to next action.
   * CONTINUE_FAILED - Action failed, but continue anyway. Does not modify the internal worldstate. Subsequent actions may not be possible.
   * REPEAT          - Action did not execute successfully. Try again.
   * TERMINATE       - Action did not execute successfully. End the action plan.
   * REPLAN          - Update the plan and continue. Use this if an external change occurs to the worldstate which we must account for.
   * BAD_CALL        - Use to flag errors unrelated to the actual execution of the action. A useful distinction for debugging.
   */
  enum RETURN_TYPE {CONTINUE, CONTINUE_FAILED, REPEAT, TERMINATE, REPLAN, BAD_CALL};
  
  // Action callback function with zero parameters
  typedef std::function<RETURN_TYPE ()> ActionCallback;
  
  // Action callback function with one parameter
  template <typename A>
  using ActionCallback_1 = std::function<RETURN_TYPE (A)>;
  
  // Action callback function with two parameters
  template <typename A, typename B>
  using ActionCallback_2 = std::function<RETURN_TYPE (A, B)>;
  
  // Action callback function with three parameters
  template <typename A, typename B, typename C>
  using ActionCallback_3 = std::function<RETURN_TYPE (A, B, C)>;
  
  // Action callback function with four parameters
  template <typename A, typename B, typename C, typename D>
  using ActionCallback_4 = std::function<RETURN_TYPE (A, B, C, D)>;
  
  // Action callback function with five parameters
  template <typename A, typename B, typename C, typename D, typename E>
  using ActionCallback_5 = std::function<RETURN_TYPE (A, B, C, D, E)>;
  
  /**
   * @brief Constructor
   * @param planner The planner to associate with this TaskManager. Immutable after construction.
   */
  TaskManager(ActionPlanner planner);
  
  /**
   * @brief Calls the createPlan method of planner_
   * @param cost Outputs the cost of the created plan.
   * @param worldstates Outputs the worldstate sequence of the created plan.
   * @return True on success. False if no valid plan was found.
   */
  bool buildPlan(int* cost, std::vector<Worldstate>* worldstates);
  
  /**
   * @brief Execute the action plan.
   * @return True if the end of the plan was reached. False if an error, BAD_CALL or TERMINATE were encountered.
   */
  bool runPlan();
  
  /**
   * @brief Associate a callback function with the specified action
   * @param action_name The name of the action.
   * @param func The callback function to associate with action. See ActionCallback definition.
   * @return True on success. False if the action was not found.
   */
  bool setActionCallback(std::string action_name, ActionCallback func);
  
  template <typename A>
  bool setActionCallback(std::string action_name, ActionCallback_1<A> func, std::vector<A> first);
  
  template <typename A, typename B>
  bool setActionCallback(std::string action_name, ActionCallback_2<A,B> func, std::vector<A> first, std::vector<B> second);
  
  template <typename A, typename B, typename C>
  bool setActionCallback(std::string action_name, ActionCallback_3<A,B,C> func, std::vector<A> first, std::vector<B> second, std::vector<C> third);
  
  template <typename A, typename B, typename C, typename D>
  bool setActionCallback(std::string action_name, ActionCallback_4 <A,B,C,D> func, std::vector<A> first, std::vector<B> second, std::vector<C> third, std::vector<D> fourth);
  
  template <typename A, typename B, typename C, typename D, typename E>
  bool setActionCallback(std::string action_name, ActionCallback_5<A,B,C,D,E> func, std::vector<A> first, std::vector<B> second, std::vector<C> third, std::vector<D> fourth, std::vector<E> fifth);
  
  /**
   * @brief Set an atom of the start state. The atom must exist in the planner.
   * @param atom Name of the atom to be modified
   * @param value The value to be set
   * @return True on success.
   */
  bool setStartStateProposition(std::string atom, bool value);
  
  /**
   * @brief Set an atom of the current state. The atom must exist in the planner.
   * @param atom Name of the atom to be modified
   * @param value The value to be set
   * @return True on success.
   */
  bool setCurrentStateProposition(std::string atom, bool value);
  
  /**
   * @brief Set an atom of the goal state. The atom must exist in the planner.
   * @param atom Name of the atom to be modified
   * @param value The value to be set
   * @return True on success.
   */
  bool setGoalStateProposition(std::string atom, bool value);
  
  /**
   * @brief Clear the start state.
   */
  void clearStartState();
  
  /**
   * @brief Clear the current state.
   */
  void clearCurrentState();
  
  /**
   * @brief Clear the goal state.
   */
  void clearGoalState();
  
  /**
   * @brief Return the plan held by the manager.
   * @return The action plan.
   */
  action_plan_t getPlan() const;
  
  /**
   * @brief Return const reference to the action planner.
   * @return The action planner
   */
  const goap::ActionPlanner& getPlanner() const;
  
  /**
   * @brief Get the current value of an atom.
   * @param atom Name of the atom to be retrieved
   * @param result Outputs the value of the atom
   * @return True on success.
   */
  bool getCurrentStateProposition(std::string atom, bool* result) const;
  
  /**
   * @breif Sets the start state equal to the current state
   */
  void setStartStatetoCurrent();
  
  /**
   * @brief Get the name of the current action while the plan is running.
   * @return The action name. Returns empty string if the plan is not running.
   */
  std::string getCurrentAction() const;
  
private:
  
  /**
   * @brief Calls the callback function of the given action.
   * @param action The action to execute
   * @return Echos the return type of the callback function. Returns BAD_CALL if the action has no associated callback.
   */
  RETURN_TYPE executeAction(std::string action) const;
  
  /**
   * @brief This function is called prior to each action when running the action stack.
   */
  virtual void preActionFunction() const;
    
  bool actionPermuteHelper(std::string action_name, ActionPlanner::action_vector* permuted_actions);
  
  // GOAP planner used by this manager. It is ummutable after the TaskManager object is constructed.
  const ActionPlanner planner_;
  
  // Stores generated action plans.
  action_plan_t plan_;
  
  // Stores the current location in the action stack.
  std::vector<std::string>::iterator action_counter_;
  
  // Track the necessary world states
  Worldstate start_state_, current_state_, goal_state_;
  
  // Stores the callback functions associated with the actions
  std::unordered_map<std::string, ActionCallback> callback_funcs_;
};

/** Template definitions **/
#include "task_manager.tpp"
  
} // namespace goap

#endif
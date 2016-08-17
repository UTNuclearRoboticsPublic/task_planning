///////////////////////////////////////////////////////////////////////////////
//      Title     : action_planner.h
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

/*
 * Provides goal oriented action planning interface.
 */

#ifndef GOAP_ACTIONPLANNER_H
#define GOAP_ACTIONPLANNER_H

#include "worldstate.h"

#include <vector>
#include <list>
#include <forward_list>
#include <string>
#include <iostream>
#include <unordered_map>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/tag.hpp>

#include <boost/bimap.hpp>

namespace goap {
  
  class ActionPlanner;
  
  typedef std::vector<std::string> action_plan_t;
  typedef int (*cost_function)(const ActionPlanner* ap, const Worldstate* ws);

// Action planner that keeps track of world state propositions and its action repertoire.
class ActionPlanner
{
  friend class TaskManager;
  
public:
  
  /**
   * @brief Produce an action plan
   * @param start       - The starting worldstate
   * @param goal        - The goal worldstate
   * @param plan        - Outputs the created action plan
   * @param worldstates - Outputs the worldstates that the plan passes through
   * @return The plan cost, or -1 if no plan was found.

      from: http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html
      OPEN = priority queue containing START
      CLOSED = empty set
      while lowest rank in OPEN is not the GOAL:
	current = remove lowest rank item from OPEN
	add current to CLOSED
	for neighbors of current:
	  cost = g(current) + movementcost(current, neighbor)
	  if neighbor in OPEN and cost less than g(neighbor):
	    remove neighbor from OPEN, because new path is better
	  if neighbor in CLOSED and cost less than g(neighbor): **
	    remove neighbor from CLOSED
	  if neighbor not in OPEN and neighbor not in CLOSED:
	    set g(neighbor) to cost
	    add neighbor to OPEN
	    set priority queue rank to g(neighbor) + h(neighbor)
	    set neighbor's parent to current
   */
  int createPlan
  (
    const Worldstate& start,
    const Worldstate& goal,
    action_plan_t* plan,
    std::vector<Worldstate>* worldstates
  ) const;
  
  /**
   * @brief Clear contents of this planner.
   */
  void clear();
  
  /**
   * @brief Add an action to be planner using ADL syntax.
   * @param definition    - The base action definition
   * @param preconditions - Vector of preconditions
   * @param effects       - Vector of effects
   * @return              - True on success.
   */
  bool addAction(std::string definition, std::vector<std::string> preconditions, std::vector<std::string> effects);
  
  /**
   * @brief Add a set of entities to be planner using ADL syntax. This version takes a vector of names to assign to the members. Member names must be unique.
   * @param name    - The name of the set
   * @param members - Vector of member names
   * @return        - True on success.
   */
  bool addEntitySet(std::string name, std::vector<std::string> members);
  
  /**
   * @brief Add a set of entities to be planner using ADL syntax. This version takes the number of members to create and assigns [set name]_i to the members.
   * @param name - The name of the set
   * @param num  - The number of members to create
   * @return     - True on success.
   */
  bool addEntitySet(std::string name, int num);
  
  /**
   * @brief Build action space from provided ADL actions and entities.
   * @return - True on success.
   */
  bool buildActionSpace();
  
  /**
   * @brief Set a constant cost for named action.
   * @param actionname - The name of the action to be modified
   * @param cost       - The new constant cost
   * @return True on success, false if the action is not in the planner.
   */
  bool setActionCost(std::string actionname, int cost);
  
  /**
   * @brief Set a cost function for named action.
   * @param actionname - The name of the action to be modified
   * @param func       - Pointer to cost function.
   * @return True on success, false if the action is not in the planner.
   */
  bool setActionCost(std::string actionname, cost_function func);
  
  /**
   * @brief Set a proposition of worldstate to specified value.
   * @param ws       - World state to be modified
   * @param name     - The name of the proposition to be modified. This can use literal or generic params.
   * @param value    - The new value of propositionname
   * @return         - True on success, failure if the proposition is not in the planner.
   */
  bool setProposition(Worldstate* ws, std::string name, bool value) const;
  
  /**
   * @brief Get the value of a proposition in a specified worldstate.
   * @param ws       - Worldstate containing the value
   * @param name     - The name of the proposition to be checked
   * @param result   - Outputs the value of the proposition
   * @return         - Teu on success. Failure if the action doesn't exist
   */
  bool getPropositionValue(const Worldstate* ws, std::string name, bool* result) const;
  
  /**
   * @brief Determine if the preconditions of an action are met by a given worldstate
   * @param ws          - The worldstate to be tested
   * @param action_name - The action name
   * @param result      - Outputs whether the action is possible.
   * @return            - True on success. False if the action doesn't exist.
   */
  bool isActionPossible(const Worldstate* ws, std::string action_name, bool* result) const;
  
  /**
   * @brief Performs an action on a worldstate, producing the resultant worldstate
   * @param actionnname - The name of the action
   * @param fr          - The starting worldstate
   * @param to          - Outputs the resultant worldspace
   * return             - True on success, false on failure
   */  
  bool doAction(std::string actionname, const Worldstate* fr, Worldstate* to) const;
  
  /**
   * @brief Get string which describes the action planner by listing all actions with preconditions and effects.
   * @param output - Description string
   */
  std::string description() const;
  
  /**
   * @brief Describe the worldstate by listing propositions that matter, in lowercase for false-valued, and uppercase for true-valued propositions.
   * @param ws - The worldstate to be described
   * @return   - String containing the output
   */
  std::string worldstateDescription(const Worldstate* ws) const;
  
  /**
   * @brief Update the size of a worldstate to match the number of propositions in this planner.
   * @param ws The worldstate to modify.
   */
  void updateWorldstateSize(Worldstate* ws) const;
  
private:

  typedef std::string param_label, param_type;
  typedef std::vector<std::string> string_vec;
  
  // Used internally by the createPlan function. Represents a node in the graph search.
  struct astarnode_t
  {
    Worldstate ws;		// The state of the world at this node.
    int g;			// The cost so far.
    int f;			// g+heuristic.
    std::string actionname;	// How did we get to this node?
    Worldstate* parentws;	// Where did we come from?
  };
  
  class NodeMap
  {
  public:
    
    struct sequenced{};
    struct hash_key {};
    
    typedef boost::multi_index::tag<sequenced> sequenced_tag;
    typedef boost::multi_index::tag<hash_key> hash_tag;
    
    typedef boost::multi_index_container<
      astarnode_t*,
      boost::multi_index::indexed_by<
	boost::multi_index::hashed_unique<
	  hash_tag,
	  boost::multi_index::member<astarnode_t, Worldstate, &astarnode_t::ws>,
	  Worldstate::hash
	>,
	boost::multi_index::sequenced<sequenced_tag>
      >
    > node_map_t;
    
    typedef node_map_t::index<hash_key>::type hash_index;
    typedef node_map_t::index<sequenced>::type sequence_index;
    typedef node_map_t::index_iterator<hash_key>::type hash_iter;
    typedef node_map_t::index_iterator<sequenced>::type sequence_iter;
    
    node_map_t& operator() () { return map_; }
    
    hash_index& getHashed() { return map_.get<hash_key>(); };
    
    sequence_index& getSequence() { return map_.get<sequenced>(); };
    
    ~NodeMap()
    {
      for (sequence_iter it = getSequence().begin(); it != getSequence().end(); ++it)
	delete *it;
    }
    
  private:
    node_map_t map_;
  };
  
  struct EntitySet
  {
    param_type name;
    
    typedef std::vector<param_label> member_list;
    member_list members;
    
    bool operator==(const param_type &rhs) const { return name == rhs; }
  };

  class Proposition
  {
  public:
    /*
     * A "variable" parameter references a parameter in the base action.
     * This causes the proposition to receive the value of the action parameter.
     * 		For example:	RobotAt(a) sets RobotAt true for the value of a.
     * 				~RobotAt(a) sets RobotAt false for the value of a.
     * 
     * A "literal" parameter references a specific entity. This makes the proposition the same for all versions of the action.
     * A litetal parameter has a '$' in front of the parameter label.
     * 		For example:	RobotAt($location_1) sets RobotAt true for location_1.
     * 				RobotAt($location_1) sets RobotAt false for location_1.
     * 
     * A "negated" parameter references all entities in a set other than the specified one. You must also provide the set name.
     * A negated parameter has a '~' in front of the parameter label.
     * 		For example: 	RobotAt(~a:location) sets RobotAt true for all values other than a in set location.
     *			 	~RobotAt(~a:location) sets RobotAt false for all values other than a in set location.
     * 
     * You can declare a parameter that is both negated and literal.
     * 		For example: 	RobotAt(~$location_1:location) sets RobotAt true for all values other than location_1 in set location.
     *			 	~RobotAt(~$location_1:location) sets RobotAt false for all values other than location_1 in set location.
     * 
     * An "ALL" parameter references all entities of a set.
     * 		For example:	RobotAt(ALL:location) sets RobotAt true for all locations.
     * 				~RobotAt(ALL:location) sets RobotAt false for all locations.
     * 
     * You cannot negate an all parameter.
     */
    typedef struct {
      std::string raw_text;		// The raw text provided by client
      param_label label;		// Holds name of parsed parameter label
      param_type entity_set_name;	// Name of set referenced by parameter
      const EntitySet::member_list* set_members; // Points to entities referenced by parameter
      
      bool negated;			// Is this a "not" param?
      bool all;				// Is this an "all" param?
      bool literal;			// Is the text a literal?
      
      bool operator==(const std::string &rhs) const { return label == rhs; }
    } parameter;
    std::vector<parameter> params;
    
    /**
     * @breif Build this proposition by parsing the user text
     * @param definition The raw text from user
     * @return True on successful parse
     */
    bool parse(std::string definition);
    
    /**
     * @breif Build the proposition name using the parameter values
     */
    void buildName();
    
    /**
     * @breif Helper function for parse(). Parses a parameter.
     * @param param The raw text to parse
     * @param parameter Output reference to the created parameter object
     * @return True on successful parse.
     */
    bool parseParam(std::string param, parameter* parameter);
    
    /**
     * @breif Search function for the parameter list
     * @param label The derived_value to search for.
     * @return Pointer to parameter, or NULL if not found.
     */
    parameter* getParameter(std::string value);
    
    std::string name, root;
    
    
    bool operator==(const Proposition &rhs) const { return name == rhs.name; }
    
    bool operator==(const std::string &rhs) const { return name == rhs; }
  };

  class Action
  {
  public:
    Action();
    Action(int);
    
    // action params are modeled like this
    typedef struct {
      param_label label;
      param_type type;
      const std::vector<param_label>* set_members;
      
      bool operator==(const param_label &rhs) const { return label == rhs; }
    } parameter;
    
    /**
     * @breif Build this action by parsing the user text
     * @param definition The raw text from user
     * @return True on successful parse
     */
    bool parse(std::string definition, string_vec preconditions, string_vec effects);
    
    /**
     * @breif Helper for parse. Parses a proposition and adds to precondition list
     * @param definition The raw text from user
     * @return True on successful parse
     */
    bool buildPrecondition(std::string definition);
    
    /**
     * @breif Helper for parse. Parses a proposition and adds to effect list
     * @param definition The raw text from user
     * @return True on successful parse
     */
    bool buildEffect(std::string definition);
    
    /**
    * @brief Set the cost to a constant value.
    * @param val - The new constant value
    */
    void set_cost(int val);
    
    /**
    * @brief Associate a cost function with the action. This allows the cost to depend on the worldstate.
    * @param func - Function pointer referencing the cost function
    */
    void set_cost(cost_function func);
    
    /**
    * @brief Get the cost of the action.
    * @param ap - The action planner. Not needed if we are not using a cost function for thsi action.
    * @param ws - The worldstate. Not needed if we are not using a cost function for thsi action.
    * @return The action cost
    */
    int get_cost(const ActionPlanner* ap = NULL, const Worldstate* ws = NULL) const;
    
    /**
     * @breif Updates the name member using current parameter values
     */
    void buildName();
    
    /**
     * @breif Search function for the parameter list
     * @param label The label to search for.
     * @return Pointer to parameter, or NULL if not found.
     */
    parameter* getParameter(param_label label);
    
    std::string name; // full name
    std::string root; // name without the parameter list
    
    typedef std::vector<parameter> param_vec;
    param_vec params; // holds the params
    
    typedef std::pair<Proposition, bool> condition_t;
    std::vector<condition_t> preconditions, effects;
    
    // Use this criterion for sorting Actions alphabetically in a vector
    bool operator<(const Action &rhs) const { return name < rhs.name; }
    
    bool operator==(const Action &rhs) const { return name == rhs.name; }
    
    bool operator==(const std::string &rhs) const { return name == rhs; }
    
    // These store the compiled preconditions and effects. The planner algorithm uses these.
    Worldstate pre, pst;
    
  private:
    int cost;
    int (*cost_func)(const ActionPlanner*, const Worldstate*);
  };
  
  typedef std::list<Action> action_vector;
  
  /**
   * @brief Add a new action to the action planner.
   * @param actionname - The desired action name
   * @return           - Reference to the action
   */
  Action* addAction(std::string actionname);
    
  /**
   * @brief Add a precondition for an action. If this proposition is not already present in the planner, it will be added.
   * @param actionname - The action to be modified.
   * @param name       - The proposition to be modified.
   * @param value      - The new value of the precondition.
   * @return           - True on success
   */
  bool setPrecondition(std::string actionname, std::string name, bool value);
  
  /**
   * @brief Add an effect for an action. If this proposition is not already present in the planner, it will be added.
   * @param actionname - The action to be modified.
   * @param name       - The proposition to be modified.
   * @param value      - The new value of precondition.
   * @return           - True on success
   */
  bool setEffect(std::string actionname, std::string name, bool value);
  
  /**
   * @brief Add a new proposition to the action planner.
   * @param propositionname - The desired proposition name
   * @param index    - Index of the proposition in proposition_names_
   * @return         - True if the success, false if proposition with that name already existed.
   */
  bool addProposition(std::string propositionname);
  
  /**
   * @brief Attempt to match an ADL proposition to existing atoms.
   * @param propositionname - The desired proposition name
   * @param index    - Index of the proposition in proposition_names_
   * @return         - True if the success, false if proposition with that name already existed.
   */
  bool setPropositionADL(Worldstate* ws, std::string name, bool value) const;
  
  /**
   * @brief Get the lowest rank node in the opened nodes.
   * @param nodes The node map to search
   * @return Iterator to the lowest ranked node
   */
  NodeMap::sequence_iter getLowestRank(NodeMap* nodes) const;
  
  /**
   * @brief Internal function to reconstruct the plan by tracing from last node to initial node.
   * @param goalnode    - The goal
   * @param plan        - Vector of actions to be taken
   * @param worldstates - The worldstates we will pass through
   */
  void reconstructPlan(astarnode_t* goalnode, NodeMap* nodes, action_plan_t* plan, std::vector<Worldstate>* worldstates) const;
  
  /**
  * @brief Given the specified 'from' state, list all possible 'to' states along with the action required, and the action cost. For internal use.
  * @param fr - The 'from' state
  * @param to - the 'to' state
  * @return The number of possible transitions
  */
  int getPossibleStateTransitions(const Worldstate* fr, std::vector<Worldstate>* to, std::vector<const std::string*>* actionnames, std::vector<int>* actioncosts) const;
  
  /**
   * @brief Perform action specified by number on a worldstate
   * @param actionnr - The index of the action to perform
   * @param fr - The worldstate to be acted on
   * @param to - The new worldstate
   * @return   - True on success. False on failure.
   */
  bool doActionInternal(const Action* action, const Worldstate* fr, Worldstate* to) const;
  
  /**
   * @brief Return the index of the indicated proposition.
   * @param name     - The desired proposition name
   * @param index    - Outputs the index of the proposition in proposition_names_
   * @return         - True if the proposition was found, else false
   */
  bool findPropositionIndex(std::string name, int* index = NULL) const;
  
  /**
   * @brief Return a pointer to the indicated action.
   * @param actionname - The desired action name
   * @return           - Reference to the action
   */
   Action* findAction(std::string actionname);
   
   /**
   * @brief Return a pointer to the indicated action. Const-overloaded version.
   * @param actionname - The desired action name
   * @return           - Reference to the action
   */
   const Action* findAction(std::string actionname) const;
   
   /**
   * @brief Return a pointer to the indicated action.
   * @param actionname - The desired action name
   * @return           - Reference to the action
   */
   Action* findUnbuiltAction(std::string actionname);
   
   /**
   * @brief Return a pointer to the indicated action. Const-overloaded version.
   * @param actionname - The desired action name
   * @return           - Reference to the action
   */
   const Action* findUnbuiltAction(std::string actionname) const;
   
   /**
   * @brief Return a pointer to the indicated EntitySet.
   * @param actionname - The desired action name
   * @return           - Reference to the action
   */
   EntitySet* findEntitySet(std::string name);
   
   /**
   * @brief Return a pointer to the indicated EntitySet. Const-overloaded version.
   * @param actionname - The desired action name
   * @return           - Reference to the action
   */
   const EntitySet* findEntitySet(std::string name) const;
  
  /**
   * @brief This is our heuristic: estimate for remaining distance is the number of mismatched propositions that matter.
   * @param fr - The starting worldspace
   * @param to - The end worldspace
   * @return The distance between the two worldspaces
   */
  int calcH(const Worldstate* fr, const Worldstate* to) const;
  
  /**
   * @breif Build permutations of an action.
   * @param action - The action to be permuted
   * @return The permutations
   */
  bool permuteAction(const Action &action, action_vector* result) const;
  
  /**
   * @breif Helper function for permuteAction to insert permuted parameters into an action.
   * @param action      - The action to be modified
   * @param original    - The param name to be replaced
   * @param replacement - The param name to be inserted
   * @return true on success
   */
  bool substituteParamLabel(Action &action, param_label original, param_label replacement) const;
  
  /**
   * @brief Helper function for permuteAction to verify the mutual exclusion rule for action parameters.
   * @param labels - The parameter labels
   * @param v      - The parameter values
   * @return True if the mutual exclusion rule is met
   */
  template <typename T>
  bool elementUniqueness(const string_vec& labels, const std::vector<T>& v) const;
  
  /**
   * @breif Links the action definitions to the entity sets.
   * @return true on success
   */
  bool linkEntities(Action& action) const;
  
  /**
   * @breif Expands parameters set ALL.
   * @return true on success
   */
  bool expandAllParams(Action &action) const;
  
  /**
   * @breif Expands parameters set ALL.
   * @return The Propositions produced by the expansion
   */
  std::vector<ActionPlanner::Action::condition_t> expandAllCondition(Action::condition_t condition) const;
  
  /**
   * @breif Expands negated parameters.
   * @return true on success
   */
  bool expandNegatedParams(Action &action) const;
  
  /**
   * @breif Expands negated parameters.
   * @return The Propositions produced by the expansion
   */
  std::vector<ActionPlanner::Action::condition_t> expandNegatedCondition(Action::condition_t condition) const;
  
  action_vector actions_;
  action_vector unbuilt_actions_;
  
  typedef std::forward_list<EntitySet> entity_list;
  entity_list entities_;
  
  typedef boost::bimaps::bimap<std::size_t, std::string> prop_index_map_t;
  typedef prop_index_map_t::value_type prop_index_t;
  prop_index_map_t proposition_index_;
};

} // namespace goap

#endif
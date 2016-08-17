# task_planning
Goal Oriented Action Planning (GOAP) and task execution.

The Task Planning package was initially written at Los Alamos National Laboratory (LANL). Later revisions were done at The University of Texas in the Nuclear Robotics Group. The software is open source, see License.txt.

TASK_PLANNING ROS PACKAGE GUIDE
----------------------------------------------------------------
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
----------------------------------------------------------------

TABLE OF CONTENTS
  1 Introduction

  2 Overview

  3 Usage

  4 Contact

1 INTRODUCTION

  This package provides a library of classes implementing Goal Oriented Action Planning (GOAP). Although this is organized as a ROS package, the library itself does not utilize any ROS features.

2 OVERVIEW

  The following classes implement the GOAP algorithm:

  Worldstate - Stores a vector of boolean variables (atoms) which represent the state of the world and robot.

  Action - Modifies the world atoms in some way. An action is defined by its preconditions, postconditions, and cost function.

  ActionPlanner - Stores the set of actions and produces action plans using the createPlan() function.

  TaskManager - Top level class providing both task planning and execution. It executes task plans by associating callback functions with each action.

3 USAGE

  See GOAP.pdf for explanation of action planning principles and example code.
  
  rosrun task_planning goap_test - Performs a test of the ActionPlanner class.

4 CONTACT
  
  The lab members responsible for this package are listed below with their contact information.

  Blake Anderson 
  blakeanderson@utexas.edu


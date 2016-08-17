///////////////////////////////////////////////////////////////////////////////
//      Title     : worldstate.h
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
 * Defines a world state using a set of boolean variables.
 */

#ifndef GOAP_WORLDSTATE_H
#define GOAP_WORLDSTATE_H

#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS // needed for hash_value to access m_bits
#include <boost/dynamic_bitset.hpp>
#include <boost/functional/hash/hash.hpp>

namespace goap {

// Describes the world state by listing values (t/f) for all known atoms.
class Worldstate
{
public:
  boost::dynamic_bitset<> values;  // Values for atoms.
  boost::dynamic_bitset<> care;	   // Mask for atoms that do not matter.
  
  /**
   * @breif Resize values and care
   * @param new_size Set values and care to this size.
   */
  void initialize(std::size_t new_size);
  
  /**
   * @breif Clears values and care.
   */
  void clear();
  
  bool operator==(const Worldstate &rhs) const { return values == rhs.values; }
  
  bool operator<(const Worldstate &rhs) const { 
    hash hasher;
    return hasher(*this) < hasher(rhs);
  }
  
  struct hash
  {
    size_t operator()(const Worldstate& ws) const {
      return boost::hash_value(ws.values.m_bits);
    }
  };
};

} // namespace goap

#endif
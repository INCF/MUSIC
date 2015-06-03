/*
 *  This file is distributed together with MUSIC.
 *  Copyright (C) 2008, 2009 Mikael Djurfeldt
 *
 *  This interval tree implementation is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software
 *  Foundation; either version 3 of the License, or (at your option)
 *  any later version.
 *
 *  The interval tree implementation is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MUSIC_INTERVAL_TREE_HH

#include <vector>
#include <limits>
#include <algorithm>

namespace MUSIC {

  template<class PointType, class IntervalType, class DataType>
  class IntervalTree {
    static const int ROOT = 0;

    // class NodeType needs to be public on BG machines
  public:
    class NodeType {
      IntervalType ival_;
      PointType maxEnd_;
      DataType data_;
    public:
      NodeType () { }
      NodeType (const IntervalType& i, const DataType& d)
	: ival_ (i), maxEnd_ (minVal ()), data_ (d) { }
      NodeType (const IntervalType& i, const PointType m, const DataType& d)
	: ival_ (i), maxEnd_ (m), data_ (d) { }
      NodeType& operator= (const NodeType& node)
      {
	ival_ = node.ival_;
	maxEnd_ = node.maxEnd_;
	data_ = node.data_;
	return *this;
      }
      static PointType minVal () {
	return std::numeric_limits<PointType>::min ();
      }
      bool operator< (const NodeType& other) const {
	return begin () < other.begin ();
      }
      IntervalType& interval () { return ival_; }
      DataType& data () { return data_; }
      PointType begin () const { return ival_.begin (); }
      PointType end () const { return ival_.end (); }
      PointType maxEnd () const { return maxEnd_; }
    };

  private:
    std::vector<NodeType> nodes;
    int leftChild (int i) const { return 2 * i + 1; }
    int rightChild (int i) const { return 2 * i + 2; }
    int computeSize () const;
    int maxIndex (int l, int r, int i) const;
    PointType build (std::vector<NodeType>& dest, int l, int r, int i);

  public:
    class Action {
    public:
      virtual void operator() (DataType& data) = 0;
    };

  private:
    void search (unsigned int i, PointType point, Action* a);

  public:
    void add (const IntervalType& i, const DataType& data);
    void build ();
    void search (PointType point, Action* a);
    int size () const { return nodes.size (); }
  };

  template<class PointType, class IntervalType, class DataType>
  void
  IntervalTree<PointType, IntervalType, DataType>::add (const IntervalType& i,
							const DataType& data)
  {
    nodes.push_back (NodeType (i, data));
  }


  template<class PointType, class IntervalType, class DataType>
  int
  IntervalTree<PointType, IntervalType, DataType>::computeSize () const
  {
    return maxIndex (0, nodes.size (), ROOT) + 1;
  }

  
  template<class PointType, class IntervalType, class DataType>
  int
  IntervalTree<PointType, IntervalType, DataType>::maxIndex (int l, int r, int i) const
  {
    if (l >= r)
      return -1;
    int m = (l + r) / 2;
    int max = i;
    max = std::max (max, maxIndex (l, m, leftChild (i)));
    return std::max (max, maxIndex (m + 1, r, rightChild (i)));
  }  

  
  template<class PointType, class IntervalType, class DataType>
  void
  IntervalTree<PointType, IntervalType, DataType>::build ()
  {
    sort (nodes.begin (), nodes.end ());
    std::vector<NodeType> newNodes (computeSize ());
    build (newNodes, 0, nodes.size (), ROOT);
    nodes = newNodes;
  }
  
  
  template<class PointType, class IntervalType, class DataType>
  PointType
  IntervalTree<PointType, IntervalType, DataType>::build (std::vector<NodeType>& dest,
							  int l,
							  int r,
							  int i)
  {
    if (l < r) // sequence not empty
      {
	int m = (l + r) / 2;
	PointType max = nodes[m].end ();
	PointType end = build (dest, l, m, leftChild (i));
	max = end < max ? max : end;
	end = build (dest, m + 1, r, rightChild (i));
	max = end < max ? max : end;
	dest[i] = NodeType (nodes[m].interval (), max, nodes[m].data ());
	return max;
      }
    else
      return NodeType::minVal ();
  }


  template<class PointType, class IntervalType, class DataType>
  void
  IntervalTree<PointType, IntervalType, DataType>::search (PointType p, Action* a)
  {
    search (ROOT, p, a);
  }

  template<class PointType, class IntervalType, class DataType>
  void
  IntervalTree<PointType, IntervalType, DataType>::search (unsigned int i,
							   PointType p,
							   Action* a)
  {
    // Newly introduced extra stop condition enabling less memory usage
    if (i >= nodes.size ())
      return;

    // this condition both checks if the point is to the right of all
    // nodes in this subtree (and previously stopped recursion at
    // noNode values)
    if (p >= nodes[i].maxEnd ())
      return;

    search (leftChild (i), p, a);

    if (p < nodes[i].begin ())
      // point is to the left of this interval and of the right subtree
      return;

    if (p < nodes[i].end ())
      // perform action on this interval
      (*a) (nodes[i].data ());

    search (rightChild (i), p, a);
  }
}

#define MUSIC_INTERVAL_TREE_HH
#endif

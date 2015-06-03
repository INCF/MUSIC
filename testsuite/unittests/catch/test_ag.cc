/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009 INCF
 *
 *  MUSIC is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  MUSIC is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#define CATCH_CONFIG_NO_VARIADIC_MACROS
#include "music/application_graph.hh"
#include <string>

#include "catch.hpp"
/*#define CATCH_CONFIG_RUNNER
 #include <mpi.h>
 int main( int argc, char* argv[] )
 {
 MPI_Init(&argc, &argv);

 int result = Catch::Session().run( argc, argv );

 MPI_Finalize();

 return result;
 }*/
using namespace MUSIC;
typedef AGraph<std::string, char> T_AGraph;
typedef ANode<std::string, char> T_ANode;
typedef AEdge<std::string, char> T_AEdge;

class TC_AGraph : public T_AGraph
{
protected:
  int c_loops_;
public:
  TC_AGraph() :
      T_AGraph(1, 2, 1), c_loops_(0)
  {
  }
  ;
protected:
  void
  handleLoop(T_ANode &x, std::vector<T_AEdge> & path)
  {
    c_loops_++;
    T_AGraph::handleLoop(x, path);
  }
  void
  reset()
  {
    c_loops_ = 0;
    T_AGraph::reset();

  }
};

TEST_CASE_METHOD(TC_AGraph, "Create AGraph with nAppls = 2; nEdges = 1, color = 1", "[agraph]" )
{

  REQUIRE( n_apps_ == 2);
  REQUIRE( c_apps_ == 0);
  REQUIRE( n_edges_ == 1);
  REQUIRE( c_edges_ == 0);
  REQUIRE( color_ == 1);
  REQUIRE( nNodes() == c_apps_);
  REQUIRE(nEdges() == c_edges_);
  REQUIRE(local_node_color() == color_);

  bool edge_ = 1;
  std::string data1("node1");
  std::string data2("node2");
  addNode(T_ANode(edge_, !edge_, data1), color_);
  addNode(T_ANode(!edge_, edge_, data2), color_ - 1);
  SECTION( "TEST TemporalNegotiatorGraph::addNode()", "[agraph]" )
    {
      REQUIRE( c_apps_ == 2);
    }

  T_ANode &node1 = at(color_);
  T_ANode &node2 = at(color_ - 1);
  SECTION( "TEST TemporalNegotiatorGraph::at()", "[agraph]" )
    {
      REQUIRE( &(node1.data()) == &data1);
      REQUIRE(&(node2.data()) == &data2);
    }
  SECTION( "TEST TemporalNegotiatorGraph::local_node()", "[agraph]" )
    {
      T_ANode &lnode = local_node();
      REQUIRE( &lnode == &node1);
    }

  SECTION( "TEST TemporalNegotiatorGraph::iterator", "[agraph]" )
    {
      iterator it;
      int k = 1; //nodes are stored in the order according to the color
      for (it = begin(); it < end(); ++it, ++k)
        REQUIRE((it->data()[4] - 48) == (k % 2 + 1));
    }

  T_AEdge &cur_edge = addEdge(T_AEdge(node1, node2, data1[4]));
  SECTION( "TEST TemporalNegotiatorGraph::addEdge()", "[agraph]" )
    {
      REQUIRE( c_edges_ == 1);
      REQUIRE(&(cur_edge.data()) == &data1[4]);
      T_ANode::edge_iterator it;
      int c_edges = 0;
      for (it = node1.begin_o(); it < node1.end_o(); ++it, ++c_edges)
        {
          REQUIRE( &((*it)->pre()) == &node1);
          REQUIRE( &((*it)->post()) == &node2);
        }
      REQUIRE(c_edges == 1);
      for (it = node2.begin_o(); it < node2.end_o(); ++it)
        {
          REQUIRE(false);
        }
      for (it = node1.begin_i(); it < node1.end_i(); ++it)
        {
          REQUIRE(false);
        }
      c_edges = 0;
      for (it = node2.begin_i(); it < node2.end_i(); ++it, ++c_edges)
        {
          REQUIRE( &((*it)->pre()) == &node1);
          REQUIRE( &((*it)->post()) == &node2);
        }
      REQUIRE(c_edges == 1);
    }
  SECTION( "TEST TemporalNegotiatorGraph::edge()", "[agraph]" )
    {
      T_AEdge tmp_edge = edge(node1, 0);
      REQUIRE( &tmp_edge.data() == &data1[4]);
    }
  std::vector<T_AEdge> path;
  depthFirst(node1, path);
  //very poor check
  SECTION( "TEST TemporalNegotiatorGraph::depthFirst()", "[agraph]" )
    {
      REQUIRE(c_loops_ == 0);
      iterator it;
      for (it = begin(); it < end(); ++it)
        REQUIRE(it->visited);

    }

  reset();
  SECTION( "TEST TemporalNegotiatorGraph::reset()", "[agraph]" )
    {
      iterator it;
      for (it = begin(); it < end(); ++it)
        REQUIRE_FALSE(it->visited);

      for (it = begin(); it < end(); ++it)
        REQUIRE_FALSE(it->in_path);
    }

}

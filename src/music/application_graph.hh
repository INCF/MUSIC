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

#ifndef MUSIC_APPLICATION_GRAPH_HH
#define MUSIC_APPLICATION_GRAPH_HH
#include <assert.h>
#include <vector>
#include <cstddef>
namespace MUSIC
{
  template<typename NodeData, typename EdgeData>
    class ANode;

  template<typename NodeData, typename EdgeData>
    class AEdge
    {
      ANode<NodeData, EdgeData> *pre_;
      ANode<NodeData, EdgeData> *post_;
      EdgeData *data_;
    public:
      AEdge () :
          pre_ (0), post_ (0), data_ (0)
      {
      }

      AEdge (ANode<NodeData, EdgeData> &pre, ANode<NodeData, EdgeData> &post,
          EdgeData &data) :
          pre_ (&pre), post_ (&post), data_ (&data)
      {
      }

      virtual
      ~AEdge ()
      {
      }

      ANode<NodeData, EdgeData> &
      post ()
      {
        return *post_;
      }

      ANode<NodeData, EdgeData> &
      pre ()
      {
        return *pre_;
      }

      EdgeData &
      data ()
      {
        return *data_;
      }
    };

  template<typename NodeData, typename EdgeData>
    class ANode
    {
      typedef AEdge<NodeData, EdgeData>* AEdgePtr;
      AEdgePtr *out_edges_;
      AEdgePtr *in_edges_;

      int n_out_edges_;
      int n_in_edges_;

      int c_out_;
      int c_in_;

      NodeData *data_;

    public:
      bool in_path;
      bool visited;

      ANode () :
          data_ (0)
      {
        init (0, 0);
      }

      ANode (int nOut, int nIn, NodeData &data) :
          data_ (&data)
      {
        init (nOut, nIn);
      }

      ANode (const ANode<NodeData, EdgeData>& other)
      {
        init (other.n_out_edges_, other.n_in_edges_);

        data_ = other.data_;
        in_path = other.in_path;
        visited = other.visited;
        c_out_ = other.c_out_;
        c_in_ = other.c_in_;

        for (int i = 0; i < c_out_; ++i)
          out_edges_[i] = other.out_edges_[i];
        for (int i = 0; i < c_in_; ++i)
          in_edges_[i] = other.in_edges_[i];
      }

      virtual
      ~ANode ()
      {
        delete[] out_edges_;
        delete[] in_edges_;
      }

      ANode<NodeData, EdgeData>&
      operator= (const ANode<NodeData, EdgeData>& that)
      {
        if (this != &that)
          {
            delete[] out_edges_;
            delete[] in_edges_;
            init (that.n_out_edges_, that.n_in_edges_);

            data_ = that.data_;
            in_path = that.in_path;
            visited = that.visited;
            c_out_ = that.c_out_;
            c_in_ = that.c_in_;

            for (int i = 0; i < c_out_; ++i)
              out_edges_[i] = that.out_edges_[i];
            for (int i = 0; i < c_in_; ++i)
              in_edges_[i] = that.in_edges_[i];
          }
        return *this;
      }

      void
      addEdge (AEdge<NodeData, EdgeData> &edge, bool input = false)
      {
        if (input)
          {
            assert (c_in_ < n_in_edges_);
            in_edges_[c_in_++] = &edge;
          }
        else
          {
            assert (c_out_ < n_out_edges_);
            out_edges_[c_out_++] = &edge;
          }
      }

      void
      reset ()
      {
        in_path = false;
        visited = false;
      }

      int
      nOutEdges ()
      {
        return n_out_edges_;
      }

      int
      nInEdges ()
      {
        return n_in_edges_;
      }

      AEdge<NodeData, EdgeData> &
      outEdge (int c)
      {
        assert (c < c_out_);
        return *out_edges_[c];
      }

      typedef AEdgePtr* edge_iterator;

      edge_iterator
      begin_o ()
      {
        return &out_edges_[0];
      }

      edge_iterator
      end_o ()
      {
        return &out_edges_[c_out_];
      }

      edge_iterator
      begin_i ()
      {
        return &in_edges_[0];
      }

      edge_iterator
      end_i ()
      {
        return &in_edges_[c_in_];
      }

      NodeData &
      data ()
      {
        assert (data_ != 0);
        return *data_;
      }

    private:
      void
      init (int nOut, int nIn)
      {
        n_out_edges_ = nOut;
        n_in_edges_ = nIn;
        out_edges_ = new AEdgePtr[nOut + 1];
        in_edges_ = new AEdgePtr[nIn + 1];
        c_in_ = 0;
        c_out_ = 0;
        reset ();
      }
    };

  template<typename NodeData, typename EdgeData>
    class AGraph
    {
    protected:
      ANode<NodeData, EdgeData> *nodes_;
      int c_apps_;

      AEdge<NodeData, EdgeData> *edges_;
      int c_edges_;

      int color_;

      int n_apps_;
      int n_edges_;

    public:
      AGraph (int color, int nApps, int nEdges) :
          c_apps_ (0), c_edges_ (0), color_ (color), n_apps_ (nApps), n_edges_ (
              nEdges)
      {
        nodes_ = new ANode<NodeData, EdgeData> [nApps + 1];
        edges_ = new AEdge<NodeData, EdgeData> [nEdges];
      }

      //AGraph can be a simple array of nodes
      AGraph (int color, int nApps) :
          c_apps_ (0), c_edges_ (0), color_ (color), n_apps_ (nApps), n_edges_ (
              0)
      {
        nodes_ = new ANode<NodeData, EdgeData> [nApps + 1];
        edges_ = NULL;
      }

      virtual
      ~AGraph ()
      {
        if (edges_ != NULL)
          delete[] edges_;
        delete[] nodes_;

      }

      int
      nNodes ()
      {
        return c_apps_;
      }

      int
      nEdges ()
      {
        return c_edges_;
      }

      ANode<NodeData, EdgeData> &
      addNode (ANode<NodeData, EdgeData> node, int color)
      {
        assert (color < n_apps_);
        nodes_[color] = node;
        assert (c_apps_ < n_apps_);
        c_apps_++;
        return nodes_[color];
      }

      AEdge<NodeData, EdgeData> &
      addEdge (AEdge<NodeData, EdgeData> edge)
      {
        assert (c_edges_ < n_edges_);
        edges_[c_edges_] = edge;
        edge.pre ().addEdge (edges_[c_edges_]);

        edge.post ().addEdge (edges_[c_edges_], true);

        c_edges_++;
        return edges_[c_edges_ - 1];
      }

      typedef ANode<NodeData, EdgeData> *iterator;

      iterator
      begin ()
      {
        return &nodes_[0];
      }

      iterator
      end ()
      {
        return &nodes_[c_apps_];
      }

      ANode<NodeData, EdgeData> &
      at (int color)
      {
        assert (color < c_apps_);
        return nodes_[color];
      }

      ANode<NodeData, EdgeData> &
      local_node ()
      {
        assert (color_ < c_apps_);
        return nodes_[color_];
      }

      int
      local_node_color ()
      {
        return color_;
      }

      virtual void
      reset ()
      {
        for (int i = 0; i < c_apps_; ++i)
          nodes_[i].reset ();
      }

      void
      depthFirst (ANode<NodeData, EdgeData> &x,
          std::vector<AEdge<NodeData, EdgeData> > & path)
      {
        if (x.in_path)
          {
            handleLoop (x, path); //x.handle_in_path(path);
            return;
          }

        // Mark as processed (remove from main loop forest)
        x.visited = true;
        x.in_path = true;

        for (int c = 0; c < x.nOutEdges (); ++c)
          {
            path.push_back (edge (x, c));
            depthFirst (edge (x, c).post (), path);
            path.pop_back ();
          }

        x.in_path = false;
      }

    protected:
      virtual AEdge<NodeData, EdgeData>
      edge (ANode<NodeData, EdgeData> &x, int c)
      {
        return x.outEdge (c);
      }

      virtual void
      handleLoop (ANode<NodeData, EdgeData> &x,
          std::vector<AEdge<NodeData, EdgeData> > & path)
      {
      }

    };
}
#endif /* MUSIC_APPLICATION_GRAPH_HH */

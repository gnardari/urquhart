#pragma once

#include <distance.hpp>
#include <polygon.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/graphviz.hpp>

using namespace boost;

using BoostGraph = adjacency_list<vecS, vecS, bidirectionalS, urquhart::Polygon>;
using BoostRGraph = reverse_graph<BoostGraph>;
using BoostVertexT = graph_traits<BoostGraph>::vertex_descriptor;
using BoostEdgeT = graph_traits<BoostGraph>::edge_descriptor;
using BoostEdgeIter = graph_traits<BoostGraph>::out_edge_iterator; 

class BFSVisitor : public boost::default_bfs_visitor {
    public:    
        BFSVisitor(std::vector<BoostVertexT>& _visited) : visited(_visited){}
        void discover_vertex(BoostVertexT s, const BoostGraph &g) { if (s != 0) visited.push_back(s); }
        void discover_vertex(BoostVertexT s, const BoostRGraph &g) { if (s != 0) visited.push_back(s); }
        std::vector<BoostVertexT>& visited;
};

namespace urquhart {

    class Tree {
        public:
            explicit Tree(const std::vector<Polygon>& polygons);
            void view_tree();
            void view_h2_polygons();
            // Merges two polygons of the tree. If i or j have more than 3 vertices, they are
            // disconnected from the main structure and their children are connected to the new polygon.
            // They are not deleted to keep the incremental indices (this could be improved to reduce memory consumption).
            BoostVertexT merge_op(BoostVertexT i, BoostVertexT j, const Polygon& data);
            Polygon get_vertex(const BoostVertexT v);
            // Gets the parent of a polygon
            BoostVertexT get_ancestor(const BoostVertexT v);
            // Gets the children of a vertex. This function reverses the graph and does a search,
            // TODO: this could be simplified by getting the in_edges of v which should be always 1
            std::vector<BoostVertexT> get_children(const BoostVertexT v);
            // Does a BFSearch from the root
            std::vector<BoostVertexT> traverse();
            // Does a BFSearch from v
            std::vector<BoostVertexT> traverse(const BoostVertexT v);
            BoostGraph graph;
            BoostVertexT root;
    };
}
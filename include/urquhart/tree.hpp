#pragma once

#include <utils.hpp>
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
            void view();
            BoostVertexT merge_op(BoostVertexT i, BoostVertexT j, const Polygon& data);
            Polygon get_vertex(const BoostVertexT v);
            BoostVertexT get_ancestor(const BoostVertexT v);
            std::vector<BoostVertexT> get_children(const BoostVertexT v);
            std::vector<BoostVertexT> traverse();
            std::vector<BoostVertexT> traverse(const BoostVertexT v);
            BoostGraph graph;
            BoostVertexT root;
    };
}
#include <tree.hpp>

namespace urquhart {

Tree::Tree(const std::vector<Polygon>& polygons){
    // create the root node
    root = add_vertex(Polygon(), graph);
    // add all elements and point them to the root
    for(auto p : polygons){
        BoostVertexT v = add_vertex(p, graph);
        add_edge(root, v, graph);
    }
}

BoostVertexT Tree::merge_op(BoostVertexT i, BoostVertexT j, const Polygon& data){
    BoostVertexT v = add_vertex(data, graph);
    add_edge(root, v, graph);

    // add v as the new parent of i and j
    // if i or j are already a polygon, we remove it from the tree
    // since we don't use intermediary polygons
    if (graph[i].edges.size() > 3){
        BoostEdgeIter ei, ei_end;
        for (boost::tie(ei, ei_end) = out_edges(i, graph); ei != ei_end; ++ei) {
            BoostVertexT edgeTarget = target(*ei, graph);
            add_edge(v, edgeTarget, graph);
        } 
        remove_vertex(i, graph);
    } else {
        clear_in_edges(i, graph);
        add_edge(v, i, graph);
    }

    if (graph[j].edges.size() > 3){
        BoostEdgeIter ei, ei_end;
        for (boost::tie(ei, ei_end) = out_edges(j, graph); ei != ei_end; ++ei) {
            BoostVertexT edgeTarget = target(*ei, graph);
            add_edge(v, edgeTarget, graph);
        } 
        remove_vertex(j, graph);
    } else {
        clear_in_edges(j, graph);
        add_edge(v, j, graph);
    }

    return v;
}

std::vector<BoostVertexT> Tree::get_ancestor(const BoostVertexT v){
    BoostRGraph reversedGraph(graph);
    std::vector<BoostVertexT> polygons;
    BFSVisitor vis(polygons);
    breadth_first_search(reversedGraph, v, visitor(vis));
    return polygons;
}

std::vector<BoostVertexT> Tree::traverse(){
    std::vector<BoostVertexT> polygons;
    BFSVisitor vis(polygons);
    breadth_first_search(graph, root, visitor(vis));
    return polygons;
}

std::vector<BoostVertexT> Tree::traverse(const BoostVertexT v){
    std::vector<BoostVertexT> polygons;
    BFSVisitor vis(polygons);
    breadth_first_search(graph, v, visitor(vis));
    return polygons;
}

}
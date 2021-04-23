#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <utility> 
#include <algorithm>
#include <string>
#include <cstdlib>
#include <map>
#include <climits>

using namespace std;

template <typename T> // the template allows the weight of an edge to take any numeric data type (denoted by T).
class Graph {
	public:
        
        /* define your data structure to represent a weighted undirected graph */
        map<string, map<string, T>> adj_list;
        vector<string> v_list; // vector cotains only vertices
        
        /* test1 */
		Graph(); // the contructor function.
		~Graph(); // the destructor function.
		size_t num_vertices(); // returns the total number of vertices in the graph.
		size_t num_edges(); // returns the total number of edges in the graph.

        /* test2 */
        void add_vertex(const string&); // adds a vertex to the graph -- every vertex uses a string as its unique identifier.
        bool contains(const string&); // checks if a vertex is in the graph -- returns true if the graph contains the given vertex; otherwise, returns false.
        
        /* test3 */
        vector<string> get_vertices(); // returns a vector of all the vertices in the graph.

        /* test4 */
        void add_edge(const string&, const string&, const T&); // adds a weighted edge to the graph -- the two strings represent the incident vertices; the third parameter represents the edge's weight.
        bool adjacent(const string&, const string&); // check if there is an edge between the two vertices in the graph -- returns true if the edge exists; otherwise, returns false.
		
        /* test5 */
        vector<pair<string,string>> get_edges(); // returns a vector of all the edges in the graph -- each edge is represented by a pair of vertices incident to the edge.
        
        /* test6 */
        vector<string> get_neighbours(const string&); // returns a vector of all the vertices, each of which is directly connected with the given vertex by an edge.
        size_t degree(const string&); // returns the degree of a vertex.

        /* test7 */
		void remove_edge(const string&, const string&); // removes the edge between two vertices, if it exists.
        
        /* test8 */
        void remove_vertex(const string&); // delete the given vertex from the graph -- note that, all incident edges of the vertex should be deleted as well.

        /* test9 */
		vector<string> depth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a depth-first traversal from the given vertex.
		
        /* test10 */
        vector<string> breadth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a breadth-first traversal from the given vertex.
        
        /* test11 */
		bool contain_cycles(); // check if the graph contains any cycle -- returns true if there exists a path from a vertex to itself; otherwise, return false.
        bool contain_cycles_util(const string& u, const string& parent, vector<string>& visited);

        /* test12 */
		Graph<T> minimum_spanning_tree(); // returns a spanning tree of the graph -- the returned tree is preferably a minimum spanning tree.
		
};

/* test1 */

template <typename T>
Graph<T>::Graph() {}

template <typename T>
Graph<T>::~Graph() {}


template <typename T>
size_t Graph<T>::num_vertices() {
    return v_list.size();
}

template <typename T>
size_t Graph<T>::num_edges() {
    size_t no_edges = 0;
    for(auto & v : v_list){
        no_edges += adj_list[v].size();       
    }
    return no_edges/2;
}

/* test2 */

template <typename T>
void Graph<T>::add_vertex(const string& u) {
    if(!contains(u)){
        v_list.push_back(u);
    }
}

template <typename T>
bool Graph<T>::contains(const string& u) { 
    return find(v_list.begin(),  v_list.end(), u) != v_list.end();    
}

/* test3 */

template <typename T>
vector<string> Graph<T>::get_vertices() {
    return v_list;
}

/* test4 */

template <typename T>
void Graph<T>::add_edge(const string& u, const string& v, const T& weight) {
    if(contains(u) && contains(v)){       
       
            adj_list[u].insert(pair<string, T>(v, weight));
            adj_list[v].insert(pair<string, T>(u, weight));
    }
}
template <typename T>
bool Graph<T>::adjacent(const string& u, const string& v) {
    if(contains(u) && contains(v)){
        return adj_list.at(u).find(v) != adj_list.at(u).end();
    }
    return false;
}

/* test5 */

template <typename T>
vector<pair<string,string>> Graph<T>::get_edges() {
    vector<pair<string, string>> result;
    vector<string> vertices;

    for(auto & u : v_list){
        for(auto & v: adj_list[u]){
            if(find(vertices.begin(), vertices.end(), v.first) == vertices.end()){
                pair<string, string> edge(u, v.first);
                result.push_back(edge);
            }
        }
        vertices.push_back(u);
    }
    return result;
}

/* test6 */

template <typename T>
vector<string> Graph<T>::get_neighbours(const string& u) {
    vector<string> result;
    auto var = adj_list.find(u);
    for( auto it = var->second.begin(); it != var->second.end(); ++it){
        result.push_back(it->first);
    }
    return result;
}

template <typename T>
size_t Graph<T>::degree(const string& u) {
    return get_neighbours(u).size();
}

/* test7 */

template <typename T>
void Graph<T>::remove_edge(const string& u, const string& v) {
	if(contains(u) && contains(v)){ //check 2 vertices
        if(adj_list[u].find(v) != adj_list[u].end()){
            adj_list[u].erase(v);
            adj_list[v].erase(u);
        }
    }
}

/* test8 */

template <typename T>
void Graph<T>::remove_vertex(const string& u) {
    for(int i = 0; i<v_list.size(); i++){ // loop through size of vertex list
        if(v_list[i] == u){ // if vertex at position i == u 
            for(auto & v : v_list){ 
                remove_edge(u, v); //remove edges that u is connected to
                remove_edge(v, u);
            }
            v_list.erase(v_list.begin() + i); //remove vertex u from vertex list
            break;
        }
    }
}

/* test9 */

template <typename T>
vector<string> Graph<T>::depth_first_traversal(const string& u) {
    vector<string> result; // this will be output
    stack<string> stack; //this stores elements that will be searched next
    map<string, bool> visited; // this will be use to keep track of which vertices had been visited

    for(auto & v : visited){
        visited[v.first] = false; // set all the value of the key from visited list to false
    }
    stack.push(u); 

     while(!stack.empty()){
         string v = stack.top(); 
         stack.pop(); 
         if(!visited[v]){ 
             visited[v] = true;
             result.push_back(v);
             for(auto & t : get_neighbours(v)){ // for each neighbour of t of vertex v
                 if(!visited[t]){ // if t not visited
                     stack.push(t); // push t to stack
                 }
             }
         }
     }
    return result;
}

/* test10 */

template <typename T>
vector<string> Graph<T>::breadth_first_traversal(const string& u) {
    vector<string> result; // this will be output
    queue<string> queue; //this stores elements that will be searched next
    map<string, bool> visited; // this will be use to keep track of which vertices had been visited

    for(auto & v : visited){
        visited[v.first] = false; // set all the value of the key from visited list to false
    }
    queue.push(u);

    while(!queue.empty()){
        string v = queue.front();
        queue.pop();
        if(!visited[v]){
            visited[v] = true;
            result.push_back(v);
            for(auto & t :get_neighbours(v)){ // for each neighbour t of vertex v
                if(!visited[t]){ // if t not visited
                    queue.push(t); // push t to stack
                }
            }
        }
    }
    return result;
}

/* test11 */

//The purpose of this function is to use visited[] and parent to detect cycle from vertex u
template <typename T>
bool Graph<T>::contain_cycles_util(const string& u, const string& parent, vector<string>& visited){
    visited.push_back(u); //push u to visited list to mark as visited

    for(auto & v : get_neighbours(u)){ // for each neighbour v of vertex u
        if(find(visited.begin(), visited.end(), v) == visited.end()){ // if v is not in visited
            if(contain_cycles_util(v, u, visited)){ // recur for that adjacent (i.e: v)
                return true;
            }
        }
        // If an adjacent vertex is visited and is not parent of current vertex, then there exists a cycle in the graph.
        else if(v != parent){
            return true;
        }
    }
    return false;
}

//return true if graph contains cycle, else return false
template <typename T>
bool Graph<T>::contain_cycles() {    
    vector<string> visited;
    for(auto & v : v_list){ //for each vertex in vertex list
        if(find(visited.begin(), visited.end(), v) == visited.end()){ // if v is not in visited
            if(contain_cycles_util(v, "a", visited)){ //recur for v
                return true;
            }
        }
    }  
    return false;
}

/* test12 */

template <typename T>
Graph<T> Graph<T>::minimum_spanning_tree() {
    Graph<T> mst;
    vector<string> visited;
    visited.push_back(adj_list.begin()->first);
    mst.add_vertex(adj_list.begin()->first);

    while (mst.num_vertices() < num_vertices()) { // while the mst has less vertices than current graph vertices
        int min = INT_MAX, min_index; // set the lowest value to infinite 
        pair<string, string> link;
        for (auto& u : visited) { // for each vertex in visited
            for (auto& v : adj_list[u]) { //for each vertices in adjacency list at 'vertex'
                if (find(visited.begin(), visited.end(), v.first) == visited.end() && v.second < min) { //if visited doesn't have adjacent key && adjacent value < min value
                    min = v.second; // let min value equals to adjacent value
                    link.first = u; // link pair now contains (vertex, adjacent key)
                    link.second = v.first;
                }
            }
        }
        visited.push_back(link.second);
        mst.add_vertex(link.second); 
        mst.add_edge(link.first, link.second, adj_list[link.first][link.second]); // add edges to mst
    }
    return mst;
}

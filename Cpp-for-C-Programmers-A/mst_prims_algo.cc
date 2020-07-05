/**
 * Implementation of Prim's Minimum Spanning Tree Algorithm using adjacency list.
 * 
 * Used the same Graph as used in prev. assignment as mentioned
 * added 1 constructor to read the content from the file.
 * modified the copy constructor of Graph class.
 * 
 * Author: Ravi Ranjan
*/
#include <iostream>
#include <vector>
#include <queue>
#include <fstream>

using namespace std;

const int infinity = INT32_MAX;

// this Node to contain the vertex and its cost (undirected graph)
typedef pair<int, int> Node;

// inline methods for random probability, distance within range and vertexes to chose from.
inline float probability() { return static_cast<float>(rand() % 100) / 100; }
inline int distance(int min, int max) { return rand() % (max + 1 - min) + min; }
inline int random_vertex(int vertices) { return rand() % vertices; }

class Graph
{
public:
    // constructor with initializer syntax.
    Graph(int vertices, float density, int min, int max) : vertices(vertices), density(density), min_range(min), max_range(max)
    {
        generate_random_graph();
    }

    // constructor to initialize using the files
    Graph(string filename)
    {
        // reading data from the file ideally should be presenet in the same folder
        fstream input(filename);

        char delimiter = ' ';
        string line;

        for (int i = 0; getline(input, line);)
        {
            // number of vertices in the first line.
            if (i == 0)
            {
                vertices = stoi(line);
                i += 1;

                // initialize the graph
                graph = new vector<Node>[vertices];
            }
            else
            {
                int position = 0, arr[3];

                // parsing the line into first, second ele and weight in the array
                for (int j = 0; (position = line.find(delimiter)) != string::npos; j++)
                {
                    arr[j] = stoi(line.substr(0, position));
                    line.erase(0, position + 1);
                }

                addEdge(arr[0], arr[1], arr[2]);
            }
        }
    }

    // constructor to initialize from existing graph
    Graph(vector<Node> *graph = nullptr) : graph(graph) {}

    // copy constructor needed for deep copy - since we are using random graph generate a new one.
    Graph(const Graph &that)
    {
        vertices = that.vertices;

        // graph is being created randomly
        if (that.min_range != 0 && that.max_range != 10)
        {
            density = that.density;
            min_range = that.min_range;
            max_range = that.max_range;
            generate_random_graph();
        }
        else
        {
            // deep copy as graph must have been created using file and vector copy is default a deep copy.
            graph = new vector<Node>[that.vertices];

            for (int curr_ele = 0; curr_ele < that.vertices; curr_ele++)
                graph[curr_ele] = that.graph[curr_ele];
        }
    }

    //destructor - delete the vector pair as created on heap using new operator
    ~Graph()
    {
        if (graph != NULL)
            delete[] graph;
        graph = NULL;
    }

    //accessor methods
    // used to add an edge inside the adjacency list - undirected edges
    void addEdge(int first_element, int second_element, int weight)
    {
        Node val = make_pair(second_element, weight);
        add_directed_edge(first_element, val);

        val = make_pair(first_element, weight);
        add_directed_edge(second_element, val);
    }

    // directed edge from one node to another
    void add_directed_edge(int element, Node value)
    {
        graph[element].push_back(value);
    }

    int get_edge_value(int first_ele, int second_ele)
    {
        for (auto node : graph[first_ele])
            if (node.first == second_ele)
                return node.second;

        return -1;
    }

    void set_edge_value(int first_ele, int second_ele, int weight)
    {
        for (auto x : graph[first_ele])
            if (x.first == second_ele)
            {
                x.second = weight;
                break;
            }

        for (auto x : graph[second_ele])
            if (x.first == first_ele)
            {
                x.second = weight;
            }
    }

    // adjacent nodes is bacisally all the nodes to which we'll be able to iterate based on the element.
    vector<Node> adjacent_nodes(int element)
    {
        return graph[element];
    }

    // return the total no of vertices
    int get_vertices()
    {
        return vertices;
    }

    // total no of edges
    int get_edges()
    {
        return (vertices * (vertices - 1)) / 2;
    }

    void print_graph_adjacency_list()
    {
        for (int i = 0; i < get_vertices(); i++)
        {
            cout << i << " => ";
            for (auto i : adjacent_nodes(i))
                cout << i.first << "." << i.second << " , ";

            cout << endl;
        }
    }

private:
    //private method to generate the graph as not necessarily needed to be exposed.
    void generate_random_graph()
    {
        graph = new vector<Node>[vertices];

        // for conecting the graph using the density the value determines the edge cost from one point to another.
        for (int j = 0; j < vertices * vertices; j++)
        {
            int f_element = random_vertex(vertices);
            int s_element = random_vertex(vertices);

            if (f_element != s_element && probability() < density)
            {
                int dist = distance(min_range, max_range);
                addEdge(f_element, s_element, dist);
            }
        }
    }

    // private variables.
    vector<Node> *graph;
    float density;
    int vertices;
    int min_range = 0; // default value to be checked in copy constructor
    int max_range = 10;
};

class MinimumSpanningTree
{
public:
    // constructor with initializer
    // initializing parent and key_distance also with what should be the size and default values.
    MinimumSpanningTree(Graph graph) : graph(graph)
    {
        initialize_data();
    }

    // destructor
    ~MinimumSpanningTree() {}

    //accessor methods
    void prims_mst(int src_element)
    {
        int no_of_vertices = graph.get_vertices();

        pqueue.push(make_pair(0, src_element));
        key_distance[src_element] = 0;

        while (!pqueue.empty())
        {
            int current_element = pqueue.top().second;
            pqueue.pop();

            vector<Node> adjacent_nodes = graph.adjacent_nodes(current_element);

            present_in_MST[current_element] = true;

            for (auto node : adjacent_nodes)
            {
                int vertex = node.first;
                int weight = node.second;

                // If there is shorted path to vertex through current element.
                if (present_in_MST[vertex] == false && key_distance[vertex] > weight)
                {
                    // Updating distance of vertex
                    key_distance[vertex] = weight;

                    // push to the min heap created using pqueue.
                    pqueue.push(make_pair(key_distance[vertex], vertex));

                    parent[vertex] = current_element;
                }
            }
        }
    }

    void print_path()
    {
        // Print edges of MST using parent array
        for (int ele = 1, vertices = graph.get_vertices(); ele < vertices; ++ele)
            cout << parent[ele] << " -> " << ele << endl;
    }

    int total_cost()
    {
        int cost = 0;
        for (int i = 1, n = graph.get_vertices(); i < n; ++i)
            cost += graph.get_edge_value(parent[i], i);

        return cost;
    }

private:
    void initialize_data()
    {
        int vertices = graph.get_vertices();

        key_distance = vector<int>(vertices, infinity);
        parent = vector<int>(vertices, -1);
        present_in_MST = vector<bool>(vertices, false);
    }

    // to store the key distances between the nodes
    vector<int> key_distance;
    vector<bool> present_in_MST;

    // used in tracing the path.
    vector<int> parent;

    Graph graph;
    priority_queue<Node, vector<Node>, greater<Node>> pqueue;
};

int main(void)
{
    // Create a graph from the sample file provided should be present in the same folder.
    Graph g = Graph("data.txt");
    // g.print_graph_adjacency_list();

    // create mst from the source element which is 0.
    MinimumSpanningTree mst = MinimumSpanningTree(g);
    mst.prims_mst(0);

    cout << "Edges in the trees are between : " << endl;
    mst.print_path();

    cout << "Total cost of the MST : " << mst.total_cost() << endl;
}
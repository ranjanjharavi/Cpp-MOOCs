/**
 * Implementation of Dijkstra's Algorithm using adjacency list.
 * 
 * Author: Ravi Ranjan
*/
#include <iostream>
#include <vector>
#include <queue>

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

    // constructor to initialize from existing graph
    Graph(vector<Node> *graph = nullptr) : graph(graph) {}

    // copy constructor needed for deep copy - since we are using random graph generate a new one.
    Graph(const Graph &that)
    {
        vertices = that.vertices;
        density = that.density;
        min_range = that.min_range;
        max_range = that.max_range;

        generate_random_graph();
    }

    //destructor - delete the vector pair as created on heap using new operator
    ~Graph()
    {
        delete[] graph;
    }

    //accessor methods
    // used to add an edge inside the adjacency list
    void addEdge(int first_element, int second_element, int weight)
    {
        Node val = make_pair(second_element, weight);
        graph[first_element].push_back(val);

        val = make_pair(first_element, weight);
        graph[second_element].push_back(val);
    }

    int get_edge_value(int first_ele, int second_ele)
    {
        for (auto x : graph[first_ele])
            if (x.first == second_ele)
                return x.second;

        return -1;
    }

    void set_edge_value(int first_ele, int second_ele, int weight)
    {
        for (auto x : graph[first_ele])
        {
            if (x.first == second_ele)
            {
                x.second = weight;
                break;
            }
        }
        for (auto x : graph[second_ele])
        {
            if (x.first == first_ele)
            {
                x.second = weight;
            }
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
    int min_range;
    int max_range;
};

class PriorityQueue
{
public:
    //constructor & destructor
    PriorityQueue() {}
    ~PriorityQueue() {}

    // accessor methods
    //remove the element from the queue
    void minPrioirty()
    {
        pqueue.pop();
    }

    Node top()
    {
        return pqueue.top();
    }

    int size()
    {
        return pqueue.size();
    }

    void push(int weight, int element)
    {
        pqueue.push(make_pair(weight, element));
    }

    bool empty()
    {
        return pqueue.empty();
    }

    bool contains(int element)
    {
        int val = find_index(element);
        return val != -1;
    }

    // returns the index if found, else -1;
    int find_index(int element)
    {
        int index = 0;
        priority_queue<Node, vector<Node>, greater<Node>> temp_q = pqueue;

        while (!temp_q.empty())
        {
            if (temp_q.top().second == element)
                return index;
            temp_q.pop();
            index++;
        }
        return -1;
    }

private:
    priority_queue<Node, vector<Node>, greater<Node>> pqueue;
};

class ShortestPath
{
public:
    // constructor with initializer
    // initializing parent and shortest_distance also with what should be the size and default values.
    ShortestPath(Graph graph, PriorityQueue pq, int src_element) : graph(graph), pqueue(pq), source_element(src_element)
    {
        initialize_data();
        shortest_path(src_element);
    }

    // destructor
    ~ShortestPath() {}

    //accessor methods
    // find the shortest path between source and destination
    void path(int src_element, int dest_element)
    {
        if (src_element != source_element)
        {
            // reinitializatin of parent and distances are necesary as t contain new data and the source vetex has changed.
            initialize_data();

            while (!pqueue.empty())
                pqueue.minPrioirty();

            source_element = src_element;
            shortest_path(src_element);
        }

        string str = (shortest_distances[dest_element] != infinity) ? to_string(source_element + 1) : "Not reachable";

        cout << "Shortest path between " << (src_element + 1) << " and " << (dest_element + 1) << " is : " << str;

        if (has_path(dest_element))
            print_path(dest_element);

        cout << endl;
    }

    string path_size(int src_element, int dest_element)
    {
        return (shortest_distances[dest_element] != infinity) ? to_string(shortest_distances[dest_element]) : "Infinity";
    }

    // calculate average short path excluding the unreachable distance.
    void average_short_path()
    {
        int vertices = graph.get_vertices();
        int total_cost = 0;

        for (int i = 0; i < vertices; ++i)
            if (shortest_distances[i] != infinity)
                total_cost += shortest_distances[i];

        float avg = static_cast<float>(total_cost) / vertices;
        cout << "Average shortest distances from source = " << avg << endl;
    }

    void vetices_with_shortest_path()
    {
        for (int i = 0, V = graph.get_vertices(); i < V; ++i)
        {
            string str = (shortest_distances[i] != infinity) ? to_string(shortest_distances[i]) : "Not reachable";

            cout << "Shortest Distance from  " << (source_element + 1) << "  to  " << (i + 1) << "  is  " << str;

            if (has_path(i))
            {
                cout << "\t\tPath = " << (source_element + 1);
                print_path(i);
            }
            cout << endl;
        }
    }

private:
    void initialize_data()
    {
        shortest_distances = vector<int>(graph.get_vertices(), infinity);
        parent = vector<int>(graph.get_vertices(), -1);
    }

    bool has_path(int element)
    {
        return (shortest_distances[element] != infinity) ? true : false;
    }

    void shortest_path(int src_element)
    {
        int no_of_vertices = graph.get_vertices();

        pqueue.push(0, src_element);
        shortest_distances[src_element] = 0;

        while (!pqueue.empty())
        {
            int current_element = pqueue.top().second;
            pqueue.minPrioirty();

            vector<Node> adjacent_nodes = graph.adjacent_nodes(current_element);

            for (auto node : adjacent_nodes)
                relax_edge(node, current_element);
        }
    }

    void relax_edge(Node node, int current_element)
    {
        int vertex = node.first;
        int weight = node.second;

        int possible_new_weight = shortest_distances[current_element] + weight;

        // If there is shorted path to vertex through current element.
        if (shortest_distances[vertex] > possible_new_weight)
        {
            // Updating distance of vertex
            shortest_distances[vertex] = possible_new_weight;

            // push to the min heap created using pqueue.
            pqueue.push(possible_new_weight, vertex);
        
            parent[vertex] = current_element;
        }
    }

    void print_path(int ele)
    {
        // Base Case : If ele is source
        if (parent[ele] == -1)
            return;

        print_path(parent[ele]);
        cout << " -> " << (ele + 1);
    }

    vector<int> shortest_distances;
    vector<int> parent;
    int source_element;
    Graph graph;
    PriorityQueue pqueue;
};

int main(void)
{
    // seed for random values.
    srand(time(0));
    int min_range = 1, max_range = 10;
    float density = 0.2;

    // user input.
    int vertices;
    int source_vertex;
    int destination_vertex;

    do
    {
        cout << "Enter number of vertices (max. 50) : ";
        cin >> vertices;

    } while (vertices <= 0 || vertices > 50);

    do
    {
        cout << "Enter source vertex : (1-" << vertices << ") : ";
        cin >> source_vertex;

    } while (source_vertex < 1 || source_vertex > vertices);

    cout << "Dijkstra's Algo : " << endl;

    // create graph.
    Graph graph = Graph(vertices, density, min_range, max_range);

    // create priority queue
    PriorityQueue pri_queue = PriorityQueue();

    // calculation inside the program from 0 to vertices-1 and hence decrease in size
    source_vertex -= 1;

    // Find the shortest path
    ShortestPath s_path = ShortestPath(graph, pri_queue, source_vertex);

    s_path.average_short_path();
    cout << endl;
    s_path.vetices_with_shortest_path();
    cout << endl;

    // ask for new source and destination in the same random graph
    do
    {
        cout << "Enter source vertex : (1-" << vertices << ") : ";
        cin >> source_vertex;

    } while (source_vertex < 1 || source_vertex > vertices);

    do
    {
        cout << "Enter destination vertex : (1-" << vertices << ") : ";
        cin >> destination_vertex;

    } while (destination_vertex < 1 || destination_vertex > vertices);

    cout << endl;

    // calculation inside the program from 0 to vertices-1 and hence decrease in size
    source_vertex -= 1;
    destination_vertex -= 1;

    // calculate path anc its cost from the new source to destination in the same graph.
    s_path.path(source_vertex, destination_vertex);
    cout << "Path cost : " << s_path.path_size(source_vertex, destination_vertex) << endl;

    // all path from new source and destination in the same graph
    cout << endl
         << "New Paths and costs : " << endl;
    s_path.vetices_with_shortest_path();

    return 0;
}

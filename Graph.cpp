/**
 * File Name: Graph.cpp
 * Last Updated: 8/4/23
 * Author: Thalia Valle Chavez
 * Desc: File with definitions of Graph class methods.
 */
#include "Graph.h"


/**
 * Description: Create and initialize new vertex
 * @param to
 * @param street
 * @param dst
 * @param sL
 * @param D
 * @return pointer to VertexListNod
 */
Graph::VertexListNode *Graph::getNewVertex(std::string to, std::string street, int dst, float sL, std::string D) {

    Graph::VertexListNode *newVertex = new Graph::VertexListNode{to, street, dst,
                                                                 sL, D, nullptr, (dst * sL)};
    return newVertex;
}


/**
 * Description: Partition function for Quick Sort Algorithm
 * @param arr
 * @param start
 * @param end
 * @return int
 */
int Graph::partition(std::vector<std::string> &arr, int start, int end) {

    int pIndex = start;
    std::string pivot_value = arr[end];

    for (int i = start; i < (end); i++) {
        if (arr[i].compare(pivot_value) < 0) {
            std::swap(arr[pIndex], arr[i]);
            pIndex++;
        }
    }
    std::swap(arr[pIndex], arr[end]);
    return pIndex;

}


/**
 * Description: Implement Quick Sort algorithm in array
 * @param arr
 * @param start
 * @param end
 */
void Graph::quickSort(std::vector<std::string> &arr, int start, int end) {

    if (end <= start) return;
    int pIndex = partition(arr, start, end);
    quickSort(arr, start, pIndex - 1);
    quickSort(arr, pIndex + 1, end);
}



/**
 * Description: Initialize the intersections vector by adding to it the first element of each line
 * in the file (the intersection). Then call quickSort function, and the intersections list will
 * end up sorted after the function call.
 */
void Graph::sortIntersectionsList() {
    std::string corner;
    while (getline(myFile, corner)) {
        std::string firstElement = corner.substr(0, corner.find(','));
        intersections.push_back(firstElement);
    }
    quickSort(intersections, 0, intersections.size() - 1);

}


/**
 * Description: To open file
 * @param fileName
 */
void Graph::processFile(const std::string &fileName) {
    // Opening file in myFile
    myFile.open(fileName);
    // Check if the file was opened successfully
    if (!myFile.is_open()) {
        std::cout << "Error opening file " << std::endl;
    }
}


/**
 * Description: It processes the file. It initializes the graph with data extracted from the given file.
 */
void Graph::initGraph() {

    // Storing the intersections in a list and sorting it
    sortIntersectionsList();

    // Copying the sorted list in the graph avoiding duplicates
    // Initializing the graph
    int graph_index = 0;
    int i = 0;
    do {
        if (intersections[i] != intersections[i + 1]) {
            graph[graph_index].IntersectionName = intersections[i];
            graph_index++;
        }
        i++;
    } while (i < intersections.size());


    // getting the final number of nodes in the graph
    totalNumOfNodes = graph_index + 1;


    //Create the graph VertextList for each node using an adjacency list
    for (int i = 0; i < totalNumOfNodes; i++) {

        std::string currentInt = graph[i].IntersectionName;

        // Reset the file pointer back to the beginning of the file
        myFile.clear();
        myFile.seekg(0, std::ios::beg);

        // Start parsing the file
        std::string line;
        while (getline(myFile, line)) {

            std::string firstElement = line.substr(0, line.find(','));
            if (currentInt == firstElement) {
                std::stringstream sStream(line);  //initializing stringstream sStream  with the content of line.

                std::string fromIntersection, toIntersection, streetName, direction;
                int distance;
                float speedLimit;

                // Parse the line
                getline(sStream, fromIntersection, ',');
                getline(sStream, streetName, ',');
                getline(sStream, toIntersection, ',');
                getline(sStream, direction, ',');
                sStream >> distance; // meters
                sStream.ignore(1); // Ignore the comma
                sStream >> speedLimit; //meters per second

                // If the node in question does not have a list yet, create one, else, add the node to the end of the list
                if (!graph[i].AdjVertexList) { // AdjVertexList acts as the head pointer to the linked list that has the connected vertices
                    graph[i].AdjVertexList = getNewVertex(toIntersection, streetName, distance, speedLimit, direction);
                } else {
                    VertexListNode *temp = nullptr;
                    temp = graph[i].AdjVertexList; // temp is holding the head of the linked list
                    while (temp->next) {
                        temp = temp->next;
                    }
                    temp->next = getNewVertex(toIntersection, streetName, distance, speedLimit, direction);
                }
            }
        }
    }
}

/**
 * Description: Implementation of binary search algorithm
 * @param from
 * @return
 */
int Graph::BinarySearch(std::string from) {
    int start = 0;
    int end = totalNumOfNodes - 1;
    int mid_index;

    while (start <= end) {
        mid_index = start + (end - start) / 2; //(end-start)/2;
        if (graph[mid_index].IntersectionName == from) {
            return mid_index;
        } else if (from < graph[mid_index].IntersectionName) {
            end = mid_index - 1;
        } else if (from > graph[mid_index].IntersectionName) {
            start = mid_index + 1;
        }
    }
    return -1;
}


/**
 * Description: It calls functions tp initialize the graph
 * @param fileName
 */
Graph::Graph(const std::string &fileName) {

    processFile(fileName);
    initGraph();
}


/**
 * Description: finds the path of shortest distance between two nodes using Dijkstra's Algorithm.
 * Note: Dijkstra's Algorithm generates a Shortest Path Graph (SPG). It takes a starting node and
 * generates a graph with the best path from the starting node to each other node of the graph.
 * @param start
 * @param end
 */
void Graph::shortestPath(std::string start, std::string end) {

    // Initialization of vectors of size totalNumNodes and default values.
    std::vector<int> distance(totalNumOfNodes, INT_MAX);
    std::vector<int> parent(totalNumOfNodes, -1);
    std::vector<bool> visited(totalNumOfNodes, false);

    // Use binary search to find the index of start node in graph array
    int startIndex = BinarySearch(start);
    if (startIndex == -1) {
        std::cout << "Start intersection not found.\n";
        return;
    }

    // Set the distance of the start node to 0, so we process it first
    distance[startIndex] = 0;

    // Creating a priority queue to store the shortest path distance of each node, and using a
    // vector for this purpose (storing the priority queue) . I am creating a
    // queue of pairs, were the elements are going to be ordered in increasing order because I am using
    // std::greater, thus the top of the queue will always have the node with the smallest distance.
    // In the pair, the first element represents the path distance and the second the index of the node in the graph.
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pqNodePathDist;

    // Pushing the starting node to the queue, it has a distance of 0
    pqNodePathDist.push({0, startIndex});


    // Here starts the main part of the Dijkstra's algorithm
    while (!pqNodePathDist.empty()) {
        //assigning the index of the node with the smallest known distance to the variable node
        int node = pqNodePathDist.top().second;
        pqNodePathDist.pop();

        // Proceed if the node has not been visited
        if (!visited[node]) {
            //Marking node current visiting as visited
            visited[node] = true;
            //Iterate through all the nodes adjacent to the currently selected node
            VertexListNode *temp = graph[node].AdjVertexList;
            while (temp != nullptr) {
                int adjacentNode = BinarySearch(temp->ToIntersectionName);
                int edgeWeight = temp->Distance;
                if (distance[node] + edgeWeight < distance[adjacentNode]) {
                    distance[adjacentNode] = distance[node] + edgeWeight;
                    parent[adjacentNode] = node;
                    pqNodePathDist.push({distance[adjacentNode], adjacentNode});
                }
                temp = temp->next;

            }
        }
    }

    // Printing  shortest path. Have to check for different scenarios
    // In case end intersection does not exist
    int endIndex = BinarySearch(end);
    if (endIndex == -1) {
        std::cout << "End Intersection not found.\n";
        return;
    }
    // In case there are no path between the start and end node
    if (!visited[endIndex]) {
        std::cout << "There is no path between " << start << " and " << end << ".\n";
        return;
    }

    std::vector<int> path;
    for (int v = endIndex; v != -1; v = parent[v]) path.push_back(v);
    std::reverse(path.begin(), path.end());


    std::cout << "Shortest path from " << start << " to " << end << ": Start at ";
    for (int i = 0; i < path.size(); ++i) {
        if (i != 0) std::cout << ". Then Go from " << graph[path[i - 1]].IntersectionName << " to ";
        std::cout << graph[path[i]].IntersectionName;
    }
    std::cout << '\n';

    std::cout << "Total distance: " << distance[endIndex] << " m \n";

}

/**
 * Description: finds the path between two nodes that takes the least amount of time to walk using Dijkstra's Algorithm
 * @param start
 * @param end
 */
void Graph::fastestPath(std::string start, std::string end) {

    // Initialization of vectors of size totalNumNodes and default values.
    std::vector<float> time(totalNumOfNodes, INT_MAX);
    std::vector<int> parent(totalNumOfNodes, -1);
    std::vector<bool> visited(totalNumOfNodes, false);

    int startIndex = BinarySearch(start);
    if (startIndex == -1) {
        std::cout << "Start intersection not found.\n";
        return;
    }

    time[startIndex] = 0;

    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> pqNodeTime;

    pqNodeTime.push({0, startIndex});

    while (!pqNodeTime.empty()) {
        int node = pqNodeTime.top().second;
        pqNodeTime.pop();

        if (!visited[node]) {
            visited[node] = true;

            VertexListNode *temp = graph[node].AdjVertexList;
            while (temp != nullptr) {
                int neighbor = BinarySearch(temp->ToIntersectionName);
                double edgeWeight = temp->timeToPath;
                if (time[node] + edgeWeight < time[neighbor]) {
                    time[neighbor] = time[node] + edgeWeight;
                    parent[neighbor] = node;
                    pqNodeTime.push({time[neighbor], neighbor});
                }
                temp = temp->next;
            }
        }
    }

    int endIndex = BinarySearch(end);
    if (endIndex == -1) {
        std::cout << "End intersection not found.\n";
        return;
    }

    if (!visited[endIndex]) {
        std::cout << "There is no path between " << start << " and " << end << ".\n";
        return;
    }

    std::vector<int> path;
    for (int v = endIndex; v != -1; v = parent[v]) path.push_back(v);
    std::reverse(path.begin(), path.end());

    std::cout << "Fastest path from " << start << " to " << end << ": Start at ";
    for (int i = 0; i < path.size(); ++i) {
        if (i != 0) std::cout << ". Then Go from " << graph[path[i - 1]].IntersectionName << " to ";
        std::cout << graph[path[i]].IntersectionName;
    }
    std::cout << '\n';

    //Converting time to minutes and seconds
    int timeSeconds = time[endIndex];
    timeSeconds = (timeSeconds % 60);
    int timeInMinutes = time[endIndex] / 60;
    std::cout << "Total time: " << timeInMinutes << " minutes and " << timeSeconds << " seconds. \n";
}



/**
 * Description: Methods previously used for debugging purposes. It prints to the console all the elements in
 * the intersection vector
 */
void Graph::outIntersectionList() {
    for (int i = 0; i < intersections.size() - 1; i++) {
        std::cout << intersections[i] << "\t";
    }
}

/**
 * Description: Methods previously used for debugging purposes. It prints to the console the intersection
 * names of each node in the graph and the intersection names of each nodes in their adjacency list together with some
 * elements of the node. In other words, it provides a kind of visual representation of the graph, since it
 * shows each node and their respective adjacent nodes.
 */
void Graph::outGraph() {
    for (int i = 0; i < totalNumOfNodes - 1; i++) {
        std::cout << graph[i].IntersectionName << "---> \t";
        VertexListNode *temp = graph[i].AdjVertexList;
        while (temp) {
            std::cout << temp->ToIntersectionName << temp->Direction << temp->SpeedLimit << "--" << temp->timeToPath
                      << "\t";
            temp = temp->next;
        }

        std::cout << "\n\n\n";
    }
}




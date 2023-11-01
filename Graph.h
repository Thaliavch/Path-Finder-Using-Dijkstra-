/**
 * File Name: Graph.h
 * Last Updated: 8/4/23
 * Author: Thalia Valle Chavez
 * Desc: Header file of class Graph. Contains declaration of all variables and methods.
 */
#ifndef MAIN_CPP_GRAPH_H
#define MAIN_CPP_GRAPH_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
const int MAX_ITN = 100;


class Graph {
    //*********************************** Adjacency List graph DS *********************************
    // I am using a linked list to represent the vertices each node connects to. (Adjacency List)
    struct VertexListNode
    {
        std::string ToIntersectionName;
        std::string StreetName;
        int Distance;
        float SpeedLimit;
        std::string Direction;
        VertexListNode* next;
        float timeToPath;
    };
    struct GraphNode
    {
        std::string IntersectionName;
        VertexListNode *AdjVertexList;

        //Constructor
        GraphNode(){
            AdjVertexList = nullptr;
        }
    };


    GraphNode graph[MAX_ITN];// Declaring the graph structure
    int totalNumOfNodes;
    std::ifstream myFile; // Creating an input file
    std::vector<std::string> intersections; // dynamic array that will later hold the intersections name of each node


//****************************** Private methods start here *************************************

    VertexListNode* getNewVertex(std::string to, std::string street, int dst, float sL, std::string D);
    int partition(std::vector<std::string> &arr, int start, int end);
    void quickSort(std::vector<std::string> &arr, int start, int end);
    void sortIntersectionsList();
    void processFile(const std::string& fileName);
    void initGraph();
    int BinarySearch(std::string from);

public:
    // Constructor
    Graph(const std::string& fileName);
    void shortestPath(std::string start, std::string end);
    void fastestPath(std::string start, std::string end);
    void outIntersectionList();
    void outGraph();
};



#endif //MAIN_CPP_GRAPH_H

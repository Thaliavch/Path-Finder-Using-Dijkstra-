/**
 * File Name: main.cpp
 * Last Updated: 8/4/23
 * Author: Thalia Valle Chavez
 * Desc: Creating a class to implement graph ADS using an adjacency list,
 *       and initialize the graph with the information in the fgcu.scv file.
 *       The class includes methods to find the path with the least weight from
 *       a node to another. These methods are implemented using Dijkstra's Algorithm.
 */
#include "Graph.h"


int main() {

    Graph myGraph("fgcu.csv");
    //myGraph.outIntersectionList();
    //myGraph.outGraph();


    std::cout << "\n";
    myGraph.shortestPath("Arts_Complex", "Reed_Hall");
    std::cout << "\n";
    myGraph.fastestPath("Arts_Complex", "Reed_Hall");
    std::cout << "\n";
    myGraph.shortestPath("Parking_Garage_3", "North_Lake_Village");
    std::cout << "\n";
    myGraph.fastestPath("North_Lake_Village", "Parking_Garage_3");
    std::cout << "\n";
    myGraph.shortestPath("Alico_Arena", "Welcome_Center");
    std::cout << "\n";
    myGraph.fastestPath("Alico_Arena", "Welcome_Center");



    return 0;
}
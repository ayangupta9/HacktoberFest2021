#include <iostream>
#include <list>
#include <math.h>
#include <algorithm>
#include <vector>

using namespace std;

#define INF 9999999

struct Node
{
    int row;
    int col;
    Node *parentNode;
    vector<Node *> vecNeightbors;
    float globalGoal;
    float localGoal;
    bool isVisited = false;
    bool isWall = false;

    void printNode()
    {
        cout << row << ", " << col << endl;
    }
};

void Solve_Astar(Node *nodes, Node *startNode, Node *endNode, int mapWidth, int mapHeight)
{
    for (int i = 0; i < mapWidth; i++)
    {
        for (int j = 0; j < mapHeight; j++)
        {
            nodes[j * mapWidth + i].isVisited = false;
            nodes[j * mapWidth + i].globalGoal = INF;
            nodes[j * mapWidth + i].localGoal = INF;
            nodes[j * mapWidth + i].parentNode = nullptr;
        }
    }

    auto distance = [](Node *a, Node *b)
    {
        return sqrtf((a->row - b->row) * (a->row - b->row) + (a->col - b->col) * (a->col - b->col));
    };

    auto heuristic = [distance](Node *a, Node *b)
    {
        return distance(a, b);
    };

    Node *currentNode = startNode;
    startNode->localGoal = 0.0f;
    startNode->globalGoal = heuristic(startNode, endNode);

    list<Node *> unvisitedNodes;
    unvisitedNodes.emplace_back(startNode);

    while (!unvisitedNodes.empty() && currentNode != endNode)
    {
        unvisitedNodes.sort([](Node *lhs, Node *rhs)
                            { return lhs->globalGoal < rhs->globalGoal; });

        while (!unvisitedNodes.empty() && unvisitedNodes.front()->isVisited)
            unvisitedNodes.pop_front();

        if (unvisitedNodes.empty())
            break;

        currentNode = unvisitedNodes.front();
        currentNode->isVisited = true;

        for (auto neighbor : currentNode->vecNeightbors)
        {
            if (!neighbor->isVisited && neighbor->isWall == 0)
                unvisitedNodes.emplace_back(neighbor);

            float possibleLowerGoal = currentNode->isVisited + distance(currentNode, neighbor);

            if (possibleLowerGoal < neighbor->localGoal)
            {
                neighbor->parentNode = currentNode;
                neighbor->localGoal = possibleLowerGoal;

                neighbor->globalGoal = neighbor->localGoal + heuristic(neighbor, endNode);
            }
        }
    }
}

int main()
{
    vector<Node *> optimumPath;
    Node *nodes = nullptr;

    int mapWidth = 5;
    int mapHeight = 5;

    nodes = new Node[mapWidth * mapHeight];

    for (int i = 0; i < mapWidth; i++)
    {
        for (int j = 0; j < mapHeight; j++)
        {
            nodes[j * mapWidth + i].row = i;
            nodes[j * mapWidth + i].col = j;
            nodes[j * mapWidth + i].isWall = false;
            nodes[j * mapWidth + i].parentNode = nullptr;
            nodes[j * mapWidth + i].isVisited = false;
        }
    }

    for (int i = 0; i < mapWidth; i++)
    {
        for (int j = 0; j < mapHeight; j++)
        {
            if (j > 0)
            {
                nodes[j * mapWidth + i].vecNeightbors.emplace_back(&nodes[(j - 1) * mapWidth + i]);
            }
            if (j < mapWidth - 1)
            {
                nodes[j * mapWidth + i].vecNeightbors.emplace_back(&nodes[(j + 1) * mapWidth + i]);
            }
            if (i > 0)
            {
                nodes[j * mapWidth + i].vecNeightbors.emplace_back(&nodes[j * mapWidth + (i - 1)]);
            }
            if (i < mapHeight - 1)
            {
                nodes[j * mapWidth + i].vecNeightbors.emplace_back(&nodes[j * mapWidth + (i + 1)]);
            }
        }
    }

    Node *startNode = nullptr;
    Node *endNode = nullptr;

    startNode = &nodes[(mapWidth / 2) * mapWidth + 1];
    endNode = &nodes[(mapHeight / 2) * mapWidth + (mapWidth - 2)];

    Solve_Astar(nodes, startNode, endNode, mapWidth, mapHeight);

    if (endNode != nullptr)
    {
        Node *p = endNode;
        while (p->parentNode != nullptr)
        {
            optimumPath.emplace_back(p);
            p = p->parentNode;
        }
    }

    for (auto i : optimumPath)
    {
        i->printNode();
    }
}
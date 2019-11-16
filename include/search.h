#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <random>
#include <math.h>
#include <functional>
#include <bits/stdc++.h>
#include <KDTree.hpp>
#include <queue>
#include <boost/functional/hash.hpp>
#include <MapReader.h>

struct SearchNode
{
    int x;
    int y;
    double g;
    double h;
    double f;
    SearchNode* parent;
    SearchNode()
    {
        g = DBL_MAX;
        f = DBL_MAX;
        parent = NULL;
        parent = NULL;
    }
    SearchNode(double x_, double y_)
    {
      x = x_;
      y = y_;
      g = DBL_MAX;
      f = DBL_MAX;
      parent = NULL;
    }
};

struct CompareNode {
    bool operator()(SearchNode* const& n1, SearchNode* const& n2)
    {
        return n1->f > n2->f;
    }
};

class Search
{
  SearchNode* m_start;
  SearchNode* m_goal;
  MapReader* m_mapReader;
  double m_actions[8][2];
  double m_action_cost[8];
  double m_numActions;
  std::unordered_map<std::size_t, SearchNode*> openMap;
  std::unordered_set<std::size_t> closedSet;
  std::priority_queue<SearchNode*, std::vector<SearchNode*>, CompareNode> m_openList;
  int m_weight = 1;
  double computePriority(SearchNode* node);
  double computeHeuristic(SearchNode* node);
  bool isGoal(SearchNode* node);
  void expand(SearchNode* node);
  std::size_t getHash(int node_x, int node_y);
  std::size_t getHash(SearchNode node);
  SearchNode* getNode(int succ_x, int succ_y, std::size_t Hash);
  SearchNode* createNode(int x, int y);
  std::vector<point_t> computePath(SearchNode* goal);
  bool validSucc(int succ_x, int succ_y);
  void set_mapReader(MapReader* mapReader);
  void printOpen();
public:
  Search();
  void set_start(point_t startPoint);
  void set_goal(point_t goalPoint);
  std::vector<point_t> plan();

};

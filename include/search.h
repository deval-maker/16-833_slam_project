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
    double x;
    double y;
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
        return n1->f < n2->f;
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
  void set_start(point_t startPoint);
  void set_goal(point_t goalPoint);
  double computePriority(SearchNode* node);
  double computeHeuristic(SearchNode* node);
  std::vector<point_t> plan();
  bool isGoal(SearchNode* node);
  void expand(SearchNode* node);
  std::size_t getHash(double node_x, double node_y);
  std::size_t getHash(SearchNode node);
  SearchNode* getNode(double succ_x, double succ_y, std::size_t Hash);
  SearchNode* createNode(double x, double y);
  std::vector<point_t> computePath(SearchNode* goal);
  bool validSucc(double succ_x, double succ_y);
  Search();
  void set_mapReader(MapReader* mapReader);

};

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <random>
#include <math.h>
#include <functional>
#include <bits/stdc++.h>
#include <search.h>

Search::Search()
{
  m_actions[0][0] = 0; m_actions[0][1] = 1;
  m_actions[1][0] = 0; m_actions[1][1] = -1;
  m_actions[2][0] = 1; m_actions[2][1] = 0;
  m_actions[3][0] = -1; m_actions[3][1] = 0;

  m_actions[4][1] = 1; m_actions[4][1] = 1;
  m_actions[5][1] = -1; m_actions[5][1] = -1;
  m_actions[6][1] = 1; m_actions[6][1] = -1;
  m_actions[7][1] = -1; m_actions[7][1] = 1;

  m_action_cost[0] = 1;
  m_action_cost[1] = 1;
  m_action_cost[2] = 1;
  m_action_cost[3] = 1;
  m_action_cost[4] = 1;
  m_action_cost[5] = 1;
  m_action_cost[6] = 1;
  m_action_cost[7] = 1;

  m_start = new SearchNode;
  m_goal = new SearchNode;
  m_numActions = 8;
}


void Search::set_start(point_t startPoint)
{

  m_start->x = startPoint[0];
  m_start->y = startPoint[1];
  m_start->g = 0;
  m_start->h = computeHeuristic(m_start);
  m_start->f = computePriority(m_start);
}
void Search::set_goal(point_t goalPoint)
{
  m_goal->x = goalPoint[0];
  m_goal->y = goalPoint[1];
}
void Search::set_mapReader(MapReader* mapReader)
{
  m_mapReader = mapReader;
}

double Search::computePriority(SearchNode* node)
{
  return node->g + m_weight*node->h;
}

double Search::computeHeuristic(SearchNode* node)
{
  return pow(pow(node->x - m_goal->x,2) + pow(node->y - m_goal->y,2), 1.0/2);
}

std::vector<point_t> Search::plan()
{
  m_openList.push(m_start);
  while(!m_openList.empty())
  {
    SearchNode* topNode = m_openList.top();
    if(isGoal(topNode))
    {
      return computePath(topNode);
    }
    expand(topNode);
    printOpen();
    m_openList.pop();
  }
}

bool Search::isGoal(SearchNode* node)
{
    return (node->x == m_goal->x && node->y == m_goal->y);
}

std::vector<point_t> Search::computePath(SearchNode* goal)
{
  SearchNode* curr = goal;
  std::vector<point_t> path;
  while(curr!=NULL)
  {
    point_t state;
    state.push_back(curr->x);
    state.push_back(curr->y);
    path.push_back(state);
    curr = curr->parent;
  }
  return path;
}

std::size_t Search::getHash(SearchNode node)
{
  std::size_t seed = 0;
  boost::hash_combine(seed,node.x);
  boost::hash_combine(seed,node.y);

  return seed;
}

std::size_t Search::getHash(int node_x, int node_y)
{
  std::size_t seed = 0;
  boost::hash_combine(seed,node_x);
  boost::hash_combine(seed,node_y);

  return seed;
}

SearchNode* Search::createNode(int x, int y)
{
  SearchNode* node = new SearchNode(x,y);
  node->h = computeHeuristic(node);
  node->f = computePriority(node);
  return node;
}

SearchNode* Search::getNode(int x, int y, std::size_t Hash)
{
  SearchNode* succ;
  if(openMap.find(Hash) != openMap.end())
  {
    succ = openMap[Hash];
  }
  else
  {
    succ = createNode(x,y);
    openMap[Hash] = succ;
  }
  return succ;
}
bool Search::validSucc(int x, int y)
{
  double value = m_mapReader->query_map(int(x),int(y));
  if(value >= 255) return false;
  else return true;
}

void Search::expand(SearchNode* parent)
{
  for(int i = 0; i<m_numActions; i++)
  {
    int succ_x = parent->x + m_actions[i][0];
    int succ_y = parent->y + m_actions[i][1];

    // if(!validSucc(succ_x,succ_y)) continue;

    std::size_t succHash = getHash(succ_x,succ_y);
    if(closedSet.find(succHash) != closedSet.end()) continue;

    SearchNode* succ = getNode(succ_x, succ_y, succHash);
    double new_g = parent->g + m_action_cost[i];
    if(new_g < succ->g)
    {
      succ->g = new_g;
      succ->f = computePriority(succ);
      succ->parent = parent;
      m_openList.push(succ);
    }

  }

  closedSet.insert(getHash(*parent));
}

void Search::printOpen()
{
  std::priority_queue<SearchNode*, std::vector<SearchNode*>, CompareNode> copy = m_openList;
  for(int i=0;i<copy.size();i++)
  {
    SearchNode* node = copy.top();
    std::cout<<node->x<<" "<<node->y<<" F val "<<node->f<<'\n';
    copy.pop();
  }
}

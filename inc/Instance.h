#pragma once

#include"common.h"
#include <map>

typedef std::pair<int, int> event;

// struct TemporalEdge{
//   event source;
//   event target;

//   TemporalEdge(event source, event target): source(source), target(target) {};

// };

// Currently only works for undirected unweighted 4-neighbor grids
class Instance 
{
public:
	int num_of_cols;
	int num_of_rows;
	int map_size;

	// enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size

	Instance() {}
	Instance(const string& map_fname, const string& agent_fname,
			 int num_of_agents = 0, int num_of_rows = 0, int num_of_cols = 0, int num_of_obstacles = 0, int warehouse_width = 0);

	void printAgents() const;

	inline bool isObstacle(int loc) const { return my_map[loc]; }

	inline bool validMove(int curr, int next) const;
	list<int> getNeighbors(int curr) const;

	inline int linearizeCoordinate(int row, int col) const { return (this->num_of_cols * row + col); }
	inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
	inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
	inline pair<int, int> getCoordinate(int id) const { return make_pair(id % this->num_of_cols, id / this->num_of_cols); }
	inline int getCols() const { return num_of_cols; }

	inline int getManhattanDistance(int loc1, int loc2) const
	{
		int loc1_x = getRowCoordinate(loc1);
		int loc1_y = getColCoordinate(loc1);
		int loc2_x = getRowCoordinate(loc2);
		int loc2_y = getColCoordinate(loc2);
		return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
	}

	inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2) const
	{
		return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
	}

	int getDegree(int loc) const
	{
		assert(loc >= 0 && loc < map_size && !my_map[loc]);
		int degree = 0;
		if (0 < loc - num_of_cols && !my_map[loc - num_of_cols])
			degree++;
		if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
			degree++;
		if (loc % num_of_cols > 0 && !my_map[loc - 1])
			degree++;
		if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
			degree++;
		return degree;
	}

	int getDefaultNumberOfAgents() const { return num_of_agents; }

	// JK: Moving to CBSNode in an attempt to turn this CBS Tree implementation into a CBS Forest
	// Moving temporal constraints to the nodes would (i think) give me the power to change the constraints
	//	each time I have a new root node. 
	// I think the temporal constraints will need to be copied from parent
  // // should be moved to private
  // // vector<TemporalEdge> temporal_cons;
  // // temporal_cons[i * num_of_agents + j] = [{k, l}]
  // // The k-th task of i should happens before the l-th task of j
  // vector<vector<pair<int, int>> > temporal_cons;

	// JK: Adding a few functions here to help fill instance inside LtlMapfPlanner
	Instance(const int num_agents, const int num_rows, const int num_cols, const std::vector<int>& obstacles);

	void SetStartForAgentIndex(int idx, int start);
	int GetStartForAgentIndex(int idx)
	{
		return start_locations[idx];
	}

	bool IsObstacle(int id)
	{
		return my_map[id];
	}

	void addProposition(int loc, int prop)
	{
		std::vector<int> current_props = label_map.at(loc);
		if (std::find(current_props.begin(), current_props.end(), prop) == current_props.end())
			label_map[loc].push_back(prop);
	}

	bool isLocationAllowed(int loc, int stage) const
	{
		std::vector<int> loc_labels = label_map.at(loc);
		std::vector<int> stage_unallowed_labels = robot_i_unallowed_props_per_stage.at(stage);
		for (int& label : loc_labels)
		{
			if (std::find(stage_unallowed_labels.begin(), stage_unallowed_labels.end(), label) != stage_unallowed_labels.end())
				return false;
		}
		return true;
	}

	std::map<int, std::vector<int>> robot_i_unallowed_props_per_stage; // maps a stage (int) to (unallowed_props vector of ints)
	// JK: robot_stage_unallowed_props will need to be updated every time multi-level A* is called bc it is robot / task assignment specific!!

protected:
	// int moves_offset[MOVE_COUNT];
	vector<bool> my_map;
	string map_fname;
	string agent_fname;

	// std::map<int, bool> is_location_empty; // JK: true unless location in workspace satisfies a boolean proposition
	// JK: second attempt at making these constraints
	std::map<int, std::vector<int>> label_map; // JK: maps location (int) --> a set of labels (vector of int) that are true at such a state

	int num_of_agents;
	vector<int> start_locations;

	// JK: Moving Goal location definitions to CBSNode for my work
	// vector<vector<int>> goal_locations;


	bool loadMap();
	void printMap() const;
	void saveMap() const;

	virtual bool loadAgents();
	virtual void saveAgents() const;

	void generateConnectedRandomGrid(int rows, int cols, int obstacles); // initialize new [rows x cols] map with random obstacles
	void generateRandomAgents(int warehouse_width);
	bool addObstacle(int obstacle); // add this obstacle only if the map is still connected
	bool isConnected(int start, int goal); // run BFS to find a path between start and goal, return true if a path exists.

	int randomWalk(int loc, int steps) const;

	// Class  SingleAgentSolver can access private members of Node
	friend class SingleAgentSolver;
};


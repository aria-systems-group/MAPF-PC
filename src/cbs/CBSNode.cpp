#include "CBSNode.h"


void CBSNode::clear()
{
	conflicts.clear();
	unknownConf.clear();
	conflictGraph.clear();
}

void CBSNode::updateStagePropMap(int agent, Instance& instance) const
{
	instance.robot_i_unallowed_props_per_stage = unallowed_props_maps[agent];
}

void CBSNode::printConflictGraph(int num_of_agents) const
{
	if (conflictGraph.empty())
		return;
	cout << "	Build conflict graph in " << *this << ": ";
	for (auto e : conflictGraph)
	{
		if (e.second == 0)
			continue;
		int i = e.first / num_of_agents;
		int j = e.first % num_of_agents;
		std::cout << "(" << i << "," << j << ")=" << e.second << ",";
	}
	cout << endl;
}

std::ostream& operator<<(std::ostream& os, const CBSNode& node)
{
	os << "Node " << node.helpers_idx << " (" << node.g_val + node.h_val << " = " << node.g_val << " + " <<
	   node.h_val << " ) with " << node.conflicts.size() + node.unknownConf.size() << " conflicts and " <<
	   node.paths.size() << " new paths ";
	return os;
}
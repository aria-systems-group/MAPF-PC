#include "SingleAgentSolver.h"


list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
	list<int> rst = instance.getNeighbors(curr);
	rst.emplace_back(curr);
	return rst;
}

// JK: This is a TO-DO
void SingleAgentSolver::compute_heuristics(const vector<int>& goals)
{
	struct Node
	{
		int location;
		int value;

		Node() = default;
		Node(int location, int value) : location(location), value(value) {}

		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};

  my_heuristic.resize(goals.size());
  heuristic_landmark.resize(goals.size(), 0);

  for (int i = 0; i < goals.size(); i++){
    my_heuristic[i].resize(instance.map_size, MAX_TIMESTEP);

    // generate a heap that can save nodes (and a open_handle)
    boost::heap::pairing_heap<Node, boost::heap::compare<Node::compare_node>> heap;
    Node root(goals[i], 0);
    my_heuristic[i][goals[i]] = 0;
    heap.push(root);  // add root to heap
    while (!heap.empty())
      {
        Node curr = heap.top();
        heap.pop();
        for (int next_location : instance.getNeighbors(curr.location))
          {
            if (my_heuristic[i][next_location] > curr.value + 1)
              {
                my_heuristic[i][next_location] = curr.value + 1;
                Node next(next_location, curr.value + 1);
                heap.push(next);
              }
          }
      }
  }


  for (int i = goals.size() - 2; i >= 0; i--){
    heuristic_landmark[i] = heuristic_landmark[i + 1] + my_heuristic[i + 1][goals[i]];
  }

}

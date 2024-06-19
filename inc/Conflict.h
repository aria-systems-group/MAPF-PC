#pragma once

#include "common.h"

// chooses smallest first
enum conflict_type {MUTEX, TARGET, CORRIDOR, RECTANGLE, STANDARD, TEMPORAL, TRACE, TYPE_COUNT };

// enum conflict_type {TEMPORAL, STANDARD, TRACE, MUTEX, TARGET, CORRIDOR, RECTANGLE, TYPE_COUNT};
// enum conflict_type {TEMPORAL, TRACE, STANDARD, MUTEX, TARGET, CORRIDOR, RECTANGLE, TYPE_COUNT};
// enum conflict_type {STANDARD, TEMPORAL, TRACE, MUTEX, TARGET, CORRIDOR, RECTANGLE, TYPE_COUNT};
// enum conflict_type {STANDARD, TRACE, TEMPORAL, MUTEX, TARGET, CORRIDOR, RECTANGLE, TYPE_COUNT};
// enum conflict_type {TRACE, STANDARD, TEMPORAL, MUTEX, TARGET, CORRIDOR, RECTANGLE, TYPE_COUNT};
// enum conflict_type {TEMPORAL, TRACE, STANDARD, MUTEX, TARGET, CORRIDOR, RECTANGLE, TYPE_COUNT};

enum conflict_priority { CARDINAL, PSEUDO_CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };
// Pseudo-cardinal conflicts are semi-/non-cardinal conflicts between dependent agents.
// We prioritize them over normal semi-/non-cardinal conflicts

enum constraint_type
{
	LEQLENGTH, GLENGTH, RANGE, BARRIER, VERTEX, EDGE,
  LEQSTOP, GSTOP,
	POSITIVE_VERTEX, POSITIVE_EDGE, GPRIORITY, CONSTRAINT_COUNT
};

enum conflict_selection {RANDOM, EARLIEST, CONFLICTS, MCONSTRAINTS, FCONSTRAINTS, WIDTH, SINGLETONS};

typedef std::tuple<int, int, int, int, constraint_type> Constraint;
// <agent, loc, -1, t, VERTEX>
// <agent, loc, -1, t, POSITIVE_VERTEX>
// <agent, from, to, t, EDGE> 
// <agent, B1, B2, t, BARRIER>
// <agent, loc, t1, t2, CORRIDOR> 
// <agent, loc, -1, t, LEQLENGTH>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <agent, loc, -1, t, GLENGTH>: path of agent_id should be of length at least t + 1
// <agent, id_goal, -1, t, GSTOP>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <agent, id_goal, -1, t, LEQSTOP>: path of agent_id should be of length at least t + 1
// <id_from, id_to, -1, -1, GPRIORITY>: used for PBS

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);


class Conflict
{
public:
	int a1;
	int a2;
	list<Constraint> constraint1;
	list<Constraint> constraint2;
	vector<Constraint> trace_constraints;
	conflict_type type;
	conflict_priority priority = conflict_priority::UNKNOWN;
	double secondary_priority = 0; // used as the tie-breaking criteria for conflict selection

  // For mutex propagation
  int final_len_1;
  int final_len_2;

  // For mutex propagation in Non-cardinal
  // The actuall MDD node/edge which was used to generated the biclique;
  int t1;
  int loc1;
  int loc1_to=-1;
  int t2;
  int loc2;
  int loc2_to=-1;

	void vertexConflict(int a1, int a2, int v, int t)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1.emplace_back(a1, v, -1, t, constraint_type::VERTEX);
		this->constraint2.emplace_back(a2, v, -1, t, constraint_type::VERTEX);
		type = conflict_type::STANDARD;
	}
		
	void edgeConflict(int a1, int a2, int v1, int v2, int t)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1.emplace_back(a1, v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(a2, v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}

	void corridorConflict(int a1, int a2, int v1, int v2, int t1, int t2)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1.emplace_back(a1, v1, 0, t1, constraint_type::RANGE);
		this->constraint2.emplace_back(a2, v2, 0, t2, constraint_type::RANGE);
		type = conflict_type::CORRIDOR;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
						   int Rg_t, const list<Constraint>& constraint1, const list<Constraint>& constraint2) // For RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1 = constraint1;
		this->constraint2 = constraint2;
		type = conflict_type::RECTANGLE;
		return true;
	}


	void targetConflict(int a1, int a2, int v, int t)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1.emplace_back(a1, v, -1, t, constraint_type::LEQLENGTH);
		this->constraint2.emplace_back(a1, v, -1, t, constraint_type::GLENGTH);
		type = conflict_type::TARGET;
	}


	void mutexConflict(int a1, int a2)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		type = conflict_type::MUTEX;
		priority = conflict_priority::CARDINAL;
		// TODO add constraints from mutex reasoning
	}

    // time of a1 should be smaller than a2
    // however, t1 >= t2
    /* JK: This is how it is used
    auto from_landmark = cons.first; // JK: this is an index, not an actual goal location!
    auto to_landmark = cons.second; // JK: same as above
    if (paths[a1]->timestamps[from_landmark] >= paths[a2]->timestamps[to_landmark]){
      // cout << "Temporal conflict between " << a1  << "(" << from_landmark<< ")" << " and " << a2 << "(" << to_landmark<< ")" << endl;
      shared_ptr<Conflict> conflict(new Conflict());
      conflict->temporalConflict(a1, a2, from_landmark, to_landmark, paths[a1]->timestamps[from_landmark], paths[a2]->timestamps[to_landmark]);
      curr.conflicts.push_back(conflict);
    }
    */
    void temporalConflict(int a1, int a2, int i1, int i2, int t1, int t2)
    {
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		type = conflict_type::TEMPORAL;
		// priority = conflict_priority::CARDINAL;
		this->constraint1.emplace_back(a1, i1, -1, t1 - 1, constraint_type::LEQSTOP); // item (2) part 2
		this->constraint1.emplace_back(a2, i2, -1, t1, constraint_type::LEQSTOP); // item (1)
		this->constraint2.emplace_back(a2, i2, -1, t1, constraint_type::GSTOP); // item (2) part 1
    }

    /* 
		JK: A trace conflict is basically a vertex conflict except it only results in a single agent being constrained. 
		This does not break completeness because (unlike all other types of conflict), trace conflicts are caused 
		by only a single agent (i.e. they do not require 2 agents/paths to be fully defined)
    */
    void traceConflict(vector<int> locations, vector<int> timestamps)
	{
		type = conflict_type::TRACE;
		for (int i = 0; i < locations.size(); i++)
			trace_constraints.emplace_back(i, locations[i], -1, timestamps[i], constraint_type::VERTEX);

		// constraint1.clear();
		// constraint2.clear();
		// this->a1 = a1;
		// this->a2 = -1; // non-existent
		// this->constraint1.emplace_back(a1, v, -1, t, constraint_type::VERTEX);
		// type = conflict_type::TRACE; // we might be able to resolve trace conflicts by taking a slightly different path (but probably not)
	}

    void dumb_conflict(vector<Path*>& paths)
    {
        /*
            Dumb conflict basically adds a constraint for every single vertex, time point in the plan (including every agent)
        */
        type = conflict_type::TRACE;
        for (int i = 0; i < paths.size(); i++)
        {
            for (size_t k = 0; k < paths[i]->size(); k++)
            {
                trace_constraints.emplace_back(i, paths[i]->at(k).location, -1, k, constraint_type::VERTEX);
            }
        }
    }


  void priorityConflict(int a1, int a2)
  {
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		// type = conflict_type::TEMPORAL;
		// priority = conflict_priority::CARDINAL;
		this->constraint1.emplace_back(a1, a2, -1, -1, constraint_type::GPRIORITY);
		this->constraint2.emplace_back(a2, a1, -1, -1, constraint_type::GPRIORITY);
  }
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator<(const Conflict& conflict1, const Conflict& conflict2);

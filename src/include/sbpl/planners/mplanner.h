/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __MPLANNER_H_
#define __MPLANNER_H_


//---configuration----

//control of EPS
//initial suboptimality bound (cost solution <= cost(eps*cost optimal solution)
#define M_DEFAULT_INITIAL_EPS	    5.0
// as planning time exist, M* decreases epsilon bound
#define M_DECREASE_EPS    0.2
//final epsilon bound
#define M_FINAL_EPS	    1.0

enum {
    T_SMHA,
    T_IMHA,
    T_MPWA,
    T_MHG_REEX,
    T_MHG_NO_REEX,
    T_EES,
};


//---------------------

#define M_INCONS_LIST_ID1 0
#define M_INCONS_LIST_ID2 1 

#include <cstdio>
#include <ctime>
#include <vector>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>

class CMDP;
class CMDPSTATE;
class CMDPACTION;
class CHeap;
class CHeapArr;
class CList;
class DiscreteSpaceInformation;
class MDPConfig;
class StateChangeQuery;

//-------------------------------------------------------------


/** \brief state structure used in M* search tree
  */
typedef class MSEARCHSTATEDATA : public AbstractSearchState
{
public:
  /** \brief the MDP state itself
  */
	CMDPSTATE* MDPstate; 
	/** \brief M* relevant data
    */
	unsigned int v[MAX_NUM];
	/** \brief M* relevant data
    */
	unsigned int g[MAX_NUM];
	/** \brief M* relevant data
    */
	short unsigned int iterationclosed[MAX_NUM];
	/** \brief M* relevant data
    */
	short unsigned int callnumberaccessed[MAX_NUM];
	/** \brief M* relevant data
    */
	short unsigned int numofexpands;
	/** \brief best predecessor and the action from it, used only in forward searches
    */
	//CMDPSTATE *bestpredstate;
	/** \brief the next state if executing best action
    */
	CMDPSTATE  *bestpredstate[MAX_NUM];
	CMDPSTATE  *bestnextstate[MAX_NUM];
	unsigned int costtobestnextstate[MAX_NUM];
	int h[MAX_NUM];
public:
	MSEARCHSTATEDATA() {};	
	~MSEARCHSTATEDATA() {};
} MState;




/** \brief the statespace of M*
  */
typedef struct MSEARCHSTATESPACE
{
	double eps;
	double eps1;
	double eps2;
    	double eps_satisfied;
	CHeapArr* heap;
	CList* inconslist[MAX_NUM];
	short unsigned int searchiteration;
	short unsigned int callnumber;
	CMDPSTATE* searchgoalstate;
	CMDPSTATE* searchstartstate;
    
	CMDP searchMDP;

	bool bReevaluatefvals;
    	bool bReinitializeSearchStateSpace;
	bool bNewSearchIteration;

} MSearchStateSpace_t;



/** \brief M* planner
  */
class MPlanner : public SBPLPlanner
{

	public:

		int replan(double allocated_time_secs, std::vector<int>* solution_stateIDs_V);
		int replan(double allocated_time_sec, std::vector<int>* solution_stateIDs_V, int* solcost);

		int set_goal(int goal_stateID);
		int set_goal(int goal_stateID, int i);
		int set_start(int start_stateID);

		void costs_changed(StateChangeQuery const & stateChange);

		void costs_changed();
		void PrintOpenList(MSearchStateSpace_t* pSearchStateSpace, int a);
		int force_planning_from_scratch(); 

		int set_search_mode(bool bSearchUntilFirstSolution);

		virtual double get_solution_eps() const {return pSearchStateSpace_->eps_satisfied;};

		virtual int get_n_expands() const { return searchexpands; }
		virtual void set_initialsolution_eps(double initialsolution_eps) {finitial_eps = initialsolution_eps;};
		
        virtual void set_initialsolution_eps1(double initialsolution_eps) {finitial_eps1 = initialsolution_eps;};
		virtual void set_initialsolution_eps2(double initialsolution_eps) {finitial_eps2 = initialsolution_eps;};

		void print_searchpath(FILE* fOut);
		/** \brief constructor 
		 */
		MPlanner(DiscreteSpaceInformation* e1, int kk, bool bforwardsearch, int
			planner_type
			= T_SMHA);
		/** \brief destructor
		 */
		~MPlanner();

		double get_initial_eps(){return finitial_eps1; };

		/** \brief returns the time taken to find the first solution
		 */
		double get_initial_eps_planning_time(){return finitial_eps_planning_time;}

		/** \brief returns the time taken to get the final solution
		 */
		double get_final_eps_planning_time(){return final_eps_planning_time;};

		/** \brief returns the number of expands to find the first solution
		 */
		int get_n_expands_init_solution(){return num_of_expands_initial_solution;};

		/** \brief returns the final epsilon achieved during the search
		 */
		double get_final_epsilon(){return finitial_eps2;};
		
		bool IsExpanded(int s);
		int env_num;
	private:

		//member variables
		double finitial_eps, finitial_eps1, finitial_eps2, finitial_eps_planning_time, final_eps_planning_time, final_eps;
		int num_of_expands_initial_solution;

		MDPConfig* MDPCfg_;

		bool bforwardsearch; //if true, then search proceeds forward, otherwise backward

		bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

		MSearchStateSpace_t* pSearchStateSpace_;

		unsigned int searchexpands;
		int MaxMemoryCounter;
		clock_t TimeStarted;
		FILE *fDeb;

		int m_planner_type;


		//member functions
		void Initialize_searchinfo(CMDPSTATE* state, MSearchStateSpace_t* pSearchStateSpace);
		void Initialize_searchinfoCombined(CMDPSTATE* state, MSearchStateSpace_t* pSearchStateSpace);

		CMDPSTATE* CreateState(int stateID, MSearchStateSpace_t* pSearchStateSpace);
		CMDPSTATE* CreateStateCombined(int stateID, MSearchStateSpace_t* pSearchStateSpace);

		CMDPSTATE* GetState(int stateID, MSearchStateSpace_t* pSearchStateSpace);
		CMDPSTATE* GetStateCombined(int stateID, MSearchStateSpace_t* pSearchStateSpace);

		int ComputeHeuristic(CMDPSTATE* MDPstate, MSearchStateSpace_t* pSearchStateSpace, int i);
		int ComputeHeuristic1(CMDPSTATE* MDPstate, MSearchStateSpace_t* pSearchStateSpace, int i);
		int ComputeHeuristicCombined (CMDPSTATE* MDPstate, MSearchStateSpace_t* pSearchStateSpace);

		//initialization of a state
		void InitializeSearchStateInfo(MState* state, MSearchStateSpace_t* pSearchStateSpace);
		void InitializeSearchStateInfoCombined(MState* state, MSearchStateSpace_t* pSearchStateSpace);

		//re-initialization of a state
		void ReInitializeSearchStateInfo(MState* state, MSearchStateSpace_t* pSearchStateSpace);
		void ReInitializeSearchStateInfo(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i);
		void ReInitializeSearchStateInfoCombined (MState* state, MSearchStateSpace_t* pSearchStateSpace);

		void DeleteSearchStateData(MState* state);

		//used for backward search
		void UpdatePreds(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i);
		void UpdatePredsShared(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i);
		void UpdatePredsEES (MState* state, MSearchStateSpace_t* pSearchStateSpace);
		void UpdatePredsMHG (MState* state, MSearchStateSpace_t* pSearchStateSpace);


		//used for forward search
		void UpdateSuccs(MState* state, MSearchStateSpace_t* pSearchStateSpace);
		void UpdateSuccs(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i);
		void UpdateSuccsShared (MState* state, MSearchStateSpace_t* pSearchStateSpace, int i);
		void UpdateSuccsMPWA (MState* state, MSearchStateSpace_t* pSearchStateSpace, int i);
		void UpdateSuccsEES (MState* state, MSearchStateSpace_t* pSearchStateSpace);
		// Returns whether it has generated the goal state or not.
		bool UpdateSuccsMHG (MState* state, MSearchStateSpace_t* pSearchStateSpace,
			int i);
		void UpdateSuccsCombined(MState* state, MSearchStateSpace_t* pSearchStateSpace);

		int GetGVal(int StateID, MSearchStateSpace_t* pSearchStateSpace);

		//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
		int ImprovePath(MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);
		int ImprovePath(MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs, int i);
		int ImprovePathRoundRobin (MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);
		int ImprovePathRoundRobinShared (MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);
		int ImprovePathMPWA (MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);
		int ImprovePathCombined (MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);
		int ImprovePathEES (MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);
		int ImprovePathMHG (MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);


		void BuildNewOPENList(MSearchStateSpace_t* pSearchStateSpace);

		void Reevaluatefvals(MSearchStateSpace_t* pSearchStateSpace);

		//creates (allocates memory) search state space
		//does not initialize search statespace
		int CreateSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace);

		//deallocates memory used by SearchStateSpace
		void DeleteSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace);

		//debugging 
		void PrintSearchState(MState* state, FILE* fOut);


		//reset properly search state space
		//needs to be done before deleting states
		int ResetSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace);

		//initialization before each search
		void ReInitializeSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace);

		//very first initialization
		int InitializeSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace);

		int SetSearchGoalState(int SearchGoalStateID, MSearchStateSpace_t* pSearchStateSpace);
		int SetSearchGoalState(int SearchGoalStateID, MSearchStateSpace_t* pSearchStateSpace, int i);


		int SetSearchStartState(int SearchStartStateID, MSearchStateSpace_t* pSearchStateSpace);

		//reconstruct path functions are only relevant for forward search
		int ReconstructPath(MSearchStateSpace_t* pSearchStateSpace);


		void PrintSearchPath(MSearchStateSpace_t* pSearchStateSpace, FILE* fOut);

		int getHeurValue(MSearchStateSpace_t* pSearchStateSpace, int StateID);

		//get path 
		std::vector<int> GetSearchPath(MSearchStateSpace_t* pSearchStateSpace, int& solcost);
		//std::vector<int> GetSearchPath_d(MSearchStateSpace_t* pSearchStateSpace, int& solcost);


		bool Search(MSearchStateSpace_t* pSearchStateSpace, std::vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs);


};


#endif




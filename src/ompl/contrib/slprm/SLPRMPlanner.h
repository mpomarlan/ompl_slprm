#ifndef __SLPRMPLANNER_H__

#define __SLPRMPLANNER_H__

#include <string>
#include <vector>
#include <set>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include <ompl/contrib/slprm/SLPRMTypes.h>
#include <ompl/contrib/slprm/SLPRMEdge.h>
#include <ompl/contrib/slprm/SLPRMVertex.h>
#include <ompl/contrib/slprm/SetUnion.h>

#include <ompl/base/State.h>
#include <ompl/base/StateStorage.h>

#include <fstream>


namespace ompl
{
namespace geometric
{
namespace SLPRM
{    

    typedef std::vector<SLPRMVertex*> WorkingVertexVector;
    typedef std::vector<SLPRMEdge*> WorkingEdgeVector;
    class SLPRMPlanner: public ompl::base::Planner
    {
      public:
        SLPRMPlanner(const ompl::base::SpaceInformationPtr &si, const std::string &name);
        SLPRMPlanner(const ompl::base::SpaceInformationPtr &si);
        virtual ~SLPRMPlanner();

        // Haxx for moveit
        void setSI(const ompl::base::SpaceInformationPtr &si);

        // Roadmap path and flag functions
        void setRoadmapPath(std::string const& path);
        void getRoadmapPath(std::string &path) const;
        void setTrustRoadmapFlag(bool trustRoadmap);
        bool getTrustRoadmapFlag(void) const;

        // Cost bump get/set
        double getBumpRadius(void) const;
        void setBumpRadius(double radius);
        double getUnbumpRadius(void) const;
        void setUnbumpRadius(double radius);
        double getBumpValue(void) const;
        void setBumpValue(double value);
        double getUnbumpValue(void) const;
        void setUnbumpValue(double value);

        // OMPL interface functions
        virtual void setProblemDefinition (const ompl::base::ProblemDefinitionPtr &pdef);
        virtual void checkValidity(void);
        virtual base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);
        virtual void clear(void);
        virtual void setup(void);
        virtual void getPlannerData(ompl::base::PlannerData &data) const;
        virtual void printProperties(std::ostream &out) const;
        virtual void printSettings(std::ostream &out) const;

        void getNodeCosts(std::vector<double> &costs) const;
        void getNodeCCs(std::vector<ompl::geometric::SLPRM::tIndex> &CCIndexes) const;
        void getStates(std::vector<ompl::base::State*> &states) const;

        void addVertex(const ompl::base::State *newState);
        //Loading from/storing to file: full graph load/store, edges included. Faster load
        void storeVerticesWithMetaData(void) const;
        void loadVerticesWithMetaData(void);

      protected:

      private:

        //Loading from/storing to file: the slow versions, as edges are not loaded, rather retried
        void storeVertices(void) const;
        void loadVertices(void);
        //Debug function: check whether maintained CCs are maximal
        bool verifyCC(tIndex vIndex) const;

        void resetVisitation(void);

        tIndex getKNeighborhood(tIndex index, tIndex k, std::vector<tIndex> &neighbors) const;
        tIndex getRNeighborhood(tIndex index, double r, std::vector<tIndex> &neighbors) const;
        tIndex getValidKNeighborhood(tIndex index, tIndex k, std::vector<tIndex> &neighbors) const;
        tIndex getValidRNeighborhood(tIndex index, double r, std::vector<tIndex> &neighbors) const;
        tIndex getValidNeighborhood(tIndex index, std::vector<tIndex> &neighbors) const;

        tIndex getNeighborCCs(std::vector<tIndex> const &neighbors, std::set<tIndex> &neighborCCs) const;
        tIndex getCommonCCs(std::set<tIndex> const& neighborACCs, std::set<tIndex> const& neighborBCCs, std::set<tIndex> & commonCCs) const;
        bool areInSameCC(tIndex A, tIndex B) const;

        void addEdges(tIndex index, std::vector<tIndex> const& neighbors, bool i2n, bool n2i);

        void sampleAndConnect(const ompl::base::PlannerTerminationCondition &ptc);

        void costBump(tIndex fromVertex);
        void costUnbump(tIndex fromVertex);

        bool DijkstraSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::SLPRM::WorkingVertexVector &pathCandidate);
        bool AStarSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::SLPRM::WorkingVertexVector &pathCandidate);
        bool graphSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::PathGeometric &solution, bool useAStar = true);
        bool checkPathCandidate(const ompl::base::PlannerTerminationCondition &ptc, WorkingVertexVector &pathCandidate);

        void clearScratch(void);

        static double distance(SLPRMPlanner const* planner, tIndex a, tIndex b);

        ompl::base::ValidStateSamplerPtr validSampler_;
        //Disjoint set structure. Querying which set an object belongs to (which is defined as a const query for the planner) may change this structure.
        mutable ComponentMap CCs_;
        NearQueryStructure nearFinder_;
        std::vector<SLPRMVertex> vertices_;
        SLPRMVertex workingVertex_;
        tIndex crVisitation_;
        WorkingVertexVector exactVertexChecks_;
        WorkingEdgeVector exactEdgeChecks_;
        double bumpRadius_;
        double unbumpRadius_;
        double bumpValue_;
        double unbumpValue_;
        std::string roadmapPath_;
        bool roadmapLoaded_;
        //If true, vertices and edges in the roadmap are not checked for validity.
        bool trustRoadmap_;
        //If true, extend roadmap if it fails to connect start and goal states. Currently, fixed to the same value as trustRoadmap_.
        bool allowRoadmapExtension_;
    };
}
}
}

#endif //ndef __SLPRMPLANNER_H__

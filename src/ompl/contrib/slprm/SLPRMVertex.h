#ifndef __SLPRMVERTEX_H__

#define __SLPRMVERTEX_H__

#include <vector>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include <ompl/contrib/slprm/SLPRMTypes.h>
#include <ompl/contrib/slprm/SLPRMEdge.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

namespace ompl
{
namespace geometric
{
namespace SLPRM
{
    class SLPRMVertexMetaData;
    class SLPRMVertex;

    class SLPRMVertexMetaData
    {
      friend class boost::serialization::access;
      friend class SLPRMVertex;

      public:
        SLPRMVertexMetaData();

        SLPRMVertexMetaData(SLPRMVertex const& vertex);
        SLPRMVertexMetaData(SLPRMVertex const* vertex);

        SLPRMVertexMetaData(SLPRMVertexMetaData const& vertex);
        SLPRMVertexMetaData(SLPRMVertexMetaData const* vertex);

        ~SLPRMVertexMetaData();

        template<class Archive> void serialize(Archive & ar, const unsigned int version)
        {
          ar & this->edges_;
          ar & this->nodeCost_;
          ar & this->negativeDist_;
          ar & this->CCindex_;
          ar & this->preceding_;
          ar & this->precedingEdge_;
          ar & this->index_;
          ar & this->lastInitAlg_;
          ar & this->lastVisitation_;
          ar & this->exactInvalid_;
          ar & this->exactValid_;
          ar & this->isStart_;
          ar & this->isGoal_;
        }

      private:
        std::vector<SLPRMEdge> edges_;
        double nodeCost_;
        double negativeDist_;
        tIndex CCindex_;
        tIndex preceding_;
        tIndex precedingEdge_;
        tIndex index_;
        tIndex lastInitAlg_;
        tIndex lastVisitation_;
        bool exactInvalid_;
        bool exactValid_;
        bool isStart_;
        bool isGoal_;
    };

    class SLPRMVertex
    {
      friend class SLPRMVertexMetaData;
      friend class SLPRMPlanner;
      public:
        SLPRMVertex();
        SLPRMVertex(const ompl::base::SpaceInformationPtr &si);
        SLPRMVertex(const ompl::base::SpaceInformationPtr &si, const ompl::base::State * state);
        SLPRMVertex(SLPRMVertex const& orig);
        SLPRMVertex(SLPRMVertex const* orig);
        virtual ~SLPRMVertex();

        //Handling metadata: reading vertex' member variables
        void readNonFlagsFromMetaData(SLPRMVertexMetaData const& metadata);
        void readAllFromMetaData(SLPRMVertexMetaData const& metadata);
        void readNonFlagsFromMetaData(SLPRMVertexMetaData const* metadata);
        void readAllFromMetaData(SLPRMVertexMetaData const* metadata);

        //Start/Goal flag handling
        void setIsStart(bool isStart = true);
        void setIsGoal(bool isGoal = true);
        bool getIsStart(void) const;
        bool getIsGoal(void) const;

        //Connected component index
        void setCCIndex(tIndex index);
        tIndex getCCIndex(void) const;

        //Self-index (originally used to identify start/goal vertices, and to specify preceding vertices in paths)
        void setIndex(tIndex index);
        tIndex getIndex(void) const;

        //Edge related functions
        void clearEdges(void);
        tIndex getEdgeCount(void) const;
        void popEdge(void);
        void getEdge(tIndex index, double &weight, tIndex &towards) const;
        void addEdge(double weight, tIndex towards);
        bool isInterestingEdge(tIndex index) const;

        //Collision check result flags
        bool isInteresting(void) const;
        void setExactInvalid(bool isFlagged = true);
        void clearExactInvalid();
        void setExactValid(bool isValid = true);
        void clearExactValid();
        bool isExactValid() const;
        bool isExactInvalid() const;

        //Visitation/initialization flags for graph search algorithms
        void incVisitation(void);
        void clearVisitation(void);
        void setVisitation(tIndex newVal);
        tIndex getVisitation(void) const;
        void incInitAlg(void);
        void clearInitAlg(void);
        void setInitAlg(tIndex newVal);
        tIndex getInitAlg(void) const;

        //Graph-search related auxiliaries
        void setNegativeDistance(double negativeDistance);
        double getNegativeDistance(void) const;
        void setLastInitAlg(tIndex lastInitAlg);
        tIndex getLastInitAlg(void) const;
        void setLastVisitation(tIndex lastVisitation);
        tIndex getLastVisitation(void) const;
        void setNodeCost(double nodeCost);
        void bumpCost(double val);
        void unbumpCost(double val);
        double getNodeCost(void) const;
        void setPreceding(tIndex preceding);
        tIndex getPreceding(void) const;
        void setPrecedingEdge(tIndex precedingEdge);
        tIndex getPrecedingEdge(void) const;

        //Access to state data
        ompl::base::State const* getStateData(void) const;
        void setStateData(ompl::base::State const* stateData);

        //Auxiliary functions for distance queries
        static bool SLPRMVertexCompare(SLPRMVertex const* a, SLPRMVertex const* b);

      protected:
   
      private:
        ompl::base::SpaceInformationPtr si_;
        std::vector<SLPRMEdge> edges_;
        ompl::base::State * stateData_;
        double nodeCost_;
        double negativeDist_;
        tIndex CCindex_;
        tIndex preceding_;
        tIndex precedingEdge_;
        tIndex index_;
        tIndex lastInitAlg_;
        tIndex lastVisitation_;
        bool exactInvalid_;
        bool exactValid_;
        bool isStart_;
        bool isGoal_;
    };
}
}
}

#endif //ndef __SLPRMVERTEX_H__

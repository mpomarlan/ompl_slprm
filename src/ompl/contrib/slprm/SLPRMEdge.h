#ifndef __SLPRMEDGE_H__

#define __SLPRMEDGE_H__

#include <vector>

#include <ompl/contrib/slprm/SLPRMTypes.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>


namespace ompl
{
namespace geometric
{
namespace SLPRM
{    
    class SLPRMEdge
    {
      friend class boost::serialization::access;
      friend class SLPRMPlanner;

      public:
        SLPRMEdge();
        SLPRMEdge(double weight, tIndex towards);
        SLPRMEdge(SLPRMEdge const &orig);
        SLPRMEdge(SLPRMEdge const *orig);

        virtual ~SLPRMEdge();

        void initialize(double weight, tIndex towards);
        void getEdgeData(double &weight, tIndex &towards) const;

        bool isInteresting(void) const;
        void setExactInvalid(bool isFlagged = true);
        void clearExactInvalid();
        void setExactValid(bool isValid = true);
        void clearExactValid();
        bool isExactValid() const;
        bool isExactInvalid() const;

        template<class Archive> void serialize(Archive & ar, const unsigned int version)
        {
          ar & this->weight_;
          ar & this->towards_;
          ar & this->exactInvalid_;
          ar & this->exactValid_;
        }

      private:
        tIndex towards_;
        double weight_;
        bool exactInvalid_;
        bool exactValid_;
    };
}
}
}
#endif //ndef __SLPRMEDGE_H__

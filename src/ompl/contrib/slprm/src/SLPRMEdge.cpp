#include <ompl/contrib/slprm/SLPRMEdge.h>

ompl::geometric::SLPRM::SLPRMEdge::SLPRMEdge():
  towards_(0),
  weight_(0.0),
  exactInvalid_(false),
  exactValid_(false)
{
}
ompl::geometric::SLPRM::SLPRMEdge::SLPRMEdge(double weight, ompl::geometric::SLPRM::tIndex towards):
  towards_(towards),
  weight_(weight),
  exactInvalid_(false),
  exactValid_(false)
{
}
ompl::geometric::SLPRM::SLPRMEdge::SLPRMEdge(ompl::geometric::SLPRM::SLPRMEdge const &orig):
  towards_(orig.towards_),
  weight_(orig.weight_),
  exactInvalid_(orig.exactInvalid_),
  exactValid_(orig.exactValid_)
{
}
ompl::geometric::SLPRM::SLPRMEdge::SLPRMEdge(ompl::geometric::SLPRM::SLPRMEdge const *orig):
  towards_(orig->towards_),
  weight_(orig->weight_),
  exactInvalid_(orig->exactInvalid_),
  exactValid_(orig->exactValid_)
{
}
ompl::geometric::SLPRM::SLPRMEdge::~SLPRMEdge()
{
}

void ompl::geometric::SLPRM::SLPRMEdge::initialize(double weight, ompl::geometric::SLPRM::tIndex towards)
{
  weight_ = weight;
  towards_ = towards;
  exactInvalid_ = false;
  exactValid_ = false;
}
void ompl::geometric::SLPRM::SLPRMEdge::getEdgeData(double &weight, ompl::geometric::SLPRM::tIndex &towards) const
{
  weight = weight_;
  towards = towards_;
}

bool ompl::geometric::SLPRM::SLPRMEdge::isInteresting(void) const
{
  return (false == exactInvalid_);
}
void ompl::geometric::SLPRM::SLPRMEdge::setExactInvalid(bool isFlagged)
{
  exactInvalid_ = isFlagged;
  if(exactInvalid_)
  {
    exactValid_ = false;
  }
}
void ompl::geometric::SLPRM::SLPRMEdge::clearExactInvalid()
{
  exactInvalid_ = false;
}
void ompl::geometric::SLPRM::SLPRMEdge::setExactValid(bool isValid)
{
  exactValid_ = isValid;
  if(exactValid_)
  {
    exactInvalid_ = false;
  }
}
void ompl::geometric::SLPRM::SLPRMEdge::clearExactValid()
{
  exactValid_ = false;
}
bool ompl::geometric::SLPRM::SLPRMEdge::isExactValid() const
{
  return exactValid_;
}
bool ompl::geometric::SLPRM::SLPRMEdge::isExactInvalid() const
{
  return exactInvalid_;
}


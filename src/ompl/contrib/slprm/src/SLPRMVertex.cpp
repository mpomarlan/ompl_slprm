#include <ompl/contrib/slprm/SLPRMVertex.h>
#include <ompl/contrib/slprm/SLPRMEdge.h>

ompl::geometric::SLPRM::SLPRMVertexMetaData::SLPRMVertexMetaData():
  edges_(),
  nodeCost_(0.0),
  negativeDist_(0.0),
  CCindex_(0),
  preceding_(0),
  precedingEdge_(0),
  index_(0),
  lastInitAlg_(0),
  lastVisitation_(0),
  exactInvalid_(false),
  exactValid_(false),
  isStart_(false),
  isGoal_(false)
{
}

ompl::geometric::SLPRM::SLPRMVertexMetaData::SLPRMVertexMetaData(ompl::geometric::SLPRM::SLPRMVertex const& vertex):
  edges_(vertex.edges_),
  nodeCost_(vertex.nodeCost_),
  negativeDist_(vertex.negativeDist_),
  CCindex_(vertex.CCindex_),
  preceding_(vertex.preceding_),
  precedingEdge_(vertex.precedingEdge_),
  index_(vertex.index_),
  lastInitAlg_(vertex.lastInitAlg_),
  lastVisitation_(vertex.lastVisitation_),
  exactInvalid_(vertex.exactInvalid_),
  exactValid_(vertex.exactValid_),
  isStart_(vertex.isStart_),
  isGoal_(vertex.isGoal_)
{
}
ompl::geometric::SLPRM::SLPRMVertexMetaData::SLPRMVertexMetaData(ompl::geometric::SLPRM::SLPRMVertex const* vertex):
  edges_(vertex->edges_),
  nodeCost_(vertex->nodeCost_),
  negativeDist_(vertex->negativeDist_),
  CCindex_(vertex->CCindex_),
  preceding_(vertex->preceding_),
  precedingEdge_(vertex->precedingEdge_),
  index_(vertex->index_),
  lastInitAlg_(vertex->lastInitAlg_),
  lastVisitation_(vertex->lastVisitation_),
  exactInvalid_(vertex->exactInvalid_),
  exactValid_(vertex->exactValid_),
  isStart_(vertex->isStart_),
  isGoal_(vertex->isGoal_)
{
}

ompl::geometric::SLPRM::SLPRMVertexMetaData::SLPRMVertexMetaData(ompl::geometric::SLPRM::SLPRMVertexMetaData const& vertex):
  edges_(vertex.edges_),
  nodeCost_(vertex.nodeCost_),
  negativeDist_(vertex.negativeDist_),
  CCindex_(vertex.CCindex_),
  preceding_(vertex.preceding_),
  precedingEdge_(vertex.precedingEdge_),
  index_(vertex.index_),
  lastInitAlg_(vertex.lastInitAlg_),
  lastVisitation_(vertex.lastVisitation_),
  exactInvalid_(vertex.exactInvalid_),
  exactValid_(vertex.exactValid_),
  isStart_(vertex.isStart_),
  isGoal_(vertex.isGoal_)
{
}
ompl::geometric::SLPRM::SLPRMVertexMetaData::SLPRMVertexMetaData(ompl::geometric::SLPRM::SLPRMVertexMetaData const* vertex):
  edges_(vertex->edges_),
  nodeCost_(vertex->nodeCost_),
  negativeDist_(vertex->negativeDist_),
  CCindex_(vertex->CCindex_),
  preceding_(vertex->preceding_),
  precedingEdge_(vertex->precedingEdge_),
  index_(vertex->index_),
  lastInitAlg_(vertex->lastInitAlg_),
  lastVisitation_(vertex->lastVisitation_),
  exactInvalid_(vertex->exactInvalid_),
  exactValid_(vertex->exactValid_),
  isStart_(vertex->isStart_),
  isGoal_(vertex->isGoal_)
{
}

ompl::geometric::SLPRM::SLPRMVertexMetaData::~SLPRMVertexMetaData()
{
}

ompl::geometric::SLPRM::SLPRMVertex::SLPRMVertex():
  si_(),
  edges_(),
  stateData_(NULL),
  nodeCost_(0.0),
  negativeDist_(-1e9),
  CCindex_(0),
  preceding_(0),
  precedingEdge_(0),
  index_(0),
  lastInitAlg_(0),
  lastVisitation_(0),
  exactInvalid_(false),
  exactValid_(false),
  isStart_(false),
  isGoal_(false)
{
}
ompl::geometric::SLPRM::SLPRMVertex::SLPRMVertex(const ompl::base::SpaceInformationPtr &si):
  si_(new ompl::base::SpaceInformation(si->getStateSpace())),
  edges_(),
  stateData_(NULL),
  nodeCost_(0.0),
  negativeDist_(-1e9),
  CCindex_(0),
  preceding_(0),
  precedingEdge_(0),
  index_(0),
  lastInitAlg_(0),
  lastVisitation_(0),
  exactInvalid_(false),
  exactValid_(false),
  isStart_(false),
  isGoal_(false)
{
}
ompl::geometric::SLPRM::SLPRMVertex::SLPRMVertex(const ompl::base::SpaceInformationPtr &si, const ompl::base::State * state):
  si_(new ompl::base::SpaceInformation(si->getStateSpace())),
  edges_(),
  stateData_(NULL),
  nodeCost_(0.0),
  negativeDist_(-1e9),
  CCindex_(0),
  preceding_(0),
  precedingEdge_(0),
  index_(0),
  lastInitAlg_(0),
  lastVisitation_(0),
  exactInvalid_(false),
  exactValid_(false),
  isStart_(false),
  isGoal_(false)
{
  stateData_ = si_->cloneState(state);
}
ompl::geometric::SLPRM::SLPRMVertex::SLPRMVertex(ompl::geometric::SLPRM::SLPRMVertex const& orig):
  si_(new ompl::base::SpaceInformation(orig.si_->getStateSpace())),
  edges_(orig.edges_),
  stateData_(NULL),
  nodeCost_(orig.nodeCost_),
  negativeDist_(orig.negativeDist_),
  CCindex_(orig.CCindex_),
  preceding_(orig.preceding_),
  precedingEdge_(orig.precedingEdge_),
  index_(orig.index_),
  lastInitAlg_(orig.lastInitAlg_),
  lastVisitation_(orig.lastVisitation_),
  exactInvalid_(orig.exactInvalid_),
  exactValid_(orig.exactValid_),
  isStart_(orig.isStart_),
  isGoal_(orig.isGoal_)
{
  stateData_ = si_->cloneState(orig.stateData_);
}
ompl::geometric::SLPRM::SLPRMVertex::SLPRMVertex(ompl::geometric::SLPRM::SLPRMVertex const* orig):
  si_(new ompl::base::SpaceInformation(orig->si_->getStateSpace())),
  edges_(orig->edges_),
  stateData_(orig->stateData_),
  nodeCost_(orig->nodeCost_),
  negativeDist_(orig->negativeDist_),
  CCindex_(orig->CCindex_),
  preceding_(orig->preceding_),
  precedingEdge_(orig->precedingEdge_),
  index_(orig->index_),
  lastInitAlg_(orig->lastInitAlg_),
  lastVisitation_(orig->lastVisitation_),
  exactInvalid_(orig->exactInvalid_),
  exactValid_(orig->exactValid_),
  isStart_(orig->isStart_),
  isGoal_(orig->isGoal_)
{
  stateData_ = si_->cloneState(orig->stateData_);
}

ompl::geometric::SLPRM::SLPRMVertex::~SLPRMVertex()
{
  if(stateData_)
  {
    si_->freeState(stateData_);
    stateData_ = NULL;
  }
}

void ompl::geometric::SLPRM::SLPRMVertex::readNonFlagsFromMetaData(ompl::geometric::SLPRM::SLPRMVertexMetaData const& metadata)
{
  edges_.clear();
  edges_ = metadata.edges_;
  nodeCost_ = metadata.nodeCost_;
  CCindex_ = metadata.CCindex_;
  index_ = metadata.index_;
  negativeDist_ = -1e9;
  preceding_ = 0;
  precedingEdge_ = 0;
  lastInitAlg_ = 0;
  lastVisitation_ = 0;
  exactInvalid_ = false;
  exactValid_ = false;
  isStart_ = false;
  isGoal_ = false;
}
void ompl::geometric::SLPRM::SLPRMVertex::readAllFromMetaData(ompl::geometric::SLPRM::SLPRMVertexMetaData const& metadata)
{
  readNonFlagsFromMetaData(metadata);
  negativeDist_ = metadata.negativeDist_;
  preceding_ = metadata.preceding_;
  precedingEdge_ = metadata.precedingEdge_;
  lastInitAlg_ = metadata.lastInitAlg_;
  lastVisitation_ = metadata.lastVisitation_;
  exactInvalid_ = metadata.exactInvalid_;
  exactValid_ = metadata.exactValid_;
  isStart_ = metadata.isStart_;
  isGoal_ = metadata.isGoal_;
}
void ompl::geometric::SLPRM::SLPRMVertex::readNonFlagsFromMetaData(ompl::geometric::SLPRM::SLPRMVertexMetaData const* metadata)
{
  readNonFlagsFromMetaData(*metadata);
}
void ompl::geometric::SLPRM::SLPRMVertex::readAllFromMetaData(ompl::geometric::SLPRM::SLPRMVertexMetaData const* metadata)
{
  readAllFromMetaData(*metadata);
}

void ompl::geometric::SLPRM::SLPRMVertex::setIsStart(bool isStart)
{
  isStart_ = isStart;
}
void ompl::geometric::SLPRM::SLPRMVertex::setIsGoal(bool isGoal)
{
  isGoal_ = isGoal;
}
bool ompl::geometric::SLPRM::SLPRMVertex::getIsStart(void) const
{
  return isStart_;
}
bool ompl::geometric::SLPRM::SLPRMVertex::getIsGoal(void) const
{
  return isGoal_;
}

void ompl::geometric::SLPRM::SLPRMVertex::setIndex(ompl::geometric::SLPRM::tIndex index)
{
  index_ = index;
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getIndex(void) const
{
  return index_;
}

void ompl::geometric::SLPRM::SLPRMVertex::setCCIndex(ompl::geometric::SLPRM::tIndex index)
{
  CCindex_ = index;
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getCCIndex(void) const
{
  return CCindex_;
}

void ompl::geometric::SLPRM::SLPRMVertex::clearEdges(void)
{
  edges_.clear();
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getEdgeCount(void) const
{
  return edges_.size();
}
void ompl::geometric::SLPRM::SLPRMVertex::popEdge(void)
{
  edges_.pop_back();
}
void ompl::geometric::SLPRM::SLPRMVertex::getEdge(ompl::geometric::SLPRM::tIndex index, double &weight, ompl::geometric::SLPRM::tIndex &towards) const
{
  edges_[index].getEdgeData(weight, towards);
}
void ompl::geometric::SLPRM::SLPRMVertex::addEdge(double weight, ompl::geometric::SLPRM::tIndex towards)
{
  edges_.push_back(SLPRMEdge(weight, towards));
}

bool ompl::geometric::SLPRM::SLPRMVertex::isInterestingEdge(tIndex index) const
{
  return edges_[index].isInteresting();
}

bool ompl::geometric::SLPRM::SLPRMVertex::isInteresting(void) const
{
  return (false == exactInvalid_);
}
void ompl::geometric::SLPRM::SLPRMVertex::setExactInvalid(bool isFlagged)
{
  exactInvalid_ = isFlagged;
  if(isFlagged)
  {
    setExactValid(false);
  }
}
void ompl::geometric::SLPRM::SLPRMVertex::clearExactInvalid()
{
  exactInvalid_ = false;
}
void ompl::geometric::SLPRM::SLPRMVertex::setExactValid(bool isValid)
{
  exactValid_ = isValid;
  if(isValid)
  {
    setExactInvalid(false);
  }
}
void ompl::geometric::SLPRM::SLPRMVertex::clearExactValid()
{
  exactValid_ = false;
}
bool ompl::geometric::SLPRM::SLPRMVertex::isExactValid() const
{
  return exactValid_;
}
bool ompl::geometric::SLPRM::SLPRMVertex::isExactInvalid() const
{
  return exactInvalid_;
}

void ompl::geometric::SLPRM::SLPRMVertex::incVisitation(void)
{
  ++lastVisitation_;
}
void ompl::geometric::SLPRM::SLPRMVertex::clearVisitation(void)
{
  lastVisitation_ = 0;
}
void ompl::geometric::SLPRM::SLPRMVertex::setVisitation(ompl::geometric::SLPRM::tIndex newVal)
{
  lastVisitation_ = newVal;
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getVisitation(void) const
{
  return lastVisitation_;
}
void ompl::geometric::SLPRM::SLPRMVertex::incInitAlg(void)
{
  ++lastInitAlg_;
}
void ompl::geometric::SLPRM::SLPRMVertex::clearInitAlg(void)
{
  lastInitAlg_ = 0;
}
void ompl::geometric::SLPRM::SLPRMVertex::setInitAlg(ompl::geometric::SLPRM::tIndex newVal)
{
  lastInitAlg_ = newVal;
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getInitAlg(void) const
{
  return lastInitAlg_;
}

ompl::base::State const* ompl::geometric::SLPRM::SLPRMVertex::getStateData(void) const
{
  return stateData_;
}
void ompl::geometric::SLPRM::SLPRMVertex::setStateData(ompl::base::State const* stateData)
{
  if(NULL == stateData_)
  {
    stateData_ = si_->cloneState(stateData);
  }
  else
  {
    si_->copyState(stateData_, stateData);
  }
}

bool ompl::geometric::SLPRM::SLPRMVertex::SLPRMVertexCompare(ompl::geometric::SLPRM::SLPRMVertex const* a, ompl::geometric::SLPRM::SLPRMVertex const* b)
{
  if((a->lastInitAlg_ < b->lastInitAlg_) || ((a->lastInitAlg_ == b->lastInitAlg_) && (a->negativeDist_ < b->negativeDist_)))
  {
    return true;
  }
  return false;
}

void ompl::geometric::SLPRM::SLPRMVertex::setNegativeDistance(double negativeDistance)
{
  negativeDist_ = negativeDistance;
}
double ompl::geometric::SLPRM::SLPRMVertex::getNegativeDistance(void) const
{
  return negativeDist_;
}
void ompl::geometric::SLPRM::SLPRMVertex::setLastInitAlg(ompl::geometric::SLPRM::tIndex lastInitAlg)
{
  lastInitAlg_ = lastInitAlg;
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getLastInitAlg(void) const
{
  return lastInitAlg_;
}
void ompl::geometric::SLPRM::SLPRMVertex::setLastVisitation(ompl::geometric::SLPRM::tIndex lastVisitation)
{
  lastVisitation_ = lastVisitation;
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getLastVisitation(void) const
{
  return lastVisitation_;
}
void ompl::geometric::SLPRM::SLPRMVertex::setNodeCost(double nodeCost)
{
  nodeCost_ = nodeCost;
}
void ompl::geometric::SLPRM::SLPRMVertex::bumpCost(double val)
{
  val = (0.0 < val)?(val):(0.0);
  nodeCost_ += val;
}
void ompl::geometric::SLPRM::SLPRMVertex::unbumpCost(double val)
{
  val = (0.0 < val)?(val):(0.0);
  nodeCost_ -= val;
  nodeCost_ = (0.0 < nodeCost_)?(nodeCost_):(0.0);
}

double ompl::geometric::SLPRM::SLPRMVertex::getNodeCost(void) const
{
  return nodeCost_;
}
void ompl::geometric::SLPRM::SLPRMVertex::setPreceding(ompl::geometric::SLPRM::tIndex preceding)
{
  preceding_ = preceding;
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getPreceding(void) const
{
  return preceding_;
}
void ompl::geometric::SLPRM::SLPRMVertex::setPrecedingEdge(ompl::geometric::SLPRM::tIndex precedingEdge)
{
  precedingEdge_ = precedingEdge;
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMVertex::getPrecedingEdge(void) const
{
  return precedingEdge_;
}


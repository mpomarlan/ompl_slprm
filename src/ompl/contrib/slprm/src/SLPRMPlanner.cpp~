#include <ompl/contrib/slprm/SLPRMPlanner.h>
#include <ompl/tools/debug/Profiler.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <cmath>


namespace ompl
{
  namespace geometric
  {
    namespace SLPRM
    {
      typedef ompl::base::StateStorageWithMetadata<ompl::geometric::SLPRM::SLPRMVertexMetaData> VertexQuickStorage;
    }
  }
}

ompl::geometric::SLPRM::SLPRMPlanner::SLPRMPlanner(const ompl::base::SpaceInformationPtr &si, const std::string &name):
  ompl::base::Planner(si, name),
  validSampler_(),
  CCs_(),
  nearFinder_(4, 2, 6, 50, 50, true),
  vertices_(),
  workingVertex_(si),
  crVisitation_(0),
  exactVertexChecks_(),
  exactEdgeChecks_(),
  bumpRadius_(0.0),
  unbumpRadius_(0.0),
  bumpValue_(0.0),
  unbumpValue_(0.0),
  roadmapPath_(""),
  roadmapLoaded_(false),
  trustRoadmap_(true),
  allowRoadmapExtension_(true)
{
  nearFinder_.setDistanceFunction(boost::bind(SLPRMPlanner::distance, this, _1, _2));
  workingVertex_.setStateData(si_->allocState());

  validSampler_ = si_->allocValidStateSampler();

  bumpValue_ = si_->getMaximumExtent();
  unbumpValue_ = 0.2*bumpValue_;
  bumpRadius_ = 0.05*bumpValue_;
  unbumpRadius_ = 0.1*bumpValue_;

  specs_.approximateSolutions = false;
  specs_.multithreaded = false;
  specs_.optimizingPaths = false;
  specs_.directed = false;
  specs_.provingSolutionNonExistence = false;
  specs_.recognizedGoal = (ompl::base::GOAL_ANY);
}
ompl::geometric::SLPRM::SLPRMPlanner::SLPRMPlanner(const ompl::base::SpaceInformationPtr &si):
  ompl::base::Planner(si, "SLPRM"),
  validSampler_(),
  CCs_(),
  nearFinder_(4, 2, 6, 50, 50, true),
  vertices_(),
  workingVertex_(si),
  crVisitation_(0),
  exactVertexChecks_(),
  exactEdgeChecks_(),
  bumpRadius_(0.0),
  unbumpRadius_(0.0),
  bumpValue_(0.0),
  unbumpValue_(0.0),
  roadmapPath_(""),
  roadmapLoaded_(false),
  trustRoadmap_(true),
  allowRoadmapExtension_(true)
{
  nearFinder_.setDistanceFunction(boost::bind(SLPRMPlanner::distance, this, _1, _2));
  workingVertex_.setStateData(si_->allocState());

  validSampler_ = si_->allocValidStateSampler();

  bumpValue_ = si_->getMaximumExtent();
  unbumpValue_ = 0.2*bumpValue_;
  bumpRadius_ = 0.05*bumpValue_;
  unbumpRadius_ = 0.1*bumpValue_;

  specs_.approximateSolutions = false;
  specs_.multithreaded = false;
  specs_.optimizingPaths = false;
  specs_.directed = false;
  specs_.provingSolutionNonExistence = false;
  specs_.recognizedGoal = (ompl::base::GOAL_ANY);
}
ompl::geometric::SLPRM::SLPRMPlanner::~SLPRMPlanner()
{
}

void ompl::geometric::SLPRM::SLPRMPlanner::setSI(const ompl::base::SpaceInformationPtr &si)
{
  si_.reset(si.get());
}

void ompl::geometric::SLPRM::SLPRMPlanner::resetVisitation(void)
{
  crVisitation_ = 0;
  tIndex kMax = vertices_.size();
  for(tIndex k = 0; k < kMax; k++)
  {
    vertices_[k].clearVisitation();
    vertices_[k].clearInitAlg();
  }
}

double ompl::geometric::SLPRM::SLPRMPlanner::distance(ompl::geometric::SLPRM::SLPRMPlanner const* planner, ompl::geometric::SLPRM::tIndex a, ompl::geometric::SLPRM::tIndex b)
{
  return planner->si_->distance(planner->vertices_[a].getStateData(), planner->vertices_[b].getStateData());
}


ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMPlanner::getValidNeighborhood(ompl::geometric::SLPRM::tIndex index, std::vector<ompl::geometric::SLPRM::tIndex> &neighbors) const
{
  for(std::vector<tIndex>::iterator iter = neighbors.begin(); iter != neighbors.end(); )
  {
    if((si_->checkMotion(vertices_[index].getStateData(), vertices_[(*iter)].getStateData())))
    {
      iter++;
    }
    else
    {
      iter = neighbors.erase(iter);
    }
  }
  return neighbors.size();
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMPlanner::getValidKNeighborhood(ompl::geometric::SLPRM::tIndex index, ompl::geometric::SLPRM::tIndex k, std::vector<ompl::geometric::SLPRM::tIndex> &neighbors) const
{
  neighbors.clear();
  nearFinder_.nearestK(index, k, neighbors);
  return getValidNeighborhood(index, neighbors);
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMPlanner::getValidRNeighborhood(ompl::geometric::SLPRM::tIndex index, double r, std::vector<ompl::geometric::SLPRM::tIndex> &neighbors) const
{
  neighbors.clear();
  nearFinder_.nearestR(index, r, neighbors);
  return getValidNeighborhood(index, neighbors);
}

ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMPlanner::getKNeighborhood(ompl::geometric::SLPRM::tIndex index, ompl::geometric::SLPRM::tIndex k, std::vector<ompl::geometric::SLPRM::tIndex> &neighbors) const
{
  neighbors.clear();
  nearFinder_.nearestK(index, k, neighbors);
  return neighbors.size();
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMPlanner::getRNeighborhood(ompl::geometric::SLPRM::tIndex index, double r, std::vector<ompl::geometric::SLPRM::tIndex> &neighbors) const
{
  neighbors.clear();
  nearFinder_.nearestR(index, r, neighbors);
  return neighbors.size();
}

ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMPlanner::getNeighborCCs(std::vector<ompl::geometric::SLPRM::tIndex> const &neighbors, std::set<ompl::geometric::SLPRM::tIndex> &neighborCCs) const
{
  neighborCCs.clear();
  ompl::geometric::SLPRM::tIndex kMax = neighbors.size();
  for(ompl::geometric::SLPRM::tIndex k = 0; k < kMax; k++)
  {
    neighborCCs.insert(CCs_.find(vertices_[neighbors[k]].getCCIndex()));
  }
  return neighborCCs.size();
}
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::SLPRMPlanner::getCommonCCs(std::set<ompl::geometric::SLPRM::tIndex> const& neighborACCs, std::set<ompl::geometric::SLPRM::tIndex> const& neighborBCCs, std::set<ompl::geometric::SLPRM::tIndex> & commonCCs) const
{
  commonCCs.clear();
  for(std::set<tIndex>::const_iterator iter = neighborACCs.begin(); iter != neighborACCs.end(); iter++)
  {
    if(neighborBCCs.count((*iter)))
    {
      commonCCs.insert((*iter));
    }
  }
  return commonCCs.size();
}

void ompl::geometric::SLPRM::SLPRMPlanner::addEdges(ompl::geometric::SLPRM::tIndex index, std::vector<ompl::geometric::SLPRM::tIndex> const& neighbors, bool i2n, bool n2i)
{
  //Only merge CCs if adding bidirectional edges (index <-> neighbor)
  bool trackCCs = (i2n && n2i);
  tIndex maxK = neighbors.size();
  for(tIndex k = 0; k < maxK; k++)
  {
    double weight = si_->distance(vertices_[index].getStateData(), vertices_[neighbors[k]].getStateData());
    if(i2n)
    {
      vertices_[index].addEdge(weight, neighbors[k]);
    }
    if(n2i)
    {
      vertices_[neighbors[k]].addEdge(weight, index);
    }
    if(trackCCs)
    {
      tIndex CCA, CCB, CCR;
      CCA = CCs_.find(vertices_[index].getCCIndex());
      CCB = CCs_.find(vertices_[neighbors[k]].getCCIndex());
      //CC index 0 is reserved. May be used in the future as temp vertex marker, for example.
      if(CCA && CCB)
      {
        CCs_.setUnion(CCA, CCB);
        CCR = CCs_.find(CCA);
        vertices_[index].setCCIndex(CCR);
        vertices_[neighbors[k]].setCCIndex(CCR);
      }
    }
  }
}

bool ompl::geometric::SLPRM::SLPRMPlanner::areInSameCC(ompl::geometric::SLPRM::tIndex A, ompl::geometric::SLPRM::tIndex B) const
{
  tIndex CCA, CCB;
  CCA = CCs_.find(vertices_[A].getCCIndex());
  CCB = CCs_.find(vertices_[B].getCCIndex());
  return (CCA == CCB);
}

void ompl::geometric::SLPRM::SLPRMPlanner::addVertex(const ompl::base::State *newState)
{

std::cout << "SLPRMPlanner::addVertex" << std::endl;

  tIndex newIndex = vertices_.size();

std::cout << "SLPRMPlanner::addVertex - new vertex" << std::endl;

  SLPRMVertex dummy(si_, newState);

std::cout << "SLPRMPlanner::addVertex - pushing into containers" << std::endl;

  vertices_.push_back(dummy);
  vertices_[newIndex].setIndex(newIndex);
  vertices_[newIndex].setCCIndex(CCs_.makeNewComponent());
  nearFinder_.add(newIndex);

std::cout << "SLPRMPlanner::addVertex - add edges" << std::endl;

  if(newIndex)
  {
    std::vector<tIndex> neighbors;

std::cout << "SLPRMPlanner::addVertex - get valid neighborhood" << std::endl;

    getValidKNeighborhood(newIndex, 15, neighbors);

    if(!neighbors.size())
    {
      getValidKNeighborhood(newIndex, newIndex, neighbors);
    }

std::cout << "SLPRMPlanner::addVertex - add edges: " << neighbors.size() << std::endl;

    addEdges(newIndex, neighbors, true, true);
  }

std::cout << "SLPRMPlanner::addVertex - done" << std::endl;

}


void ompl::geometric::SLPRM::SLPRMPlanner::sampleAndConnect(const ompl::base::PlannerTerminationCondition &ptc)
{
  assert(allowRoadmapExtension_);
  bool extended = false;
  while((!extended) && (false == ptc()))
  {
    ompl::base::State* newState = si_->allocState();
ompl::tools::Profiler::Begin("sampling");
    while((!validSampler_->sample(newState)) && (false == ptc()))
    {
    }
ompl::tools::Profiler::End("sampling");
    if(false == ptc())
    {
      //Valid state found. Create a temp vertex for it
      tIndex newIndex = vertices_.size();
      SLPRMVertex dummy(si_, newState);
      vertices_.push_back(dummy);
      //Find neighbors and their CCs
      double r = si_->getMaximumExtent()/std::pow(1.0 + vertices_.size(), 1.0/(1.0*si_->getStateDimension()));
      std::vector<tIndex> neighbors;
      getValidRNeighborhood(newIndex, r, neighbors);
      std::set<tIndex> neighborCCs;
      getNeighborCCs(neighbors, neighborCCs);
      if(0 == neighbors.size())
      {
ompl::tools::Profiler::Event("new CC");
        extended = true;
        //No connection to vertices in the roadmap: node improves coverage. Add to roadmap.
        vertices_[newIndex].setIndex(newIndex);
        nearFinder_.add(newIndex);
        vertices_[newIndex].setCCIndex(CCs_.makeNewComponent());
      }
      else if((1 == neighbors.size()) || (1 < neighborCCs.size()))
      {
ompl::tools::Profiler::Event("new Connection");
        extended = true;
        //Either extends in a hard to reach area or improves connectivity. Add to roadmap.
        nearFinder_.add(newIndex);
        vertices_[newIndex].setCCIndex(CCs_.makeNewComponent());
        vertices_[newIndex].setIndex(newIndex);
        addEdges(newIndex, neighbors, true, true);
      }
      //TODO: add useful loop/path shortening heuristic
      else
      {
        //Remove vertex from the roadmap, it does not appear useful.
ompl::tools::Profiler::Event("rejection");
        vertices_.pop_back();
      }
    }
    si_->freeState(newState);
  }
}

bool ompl::geometric::SLPRM::SLPRMPlanner::DijkstraSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::SLPRM::WorkingVertexVector &pathCandidate)
{
  ++crVisitation_;
  WorkingVertexVector pqueue;
  bool reachedGoal = false;

  tIndex goalIndexReached = 0;
  pathCandidate.clear();
  pqueue.clear();
  tIndex maxK = vertices_.size();
  for(tIndex k = 0; k < maxK; k++)
  {
    if(vertices_[k].getIsStart())
    {
      vertices_[k].setNegativeDistance(0);
      vertices_[k].setLastInitAlg(crVisitation_);
      pqueue.push_back(&vertices_[k]);
    }
  }
  make_heap(pqueue.begin(), pqueue.end(), SLPRMVertex::SLPRMVertexCompare);

  while(pqueue.size() && (ptc == false))
  {
    SLPRMVertex *cVertex(pqueue.front());
    pop_heap(pqueue.begin(), pqueue.end(), SLPRMVertex::SLPRMVertexCompare); pqueue.pop_back();

    if(cVertex->getLastVisitation() < crVisitation_)
    {
      cVertex->setLastVisitation(crVisitation_);
      if(cVertex->getIsGoal())
      {
        reachedGoal = true;
        goalIndexReached = cVertex->getIndex();
        break;
      }
      tIndex kMax = cVertex->getEdgeCount();
      for(tIndex k = 0; k < kMax; k++)
      {
        double weight;
        tIndex towards;
        cVertex->getEdge(k, weight, towards);
        SLPRMVertex *cTarget(&vertices_[towards]);
        if(cTarget->isInteresting() && cVertex->isInterestingEdge(k))
        {
          //STL heap is a max heap, so we use negative edge weights to 'convert' it to a min-heap
          if(cTarget->getLastInitAlg() < crVisitation_)
          {
            cTarget->setLastInitAlg(crVisitation_);
            cTarget->setNegativeDistance(cVertex->getNegativeDistance() - weight - cTarget->getNodeCost());
            cTarget->setPreceding(cVertex->getIndex());
            cTarget->setPrecedingEdge(k);
            pqueue.push_back(cTarget); push_heap(pqueue.begin(), pqueue.end(), SLPRMVertex::SLPRMVertexCompare);
          }
          else
          {
            double negativeDist = cVertex->getNegativeDistance() - weight - cTarget->getNodeCost();
            if(negativeDist > cTarget->getNegativeDistance())
            {
              cTarget->setNegativeDistance(negativeDist);
              cTarget->setPreceding(cVertex->getIndex());
              cTarget->setPrecedingEdge(k);
              pqueue.push_back(cTarget); push_heap(pqueue.begin(), pqueue.end(), SLPRMVertex::SLPRMVertexCompare);
            }
          }
        }
      }
    }
  }

  if(reachedGoal)
  {
    tIndex cIndex = goalIndexReached;
    while(!vertices_[cIndex].getIsStart())
    {
      pathCandidate.push_back(&vertices_[cIndex]);
      cIndex = vertices_[cIndex].getPreceding();
    }
    pathCandidate.push_back(&vertices_[cIndex]);
  }
  return reachedGoal;
}
bool ompl::geometric::SLPRM::SLPRMPlanner::AStarSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::SLPRM::WorkingVertexVector &pathCandidate)
{
  //TODO: implement
  return false;
}

bool ompl::geometric::SLPRM::SLPRMPlanner::checkPathCandidate(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::SLPRM::WorkingVertexVector &pathCandidate)
{
ompl::tools::Profiler::Begin("LazyCheck");
  unsigned int kMax = pathCandidate.size();
  bool allVerticesValid = true;
  bool allEdgesValid = true;

  for(unsigned int k = 0; (k < kMax) && (false == ptc) && (allVerticesValid); k++)
  {
    unsigned int adjK = kMax - k - 1;
    if((!pathCandidate[adjK]->isExactValid()) && (!pathCandidate[adjK]->isExactInvalid()))
    {
ompl::tools::Profiler::Event("VertexCheck");
      exactVertexChecks_.push_back(pathCandidate[adjK]);
      if(!si_->isValid(pathCandidate[adjK]->getStateData()))
      {
        pathCandidate[adjK]->setExactInvalid(true);
        allVerticesValid = false;
ompl::tools::Profiler::Begin("VertexBump");
        costBump(pathCandidate[adjK]->getIndex());
ompl::tools::Profiler::End("VertexBump");
      }
      else
      {
ompl::tools::Profiler::Event("VertexValid");
        pathCandidate[adjK]->setExactValid(true);
ompl::tools::Profiler::Begin("VertexUnbump");
        costUnbump(pathCandidate[adjK]->getIndex());
ompl::tools::Profiler::End("VertexUnbump");
      }
    }
  }

  if((false != ptc) || (!allVerticesValid))
  {
    //force quick termination (avoid more collision checks)
ompl::tools::Profiler::End("LazyCheck");
    return false;
  }
  //only if all vertices are valid, check the edges (an edge with one adjacent invalid vertex is never used)
  if((1 < kMax))
  {
    for(unsigned int k = 0; (k < kMax - 1) && (ptc == false) && (allEdgesValid); k++)
    {
      unsigned int adjK = kMax - k - 2;
      if((!vertices_[pathCandidate[adjK]->getPreceding()].edges_[pathCandidate[adjK]->getPrecedingEdge()].isExactValid()) && (!vertices_[pathCandidate[adjK]->getPreceding()].edges_[pathCandidate[adjK]->getPrecedingEdge()].isExactInvalid()))
      {
        exactEdgeChecks_.push_back(&(vertices_[pathCandidate[adjK]->getPreceding()].edges_[pathCandidate[adjK]->getPrecedingEdge()]));
ompl::tools::Profiler::Event("EdgeCheck");
        std::pair< ompl::base::State *, double > dummyPair;
        dummyPair.first = workingVertex_.stateData_;
        if(!si_->checkMotion(vertices_[pathCandidate[adjK]->getPreceding()].getStateData(), pathCandidate[adjK]->getStateData(), dummyPair))
        {
          vertices_[pathCandidate[adjK]->preceding_].edges_[pathCandidate[adjK]->precedingEdge_].setExactInvalid(true);
          allEdgesValid = false;
ompl::tools::Profiler::Begin("EdgeBump");
          costBump(vertices_.size());
ompl::tools::Profiler::End("EdgeBump");
        }
        else
        {
ompl::tools::Profiler::Event("EdgeValid");
          vertices_[pathCandidate[adjK]->getPreceding()].edges_[pathCandidate[adjK]->getPrecedingEdge()].setExactValid(true);
        }
      }
    }
  }

  if((false != ptc))
  {
    //ptc forced termination, motion check not completed, cannot trust path
    allEdgesValid = false;
  }
ompl::tools::Profiler::End("LazyCheck");
  return allEdgesValid;
}

bool ompl::geometric::SLPRM::SLPRMPlanner::graphSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::PathGeometric &solution, bool useAStar)
{
  //update the value we use to check that a vertex has been initialized, or visited, in the current search
  ++crVisitation_;
  WorkingVertexVector pqueue;
  WorkingVertexVector pathCandidate;
  bool reachedGoal = false;
  bool goalDisconnected = false;

  tIndex goalIndexReached = 0;

  while((false == ptc) && (!reachedGoal) && (!goalDisconnected))
  {
    reachedGoal = (useAStar)?(AStarSearch(ptc, pathCandidate)):(DijkstraSearch(ptc, pathCandidate));
    if(!reachedGoal)
    {
      goalDisconnected = true;
    }
    if((!trustRoadmap_) && (reachedGoal))
    {
      reachedGoal = checkPathCandidate(ptc, pathCandidate);
    }
  }

  if(reachedGoal)
  {
    tIndex kMax = pathCandidate.size();
    for(tIndex k = 0, kPrev = 0; (k < kMax); k++)
    {
      if(0 < k)
      {
        if(0.000001 < si_->distance(pathCandidate[kMax - 1 - k]->getStateData(), pathCandidate[kMax - 1 - kPrev]->getStateData()))
        {
          solution.append(pathCandidate[kMax - 1 - k]->getStateData());
          kPrev = k;
        }
      }
      else
      {
        solution.append(pathCandidate[kMax - 1 - k]->getStateData());
      }
    }
  }
  clearScratch();
  return reachedGoal;
}

void ompl::geometric::SLPRM::SLPRMPlanner::loadVertices(void)
{
  CCs_.reset();
  nearFinder_.clear();
  vertices_.clear();
  exactVertexChecks_.clear();
  exactEdgeChecks_.clear();
  roadmapLoaded_ = false;

  if("" == roadmapPath_)
  {
    return;
  }
  ompl::base::StateStorage storage(si_->getStateSpace());
  storage.load(roadmapPath_.c_str());
  tIndex maxK = storage.size();
  for(tIndex k = 0; k < maxK; k++)
  {
    if(si_->isValid(storage.getState(k)))
      addVertex(storage.getState(k));
  }
  for(tIndex k = 0; k < vertices_.size(); k++)
  {
    vertices_[k].setCCIndex(CCs_.find(vertices_[k].getCCIndex()));
  }
  roadmapLoaded_ = true;
}

void ompl::geometric::SLPRM::SLPRMPlanner::storeVertices(void) const
{
  if("" == roadmapPath_)
  {
    return;
  }
  ompl::base::StateStorage storage(si_->getStateSpace());
  tIndex maxK = vertices_.size();
  for(tIndex k = 0; k < maxK; k++)
  {
    storage.addState(vertices_[k].getStateData());
  }
  storage.store(roadmapPath_.c_str());
}

void ompl::geometric::SLPRM::SLPRMPlanner::storeVerticesWithMetaData(void) const
{
  if("" == roadmapPath_)
  {
    return;
  }
  std::ofstream roadmapStream(roadmapPath_.c_str(), std::ios::binary);
  VertexQuickStorage quickStorage(si_->getStateSpace());
  tIndex maxK = vertices_.size();
  for(tIndex k = 0; k < maxK; k++)
  {
    quickStorage.addState(vertices_[k].getStateData(), SLPRMVertexMetaData(vertices_[k]));
  }
  quickStorage.store(roadmapStream);
  boost::archive::binary_oarchive oa(roadmapStream);
  oa << CCs_;
  roadmapStream.close();
}
void ompl::geometric::SLPRM::SLPRMPlanner::loadVerticesWithMetaData(void)
{

std::cout << "SLPRMPlanner::loadVerticesWithMetaData" << std::endl;

  CCs_.reset();
  nearFinder_.clear();
  vertices_.clear();
  exactVertexChecks_.clear();
  exactEdgeChecks_.clear();
  roadmapLoaded_ = false;

  if("" == roadmapPath_)
  {

std::cout << "SLPRMPlanner::loadVerticesWithMetaData - empty path, nothing to load " << std::endl;

    return;
  }
  std::ifstream roadmapStream(roadmapPath_.c_str(), std::ios::binary);
  VertexQuickStorage quickStorage(si_->getStateSpace());

std::cout << "SLPRMPlanner::loadVerticesWithMetaData - load vertices from quickstorage ... " << std::endl;

  quickStorage.load(roadmapStream);

std::cout << "SLPRMPlanner::loadVerticesWithMetaData - quickstorage vertex load done " << std::endl;

  tIndex maxK = quickStorage.size();
  vertices_.clear();
  nearFinder_.clear();
  for(tIndex k = 0; k < maxK; k++)
  {
    SLPRMVertex dummy(si_, quickStorage.getState(k));
    dummy.readNonFlagsFromMetaData(quickStorage.getMetadata(k));
    vertices_.push_back(dummy);
    nearFinder_.add(k);
  }

std::cout << "SLPRMPlanner::loadVerticesWithMetaData - load CCs from quickstorage ... " << std::endl;

  boost::archive::binary_iarchive ia(roadmapStream);
  ia >> CCs_;
  for(tIndex k = 0; k < maxK; k++)
  {
    vertices_[k].setCCIndex(CCs_.find(vertices_[k].getCCIndex()));
  }
  roadmapStream.close();
  roadmapLoaded_ = true;

std::cout << "SLPRMPlanner::loadVerticesWithMetaData - finished " << std::endl;
}

void ompl::geometric::SLPRM::SLPRMPlanner::setRoadmapPath(std::string const& path)
{
  roadmapPath_ = path;
}
void ompl::geometric::SLPRM::SLPRMPlanner::getRoadmapPath(std::string &path) const
{
  path = roadmapPath_;
}
void ompl::geometric::SLPRM::SLPRMPlanner::setTrustRoadmapFlag(bool trustRoadmap)
{
  trustRoadmap_ = trustRoadmap;
  //Currently, the two flags (trustRoadmap_ and allowRoadmapExtension_) are locked together
  allowRoadmapExtension_ = trustRoadmap_;
}
bool ompl::geometric::SLPRM::SLPRMPlanner::getTrustRoadmapFlag(void) const
{
  return trustRoadmap_;
}

double ompl::geometric::SLPRM::SLPRMPlanner::getBumpRadius(void) const
{
  return bumpRadius_;
}
void ompl::geometric::SLPRM::SLPRMPlanner::setBumpRadius(double radius)
{
  bumpRadius_ = radius;
}
double ompl::geometric::SLPRM::SLPRMPlanner::getUnbumpRadius(void) const
{
  return unbumpRadius_;
}
void ompl::geometric::SLPRM::SLPRMPlanner::setUnbumpRadius(double radius)
{
  unbumpRadius_ = radius;
}
double ompl::geometric::SLPRM::SLPRMPlanner::getBumpValue(void) const
{
  return bumpValue_;
}
void ompl::geometric::SLPRM::SLPRMPlanner::setBumpValue(double value)
{
  bumpValue_ = value;
}
double ompl::geometric::SLPRM::SLPRMPlanner::getUnbumpValue(void) const
{
  return unbumpValue_;
}
void ompl::geometric::SLPRM::SLPRMPlanner::setUnbumpValue(double value)
{
  unbumpValue_ = value;
}

void ompl::geometric::SLPRM::SLPRMPlanner::costBump(ompl::geometric::SLPRM::tIndex fromVertex)
{
  tIndex iMax = vertices_.size();
  ompl::base::State const* center;
  if(iMax <= fromVertex)
  {
    center = workingVertex_.getStateData();
  }
  else
  {
    center = vertices_[fromVertex].getStateData();
  }
  for(tIndex i = 0; i < iMax; i++)
  {
    if((!vertices_[i].getIsStart()) && (!vertices_[i].getIsGoal()))
    {
      double d = (si_->distance(vertices_[i].getStateData(), center))/bumpRadius_;
      vertices_[i].bumpCost((bumpValue_)/(1 + d*d));
    }
  }
}
void ompl::geometric::SLPRM::SLPRMPlanner::costUnbump(ompl::geometric::SLPRM::tIndex fromVertex)
{
  tIndex iMax = vertices_.size();
  ompl::base::State const* center;
  if(iMax <= fromVertex)
  {
    center = workingVertex_.getStateData();
  }
  else
  {
    center = vertices_[fromVertex].getStateData();
  }
  for(tIndex i = 0; i < iMax; i++)
  {
    if((!vertices_[i].getIsStart()) && (!vertices_[i].getIsGoal()))
    {
      double d = (si_->distance(vertices_[i].getStateData(), center))/unbumpRadius_;
      vertices_[i].unbumpCost((unbumpValue_)/(1 + d*d));
    }
  }
}
void ompl::geometric::SLPRM::SLPRMPlanner::clearScratch(void)
{
  tIndex kMax = exactVertexChecks_.size();
  for(tIndex k = 0; k < kMax; k++)
  {
    exactVertexChecks_[k]->setExactValid(false);
    exactVertexChecks_[k]->setExactInvalid(false);
  }
  kMax = exactEdgeChecks_.size();
  for(tIndex k = 0; k < kMax; k++)
  {
    exactEdgeChecks_[k]->setExactValid(false);
    exactEdgeChecks_[k]->setExactInvalid(false);
  }
  exactVertexChecks_.clear();
  exactEdgeChecks_.clear();
}

bool ompl::geometric::SLPRM::SLPRMPlanner::verifyCC(tIndex vIndex) const
{
  std::vector<tIndex> toVisit;
  std::set<tIndex> visited;
  toVisit.clear();
  visited.clear();
  toVisit.push_back(vIndex);
  tIndex expCC = CCs_.find(vertices_[vIndex].getCCIndex());
  bool retq = true;
  while(toVisit.size() && retq)
  {
    tIndex newIndex = toVisit[toVisit.size() - 1];
    tIndex numEdge = vertices_[newIndex].getEdgeCount();
    tIndex foundCC = CCs_.find(vertices_[newIndex].getCCIndex());
    if(expCC != foundCC)
    {
      std::cout << "  Nonmaximal CC:  set id " << expCC << " can also reach " << foundCC << std::endl;
      retq = false;
    }
    else
    {
      visited.insert(newIndex);
      toVisit.pop_back();
      for(tIndex j = 0; j < numEdge; j++)
      {
        tIndex towards;
        double weight;
        vertices_[newIndex].getEdge(j, weight, towards);
        if(0 == visited.count(towards))
        {
          toVisit.push_back(towards);
        }
      }
    }
  }
  return retq;
}

ompl::base::PlannerStatus ompl::geometric::SLPRM::SLPRMPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc)
{

ompl::tools::Profiler::Clear();
ompl::tools::Profiler::Start();

std::cout << "SLPRMPlanner::solve - starting" << std::endl;

  if(!roadmapLoaded_)
  {
ompl::tools::Profiler::Begin("LoadRoadmap");
    loadVerticesWithMetaData();
ompl::tools::Profiler::End("LoadRoadmap");
  }

std::cout << "SLPRMPlanner::solve - roadmap loaded" << std::endl;

  //prevent nasty overflows of the crVisitation_ index (unlikely though they are)
  if(ULONG_MAX - vertices_.size() < crVisitation_)
  {
    resetVisitation();
  }

  // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
  // ensures that there is at least one input state and a ompl::base::Goal object specified
  checkValidity();

std::cout << "SLPRMPlanner::solve - validity checked" << std::endl;

  // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
  ompl::base::Goal *goal = pdef_->getGoal().get();
  ompl::geometric::PathGeometric solution(si_);
  bool pathFound = false;
  bool pathStored = false;


  //Obtain one start and one goal state.
  //TODO: possibly make this more flexible to allow several start/goal states
  const ompl::base::State *startState = pis_.nextStart();
  const ompl::base::State *goalState = pis_.nextGoal(ptc);

  if(!startState)
  {
    return ompl::base::PlannerStatus::INVALID_START;
  }
  if(!goalState)
  {
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  //Create temp vertices for the start and goal states
  tIndex startIndex = vertices_.size();
  tIndex goalIndex = startIndex + 1;

  vertices_.push_back(SLPRMVertex(si_, startState));
  vertices_.push_back(SLPRMVertex(si_, goalState));
  vertices_[startIndex].setIsStart();
  vertices_[startIndex].setIndex(startIndex);
  vertices_[goalIndex].setIsGoal();
  vertices_[goalIndex].setIndex(goalIndex);

  //Look for neighbors of the start/goal states in the roadmap
  //  do motion validation here. Keep only reachable neighbors. Lazy check not implemented.
  std::vector<tIndex> startNeighbors;
  std::vector<tIndex> goalNeighbors;
  if(trustRoadmap_ && allowRoadmapExtension_)
  {
    getValidKNeighborhood(startIndex, 40, startNeighbors);
    getValidKNeighborhood(goalIndex, 40, goalNeighbors);
  }
  else
  {
    getKNeighborhood(startIndex, 40, startNeighbors);
    getKNeighborhood(goalIndex, 40, goalNeighbors);
  }


  //Check if the there is some path between a start and goal state: one pair of start/goal states
  //connects to the same connected component
  std::set<tIndex> startNeighborCCs;
  std::set<tIndex> goalNeighborCCs;
  std::set<tIndex> commonCCs;
  getNeighborCCs(startNeighbors, startNeighborCCs);
  getNeighborCCs(goalNeighbors, goalNeighborCCs);
  getCommonCCs(startNeighborCCs, goalNeighborCCs, commonCCs);

  vertices_[startIndex].setExactValid();
  vertices_[goalIndex].setExactValid();

  if((!allowRoadmapExtension_) || (commonCCs.size() && (1 == goalNeighborCCs.size()) && (1 == startNeighborCCs.size())))
  {
    //If path between start and goal exists through a component C, or we are not allowed to change the roadmap anyway:
    //  add connectable start/goal to roadmap as temporary vertices
    //    {current implementation uses only one start and one goal state, and they were already added above}
    //  add temporary edges start->{neighbor vertices in C}
    addEdges(startIndex, startNeighbors, true, false);
    //  add temporary edges {neighbor vertices in C}->goal
    addEdges(goalIndex, goalNeighbors, false, true);
    //  check whether the problem is trivially solvable
    if(si_->checkMotion(vertices_[startIndex].getStateData(), vertices_[goalIndex].getStateData()))
    {
      std::vector<tIndex> dummy;
      dummy.clear(); dummy.push_back(goalIndex);
      addEdges(startIndex, dummy, true, false);
      vertices_[startIndex].edges_[vertices_[startIndex].edges_.size() - 1].setExactValid();
    }
    //  graph search, extract path
    pathFound = graphSearch(ptc, solution, false);
    //  if path not found, fall back on RRTConnect
    if(!pathFound)
    {
      ompl::geometric::RRTConnect rrtConnect(si_);
      rrtConnect.setProblemDefinition(pdef_);
      rrtConnect.setup();
      ompl::base::PlannerStatus result = rrtConnect.solve(ptc);
      if((ompl::base::PlannerStatus::APPROXIMATE_SOLUTION == result) || (ompl::base::PlannerStatus::EXACT_SOLUTION == result))
      {
        pathFound = true;
        pathStored = true;
      }
    }
    //  remove temporary edges and vertices
    tIndex maxK = goalNeighbors.size();
    for(tIndex k = 0; k < maxK; k++)
    {
      vertices_[goalNeighbors[k]].popEdge();
    }
    vertices_.pop_back();
    vertices_.pop_back();
  }
  else if(allowRoadmapExtension_)
  {
    //If no path exists between start and goal through current roadmap and we are allowed to change the roadmap:
    //  add the start/goal vertices to the roadmap as permanent vertices
    //    {current implementation uses only one start and one goal state; to fully add them to the roadmap:
    //       - add them to the nearFinder_ too
    //       - provide a CCindex
    //    }
    nearFinder_.add(startIndex);
    nearFinder_.add(goalIndex);
    vertices_[startIndex].setCCIndex(CCs_.makeNewComponent());
    vertices_[goalIndex].setCCIndex(CCs_.makeNewComponent());
    //  add edges to neighbors, merge connected components as appropriate
    addEdges(startIndex, startNeighbors, true, true);
    addEdges(goalIndex, goalNeighbors, true, true);
    //  check whether the problem is trivially solvable
    if(si_->checkMotion(vertices_[startIndex].getStateData(), vertices_[goalIndex].getStateData()))
    {
      std::vector<tIndex> dummy;
      dummy.clear(); dummy.push_back(goalIndex);
      addEdges(startIndex, dummy, true, true);
      //vertices_[startIndex].edges_[vertices_[startIndex].edges_.size() - 1].setExactValid();
    }
    //  test whether start/goal get in same CC
    while((!areInSameCC(startIndex, goalIndex)) && (false == ptc()))
    {
      //  grow roadmap if not
ompl::tools::Profiler::Begin("sampleAndConnect");
      sampleAndConnect(ptc);
ompl::tools::Profiler::End("sampleAndConnect");
    }
    //  if start/goal in same CC
    if(areInSameCC(startIndex, goalIndex))
    {
      //  search for and extract path
ompl::tools::Profiler::Begin("graphSearch");
      pathFound = graphSearch(ptc, solution, false);
ompl::tools::Profiler::End("graphSearch");
    }

    //  flag clearing
    vertices_[startIndex].setIsStart(false);
    vertices_[goalIndex].setIsGoal(false);
    storeVerticesWithMetaData();
  }
  // When a solution path is computed, save it here
  if(pathFound && (!pathStored))
  {
    solution.interpolate();
    ompl::base::PathPtr solutionPtr(new ompl::geometric::PathGeometric(solution));
    pdef_->addSolutionPath(solutionPtr);
  }
ompl::tools::Profiler::Stop();
ompl::tools::Profiler::Console();
ompl::tools::Profiler::Clear();
  // Return a value from the PlannerStatus enumeration.
  if(pathFound)
  {
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
  else
  {
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}  
void ompl::geometric::SLPRM::SLPRMPlanner::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{

std::cout << "SLPRMPlanner::setProblemDefinition" << std::endl;

  ompl::base::Planner::setProblemDefinition(pdef);
}
void ompl::geometric::SLPRM::SLPRMPlanner::checkValidity(void)
{

std::cout << "SLPRMPlanner::checkValidity" << std::endl;

  ompl::base::Planner::checkValidity();
}
void ompl::geometric::SLPRM::SLPRMPlanner::clear(void)
{

std::cout << "SLPRMPlanner::clear" << std::endl;

  ompl::base::Planner::clear();

std::cout << "SLPRMPlanner::clear - clearing SLPRM specific structures" << std::endl;

  // clear the data structures here
  CCs_.reset();
  nearFinder_.clear();
  vertices_.clear();
  exactVertexChecks_.clear();
  exactEdgeChecks_.clear();
  roadmapLoaded_ = false;
  crVisitation_ = 0;

std::cout << "SLPRMPlanner::clear - done" << std::endl;

}
// optional, if additional setup/configuration is needed, the setup() method can be implemented
void ompl::geometric::SLPRM::SLPRMPlanner::setup(void)
{

std::cout << "SLPRMPlanner::setup" << std::endl;

  ompl::base::Planner::setup();
  if(!roadmapLoaded_)
  {
    loadVerticesWithMetaData();
  }
}
void ompl::geometric::SLPRM::SLPRMPlanner::getPlannerData(ompl::base::PlannerData &data) const
{
  // fill data with the states and edges that were created
  // in the exploration data structure
  // perhaps also fill control::PlannerData
  // TODO: currently, only vertices are added
  data.clear();
  std::vector<ompl::base::PlannerDataVertex> pdVertices;
  tIndex kMax = vertices_.size();
  for(tIndex k = 0; k < kMax; k++)
  {
    ompl::base::PlannerDataVertex dummy(vertices_[k].getStateData());
    pdVertices.push_back(dummy);
  }
  for(tIndex k = 0; k < kMax; k++)
  {
    data.addVertex(pdVertices[k]);
  }
  data.decoupleFromPlanner();
}

void ompl::geometric::SLPRM::SLPRMPlanner::getNodeCCs(std::vector<ompl::geometric::SLPRM::tIndex> &CCIndexes) const
{
  CCIndexes.clear();
  tIndex kMax = vertices_.size();
  for(tIndex k = 0; k < kMax; k++)
  {
    CCIndexes.push_back(vertices_[k].getCCIndex());
  }
}

void ompl::geometric::SLPRM::SLPRMPlanner::getNodeCosts(std::vector<double> &costs) const
{
  costs.clear();
  tIndex kMax = vertices_.size();
  for(tIndex k = 0; k < kMax; k++)
  {
    costs.push_back(vertices_[k].getNodeCost());
  }
}

void ompl::geometric::SLPRM::SLPRMPlanner::getStates(std::vector<ompl::base::State*> &states) const
{
  states.clear();
  tIndex kMax = vertices_.size();
  for(tIndex k = 0; k < kMax; k++)
  {
    ompl::base::State* dummy = si_->cloneState(vertices_[k].getStateData());
    states.push_back(dummy);
  }
}

void ompl::geometric::SLPRM::SLPRMPlanner::printProperties(std::ostream &out) const
{
  out << "SLPRM::printProperties: TBD." << std::endl;
  //TODO
}
void ompl::geometric::SLPRM::SLPRMPlanner::printSettings(std::ostream &out) const
{
  out << "SLPRM::printSettings: TBD." << std::endl;
  //TODO
}


#include <stdio.h>
#include <vector>

#include <ompl/contrib/slprm/SetUnion.h>

ompl::geometric::SLPRM::ConnectedComponent::ConnectedComponent(): index_(0), parent_(0), rank_(0)
{
}
ompl::geometric::SLPRM::ConnectedComponent::ConnectedComponent(ompl::geometric::SLPRM::tIndex indexP): index_(indexP), parent_(indexP), rank_(0)
{
}


ompl::geometric::SLPRM::ComponentMap::ComponentMap(): components_(1)
{
}

ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::ComponentMap::makeNewComponent(void)
{
  tIndex retq = components_.size();
  components_.push_back(ConnectedComponent(components_.size()));
  return(retq);
}
		
ompl::geometric::SLPRM::tIndex ompl::geometric::SLPRM::ComponentMap::find(ompl::geometric::SLPRM::tIndex index)
{
  if(components_[index].parent_ != index)
  {
    components_[index].parent_ = find(components_[index].parent_);
  }
  return components_[index].parent_;
}

void ompl::geometric::SLPRM::ComponentMap::setUnion(ompl::geometric::SLPRM::tIndex indexA, ompl::geometric::SLPRM::tIndex indexB)
{
  if((components_.size() <= indexA) || (components_.size() <= indexB))
  {
    return;
  }
  tIndex compA, compB;
  compA = find(indexA);
  compB = find(indexB);
  if(compA == compB)
  {
    return;
  }
  if(components_[compA].rank_ < components_[compB].rank_)
  {
    components_[compA].parent_ = compB;
    components_[indexA].parent_ = compB;
  }
  else if(components_[compA].rank_ > components_[compB].rank_)
  {
    components_[compB].parent_ = compA;
    components_[indexB].parent_ = compA;
  }
  else
  {
    components_[compB].parent_ = compA;
    components_[indexB].parent_ = compA;
    components_[compA].rank_ = components_[compA].rank_ + 1;
  }
}

void ompl::geometric::SLPRM::ComponentMap::reset(void)
{
  components_.clear();
  components_.push_back(ConnectedComponent(components_.size()));
}



/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Alejandro Perez, Sertac Karaman, Ioan Sucan */

#include "ompl/contrib/rrt_star/RRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <algorithm>
#include <limits>
#include <map>

ompl::geometric::RRTstar::RRTstar(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    ballRadiusMax_ = 0.0;
    ballRadiusConst_ = 0.0;
    delayCC_ = true;

    Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("ball_radius_constant", this, &RRTstar::setBallRadiusConstant, &RRTstar::getBallRadiusConstant);
    Planner::declareParam<double>("max_ball_radius", this, &RRTstar::setMaxBallRadius, &RRTstar::getMaxBallRadius);
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC, "0,1");
}

ompl::geometric::RRTstar::~RRTstar(void)
{
    freeMemory();
}

void ompl::geometric::RRTstar::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (ballRadiusMax_ == 0.0)
        ballRadiusMax_ = si_->getMaximumExtent();
    if (ballRadiusConst_ == 0.0)
        ballRadiusConst_ = maxDistance_ * sqrt((double)si_->getStateSpace()->getDimension());

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));
}

void ompl::geometric::RRTstar::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

ompl::base::PlannerStatus ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                  *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    base::OptimizationObjective *opt    = pdef_->getOptimizationObjective().get();
    
    // when no optimization objective is specified, we create a temporary one (we should not modify the ProblemDefinition)
    boost::scoped_ptr<base::OptimizationObjective> temporaryOptimizationObjective;

    if (opt && !dynamic_cast<base::PathLengthOptimizationObjective*>(opt))
    {
        opt = NULL;
        OMPL_WARN("Optimization objective '%s' specified, but such an objective is not appropriate for %s. Only path length can be optimized.", getName().c_str(), opt->getDescription().c_str());
    }
    
    if (!opt)
    { 
        // by default, optimize path length and run until completion
        opt = new base::PathLengthOptimizationObjective(si_, std::numeric_limits<double>::epsilon());
        temporaryOptimizationObjective.reset(opt);
        OMPL_INFORM("No optimization objective specified. Defaulting to optimization of path length for the allowed planning time.");
    }

    if (!goal)
    {
        OMPL_ERROR("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("Starting with %u states", nn_->size());

    Motion *solution       = NULL;
    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion     = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    std::vector<Motion*> solCheck;
    std::vector<Motion*> nbh;
    std::vector<double>  dists;
    std::vector<int>     valid;
    unsigned int         rewireTest = 0;
    double               stateSpaceDimensionConstant = 1.0 / (double)si_->getStateSpace()->getDimension();

    while (ptc == false)
    {
        // sample random state (with goal biasing)
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        base::State *dstate = rstate;

        // find state to add
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            double distN = si_->distance(dstate, nmotion->state);
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->cost = nmotion->cost + distN;

            // find nearby neighbors
            double r = std::min(ballRadiusConst_ * pow(log((double)(1 + nn_->size())) / (double)(nn_->size()), stateSpaceDimensionConstant),
                                ballRadiusMax_);

            nn_->nearestR(motion, r, nbh);
            rewireTest += nbh.size();

            // cache for distance computations
            dists.resize(nbh.size());
            // cache for motion validity
            valid.resize(nbh.size());
            std::fill(valid.begin(), valid.end(), 0);

            if (delayCC_)
            {
                // calculate all costs and distances
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                    nbh[i]->cost += si_->distance(nbh[i]->state, dstate);

                // sort the nodes
                std::sort(nbh.begin(), nbh.end(), compareMotion);

                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    dists[i] = si_->distance(nbh[i]->state, dstate);
                    nbh[i]->cost -= dists[i];
                }

                // collision check until a valid motion is found
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        double c = nbh[i]->cost + dists[i];
                        if (c < motion->cost)
                        {
                            if (si_->checkMotion(nbh[i]->state, dstate))
                            {
                                motion->cost = c;
                                motion->parent = nbh[i];
                                valid[i] = 1;
                                break;
                            }
                            else
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        valid[i] = 1;
                        dists[i] = distN;
                        break;
                    }
                }
            }
            else
            {
                // find which one we connect the new state to
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        dists[i] = si_->distance(nbh[i]->state, dstate);
                        double c = nbh[i]->cost + dists[i];
                        if (c < motion->cost)
                        {
                            if (si_->checkMotion(nbh[i]->state, dstate))
                            {
                                motion->cost = c;
                                motion->parent = nbh[i];
                                valid[i] = 1;
                            }
                            else
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        valid[i] = 1;
                        dists[i] = distN;
                    }
                }
            }

            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);

            solCheck.resize(1);
            solCheck[0] = motion;

            // rewire tree if needed
            for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                if (nbh[i] != motion->parent)
                {
                    double c = motion->cost + dists[i];
                    if (c < nbh[i]->cost)
                    {
                        bool v = valid[i] == 0 ? si_->checkMotion(nbh[i]->state, dstate) : valid[i] == 1;
                        if (v)
                        {
                            // Remove this node from its parent list
                            removeFromParent (nbh[i]);
                            double delta = c - nbh[i]->cost;

                            // Add this node to the new parent
                            nbh[i]->parent = motion;
                            nbh[i]->cost = c;
                            nbh[i]->parent->children.push_back(nbh[i]);
                            solCheck.push_back(nbh[i]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i], delta);
                        }
                    }
                }

            // Make sure to check the existing solution for improvement
            if (solution)
                solCheck.push_back(solution);

            // check if we found a solution
            for (unsigned int i = 0 ; i < solCheck.size() ; ++i)
            {
                double dist = 0.0;
                bool solved = goal->isSatisfied(solCheck[i]->state, &dist);
                sufficientlyShort = solved ? opt->isSatisfied(solCheck[i]->cost) : false;

                if (solved)
                {
                    if (sufficientlyShort)
                    {
                        solution = solCheck[i];
                        break;
                    }
                    else if (!solution || (solCheck[i]->cost < solution->cost))
                    {
                        solution = solCheck[i];
                    }
                }
                else if (!solution && dist < approximatedist)
                {
                    approximation = solCheck[i];
                    approximatedist = dist;
                }
            }
        }

        // terminate if a sufficient solution is found
        if (solution && sufficientlyShort)
            break;
    }
    
    double solutionCost;
    bool approximate = (solution == NULL);
    bool addedSolution = false;
    if (approximate)
    {
        solution = approximation;
        solutionCost = approximatedist;
    }
    else
        solutionCost = solution->cost;

    if (solution != NULL)
    {
        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), approximate, solutionCost);
        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("Created %u states. Checked %lu rewire options.", nn_->size(), rewireTest);

    return base::PlannerStatus(addedSolution, approximate);
}

void ompl::geometric::RRTstar::removeFromParent(Motion *m)
{
    std::vector<Motion*>::iterator it = m->parent->children.begin ();
    while (it != m->parent->children.end ())
    {
        if (*it == m)
        {
            it = m->parent->children.erase(it);
            it = m->parent->children.end ();
        }
        else
            ++it;
    }
}

void ompl::geometric::RRTstar::updateChildCosts(Motion *m, double delta)
{
    for (size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost += delta;
        updateChildCosts(m->children[i], delta);
    }
}

void ompl::geometric::RRTstar::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::RRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.addEdge (base::PlannerDataVertex (motions[i]->parent ? motions[i]->parent->state : NULL),
                      base::PlannerDataVertex (motions[i]->state));
}

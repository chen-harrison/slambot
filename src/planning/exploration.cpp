#include <planning/exploration.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unistd.h>
#include <cassert>

typedef Point<int> cell_t;
const float kReachedPositionThreshold = 0.05f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool operator==(const pose_xyt_t& lhs, const pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}


Exploration::Exploration(int32_t teamNumber,
                         lcm::LCM* lcmInstance)
: teamNumber_(teamNumber)
, state_(exploration_status_t::STATE_INITIALIZING)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
, pathReceived_(false)
{
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Exploration::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Exploration::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Exploration::handleConfirmation, this);
    
    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    MotionPlannerParams params;
    // params.robotRadius = 0.2;
    params.robotRadius = 0.17;
    robotRadius_ = params.robotRadius;
    planner_.setParams(params);
}


bool Exploration::exploreEnvironment()
{
    while((state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runExploration();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }
    }
    
    // If the state is completed, then we didn't fail
    return state_ == exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

void Exploration::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Exploration::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}

void Exploration::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}

bool Exploration::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Exploration::runExploration(void)
{
    assert(isReadyToUpdate());
    
    copyDataForUpdate();
    executeStateMachine();
}


void Exploration::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    
    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
    }
    
    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;
    
    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        haveHomePose_ = true;
        std::cout << "INFO: Exploration: Set home pose:" << homePose_.x << ',' << homePose_.y << ',' 
            << homePose_.theta << '\n';
    }
}


void Exploration::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;
    
    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;
    
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing();
                break;
            case exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeExploringMap(stateChanged);
                break;
                
            case exploration_status_t::STATE_RETURNING_HOME:
                nextState = executeReturningHome(stateChanged);
                break;

            case exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;
                
            case exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;
        }
        
        stateChanged = nextState != state_;
        state_ = nextState;
        
    } while(stateChanged);

    // If path confirmation was not received, resend path
    if(!pathReceived_)
    {
        std::cout << "the current path was not received by motion_controller, attempting to send again:\n";

        std::cout << "timestamp: " << currentPath_.utime << "\n";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }

    // If path changed, send current path
    if(previousPath.path != currentPath_.path)
    { 

        std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Mbot:\n";

        std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}


int8_t Exploration::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to exploring once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    return exploration_status_t::STATE_EXPLORING_MAP;
}


int8_t Exploration::executeExploringMap(bool initialize)
{
    //////////////////////// TODO: Implement your method for exploring the map ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) frontiers_.empty() == true      : all frontiers have been explored as determined by find_map_frontiers()
    *       (2) currentPath_.path_length > 1 : currently following a path to the next frontier
    * 
    *   - Use the provided function find_map_frontiers() to find all frontiers in the map.
    *   - You will need to implement logic to select which frontier to explore.
    *   - You will need to implement logic to decide when to select a new frontier to explore. Take into consideration:
    *       -- The map is evolving as you drive, so what previously looked like a safe path might not be once you have
    *           explored more of the map.
    *       -- You will likely be able to see the frontier before actually reaching the end of the path leading to it.
    */
    
    planner_.setMap(currentMap_);
    double distToTarget = distance_between_points(Point<float>(currentTarget_.x, currentTarget_.y),
                                                  Point<float>(currentPose_.x, currentPose_.y));
    double distToStart = 0.0;
    if(!(currentPose_ == homePose_))
    {    
        distToStart = distance_between_points(Point<float>(currentPath_.path.front().x, currentPath_.path.front().y),
                                                     Point<float>(currentPose_.x, currentPose_.y));
    }
    
    if(distToTarget <= 4.0 * currentMap_.metersPerCell() ||     // arrived at frontier
       currentPose_ == homePose_ ||                             // just starting
       2.0 * distToTarget < distToStart)                        // time to update path to goal
    {
        if(2.0 * distToTarget < distToStart)
        {
            printf("REFRESHING PATH! to target = %lf, to start = %lf\n", distToTarget, distToStart);
            printf("current: (%f, %f)\n", currentPose_.x, currentPose_.y);
            printf("start: (%f, %f)\ntarget: (%f, %f)\n", currentPath_.path.front().x, currentPath_.path.front().y, currentTarget_.x, currentTarget_.y);
        }
        frontiers_ = find_map_frontiers(currentMap_, currentPose_);
        planner_.setNumFrontiers(frontiers_.size());

        if(frontiers_.empty())
        {
            printf("empty frontiers!\n");
            currentPath_ = planner_.planPath(currentPose_, currentPose_);
            currentTarget_ = currentPose_;
        }
        else
        {
            std::vector<pose_xyt_t> frontier_pts;
            std::vector<float> frontier_dists;
            for(auto& f : frontiers_)
            {
                pose_xyt_t avg = {currentPose_.utime, 0.0, 0.0, 0.0};
                for(auto& p : f.cells)
                {
                    avg.x += p.x;
                    avg.y += p.y;
                }
                avg.x /= (float)f.cells.size();
                avg.y /= (float)f.cells.size();
                frontier_dists.push_back(distance_between_points(Point<float>(avg.x, avg.y),
                                                                Point<float>(currentPose_.x, currentPose_.y)));
                frontier_pts.push_back(avg);
            }
            std::vector<float>::iterator min_iter = std::min_element(frontier_dists.begin(), frontier_dists.end());
            int min_idx = std::distance(frontier_dists.begin(), min_iter);
            // printf("min idx: %d\n", min_idx);

            pose_xyt_t new_target = frontier_pts[min_idx];
            printf("new target point: (%f, %f)\n", new_target.x, new_target.y);

            if(planner_.isValidGoal(new_target))
            {
                currentPath_ = planner_.planPath(currentPose_, new_target);
                currentTarget_ = new_target;
            }
            else
            {
                printf("starting breadth-first search\n");
                cell_t root_cell = global_position_to_grid_cell(Point<double>(new_target.x, new_target.y), planner_.obstacleDistances());
                std::vector<cell_t> old_queue = {root_cell};
                std::vector<cell_t> new_queue;
                std::vector<cell_t> closed;
                bool found_path = false;
                while(!found_path)
                {
                    new_queue.clear();
                    for(auto& o : old_queue)
                    {
                        planner_.setMap(currentMap_);
                        std::vector<cell_t> adj = {cell_t((o.x - 1), o.y), cell_t((o.x + 1), o.y),
                                                cell_t(o.x, (o.y - 1)), cell_t(o.x, (o.y + 1))};
                        
                        for(auto& a : adj)
                        {
                            if((planner_.obstacleDistances().isCellInGrid(a.x, a.y)) &&
                               (std::find(closed.begin(), closed.end(), a) == closed.end()))
                            {
                                if(planner_.obstacleDistances()(a.x, a.y) > 1.25*robotRadius_)
                                {
                                    Point<double> check_point = grid_position_to_global_position(a, planner_.obstacleDistances());
                                    pose_xyt_t check_pose = {currentPose_.utime, (float)check_point.x, (float)check_point.y, 0.0};
                                    robot_path_t check_path = planner_.planPath(currentPose_, check_pose);
                                    if(check_path.path_length > 1)
                                    {
                                        currentPath_ = check_path;
                                        currentTarget_ = check_pose;
                                        printf("goal found! (%f, %f)\n", currentTarget_.x, currentTarget_.y);
                                        found_path = true;
                                        break;
                                    }
                                    // if(planner_.isValidGoal(check_pose))
                                    // {
                                    //     new_target = check_pose;
                                    //     found_path = true;
                                    //     printf("goal found! (%f, %f)\n", new_target.x, new_target.y);
                                    //     break;
                                    // }
                                }
                                // if cell doesn't give a valid path, store in new_queue to expand it
                                if(std::find(new_queue.begin(), new_queue.end(), a) == new_queue.end() &&
                                   std::find(closed.begin(), closed.end(), a) == closed.end())
                                {
                                    new_queue.push_back(a);
                                }
                            }
                        }
                        closed.push_back(o);

                        if(found_path)
                        {
                            break;
                        }
                    }
                    old_queue = new_queue;
                    if(old_queue.empty())
                    {
                        printf("no nodes left\n");
                        // distToTarget = distance_between_points(Point<float>(currentTarget_.x, currentTarget_.y),
                        //                                        Point<float>(currentPose_.x, currentPose_.y));
                        // if(distToTarget <= 4.0*currentMap_.metersPerCell())
                        // {
                        //     break;
                        // }
                        break;
                    }
                }
            }          
        }  
    }

    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    
    // If no frontiers remain, then exploration is complete
    if(frontiers_.empty())
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else if there's a path to follow, then we're still in the process of exploring
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case exploration_status_t::STATUS_IN_PROGRESS:
            return exploration_status_t::STATE_EXPLORING_MAP;
            
        // If exploration is completed, then head home
        case exploration_status_t::STATUS_COMPLETE:
            return exploration_status_t::STATE_RETURNING_HOME;
            
        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case exploration_status_t::STATUS_FAILED:
            return exploration_status_t::STATE_FAILED_EXPLORATION;
            
        default:
            std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
            return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeReturningHome(bool initialize)
{
    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */
    
    if(!(currentTarget_ == homePose_))
    {
        currentPath_ = planner_.planPath(currentPose_, homePose_);
        currentTarget_ = homePose_;
    }

    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_RETURNING_HOME;
    
    double distToHome = distance_between_points(Point<float>(homePose_.x, homePose_.y), 
                                                Point<float>(currentPose_.x, currentPose_.y));
    // If we're within the threshold of home, then we're done.
    if(distToHome <= kReachedPositionThreshold)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_RETURNING_HOME;
    }
    else // if(status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}

int8_t Exploration::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
}


int8_t Exploration::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}
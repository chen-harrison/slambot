#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>

typedef Point<int> cell_t;
struct CostNode
{
    cell_t cell;
    cell_t parent;
    float cost;

    CostNode(cell_t c,
             cell_t p,
             float k)
    {
        cell = c;
        parent = p;
        cost = k;
    }

    bool operator>(const CostNode& rhs) const
    {
        return cost > rhs.cost;
    }
};


float calc_cost(cell_t current,
                cell_t parent,
                cell_t start,
                cell_t goal,
                const ObstacleDistanceGrid& distances,
                const SearchParams& params)
{
    float g = (float)std::abs(parent.x - start.x) + (float)std::abs(parent.y - start.y) + 1.0;
    float h = (float)std::abs(goal.x - current.x) + (float)std::abs(goal.y - current.y);
    float d = 0.0;

    if(distances(current.x, current.y) < params.maxDistanceWithCost)
    {
        d = std::pow(params.maxDistanceWithCost - distances(current.x, current.y),
                     params.distanceCostExponent);
    }

    return g + h + d;
}


void make_path(robot_path_t& path,
               pose_xyt_t start,
               pose_xyt_t goal,
               const ObstacleDistanceGrid& distances,
               CostNode n,
               std::vector<CostNode> closed)
{
    std::vector< pose_xyt_t > hold = {goal};

    CostNode current = n;
    while(current.cell != current.parent)
    {
        Point<float> glob = grid_position_to_global_position((Point<double>)current.cell, distances);
        float theta = std::atan2(hold.back().y - glob.y, hold.back().x - glob.x);
        pose_xyt_t newPose = {start.utime, glob.x, glob.y, theta};
        hold.push_back(newPose);

        std::vector<CostNode>::iterator next = std::find_if(closed.begin(), closed.end(),
                                                            [current](const CostNode& cn){return cn.cell == current.parent;});
        current = *next;
    }
    hold.push_back(start);
    std::reverse(hold.begin(), hold.end());

    path.path = {hold.front()};
    float oldMag = 1.0;
    Point<float> oldDelta = Point<float>(0.0, 0.0);
    for(int i = 1; i < (int)hold.size(); i++)
    {
        float newMag = distance_between_points(Point<float>(hold[i].x, hold[i].y),
                                               Point<float>(path.path.back().x, path.path.back().y));
        
        // for testing: make newDelta a unit vector
        Point<float> newDelta = Point<float>((hold[i].x - path.path.back().x),
                                             (hold[i].y - path.path.back().y));

        if((newDelta.x == oldDelta.x) &&
           (newDelta.y == oldDelta.y))
        {
            // "remove" previous point by replacing it
            path.path.back() = hold[i];
            oldDelta = Point<float>(newDelta.x + oldDelta.x, newDelta.y + oldDelta.y);
            oldMag += newMag;
        }
        // else if((std::abs(newDelta.x) + std::abs(oldDelta.x) == 1.0) &&
        //         (std::abs(newDelta.y) + std::abs(oldDelta.y) == 1.0))
        // {
        //     printf("remove diagonal!\n olddelta = (%f, %f)\nnewdelta = (%f, %f)\n", oldDelta.x, oldDelta.y, newDelta.x, newDelta.y);
        //     path.path.back() = hold[i];
        //     oldDelta = Point<float>(newDelta.x + oldDelta.x, newDelta.y + oldDelta.y);
        //     oldMag = std::sqrt(2.0);
        // }
        else
        {
            // different direction of movement, so append
            path.path.push_back(hold[i]);
            oldDelta = newDelta;
            oldMag = newMag;
        }
        
    }

    path.path_length = static_cast<int32_t>(path.path.size());
}


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    robot_path_t path;
    path.utime = start.utime;

    if(start.x == goal.x && start.y == goal.y)
    {
        printf("WHY AM I HERE\n");
        path.path.push_back(start);
        path.path_length = static_cast<int32_t>(path.path.size());
        return path;
    }

    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    std::vector<CostNode> open = {CostNode(startCell, startCell, 0.0)};
    std::vector<CostNode> closed;

    while(open.size() != 0)
    {
        std::sort(open.begin(), open.end(), std::greater<CostNode>());
        CostNode n = {open.back().cell, open.back().parent, open.back().cost};
        open.pop_back();
        
        // printf("node n: (%d, %d), distance = %f\n", n.cell.x, n.cell.y, distances(n.cell.x, n.cell.y));
        std::vector<cell_t> kids = {cell_t(n.cell.x - 1, n.cell.y),
                                    cell_t(n.cell.x + 1, n.cell.y),
                                    cell_t(n.cell.x, n.cell.y - 1),
                                    cell_t(n.cell.x, n.cell.y + 1)};
        for(auto& k : kids)
        {
            // printf("kid: (%d, %d), distance = %f\n", k.x, k.y, distances(k.x, k.y) - 0.5*distances.metersPerCell());
            // printf("min distance: %lf\n", params.minDistanceToObstacle);
            if(distances.isCellInGrid(k.x, k.y) &&
               (distances(k.x, k.y) - 0.5*distances.metersPerCell() > params.minDistanceToObstacle || distances(k.x, k.y) == -1.0) &&
            //    (distances(k.x, k.y) > params.minDistanceToObstacle || distances(k.x, k.y) == -1.0) &&
               std::find_if(closed.begin(), closed.end(),
               [k](const CostNode& cn){return cn.cell == k;}) == closed.end())
            {
                // printf("satisfied conditions! (%d, %d)\n", k.x, k.y);
                float cost = calc_cost(k, n.cell, startCell, goalCell, distances, params);
                CostNode kidNode = {k, n.cell, cost};

                if(kidNode.cell == goalCell)
                {
                    make_path(path, start, goal, distances, n, closed);
                    return path;
                }
                // Only add to open if it's not already there
                if(std::find_if(open.begin(), open.end(),
                   [kidNode](const CostNode& cn){return cn.cell == kidNode.cell;}) == open.end())
                {
                    open.push_back(kidNode);
                }
                
            }
        }

        closed.push_back(n);
    }

    printf("no path found to goal\n");
    path.path.push_back(start);
    path.path_length = static_cast<int32_t>(path.path.size());
    return path;
}

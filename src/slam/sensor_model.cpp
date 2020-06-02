#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
: kObsScore_((int)(0.75 * std::numeric_limits<CellOdds>::max()))
, kAtObs_(-0.004f)
, kBeforeObs_(-0.4f)
, kAfterObs_(-0.8f)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double logScore = 0.0;

    for(auto& ray : movingScan)
    {
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayEnd;

        rayEnd.x = static_cast<int>(rayStart.x + (ray.range * std::cos(ray.theta) * map.cellsPerMeter()));
        rayEnd.y = static_cast<int>(rayStart.y + (ray.range * std::sin(ray.theta) * map.cellsPerMeter()));

        int dx = std::abs(rayEnd.x - rayStart.x);
        int dy = std::abs(rayEnd.y - rayStart.y);
        int sx = (rayStart.x < rayEnd.x) ? 1 : -1;
        int sy = (rayStart.y < rayEnd.y) ? 1 : -1;
        int err = dx - dy;
        int e2 = 2 * err;
        int x = rayStart.x;
        int y = rayStart.y;

        int afterObs = 0;
        bool atObs = false;
        while (x != rayEnd.x || y != rayEnd.y)
        {
            if(x != rayStart.x || y != rayStart.y)
            {
                if(map(x, y) >= kObsScore_ &&
                   std::sqrt(std::pow(rayEnd.x - x, 2) + std::pow(rayEnd.y - y, 2)) > std::sqrt(2.0))
                {
                    afterObs++;
                    if(afterObs >= 2)
                    {
                        logScore += kAfterObs_;
                        break;
                    }                    
                }
            }

            e2 = 2 * err;
            if(e2 >= -dy)
            {
                err -= dy;
                x += sx;
            }
            if(e2 <= dx)
            {
                err += dx;
                y += sy;
            }
        }

        if(afterObs < 2)
        {
            // move back a step to grab cell before beam end
            int xBefore = x;
            int yBefore = y;
            if(e2 >= -dy)
            {
                xBefore -= sx; 
            }
            if(e2 <= dx)
            {
                yBefore -= sy;
            }
            
            // move forward a step to grab cell after beam end
            int xAfter = x;
            int yAfter = y;
            e2 = 2 * err;
            if(e2 >= -dy)
            {
                xAfter += sx;
            }
            if(e2 <= dx)
            {
                yAfter += sy;
            }

            if(map(x, y) >= kObsScore_ ||
               map(xBefore, yBefore) >= kObsScore_ ||
               map(xAfter, yAfter) >= kObsScore_)
            {
                atObs = true;
                logScore += kAtObs_;
            }

            if(!atObs)
            {
                logScore += kBeforeObs_;
            }
        }
    }
    double score = std::exp(logScore);
    return score;
}

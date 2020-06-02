#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>

typedef Point<int> cell_t;

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    std::vector<cell_t> oldCells;
    std::vector<cell_t> newCells;

    for(int i = 0; i < width_; i++)
    {
        for(int j = 0; j < height_; j++)
        {
            if (map(i, j) >= 0)
            {
                distance(i, j) = 0.0;
                oldCells.push_back(cell_t(i, j));
            }
            else
            {
                // "Empty" space initializated as -1
                distance(i, j) = -1.0;
            }
            
        }
    }

    if(oldCells.empty())    // No obstacles
    {
        std::fill(cells_.begin(), cells_.end(), 128.0);
    }
    
    float cellValue = 1.0;
    while(true)
    {
        newCells.clear();

        for(auto& o : oldCells)
        {
            // Adjacent cells to check
            // std::vector<cell_t> adjCells = {cell_t((o.x - 1), o.y), cell_t((o.x + 1), o.y),
            //                                 cell_t(o.x, (o.y - 1)), cell_t(o.x, (o.y + 1))};
            // for(auto& a : adjCells)
            // {
            //     if(isCellInGrid(a.x, a.y) && (distance(a.x, a.y) == -1.0))
            //     {
            //         distance(a.x, a.y) = cellValue * metersPerCell_;
            //         newCells.push_back(a);
            //     }
            // }
            for(int i = o.x-1; i <= o.x+1; i++)
            {
                for(int j = o.y-1; j <= o.y+1; j++)
                {
                    if(isCellInGrid(i, j) && (distance(i, j) == -1.0))
                    {
                        distance(i, j) = cellValue * metersPerCell_;
                        newCells.push_back(cell_t(i, j));
                    }
                }
            }
        }
        
        if(newCells.empty())
        {
            break;
        }

        oldCells = newCells;
        cellValue++;
    }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}

#pragma once
#include <vector>
#include "run.h"
#include "common.h"

namespace haoxing
{
    namespace route2
    {
        class  RoadPlanningService {
            public:
                RoadPlanningService();

                ~RoadPlanningService();

                void running(std::vector<PointXYZ> &m_FNode, 
                             std::vector<Obstacle> &obstacles, PointXYZ *locationNode,
                             Machine *mac, Tractor *tra, double interSp, double typeSeeding, 
                             double ChangeOperation, double AccOverlap, double sd);

                std::vector<PointXYZ> getNode();
        }; 
    } // namespace route2
} // namespace haoxing

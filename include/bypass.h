#pragma once
#include "bypassobs.h"
#include "fishtail.h"
#include "common.h"

namespace haoxing
{
    namespace routing
    {
        class planbypass
        {
            bgy::common::math::Math m_math;
            bgy::common::Common m_common;
            haoxing::routing::bypass m_bypass;
            std::vector<V2P> RoutingPoint;
            fishtail ft;
            std::vector<std::vector<PointXYZ> > bypassNodes;
            std::vector<std::vector<PointXYZ> > obsNodes;
         
         public:
            /**
             * @brief 鱼尾
             * @param[in] EndPoint 
             * @param[in] obstacles 
             * @param[in] machine 
             * @param[in] tractor 
             * @param[in] SpacingPoint 
             * @param[in] k 
             */
            void runfishtail(V2P EndPoint,std::vector<Obstacle> obstacles, Machine machine, Tractor tractor, double SpacingPoint, int k);
            std::vector<V2P> getPoints();
        };
    }
}
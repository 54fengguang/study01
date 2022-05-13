#pragma once
#include "common.h"
#include "routing.h"
namespace bgy
{
    namespace routing 
    {
        class pear
        {
        public:
            bgy::common::Common common;
            bgy::common::math::Math math;
            bgy::routing::RoutingABCD routingabcd;
            /**
             * @brief // 根据转弯半径与幅宽关系建立梨形规划
             * @param[in] Node 
             * @param[in] machine 
             * @param[in] tractor 
             * @param[in] insterp 
             * @return V2P 
             */
            V2P planPear( V2P &Node, const Machine &machine, const Tractor &tractor, const double &insterp);
            /**
             * @brief 有田头安全区域限制功能的梨形规划
             * @param[in] Node 
             * @param[in] machine 
             * @param[in] tractor 
             * @param[in] intersp 
             * @return V2P 
             */
            V2P planPearLimit(V2P &Node, const Machine &machine, const Tractor &tractor, const double &intersp);

            /**
             * 
             * 
             * 
             *
             * 
             */  
            V2P planPearDouble(V2P &Node, const Machine &machine, const Tractor &tractorm, const double &intersp);
        };
    }
}
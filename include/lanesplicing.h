#pragma once
#include "common.h"

namespace bgy
{
    namespace routing
    {
        class lane
        {
        public:
            bgy::common::Common common;
            bgy::common::math::Math math;
            std::vector<PointXYZ> splicing_Nonparallel(std::vector<PointXYZ> &p, Machine const &mechine, Tractor const &tractor, double const &intersp);
        };
    }
}
#pragma once
#include "common.h"

class SegLane
{
public:
    bgy::common::math::Math m_math;
    bgy::common::Common common;

    /**
     * @brief 
     * @param[in] p 
     * @return std::vector<V2P> 
     */
    std::vector<std::vector<PointXYZ> > OptSegmentLane(std::vector<PointXYZ> const &p);

    /**
     * @brief 无优化版本
     * @param[in] p 
     * @return std::vector<V2P> 
     */
    std::vector<std::vector<PointXYZ> > SegmentLane(std::vector<PointXYZ> const &p);

    /**
    * @brief Routing Segment Smoothing Optimization
    * @param[in] p 
    * @return std::vector<V2P>
    */
    std::vector<std::vector<PointXYZ> > OptSegment4(std::vector<std::vector<PointXYZ> > &p);

    
};
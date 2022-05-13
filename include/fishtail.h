#pragma once
#include "common.h"

class fishtail
{
public:
    bgy::common::math::Math m_math;
    bgy::common::Common common;
    // haoxing::routing::bypass m_bypass;
    std::vector<V2P> RoutingPoint;
    /**
     * @brief 鱼尾部分为曲线的鱼尾算法规划
     * @param[in] p 直线路径点
     * @param[in] tractor 拖拉机参数
     * @param[in] machine 农具参数
     * @param[in] inster 间隔点
     * @return V2P 
     */
    V2P fishtailplan(const V2P &p, const Tractor tractor, const Machine &machine, const double &inster);
    std::vector<V2P> fishtailplan1(const V2P &p, const Tractor tractor, const Machine &machine, const double &inster);
    /**
     * @brief 鱼尾部分为直线的鱼尾规划
     * @param[in] p 
     * @param[in] tractor 
     * @param[in] machine 
     * @param[in] inster 
     * @return V2P 
     */
    V2P fishtailplan_st(const V2P &p, const Tractor tractor, const Machine &machine, const double &inster);
    /**
     * @brief 鱼尾回环收割鱼尾规划
     * @param[in] p 
     * @param[in] tractor 
     * @param[in] machine 
     * @param[in] inster 
     * @return V2P 
     */
    V2P fishtailplan_loop(const V2P &p, const Tractor tractor, const Machine &machine, const double &inster);
    bool Ori = true;

    /**
     * @brief 带直线作业边倒退的鱼尾设计方案 
     * 
     */
    std::vector<V2P> fishtailplanback(const V2P &p, const Tractor tractor, const Machine &machine, const double &inster);

    std::vector<V2P> fishtailplanNonparallel(const V2P &p, const Tractor tractor, const Machine &machine, const double &inster);

    std::vector<V2P> fishtailplanNonparallel2(const V2P &p, const Tractor tractor, const Machine &machine, const double &inster);

    
};

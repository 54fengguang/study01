/**
 * @copyright Copyright (c) 2021 Bright Dream Robotics Co.Ltd
 * @file sortopt.h
 * @brief 对作业进行套圈逻辑优化排序
 * @author tanglingmao (tanglingmao@countrygarden.com.cn)
 * @date 2021-12-09
 * @version 1.0.0
 */
#pragma once
#include "common.h"
class sortopt
{
    private:
        bgy::common::Common m_common;
        Machine m_machine;
        Tractor m_tractor;
        std::vector<PointXYZ> m_sortRes;
        std::vector<PointXYZ> m_sortPoints;
        std::vector<int> index;
        int pSize;
        /**
         * @brief 梭行排序
         */
        void adjacentSort();
        /**
         * @brief 套圈优化排序
         */
        void loopSort();
        /**
         * @brief 最小套圈优化排序
         */
        void minloopSort();

        void fishSort();
    public:

        /**
         * @brief 
         * @param[std::vector<PointXYZ>] p 
         * @param[in] sortType 
         */
        void runSort(std::vector<PointXYZ> &p, Machine &machine, Tractor &tractor, int &sortType);
        /**
         * @brief Get the Sort Res object 获取排序后结果
         * @return std::vector<PointXYZ> 
         */
        std::vector<PointXYZ> getSortRes();
};
/**
 * @copyright Copyright (c) 2021 Bright Dream Robotics Co.Ltd
 * @file bypassobs.h
 * @brief 暂时未加入额外避障, 即检测是有限的,可以采用障碍合并,规避系类问题!
 * @author tanglingmao (tanglingmao@countrygarden.com.cn)
 * @date 2021-12-25
 * @version 1.0.0
 */
#pragma once
#include "common.h"
#include "fishtail.h"
namespace haoxing
{
    namespace routing
    {
        class bypass
        {
            bool Ori = true;
            double dr = 3.0;
            bgy::common::math::Math m_math;
            bgy::common::Common m_common;
            std::vector<PointXYZ> newEnd;
            std::vector<std::vector<PointXYZ> > bypassNodes;
            std::vector<std::vector<PointXYZ> > obsNodes;
            fishtail ft;
            int m_obsLocationType = 0; // 0表示直线段,1表示曲线段
            int obsIndex = 0;

            public:
                /**
                 * @brief 处理直线段绕障路径生成
                 * @param[in] line 
                 * @param[in] mobstacleNode 
                 * @param[in] Ori 
                 * @return std::vector<PointXYZ> 
                 */
                std::vector<PointXYZ> runBypass(std::vector<PointXYZ> line, Obstacle mobstacleNode, Tractor & tra,bool Ori);
                /**
                 * @brief 交叉线链接处的绕障
                 * @param[in] line 
                 * @param[in] mobstacleNode 
                 * @param[in] Ori 
                 * @return std::vector<PointXYZ> 
                 */
                std::vector<PointXYZ> runBypassIntersect(std::vector<PointXYZ> line, Obstacle mobstacleNode, Tractor & tra,bool Ori);


                void testloopPlanGBTS(std::vector<PointXYZ> LoopNode, Tractor tractor,Machine machine, std::vector<Obstacle> obs);
                /**
                 * @brief 平行线绕障判断,并
                 * @param[in] obstacles 
                 * @param[in] tractor 
                 * @param[in] machine 
                 */
                void checkObs(std::vector<PointXYZ>EndPoint, std::vector<Obstacle> obstacles,Tractor tractor, Machine machine, double intersp, int k, int index);
                /**
                 * @brief 交互处
                 * @param[in] obstacles 
                 * @param[in] tractor 
                 * @param[in] machine 
                 * @param[in] intersp
                 */
                void checkObsIntersect(std::vector<PointXYZ>EndPoint, std::vector<Obstacle> obstacles,Tractor tractor, Machine machine, double intersp, int k, int index);

                std::vector<std::vector<PointXYZ> > getObsNodes();

                int getInterStatus();
                
                int getIndex();
        };
    }
}
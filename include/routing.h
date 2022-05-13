#pragma once 
#include "common.h"
#include "fishtail.h"
#include <iostream>


namespace bgy{
    namespace routing{
        class RoutingABCD
        {
        private:
            /* data */
        public:
            bool ori = true;
            std::vector<int> index;
            bgy::common::math::Math math;
            bgy::common::Common common;
            fishtail ft;
            /**
             * @brief 曲线点获取
             * @return V2P 
             */
            V2P CurvePoint();
            /**
             * @brief Get the New A B C D object
             * @param[in] field_point 
             * @param[in] location_point 
             * @return V2P 
             */
            V2P getNewABCD(V2P field_point, PointXYZ location_point);
            /**
             * @brief 获取作业直线点
             * @param[in] field_point 
             * @param[in] machine 
             * @param[in] tractor 
             * @return V2P 
             */
            V2P StraightEndpoint(V2P field_point, Machine machine, Tractor tractor);
            /**
             * @brief Get the Line Curve Point object
             * @param[in] EndPoint 
             * @param[in] machine 
             * @param[in] tractor 
             * @param[in] SpacingPoint 
             * @return V2P 
             */
            V2P getLineCurvePoint(V2P EndPoint, Machine machine, Tractor tractor, double SpacingPoint);
            
            /**
             * @brief 采样点为边界点
             * @param[in] field_point 
             * @param[in] machine 
             * @param[in] tractor 
             * @return V2P 
             */
            V2P RoutigFeasible(V2P &field_point, Machine &machine, Tractor &tractor);
            
            /**
             * @brief 获取曲线(圆弧分割)点
             * @param[in] M1 
             * @param[in] M2 
             * @param[in] angle_theta 
             * @param[in] switch_num 
             * @param[in] Tangle 
             * @param[in] tractor 
             * @param[in] Back 
             * @param[in] ORI 
             * @return V2P 
             */
            V2P getinfoCur(Eigen::MatrixXf M1, Eigen::MatrixXf M2, double angle_theta, int switch_num, double Tangle, Tractor const &tractor, int Back, int ORI);
            V2P ABCDRouting(V2P field_point, PointXYZ location_point, Tractor tractor, Machine machine , double SpacingPoint);
            V2P CompleteLane(V2P field_point, V2P Endpoint, Tractor tractor, Machine machine);
            V2P NewLineCurvePoint(V2P EndPoint, V2P CompletePoint, Machine machine, Tractor tractor, double SpacingPoint, int type);
            std::vector<V2P> NewLineCurvePoint2(V2P EndPoint, V2P CompletePoint, Machine machine, Tractor tractor, double SpacingPoint, int type);
            LinearEquation LineparaTran(LinearEquation& lp, double width, bool tf);
            V2P LoopBackPlanning(const V2P &EndPoint);
            /**
             * @brief 
             * @param[in] Psize 
             * @param[in] endPoints 
             * @param[in] machine 
             * @param[in] tractor 
             * @param[in] maxLoop 可选配是否采用最大间距跨行
             * @return V2P 
             */
            V2P LoopBack(const int &Psize, V2P const &endPoints, Machine const& machine, Tractor const& tractor, int const &maxLoop);

            std::vector<int> getLoopBackIndex();
            std::vector<PointXYZ> recNode;
            std::vector<PointXYZ> getRecNode();


        };
    }//namespace routing
}//namespace bdr
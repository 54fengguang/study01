/***
 * 
 * 犁地作业规划
 * 时间：2021-11-29
 */ 
#pragma once
#include "common.h"
#include "segmentlane.h"
#include "routing.h"
#include "pearshape.h"
#include "fishtail.h"
#include "lanesplicing.h"
class plough
{
    private:
        bgy::common::Common common;
        bgy::common::math::Math math;
        bgy::routing::RoutingABCD routingabcd;
        bgy::routing::pear pe;
        bgy::routing::lane lanesp;
        fishtail ft;
        SegLane SL;

        std::vector<std::vector<PointXYZ> > GlobalNodeV; // 已经完成分割路径点
        std::vector<PointXYZ> LoopNode;
        std::vector<PointXYZ> extLoopNode;
        std::vector<PointXYZ> EndNode;
        std::vector<PointXYZ> GlobalNode; // 反路径可为种植路径
        std::vector<PointXYZ> testNode;
        std::vector<PointXYZ> newEndNode;
        std::vector<PointXYZ> fisLane;
        std::vector<PointXYZ> newLoopNode;
        std::vector<PointXYZ> addLoopNode; // 大梯形
        std::vector<PointXYZ> LoopNodeUp;
        std::vector<PointXYZ> LoopNodeDown;
        std::vector<PointXYZ> FieldNode;
        std::vector<PointXYZ> recordEndNode;

        LinearEquation LpAB;
        LinearEquation LpAC;
        LinearEquation LpBD;
        LinearEquation LpCD;

        LinearEquation LpAB_;
        LinearEquation LpAC_;
        LinearEquation LpBD_;
        LinearEquation LpCD_;
        LinearEquation tlp1p2;
        std::vector<PointXYZ> AC;
        std::vector<PointXYZ> BD;
        std::vector<PointXYZ> CD;
        std::vector<PointXYZ> AB;
        std::vector<PointXYZ> CD_;
        std::vector<PointXYZ> OptAddRightNode;
        std::vector<PointXYZ> OptAddleftNode;
        std::vector<int> symbol = {0, 0};
        PointXYZ tp1;
        PointXYZ tp2;
        PointXYZ p1;
        PointXYZ p2;
        PointXYZ machine_con1, machine_con2;
        bool AcceptOverlap;
        // bool leftOrright = false;
        int leftOrright = 1; // 0 不翻转, 1:左翻 -1:右翻
        bool change_AB = false;
        bool clockwise = true; //田块方向,
        int search_type;

        int type_seeding;
        Machine machine;
        Tractor tractor;
        double intersp;
        double safeDistance;
        double increaseValue;
        double m_rightOffset = 0.001;
        double m_leftOffset = 0.001;
        double offset1, offset2;
        int ErrType = 0; // 0 默认不报错， 1:参数错误
        int addlane; // 补充作业行,但不作业;
        int loop_k1; // 用于记录套圈数
        int loop_k2;
        int optAddK;
        int size_EndNode;
        int splicing_state = 0; // 0:未拼接, 1:已拼接
        int m_i = 0;
        int pos_nav_value = 0;
        int No = 0;
        int Ori = 1; // 确定当前航向的朝向默认北向
        /**
         * @brief 计算圈数
         */
        void calk();

        /**
         * @brief 查找回字作业圈数
         */
        void searchLoopNode();

        /**
         * @brief 查找带偏移的犁地作业直线点（梭行作业）
         */
        void searchStraightNodeOffset();

        /**
         * @brief 直线端拼接
         */
        void EndNodePlan();

        /**
         * @brief 
         */
        void EndNodeCheck();

        /**
         * @brief 直线点拼接优化
         */
        void EndNodeOpt();

        void LoopNodeGBTSPlan();

        void loopPlanGBTS();

        void FeasibleRegion();
        
        /**
         * @brief 
         */
        void ParamCheck();


        
        /**
         * @brief 
         */
        void LoopofKCheck();

        void loopOpt();

        void GlobalPlan();
    public:
        /**
         * @brief 
         * @param[in] FNode 
         * @param[in] locationNode 
         * @param[in] mec 
         * @param[in] tra 
         * @param[in] interSp 
         * @param[in] typeSeeding 
         * @param[in] ChangeOperation 
         * @param[in] AccOverlap 
         * @param[in] sd 
         * @return std::vector<std::vector<PointXYZ> > 
         */
        std::vector<std::vector<PointXYZ> > ploughPlan(std::vector<PointXYZ> &FNode, PointXYZ &locationNode, Machine &mac, Tractor &tra, const double &interSp, 
        int &typeSeeding, const bool &ChangeOperation, const bool &AccOverlap, const double &sd);

        double getMileage();

                /**
         * @brief Get the Rec Node object
         * @return std::vector<PointXYZ> 
         */
        std::vector<PointXYZ> getRecNode();
};
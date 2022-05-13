/**
 * @copyright Copyright (c) 2020 Bright Dream Robotics Co.Ltd
 * @file common.h
 * @brief 规划算法中频繁使用，且具有通用性的函数集合
 * @author tanglingmao (tanglingmao@countrygarden.com.cn)
 * @date 2020-06-30
 * @version 0.1.0
 */
#pragma once
#include <vector>
#include <cmath>
//#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdint>
#include <math.h>
// 进行Kd树查找

static const double XY = 10.0115;

//enum SEEDING{fishtail_seeding_GBTS = 1, pearshape_seeding_GBTS, loop_seeding_GBTS, fishtail_seeding, pearshape_seeding, loop_seeding}; //GBTS:go back to starting point
enum ErrorType
{
    ParametersCorrect = 0,
    ParametersError,
    ParametersMismatch,
    DividendException,
    VectorEmptyorCoreDumped,

};

enum SEEDING
{
    unloop = 0,
    unfishtail = 1,
    unpearshape = 2,
    loop_seeding_GBTS = 3,
    loop_seeding = 4,
    fishtail_seeding_GBTS,
    fishtail_seeding,
    pearshape_seeding_GBTS,
    pearshape_seeding,
    loop_harest = 10,
    offsetPearshape = 12,
    SingleoffsetPearshape = 13,
    offsetPearshape_GBTS = 14,
    offsetFishtail_GBTS = 15,
};
enum ROUTETYPE
{
    loop,
    fish,
    pearshape
};
enum FARMIMPOPE
{
    HangUp,
    HangDown,
    PtoUp,
    PtoDown
};
struct FarmImplementOpDis
{
    FarmImplementOpDis(float HangUpc = 0.0, float HangDownc = 0.0, float PtoOnc = 0.0, float PtoOffc = 0.0) : HangUpDis(HangUpc), HangDownDis(HangDownc), PtoOnDis(PtoOnc), PtoOffDis(PtoOffc) {}
    float HangUpDis;
    float HangDownDis;
    float PtoOnDis;
    float PtoOffDis;
};

struct Obstacle
{
    double x;
    double y;
    double z;
    double r;   // 障碍物实际半径
    double r_w; // 运算半径
    int taskIndex; // 田块index;
};

struct LinearEquation
{
    double k; // 斜率
    double b; // 截距
    double x; // 用于表示 x = 常量，方程
    double y; // 用于表示 y = 常量，方程
};

struct PointXYZ
{
    PointXYZ(double xc = 0.0, double yc = 0.0, double zc = -1.00, double yawc = 0.0, double vc = 0.1,
             double sc = 0.0, double scur = -1.0, int Backc = 0, int StrCurc = 0, bool reversec = false, int typec = -1)
        : x(xc), y(yc), z(zc), yaw(yawc), v(vc), s(sc), cur(scur), Back(Backc), StrCur(StrCurc), reverse(reversec), type(typec) {}
    double x;   // 横坐标
    double y;   // 纵坐标
    double z;   // 高程
    double yaw; // 航向角
    double v;   // 速度
    double s;   // s-d-t坐标系
    double cur; // curvature 曲率
    int Back;   // 主要用于鱼尾 0 false 1 true或用于pearshape中的0:优化端, 1:非优化端
    int StrCur; // 1:直线作业点, 0 :非作业直线点
    bool reverse;
    int type; //0 直线点,1鱼尾点,2梨形点
};

struct Tractor
{
    //Tractor(double radiusc = 0.0, double lenghtc = 0.0, int switchGearc = 1, int typec = -1, bool reversec = false) : radius(radiusc), lenght(lenghtc), switchGear(switchGearc), type(typec), reverse(reversec){}
    double radius;  // 最小转弯半径
    double lenght;  // 车与机具整体长度;
    bool reverse;              // 是否具有倒车功能 -1 无倒车功能, 1 有倒车功能
    double overallWidth;       // 车体总体宽度
    double frontEndtoLocation; // 后轮轴到车体前端之间的间距
    double rearEndtoLocation;  // 后轮轴到车体后端之间的间距(如果加载农机具则为到农机具端之间的距离)
    double wheelBase;          // 轴距
    int type;                  
    int locationPoint;         // 本体定位点所在位置
    int hitchType; // 1：半悬挂 0：悬挂 2:机具位于前方
    int locaP;
    // std::string data;
};

struct Machine
{
    Machine(double widthc = 0.0, double rightOffsetc = 0.0, double leftOffsetc = 0.0) : width(widthc), rightOffset(rightOffsetc), leftOffset(leftOffsetc) {}
    double width;       // 机具幅宽
    double lenght;      // 机具长度
    double rightOffset; // 机具右偏移量
    double leftOffset;  // 机具左偏移量
};

struct ShiftDatum
{
    ShiftDatum(int typec = -1, double rightOffsetc = -0.0, double leftOffsetc = -0.0) : type(typec), leftOffset(leftOffsetc), rightOffset(rightOffsetc) {}
    int8_t type;        // 平移模式: 1 手动、 2 自动
    double rightOffset; // 平移右偏移量
    double leftOffset;  // 平移左偏移量
};

struct Circleparam
{
    double x = 0; // 平面圆心横坐标
    double y = 0; // 平面圆心纵坐标
    double r = 0; // 圆心半径
    double curvature = 0;
};

/**
 * @brief 系统运行姿态， Start Stop Pause Continue Restart 
 */
struct SystemState
{
    SystemState(bool startc = false, bool stopc = false, bool pausec = false, bool Continuec = false, bool restartc = false) : start(startc), stop(stopc), pause(pausec), Continue(Continuec), restart(restartc) {}
    bool start;    // 开始 true flase
    bool stop;     // 停止
    bool pause;    // 暂停
    bool Continue; // 继续
    bool restart;  // 重启
};

using V2d = std::vector<double>;
using V2P = std::vector<PointXYZ>;
struct PubDataParam
{
    std::vector<std::vector<PointXYZ> > p;
    PointXYZ location;
    int statePose; // 1:pose信息可用, 0:pose信息不可用
    double DisDown;
    double DIsUp;
};

struct FieldInfo
{
    std::vector<PointXYZ> fieldNodes;
    std::vector<Obstacle> obstacles;
    PointXYZ LocaNode;
    int taskIndex;
};

namespace bgy
{
    namespace common
    {
        namespace math
        {
            class Math
            {
            public:
                //bgy::common::Common m_common;
                LinearEquation LineparaTran(LinearEquation &lpt, double width, bool t);
                /**
                 * @brief 用于拼接非平行作业行,
                 * @param[in] p 作业行坐标点
                 * @param[in] machine 
                 * @param[in] tractor 
                 * @param[in] intersp 
                 * @return std::vector<PointXYZ> 包含了输入平行行坐标,视情况去除
                 */
                std::vector<PointXYZ> Lanesplicing(std::vector<PointXYZ> &p, Machine &machine, Tractor const &tractor, double const &intersp);

                /**
                 * @brief 用于平行直线拼接
                 */
                std::vector<PointXYZ> LanesParallelsplicing(std::vector<PointXYZ> &p, Machine & machine, Tractor const &tractor, double const &intersp);

                std::vector<PointXYZ> getCur(Eigen::MatrixXf M1, Eigen::MatrixXf M2, double angle_theta, int switch_num, double Tangle, Tractor const &tractor, int back, bool ori);

                /**
                 * @brief 求解一元二次方程解
                 * @param[in] a 
                 * @param[in] b 
                 * @param[in] c 
                 * @return std::vector<double> 空为误解,  一个解, 两个解
                 */
                std::vector<double> Quadratic(double const &a, double const &b, double const &c);

                /**
                 * @brief Bezier 曲线
                 * @param[in] p 
                 * @return std::vector<PointXYZ> 
                 */
                std::vector<PointXYZ> BezierCurve(const std::vector<PointXYZ> &p, const int &SizeOfBezierCurve);

                /** 用于判断点是否在给定线段内
                 * @brief 
                 * @param[in] lineStart 
                 * @param[in] lineEnd 
                 * @param[in] Point 
                 * @return true 
                 * @return false 
                 */
                bool dotProduct(const PointXYZ &lineStart, const PointXYZ &lineEnd, const PointXYZ &Point);

                

                double VectorAngle(const PointXYZ &V1, const PointXYZ &V2);
                /**
                 * @brief 获取点到点的距离
                 * @param[in] p1 
                 * @param[in] p2 
                 * @return double 
                 */
                double DistPoint(const PointXYZ &p1, const PointXYZ &p2);

                /**
                 * @brief 获取点到直线的距离
                 * @param[in] p1 
                 * @param[in] lp 
                 * @return double 
                 */
                double DistPoint2Line(const PointXYZ &p1, const LinearEquation &lp);

                bool Angle_HeadingLane(PointXYZ &Loaction_point, const LinearEquation &lp);
                /**
                 * @brief 获取直线方程
                 * @param[in] 直线坐标点p1 
                 * @param[in] 直线坐标点p2 
                 * @return LinearEquation 
                 */
                LinearEquation getLine(const PointXYZ &p1, const PointXYZ &p2);

                /**
                 * @brief 获取半径大小为R的圆的圆心坐标
                 * @param[in] p1 
                 * @param[in] p2 
                 * @param[in] R 设定的转弯半径 
                 * @return std::vector<PointXYZ> 
                 */
                std::vector<PointXYZ> CircleCenter(const PointXYZ &p1, const PointXYZ &p2, const double &R);

                /**
                 * @brief 获取两直线交点
                 * @param[in] lp1 
                 * @param[in] lp2 
                 * @return PointXYZ 
                 */
                PointXYZ IntersectionNode(const LinearEquation &lp1, const LinearEquation &lp2);

                /**
                 * @brief 用于按length要求,进行向量方向上的点选取
                 * @param[in] p 向量的头尾坐标点
                 * @param[in] length 
                 * @return PointXYZ 
                 */
                PointXYZ FeaturePointSelofVec(const std::vector<PointXYZ> &p, const double &length);

                double CalKappa(const std::vector<PointXYZ> &p);

                Circleparam CalKappaCenter(const std::vector<PointXYZ> &p);

                double CalYaw(const PointXYZ &p1, const PointXYZ &p2);

                double CalYaw(const std::vector<PointXYZ> &p);
                /**
                 * @brief 用于计算固定长度直线与圆相切时,直线离圆最远端点到设定参考直线的距离
                 * @param[in] r 
                 * @param[in] l 
                 * @param[in] instersp 
                 * @return double 
                 */
                double CalMaxDis(double r, double l, double intersp);
                /**
                 * 判断直线段是否为平行直线
                 */
                bool LinesParallel(LinearEquation &lp1, LinearEquation &lp2);
                void checkError(double data);
                int returnStatus();

            private:
                int ErrorStatus = 0;
            };
        } // namespace math

        class Common
        {
        public:
            bgy::common::math::Math math;

            /**
             * @brief 
             * @param[in] point 
             * @param[in] interpRes 
             * @return std::vector<PointXYZ> 
             */
            std::vector<PointXYZ> LanePoint(const std::vector<PointXYZ> &point, const double &interpRes, const bool &strcur, const int &back);

            /**
             * @brief 
             * @param[in] point 
             * @param[in] interpRes 
             * @param[in] Backward 
             * @param[in] extendType 
             * @return std::vector<PointXYZ> 
             */
            std::vector<PointXYZ> LanePoint(const std::vector<PointXYZ> &point, const double &interpRes, const int &Backward, const int &extendType, const double &extlength);

            std::vector<PointXYZ> LanePoint(const std::vector<PointXYZ> &point, const double &interpRes, const bool &strcur, const bool &back);
            /**
             * @brief 不进行插值,只进行点属性修改
             * @param[in] p 需待修改点
             * @param[bool] strcur 判断点是否为作业直线点
             * @param[bool] reverse 速度正负  false:倒退, true:前进
             * @return std::vector<PointXYZ> 
             */
            std::vector<PointXYZ> ModifyPointFeatures(const std::vector<PointXYZ> &p, const bool &strcur, const bool &reverse);

            /**
             * @brief 
             * @param[in] P 
             * @param[in] lp 
             * @return PointXYZ 
             */
            PointXYZ NearestPoint(const PointXYZ &P, const LinearEquation &lp);

            /**
             * @brief 
             * @param[in] field_point 
             * @param[in] M 
             * @return double 
             */
            double WidthTranslation(const std::vector<PointXYZ> &field_point,const double &width);

            /**
             * @brief 
             * @param[in] point 
             * @param[in] lp 
             * @param[in] machine 
             * @return std::vector<PointXYZ> 
             */
            std::vector<PointXYZ> ReferenceLane(const std::vector<PointXYZ> &point, const LinearEquation &lp, const Machine &machine);

            /**
             * @brief 判断直线外一点在直线的上方(左边) 或 下方（右边） 
             * @param[in] vp 
             * @param[in] p 
             * @return true 上方(左边)
             * @return false 下方(右边)
             */
            bool StraightBelow(const std::vector<PointXYZ> &vp, const PointXYZ &p);
            /**
             *@brief 检测是否除数为0/空 
             * 
             */
            void checkError(double data);

            
            int returnStatus();

            private:
                int ErrorStatus = 0;
        };
    } // namespace common
} // namespace bgy

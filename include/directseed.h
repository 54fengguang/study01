#pragma once
#include "common.h"
#include "routing.h"
#include "pearshape.h"
#include "lanesplicing.h"
#include "segmentlane.h"
#include "sortopt.h"
#include "bypass.h"



namespace bgy
{
     namespace routing
     {
          namespace directseeding
          {
               class water
               {
               public:
                  void ParamInit();
                    /**
                 * @brief 计算田头行数
                 */
                    void calK();

                  /**
                   * 
                   */ 
                  void calk();

                    /**
                 * @brief 查找田头作业区域坐标点
                 */
                    void searchLoopNode();

                    /**
                 * @brief 可行区域作业路径查找
                 */
                    void FeasibleRegion();
                    /**
                 * @brief 查找直线作业区域中的直线端点
                 */
                    void searchStraightNode();
                  /**
                   * @brief 
                   */
                  void searchStraightNodeOffset();
                    /**
                 * @brief 直线作业端点路径设计优化
                 */
                    void EndNodeOpt();

                    /**
                 * @brief End Node plan
                 */
                    void EndNodePlan();

                    double getMileage();

                    /**
                 * @brief 
                 */
                    void LoopNodeGBTSPlan();

                    /**
                 * @brief 
                 */
                    void loopNodePlan();

                    /**
                 * @brief 根据作业类型更新LoopNode
                 */
                    void updateLoopNode();

                    /**
                 * @brief 路径拼接
                 */
                    void GlobalPlan();


                  /**
                   * @brief loop优化
                   */
                   void loopOpt(); 

                    

                  /**
                   *ErrType = 1 参数异常检测
                   */
                    void ParamCheck();

                  /**
                   *ErrType = 2 端点异常
                   */
                    void EndNodeCheck(); 

                  /**
                   * ErrType = 3 田块尺寸初步检测
                   */ 
                  void LoopofKCheck();

                  std::vector<PointXYZ> getRecNode();

   void addRoadplan();
/**
 * 鱼尾回字
 */ 
   void loopPlanGBTS();

                  std::vector<PointXYZ> getLoopNode();

                  /**
                   * @brief 鱼尾收边
                   */
                  void LoopNodeObsfish();

                  /**
                   * @brief 套圈收边
                   */
                  void LoopNodeGBTSPlanbos();

                  /**
                   * ErrType = 3 分母为0 或越界
                   */ 
                  

                    V2P getGlobalNode();


                    std::vector<PointXYZ> getFiledNode();
               //      /**
               //   * @brief 播种路径规划启动与数据初始化函数
               //   * @param[in] FNode 田块4坐标
               //   * @param[in] locationNode 入机口
               //   * @param[in] machine 机具参数
               //   * @param[in] tractor 拖拉机参数
               //   * @param[in] intersp 点间距
               //   * @param[in] type_seeding 播种作业路径选择类型
               //   * @param[in] ChangeOperation 是否调换作业航向
               //   * @param[in] AccOverlap 是否可接受路径重覆盖
               //   * @param[in] sd  SafeDistance设置离边安全距离值
               //   */
               //      std::vector<std::vector<PointXYZ> > runDirseedPlan(std::vector<PointXYZ> &FNode, PointXYZ &locationNode, Machine &machine, Tractor &tractor, const double &intersp,
               //                                                        int &type_seeding, const bool &ChangeOperation, const bool &AccOverlap, const double &sd);
                std::vector<std::vector<PointXYZ> > runDirseedPlan(std::vector<PointXYZ> &FNode, PointXYZ &locationNode, 
                            std::vector<Obstacle> &obstacles, Machine &machine, Tractor &tractor, const double &intersp,
                                                                      int &type_seeding, const bool &ChangeOperation, const bool &AccOverlap, const double &sd);

               private:
                    bgy::common::Common common;
                    bgy::common::math::Math math;
                    bgy::routing::RoutingABCD routingabcd;
                    bgy::routing::pear pe;
                    bgy::routing::lane lanesp;
                    fishtail ft;
                    SegLane SL;
                    sortopt sopt;
                    haoxing::routing::planbypass m_planfishbypass;
                    haoxing::routing::bypass m_bypass;

                    LinearEquation LpAB;
                    LinearEquation LpAC;
                    LinearEquation LpBD;
                    LinearEquation LpCD;

                    LinearEquation LpAB_;
                    LinearEquation LpAC_;
                    LinearEquation LpBD_;
                    LinearEquation LpCD_;
                    std::vector<PointXYZ> AC;
                    std::vector<PointXYZ> BD;
                    std::vector<PointXYZ> CD;
                    std::vector<PointXYZ> AB;
                    std::vector<PointXYZ> CD_;

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

                    std::vector<Obstacle> obstacle;

                    int typeProtection = 0;

                    bool leftOrright = false; //left : false; right: true;

                    V2P OptAddRightNode;
                    V2P OptAddleftNode;

                    Machine machine;
                    Tractor tractor;

                    PointXYZ tp1;
                    PointXYZ tp2;
                    PointXYZ p1;
                    PointXYZ p2;

                    LinearEquation tlp1p2;

                    bool change_AB;
                    bool AcceptOverlap;

                    int search_type;
                    int size_short_lane;
                    int k; // 开田头圈数
                    int kk;// loop_harvest 开田头数
                    int add_k;
                    int size_EndNode;
                    int loopadd; // 
                    int type_seeding;
                    int histInterStatus;
                    int histObsIndex;
                    int m_type; // 测试 使用

                    std::vector<int> symbol = {0, 0}; // 1:+ -1:-

                    double safeDistance;
                    double intersp;
                    double mileage;
                    double increaseValue; //用于记录延长了多少的

                    int ErrType; // 0 默认不报错， 1:参数错误
                    int addlane; // 补充作业行,但不作业;
                    int splicing_state = 0; // 0:未拼接, 1:已拼接
                    int m_i = 0;
                    int optAddK;

               };
          }
     }
}

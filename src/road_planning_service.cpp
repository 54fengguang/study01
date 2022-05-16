#include <iostream>
#include <vector>
#include <fstream>
#include "road_planning_service.h"
#include "run.h"

using namespace std;

namespace haoxing
{
    namespace route2
    {
        RoadPlanningService::RoadPlanningService(){
            cout << "RoadPlanningService 构造函数被调用!!" << endl;
        }


        RoadPlanningService::~RoadPlanningService(){
            cout << "RoadPlanningService 析构函数被调用！！！" << endl;
        }

        
        void RoadPlanningService::running(vector<PointXYZ> &m_FNode, vector<Obstacle> &obstacles, PointXYZ *locationNode,
                             Machine *mac, Tractor *tra, double interSp, double typeSeeding, double ChangeOperation, 
                             double AccOverlap, double sd){
            cout << "RoadPlanningService的running函数被调用！！"  << endl;
          
            ::routing::running(m_FNode,obstacles,locationNode,mac,
                    tra,interSp,typeSeeding,ChangeOperation,AccOverlap,sd);
        }

        vector<PointXYZ> RoadPlanningService::getNode(){
            cout << "RoadPlanningService 的getNode节点被调用！！"  << endl;
            vector<PointXYZ>  pointXYZ_list = ::routing::getNode();
            return pointXYZ_list;
        }

    } // namespace route2
    
} // namespace haoxing

int main(int argc, char *argv[])
{
    haoxing::route2::RoadPlanningService  roadPlanningService;
    
    //田块坐标信息
    vector<PointXYZ> m_FNode;   
    PointXYZ pointXYZ_1;
    pointXYZ_1.x = 690726.125386399;
    pointXYZ_1.y = 2605174.532767363;
    m_FNode.push_back(pointXYZ_1);

    PointXYZ pointXYZ_2;
    pointXYZ_2.x = 690653.014657597;
    pointXYZ_2.y = 2605051.614422578;
    m_FNode.push_back(pointXYZ_2);

    PointXYZ pointXYZ_3;
    pointXYZ_3.x = 690600.054305037;
    pointXYZ_3.y = 2605082.703084111;
    m_FNode.push_back(pointXYZ_3);

    PointXYZ pointXYZ_4;
    pointXYZ_4.x = 690673.126189362;
    pointXYZ_4.y = 2605203.974061783;
    m_FNode.push_back(pointXYZ_4);



    //障碍物信息
    vector<Obstacle> obstacles; 


    PointXYZ locationNode;     //作业规划起始点坐标
    locationNode.x = 690601.759127825;
    locationNode.y = 2605081.70231815;

    //机具参数信息
    Machine mac;  
    mac.width = 2.3;       // 机具幅宽
    mac.lenght = 1.0;      // 机具长度
    mac.rightOffset = 0.001; // 机具右偏移量
    mac.leftOffset = 0.001;  // 机具左偏移量 

    /**
    "machineDto": {
		"frontEndToRearAxle": 1.7,
		"hitchType": 0,
		"length": 4.2,
		"locationPoint": -1,
		"mountType": 1,
		"overAllWidth": 6.0,
		"radius": 7.0,
		"rearEndToRearAxle": 1.7,
		"reverse": 1,
		"wheelBase": 2.4,
		"width": 2.16
	}
    **/
    //拖拉机参数信息
    Tractor tra;              
    tra.radius = 7.0;               // 最小转弯半径
    tra.lenght = 4.2;               // 车与机具整体长度;
    tra.reverse = 1;                // 是否具有倒车功能 -1 无倒车功能, 1 有倒车功能
    tra.overallWidth = 6.0;         // 车体总体宽度
    tra.frontEndtoLocation = 1.7;   // 后轮轴到车体前端之间的间距
    tra.rearEndtoLocation = 1.7;    // 后轮轴到车体后端之间的间距(如果加载农机具则为到农机具端之间的距离)
    tra.wheelBase = 2.4;            // 轴距
    tra.type;                  
    tra.locationPoint = -1 ;        // 本体定位点所在位置
    tra.hitchType  = 0;             // 1：半悬挂 0：悬挂 2:机具位于前方
    tra.locaP;


    double interSp = 0.1;               //点间距
    double typeSeeding = 1;                 //播种作业路径选择类型
    double ChangeOperation = -1;        //是否调换作业航向
    double AccOverlap = 1;              //是否接受路径重覆盖
    double sd = 0.5;                    //设置高边安全距离值

    roadPlanningService.running(m_FNode,obstacles,&locationNode,&mac,
                    &tra,interSp,typeSeeding,ChangeOperation,AccOverlap,sd);

    vector<PointXYZ>  path_points = roadPlanningService.getNode();
    if(!path_points.empty()){
        cout << "路径规划参数为不空！！" << endl;
        int count = 0;

        ofstream out("route.txt");
        if (out.is_open()) {
            for (vector<PointXYZ>::iterator iter = path_points.begin(); iter != path_points.end();iter++){
                cout << "第" << count << "个坐标点：" << (*iter).x << "," << (*iter).y ;
                cout << "  速度：" << (*iter).v << "航向角：" << (*iter).yaw << endl;
                count++;

                out << (*iter).x << "\t" << (*iter).y << "\t" << 1 << "\t" << 0 << "\n";
            }
        }
        out.close();
    }else{
        cout << "路径规划参数为空！！" << endl;
    }

    return 0;
}


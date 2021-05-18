#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#define pi 3.1415926535897932384626433832795
//下标宏定义 状态[x(m), y(m), yaw(Rad), v(m / s), w(rad / s)]
#define POSE_X          0    // 坐标 X
#define POSE_Y          1    // 坐标 Y
#define YAW_ANGLE       2    // 机器人航向角
#define V_SPD           3    // 机器人速度
#define W_ANGLE_SPD     4    // 机器人角速度
//定义Kinematic的下标含义
#define MD_MAX_V        0    // 最高速度m / s]
#define MD_MAX_W        1    // 最高旋转速度[rad / s]
#define MD_ACC          2    // 加速度[m / ss]
#define MD_VW           3    // 旋转加速度[rad / ss]
#define MD_V_RESOLUTION 4    // 速度分辨率[m / s]
#define MD_W_RESOLUTION 5    // 转速分辨率[rad / s]]

struct state{
	float x;
  float y;
  float yaw;
  float velocity;
  float angular;
};

struct controlU{
  float vt;
  float wt;
};

struct maxmotion{
  float minvel;
  float maxvel;
  float minang;
  float maxang;
};

struct eval_db{
  float vt;
  float wt;
  float heading;
  float dist;
  float vel;
  float feval;
	state pose;
};

float dt = 0.1;//时间[s]`在这里插入代码片`

class DWA
{
public:
	  DWA();
		~DWA();
    void robot2world(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y);
    void world2robot(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y);
    controlU DynamicWindowApproach(state cx, std::vector<state> *TrajDb, float *model, Eigen::Vector3f goal, float * evalParam, std::vector<Eigen::Vector3f> ob, std::vector<Eigen::Vector3f> robot_ob, float R, std::vector<Eigen::Vector3f> &dwa_predict_pose, Eigen::Vector3f &dwa_choose_pose);
    state CarState(state cx, controlU u);
    maxmotion CalcDynamicWindow(state cx, float *model);
    void Evaluation(state cx, std::vector<eval_db> *EvalDb, std::vector<state> *TrajDb, maxmotion Vr, Eigen::Vector3f goal, std::vector<Eigen::Vector3f> ob, std::vector<Eigen::Vector3f> robot_ob, float R, float *model, float predict_time);
    state GenerateTrajectory(state cx, std::vector<state> *traj, float vt, float wt, float predict_time, float *model);
    float CalcHeadingEval(state cx, Eigen::Vector3f goal);
    float CalcDistEval(state cx, std::vector<Eigen::Vector3f> ob, float R);
    void NormalizeEval(std::vector<eval_db> *EvalDb);
		bool no_path;
};

DWA::DWA()
{
		no_path = false;
}

DWA::~DWA()
{

}

void DWA::robot2world(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y)
{
		float x_world = target_pos.x()*cos(robot_pos.z() ) - target_pos.y()*sin(robot_pos.z() );
	  float y_world = target_pos.x()*sin(robot_pos.z() ) + target_pos.y()*cos(robot_pos.z() );
		x = x_world + robot_pos.x();
		y = y_world + robot_pos.y();
}

void DWA::world2robot(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y)
{
		float x_error_world = target_pos.x() - robot_pos.x();
	  float y_error_world = target_pos.y() - robot_pos.y();
	  float x_error_robot = x_error_world*cos(-1*robot_pos.z()) - y_error_world*sin(-1*robot_pos.z());
	  float y_error_robot = x_error_world*sin(-1*robot_pos.z()) + y_error_world*cos(-1*robot_pos.z());
		x = x_error_robot;
		y = y_error_robot;
}

controlU DWA::DynamicWindowApproach(state cx, std::vector<state> *TrajDb, float *model, Eigen::Vector3f goal, float * evalParam, std::vector<Eigen::Vector3f> ob, std::vector<Eigen::Vector3f> robot_ob, float R, std::vector<Eigen::Vector3f> &dwa_predict_pose, Eigen::Vector3f &dwa_choose_pose)
{
    controlU u;
    std::vector<eval_db> EvalDb;
    maxmotion cvr = CalcDynamicWindow(cx, model); //根据当前状态 和 运动模型 计算当前的参数允许范围
		if(cvr.minvel > cvr.maxvel)
		{
				float ve = cvr.minvel;
				cvr.minvel = cvr.maxvel;
				cvr.maxvel = ve;
		}
    Evaluation(cx, &EvalDb, TrajDb, cvr, goal, ob, robot_ob, R, model, evalParam[3]);  //predict_dis = evalParam(4) 评价函数参数[heading, dist, velocity, predictDistance]
    if (EvalDb.empty())
		{
				std::cout << "no path to goal!!" << std::endl;
				no_path = true;
				u.vt = 0;
				u.wt = 0;
				return u;
		}
    NormalizeEval(&EvalDb);//各评价函数正则化
    //最终评价函数的计算float heading;float dist;float vel;
		for (int i = 0; i < EvalDb.size(); ++i)
		{
				EvalDb.at(i).feval = evalParam[0] * EvalDb.at(i).heading + evalParam[1] * EvalDb.at(i).dist + evalParam[2] * EvalDb.at(i).vel;//根据评价函数参数 前三个参数分配的权重 计算每一组可用的路径参数信息的得分
				Eigen::Vector3f pose_buf(EvalDb[i].pose.x, EvalDb[i].pose.y, 0.0);
				dwa_predict_pose.push_back(pose_buf);
		}
    float maxheading = EvalDb.at(0).feval;
		float idx = 0;
		for (int i = 0; i < EvalDb.size(); ++i)
		{
				if (maxheading < EvalDb.at(i).feval)
				{
						maxheading = EvalDb.at(i).feval;
						idx = i;
				}
		}
	  u.vt = EvalDb.at(idx).vt;
		u.wt = EvalDb.at(idx).wt;
    /////////////////////////存取選擇的點
		dwa_choose_pose.x() = EvalDb[idx].pose.x;
		dwa_choose_pose.y() = EvalDb[idx].pose.y;
		dwa_choose_pose.z() = 0.0;
		/////////////////////////存取選擇的點
		EvalDb.clear();
		return u;
}

state DWA::CarState(state cx, controlU u)
{
    state result;
		result.x = cx.x + dt * cos(cx.yaw)*u.vt;
		result.y = cx.y + dt * sin(cx.yaw)*u.vt;
    result.yaw = cx.yaw + dt * u.wt;
    result.velocity = u.vt;
    result.angular = u.wt;
    return result;
}

maxmotion DWA::CalcDynamicWindow(state cx, float *model)
{
    maxmotion V_r;
		V_r.minvel = std::max(0.0f, cx.velocity - model[MD_ACC] * dt);
    V_r.maxvel = std::min(model[MD_MAX_V], cx.velocity + model[MD_ACC] * dt);
    V_r.minang = std::max(-model[MD_MAX_W], cx.angular - model[MD_VW] * dt);
    V_r.maxang = std::min(model[MD_MAX_W], cx.angular + model[MD_VW] * dt);
    return V_r;
}

void DWA::Evaluation(state cx, std::vector<eval_db> *EvalDb, std::vector<state> *TrajDb, maxmotion Vr, Eigen::Vector3f goal, std::vector<Eigen::Vector3f> ob, std::vector<Eigen::Vector3f> robot_ob, float R, float *model, float predict_time)
{
    EvalDb->clear();
    TrajDb->clear();
    std::vector<state> traj;
    for (float vt = Vr.minvel; vt <= Vr.maxvel; vt = vt + model[4])//根据速度分辨率遍历所有可用速度： 最小速度和最大速度 之间 速度分辨率 递增
    {
      	for (float wt = -0.5; wt <= 0.5; wt = wt + model[5])//根据角度分辨率遍历所有可用角速度： 最小角速度和最大角速度 之间 角度分辨率 递增
        {
        		//轨迹推测; 得到 xt : 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹（由轨迹点组成）
        		state xt = GenerateTrajectory(cx, &traj, vt, wt, predict_time, model); //predict_dis = evalParam(4), 前向模拟距
						//各评价函数的计算
        		float heading = CalcHeadingEval(xt, goal);//前项预测终点的航向得分  偏差越小分数越高 w1
        		float dist = CalcDistEval(xt, ob, R);//前项预测终点 距离最近障碍物的间隙得分 距离越远分数越高 w2
						float vel = fabs(vt);//速度得分 速度越快分越高 w3
						float feval = 0.0;
						eval_db db = { vt,wt,heading,dist,vel,feval,xt};
						if (dist > 1.2 * R)
						{
								EvalDb->push_back(db);
						}
        }
    }
		traj.clear();
}

state DWA::GenerateTrajectory(state cx, std::vector<state> *traj, float vt, float wt, float predict_time, float *model)
{
    float time = 0.0;
    controlU u = { vt, wt };
    traj->clear();
    traj->push_back(cx);
    state px = cx;

    while ((int)time < (int)predict_time*10)
    {
        time = time + 10*dt;
        px = CarState(px, u);
        traj->push_back(px);
    }
    return px;
}

float DWA::CalcHeadingEval(state cx, Eigen::Vector3f goal)
{
    float theta = cx.yaw; //机器人朝向
		float goalTheta = atan2(goal.y() - cx.y, goal.x() - cx.x);//目标点相对于机器人本身的方位
    float targetTheta;
		targetTheta = fabs(goalTheta - theta);
		if(targetTheta > M_PI)targetTheta = 2*M_PI - targetTheta;
    return M_PI - targetTheta;
}

float DWA::CalcDistEval(state cx, std::vector<Eigen::Vector3f> ob, float R)
{
    float dist = 100.0;
		int idx = 0;
    for (int i = 0; i < ob.size(); ++i)
    {
        //到第i个障碍物的距离 - 障碍物半径
        float disttmp = sqrt((ob[i].x() - cx.x)*(ob[i].x() - cx.x) + (ob[i].y() - cx.y)*(ob[i].y() - cx.y));
        if (dist > disttmp)//大于最小值 则选择最小值
				{
						dist = disttmp;
						idx = i;
				}
    }
    // if (dist >= 2.5 * R)
    //    dist = 2.5 * R;
    return dist;
}

void DWA::NormalizeEval(std::vector<eval_db> *EvalDb)
{
    //评价函数正则化
    float sum3 = 0, sum4 = 0, sum5 = 0;
    for (int i = 0; i < EvalDb->size(); ++i)
    {
        sum3 += EvalDb->at(i).heading;
        sum4 += EvalDb->at(i).dist;
        sum5 += EvalDb->at(i).vel;
    }
    if (sum3 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).heading = EvalDb->at(i).heading / sum3;
    }
    if (sum4 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).dist = EvalDb->at(i).dist / sum4;
    }
    if (sum5 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).vel = EvalDb->at(i).vel / sum5;
    }
}

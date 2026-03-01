/*
 *	FILE: astarOccMap.h
 *	---------------------------
 *	Risk-aware A* path planner based on occupancy map.
 *	Based on "Risk-Aware Planner for Quadrotor in Cluttered and Dynamic Environments"
 *	- Static risk: Rs(q) = max_{om, ||q-om||<=ds} (||q-om|| - ds)^2
 *	- Dynamic risk: Rd(q) from predicted obstacle positions (Gaussian-based)
 *	- Cost: g(n) = path_cost + risk, h(n) = h_dist + eta * h_risk
 */

#ifndef ASTAROCCMAP_H
#define ASTAROCCMAP_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <map_manager/occupancyMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <unordered_map>
#include <functional>
#include <vector>

using std::cout;
using std::endl;

namespace globalPlanner{

struct AStarNode{
	Eigen::Vector3i index;
	double gScore;
	double fScore;
	AStarNode* cameFrom;
	enum State { OPEN, CLOSED, UNDEFINED };
	State state;

	AStarNode() : gScore(1e10), fScore(1e10), cameFrom(nullptr), state(UNDEFINED) {}
};

struct IndexHash{
	size_t operator()(const Eigen::Vector3i& idx) const {
		return ((size_t)idx(0) * 73856093u) ^ ((size_t)idx(1) * 19349663u) ^ ((size_t)idx(2) * 83492791u);
	}
};

struct NodeComparator{
	bool operator()(AStarNode* a, AStarNode* b) const {
		return a->fScore > b->fScore;
	}
};

class astarOccMap{
protected:
	ros::NodeHandle nh_;
	ros::Publisher astarVisPub_;
	ros::Timer visTimer_;
	std::shared_ptr<mapManager::occMap> map_;

	Eigen::Vector3d start_;
	Eigen::Vector3d goal_;
	std::vector<Eigen::Vector3d> currPlan_;
	std::vector<Eigen::Vector3d> riskFreePlan_;  // path without risk cost (for visualization)

	bool dualPathSearch_;  // when true, search both risk-aware and risk-free paths
	ros::Publisher astarRiskAwarePathPub_;   // nav_msgs::Path
	ros::Publisher astarRiskFreePathPub_;    // nav_msgs::Path

	bool riskMapVis_;  // publish risk map for RViz visualization (2D XY slice)
	ros::Publisher riskMapPub_;  // sensor_msgs::PointCloud2
	double riskMapRes_;   // sampling resolution for risk map (m)
	double riskMapRange_; // half side of XY region (m)
	double riskMapHeight_; // fixed height for 2D slice (m)
	double riskMapMinDisplay_; // only display points with risk above this (hide zero-cost regions)
	double riskMapMinRisk_; // min risk to display, below this = no display (hide zero-cost regions)

	double mapRes_;
	double timeout_;
	double maxShortcutThresh_;
	bool ignoreUnknown_;
	bool passGoalCheck_;

	Eigen::Vector3d mapSizeMin_;
	Eigen::Vector3d mapSizeMax_;
	Eigen::Vector3i mapVoxelMin_;
	Eigen::Vector3i mapVoxelMax_;

	std::unordered_map<Eigen::Vector3i, AStarNode, IndexHash> nodeMap_;
	std::priority_queue<AStarNode*, std::vector<AStarNode*>, NodeComparator> openSet_;

	// Risk-aware planning (paper: Risk-Aware Planner for Quadrotor...)
	bool riskAware_;
	double riskDangerZone_;      // ds: safety margin for static risk
	double riskGammaStatic_;     // gamma_s: weight for static risk
	double riskGammaDynamic_;    // gamma_d: weight for dynamic risk
	double riskEta_;             // eta: risk heuristic weight in h(n)
	double riskWeight_;         // weight for risk in g(n)
	double riskPredHorizon_;     // prediction time horizon for dynamic obstacles (s)
	int riskPredSteps_;         // number of prediction time steps
	std::vector<Eigen::Vector3d> dynObstaclesPos_;
	std::vector<Eigen::Vector3d> dynObstaclesVel_;
	std::vector<Eigen::Vector3d> dynObstaclesSize_;

	// Intent-based risk (paper: 基于多模态意图的风险地图构建)
	// When non-empty, use Rd(q)=Σ_i Σ_I P(I) Σ_k ω_k r_{i,I}(q,k) with anisotropic kernel
	bool useIntentRisk_;
	std::vector<std::vector<std::vector<Eigen::Vector3d>>> dynIntentPredPos_;   // [ob][intent][k]
	std::vector<std::vector<std::vector<Eigen::Vector3d>>> dynIntentPredSize_; // [ob][intent][k]
	std::vector<Eigen::VectorXd> dynIntentProb_;  // [ob], size 4
	double riskLambda_;    // λ: confidence inflation (e.g. 2 for ~95%)
	double riskBeta_;      // β: shape param for kernel edge steepness (≥1)
	double riskGammaTime_; // γ: time decay ω_k = γ^k
	double riskElongation_; // 运动方向拉长因子，s_along = max(sx,sy)*此值

	double getHeuristic(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const;
	double getStaticRisk(const Eigen::Vector3d& q) const;
	double getDynamicRisk(const Eigen::Vector3d& q) const;
	double getDynamicRiskIntent(const Eigen::Vector3d& q) const;
	double getDynamicRiskVelocity(const Eigen::Vector3d& q) const;
	double getTotalRisk(const Eigen::Vector3d& q) const;
	bool coordToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx) const;
	Eigen::Vector3d indexToCoord(const Eigen::Vector3i& idx) const;
	bool isOccupied(const Eigen::Vector3i& idx) const;
	bool isOccupied(const Eigen::Vector3d& pos) const;
	void shortcutPath(const std::vector<Eigen::Vector3d>& planRaw, std::vector<Eigen::Vector3d>& planSc);
	void pathToNavMsg(const std::vector<Eigen::Vector3d>& plan, nav_msgs::Path& path);
	void visCB(const ros::TimerEvent&);
	void publishAstarPath();
	void publishRiskMap();
	void runAstarSearch(bool useRisk, std::vector<Eigen::Vector3d>& outPlan);

public:
	astarOccMap(const ros::NodeHandle& nh);
	~astarOccMap();
	void initParam();
	void registerPub();
	void registerCallback();
	void setMap(const std::shared_ptr<mapManager::occMap>& map);
	void updateStart(const geometry_msgs::Pose& pose);
	void updateGoal(const geometry_msgs::Pose& pose);
	void updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos,
	                            const std::vector<Eigen::Vector3d>& obstaclesVel,
	                            const std::vector<Eigen::Vector3d>& obstaclesSize);
	void updateDynamicObstaclesIntent(
		const std::vector<std::vector<std::vector<Eigen::Vector3d>>>& predPos,
		const std::vector<std::vector<std::vector<Eigen::Vector3d>>>& predSize,
		const std::vector<Eigen::VectorXd>& intentProb);
	void makePlan(nav_msgs::Path& path);
	const std::vector<Eigen::Vector3d>& getCurrPlan() const { return currPlan_; }
	const std::vector<Eigen::Vector3d>& getRiskFreePlan() const { return riskFreePlan_; }
};

}

#endif

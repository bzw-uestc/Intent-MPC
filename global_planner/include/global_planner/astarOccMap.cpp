/*
 *	FILE: astarOccMap.cpp
 *	---------------------------
 *	A* path planner implementation
 */

#include <global_planner/astarOccMap.h>

namespace globalPlanner{

astarOccMap::astarOccMap(const ros::NodeHandle& nh) : nh_(nh){
	this->initParam();
	this->registerPub();
	this->registerCallback();
}

astarOccMap::~astarOccMap(){}

void astarOccMap::initParam(){
	if (not this->nh_.getParam("rrt/map_resolution", this->mapRes_)){
		this->mapRes_ = 0.2;
		cout << "[AStarPlanner]: No map resolution param. Use default: 0.2." << endl;
	}
	else{
		cout << "[AStarPlanner]: Map resolution: " << this->mapRes_ << "." << endl;
	}

	if (not this->nh_.getParam("rrt/timeout", this->timeout_)){
		this->timeout_ = 0.5;
	}
	if (this->nh_.getParam("astar/timeout", this->timeout_)){
		cout << "[AStarPlanner]: A* timeout: " << this->timeout_ << "s." << endl;
	}
	else{
		cout << "[AStarPlanner]: Timeout: " << this->timeout_ << "s." << endl;
	}

	if (not this->nh_.getParam("rrt/max_shortcut_dist", this->maxShortcutThresh_)){
		this->maxShortcutThresh_ = 5.0;
		cout << "[AStarPlanner]: No max shortcut dist. Use default: 5.0." << endl;
	}
	else{
		cout << "[AStarPlanner]: Max shortcut distance: " << this->maxShortcutThresh_ << "m." << endl;
	}

	if (not this->nh_.getParam("rrt/ignore_unknown", this->ignoreUnknown_)){
		this->ignoreUnknown_ = false;
	}
	else{
		cout << "[AStarPlanner]: Ignore unknown: " << this->ignoreUnknown_ << "." << endl;
	}

	if (not this->nh_.getParam("rrt/pass_goal_check", this->passGoalCheck_)){
		this->passGoalCheck_ = false;
	}
	else{
		cout << "[AStarPlanner]: Pass goal check: " << this->passGoalCheck_ << "." << endl;
	}

	// Risk-aware A* parameters
	if (not this->nh_.getParam("astar/risk_aware", this->riskAware_)){
		this->riskAware_ = true;
		cout << "[AStarPlanner]: Risk-aware mode: true (default)." << endl;
	}
	else{
		cout << "[AStarPlanner]: Risk-aware mode: " << this->riskAware_ << "." << endl;
	}

	if (not this->nh_.getParam("astar/risk_danger_zone", this->riskDangerZone_)){
		this->riskDangerZone_ = 0.8;
	}
	if (not this->nh_.getParam("astar/risk_gamma_static", this->riskGammaStatic_)){
		this->riskGammaStatic_ = 1.0;
	}
	if (not this->nh_.getParam("astar/risk_gamma_dynamic", this->riskGammaDynamic_)){
		this->riskGammaDynamic_ = 2.0;
	}
	if (not this->nh_.getParam("astar/risk_eta", this->riskEta_)){
		this->riskEta_ = 0.5;
	}
	if (not this->nh_.getParam("astar/risk_weight", this->riskWeight_)){
		this->riskWeight_ = 1.0;
	}
	if (not this->nh_.getParam("astar/risk_pred_horizon", this->riskPredHorizon_)){
		this->riskPredHorizon_ = 2.0;
	}
	if (not this->nh_.getParam("astar/risk_pred_steps", this->riskPredSteps_)){
		this->riskPredSteps_ = 6;
	}

	if (not this->nh_.getParam("astar/dual_path_search", this->dualPathSearch_)){
		this->dualPathSearch_ = false;
		cout << "[AStarPlanner]: Dual path search (risk + risk-free): " << this->dualPathSearch_ << "." << endl;
	}
	else{
		cout << "[AStarPlanner]: Dual path search: " << this->dualPathSearch_ << " (publish both for visualization)." << endl;
	}
}

void astarOccMap::registerPub(){
	this->astarVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("astar/planned_path", 100);
	this->astarRiskAwarePathPub_ = this->nh_.advertise<nav_msgs::Path>("astar/risk_aware_path", 10);
	this->astarRiskFreePathPub_ = this->nh_.advertise<nav_msgs::Path>("astar/risk_free_path", 10);
}

void astarOccMap::registerCallback(){
	this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &astarOccMap::visCB, this);
}

void astarOccMap::setMap(const std::shared_ptr<mapManager::occMap>& map){
	this->map_ = map;
	this->map_->getMapRange(this->mapSizeMin_, this->mapSizeMax_);
	this->mapRes_ = this->map_->getRes();
	this->map_->posToIndex(this->mapSizeMin_, this->mapVoxelMin_);
	this->map_->posToIndex(this->mapSizeMax_, this->mapVoxelMax_);
	this->mapVoxelMax_(0) = std::max(this->mapVoxelMax_(0), this->mapVoxelMin_(0) + 1);
	this->mapVoxelMax_(1) = std::max(this->mapVoxelMax_(1), this->mapVoxelMin_(1) + 1);
	this->mapVoxelMax_(2) = std::max(this->mapVoxelMax_(2), this->mapVoxelMin_(2) + 1);
}

void astarOccMap::updateStart(const geometry_msgs::Pose& pose){
	this->start_(0) = pose.position.x;
	this->start_(1) = pose.position.y;
	this->start_(2) = pose.position.z;
}

void astarOccMap::updateGoal(const geometry_msgs::Pose& pose){
	this->goal_(0) = pose.position.x;
	this->goal_(1) = pose.position.y;
	this->goal_(2) = pose.position.z;
}

void astarOccMap::updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos,
                                         const std::vector<Eigen::Vector3d>& obstaclesVel,
                                         const std::vector<Eigen::Vector3d>& obstaclesSize){
	this->dynObstaclesPos_ = obstaclesPos;
	this->dynObstaclesVel_ = obstaclesVel;
	this->dynObstaclesSize_ = obstaclesSize;
}

double astarOccMap::getStaticRisk(const Eigen::Vector3d& q) const{
	// Rs(q) = max_{om in O, ||q-om||<=ds} (||q-om|| - ds)^2  (Eq.1 in paper)
	// Use step=2 for performance: ~8x fewer voxel checks
	if (not this->map_ or this->riskDangerZone_ <= 0) return 0.0;

	double maxRisk = 0.0;
	const int searchRadius = std::max(1, (int)std::ceil(this->riskDangerZone_ / this->mapRes_));
	const int step = 2;
	Eigen::Vector3i qIdx;
	this->map_->posToIndex(q, qIdx);

	for (int i = -searchRadius; i <= searchRadius; i += step){
		for (int j = -searchRadius; j <= searchRadius; j += step){
			for (int k = -searchRadius; k <= searchRadius; k += step){
				Eigen::Vector3i idx = qIdx + Eigen::Vector3i(i, j, k);
				if (not this->map_->isInMap(idx)) continue;
				Eigen::Vector3d pos;
				this->map_->indexToPos(idx, pos);
				if (this->map_->isInflatedOccupied(pos)){
					double d = (q - pos).norm();
					if (d <= this->riskDangerZone_){
						double r = (d - this->riskDangerZone_) * (d - this->riskDangerZone_);
						maxRisk = std::max(maxRisk, r);
					}
				}
			}
		}
	}
	return this->riskGammaStatic_ * maxRisk;
}

double astarOccMap::getDynamicRisk(const Eigen::Vector3d& q) const{
	// Rd(q): Gaussian-based risk from predicted dynamic obstacle positions (simplified Eq.6-7)
	// rd(q,t) = exp(-||q - pred_pos||^2 / (2*sigma^2)), aggregated over time
	if (this->dynObstaclesPos_.empty()) return 0.0;

	double dt = (this->riskPredSteps_ > 1) ? (this->riskPredHorizon_ / (this->riskPredSteps_ - 1)) : 0.0;
	double totalRisk = 0.0;

	for (size_t i = 0; i < this->dynObstaclesPos_.size(); ++i){
		Eigen::Vector3d pos = this->dynObstaclesPos_[i];
		Eigen::Vector3d vel = this->dynObstaclesVel_[i];
		Eigen::Vector3d size = this->dynObstaclesSize_[i];
		double baseSigma = std::max({size(0), size(1), size(2)}) * 0.5;
		double velNorm = vel.norm();
		double velSigma = std::max(0.3, velNorm * this->riskPredHorizon_ * 0.5);

		for (int t = 0; t < this->riskPredSteps_; ++t){
			Eigen::Vector3d predPos = pos + vel * (t * dt);
			double sigma = baseSigma + velSigma * (1.0 + t * 0.2);
			double distSq = (q - predPos).squaredNorm();
			double w = 1.0 / (1.0 + t * 0.3);
			totalRisk += w * std::exp(-distSq / (2.0 * sigma * sigma));
		}
	}
	return this->riskGammaDynamic_ * totalRisk;
}

double astarOccMap::getTotalRisk(const Eigen::Vector3d& q) const{
	if (not this->riskAware_) return 0.0;
	return this->getStaticRisk(q) + this->getDynamicRisk(q);
}

double astarOccMap::getHeuristic(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const{
	Eigen::Vector3d diff = (b - a).cast<double>();
	return diff.norm();
}

bool astarOccMap::coordToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx) const{
	this->map_->posToIndex(pos, idx);
	return this->map_->isInMap(idx);
}

Eigen::Vector3d astarOccMap::indexToCoord(const Eigen::Vector3i& idx) const{
	Eigen::Vector3d pos;
	this->map_->indexToPos(idx, pos);
	return pos;
}

bool astarOccMap::isOccupied(const Eigen::Vector3i& idx) const{
	Eigen::Vector3d pos = this->indexToCoord(idx);
	return this->map_->isInflatedOccupied(pos);
}

bool astarOccMap::isOccupied(const Eigen::Vector3d& pos) const{
	return this->map_->isInflatedOccupied(pos);
}

void astarOccMap::shortcutPath(const std::vector<Eigen::Vector3d>& planRaw, std::vector<Eigen::Vector3d>& planSc){
	if (planRaw.size() <= 2){
		planSc = planRaw;
		return;
	}

	size_t ptr1 = 0, ptr2 = 2;
	planSc.push_back(planRaw[ptr1]);

	while (ptr2 < planRaw.size()){
		const Eigen::Vector3d& p1 = planRaw[ptr1];
		const Eigen::Vector3d& p2 = planRaw[ptr2];
		bool lineFree = not this->map_->isInflatedOccupiedLine(p1, p2);
		double dist = (p2 - p1).norm();

		if (lineFree and dist <= this->maxShortcutThresh_){
			if (ptr2 == planRaw.size() - 1){
				planSc.push_back(p2);
				break;
			}
			++ptr2;
		}
		else{
			planSc.push_back(planRaw[ptr2 - 1]);
			ptr1 = ptr2 - 1;
			ptr2 = ptr1 + 2;
			if (ptr2 >= planRaw.size()){
				planSc.push_back(planRaw.back());
				break;
			}
		}
	}
}

void astarOccMap::pathToNavMsg(const std::vector<Eigen::Vector3d>& plan, nav_msgs::Path& path){
	path.poses.clear();
	path.header.frame_id = "map";
	path.header.stamp = ros::Time::now();
	for (const auto& p : plan){
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = p(0);
		ps.pose.position.y = p(1);
		ps.pose.position.z = p(2);
		ps.pose.orientation.w = 1.0;
		path.poses.push_back(ps);
	}
}

void astarOccMap::runAstarSearch(bool useRisk, std::vector<Eigen::Vector3d>& outPlan){
	outPlan.clear();
	Eigen::Vector3i startIdx, goalIdx;
	if (not this->coordToIndex(this->start_, startIdx) or not this->coordToIndex(this->goal_, goalIdx)){
		outPlan.push_back(this->start_);
		outPlan.push_back(this->goal_);
		return;
	}

	ros::Time startTime = ros::Time::now();
	this->nodeMap_.clear();
	while (!this->openSet_.empty()) this->openSet_.pop();

	double startRisk = useRisk ? this->getTotalRisk(this->start_) : 0.0;
	AStarNode& startNode = this->nodeMap_[startIdx];
	startNode.index = startIdx;
	startNode.gScore = useRisk ? (this->riskWeight_ * startRisk) : 0;
	double hDist = this->getHeuristic(startIdx, goalIdx);
	double hRisk = useRisk ? (this->riskEta_ * startRisk * std::max(1.0, hDist / this->mapRes_)) : 0;
	startNode.fScore = startNode.gScore + hDist + hRisk;
	startNode.cameFrom = nullptr;
	startNode.state = AStarNode::OPEN;
	this->openSet_.push(&startNode);

	AStarNode* goalNodePtr = nullptr;
	const double tieBreaker = 1.0 + 1e-6;

	static const int dx[] = {-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	static const int dy[] = {-1,-1,-1, 0, 0, 0, 1, 1, 1,-1,-1,-1, 0, 0, 1, 1, 1,-1,-1,-1, 0, 0, 0, 1, 1, 1};
	static const int dz[] = {-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1};

	while (!this->openSet_.empty()){
		if ((ros::Time::now() - startTime).toSec() > this->timeout_){
			if (useRisk) cout << "[AStarPlanner]: Timeout (" << this->timeout_ << "s)." << endl;
			break;
		}

		AStarNode* current = this->openSet_.top();
		this->openSet_.pop();

		if (current->state == AStarNode::CLOSED) continue;
		current->state = AStarNode::CLOSED;

		if (current->index == goalIdx){
			goalNodePtr = current;
			break;
		}

		Eigen::Vector3d currentPos = this->indexToCoord(current->index);

		for (int i = 0; i < 26; ++i){
			Eigen::Vector3i neighborIdx = current->index + Eigen::Vector3i(dx[i], dy[i], dz[i]);

			if (not this->map_->isInMap(neighborIdx)) continue;
			if (this->isOccupied(neighborIdx)) continue;

			Eigen::Vector3d neighborPos = this->indexToCoord(neighborIdx);
			double stepCost = (Eigen::Vector3d(dx[i], dy[i], dz[i])).norm() * this->mapRes_;
			double riskCost = 0.0;
			if (useRisk){
				double neighborRisk = this->getTotalRisk(neighborPos);
				riskCost = this->riskWeight_ * (neighborRisk + this->getTotalRisk(currentPos)) * 0.5;
			}
			double tentativeG = current->gScore + stepCost + riskCost;

			double hDist = this->getHeuristic(neighborIdx, goalIdx);
			double hRisk = 0.0;
			if (useRisk){
				double neighborRisk = this->getTotalRisk(neighborPos);
				hRisk = this->riskEta_ * neighborRisk * std::max(1.0, hDist / this->mapRes_);
			}
			double fScore = tentativeG + tieBreaker * (hDist + hRisk);

			AStarNode& neighbor = this->nodeMap_[neighborIdx];
			if (neighbor.state == AStarNode::UNDEFINED){
				neighbor.index = neighborIdx;
				neighbor.gScore = tentativeG;
				neighbor.fScore = fScore;
				neighbor.cameFrom = current;
				neighbor.state = AStarNode::OPEN;
				this->openSet_.push(&neighbor);
			}
			else if (neighbor.state == AStarNode::OPEN and tentativeG < neighbor.gScore){
				neighbor.gScore = tentativeG;
				neighbor.fScore = fScore;
				neighbor.cameFrom = current;
				this->openSet_.push(&neighbor);
			}
		}
	}

	std::vector<Eigen::Vector3d> planRaw;
	if (goalNodePtr){
		AStarNode* ptr = goalNodePtr;
		while (ptr){
			planRaw.push_back(this->indexToCoord(ptr->index));
			ptr = ptr->cameFrom;
		}
		std::reverse(planRaw.begin(), planRaw.end());
	}
	else{
		if (useRisk) cout << "[AStarPlanner]: No path found." << endl;
		planRaw.push_back(this->start_);
		planRaw.push_back(this->goal_);
	}

	this->shortcutPath(planRaw, outPlan);
}

void astarOccMap::makePlan(nav_msgs::Path& path){
	path.poses.clear();
	if (not this->map_){
		cout << "[AStarPlanner]: Map not set." << endl;
		return;
	}

	Eigen::Vector3d qGoal = this->goal_;
	if (not this->passGoalCheck_){
		if (not this->ignoreUnknown_ and this->map_->isUnknown(qGoal)){
			geometry_msgs::PoseStamped ps;
			ps.pose.position.x = this->start_(0);
			ps.pose.position.y = this->start_(1);
			ps.pose.position.z = this->start_(2);
			path.poses.push_back(ps);
			cout << "[AStarPlanner]: Goal is unknown. Please change goal." << endl;
			return;
		}
	}

	Eigen::Vector3i startIdx, goalIdx;
	if (not this->coordToIndex(this->start_, startIdx)){
		cout << "[AStarPlanner]: Start point outside map." << endl;
		return;
	}
	if (not this->coordToIndex(this->goal_, goalIdx)){
		cout << "[AStarPlanner]: Goal point outside map." << endl;
		return;
	}

	if (this->isOccupied(startIdx)){
		cout << "[AStarPlanner]: Start point in obstacle." << endl;
		return;
	}
	if (this->isOccupied(goalIdx)){
		cout << "[AStarPlanner]: Goal point in obstacle." << endl;
		return;
	}

	// 1. Risk-aware path (used for planning)
	this->runAstarSearch(this->riskAware_, this->currPlan_);
	this->pathToNavMsg(this->currPlan_, path);

	// 2. Risk-free path (only search, for visualization)
	if (this->dualPathSearch_){
		this->runAstarSearch(false, this->riskFreePlan_);
	}
}

void astarOccMap::visCB(const ros::TimerEvent&){
	this->publishAstarPath();
	if (this->dualPathSearch_){
		nav_msgs::Path riskAwarePath, riskFreePath;
		this->pathToNavMsg(this->currPlan_, riskAwarePath);
		this->pathToNavMsg(this->riskFreePlan_, riskFreePath);
		if (riskAwarePath.poses.size() >= 2){
			this->astarRiskAwarePathPub_.publish(riskAwarePath);
		}
		if (riskFreePath.poses.size() >= 2){
			this->astarRiskFreePathPub_.publish(riskFreePath);
		}
	}
}

void astarOccMap::publishAstarPath(){
	if (this->currPlan_.size() < 2) return;

	visualization_msgs::MarkerArray msg;
	std::vector<visualization_msgs::Marker> markers;
	visualization_msgs::Marker line;
	line.header.frame_id = "map";
	line.header.stamp = ros::Time::now();
	line.ns = "astar_path";
	line.id = 0;
	line.type = visualization_msgs::Marker::LINE_LIST;
	line.scale.x = 0.05;
	line.scale.y = 0.05;
	line.scale.z = 0.05;
	line.color.a = 1.0;
	line.color.r = 0.5;
	line.color.g = 0.1;
	line.color.b = 1.0;
	line.lifetime = ros::Duration(0.5);

	for (size_t i = 0; i < this->currPlan_.size() - 1; ++i){
		geometry_msgs::Point p1, p2;
		p1.x = this->currPlan_[i](0);
		p1.y = this->currPlan_[i](1);
		p1.z = this->currPlan_[i](2);
		p2.x = this->currPlan_[i+1](0);
		p2.y = this->currPlan_[i+1](1);
		p2.z = this->currPlan_[i+1](2);
		line.points.push_back(p1);
		line.points.push_back(p2);
	}
	markers.push_back(line);

	for (size_t i = 0; i < this->currPlan_.size(); ++i){
		visualization_msgs::Marker wp;
		wp.header.frame_id = "map";
		wp.header.stamp = ros::Time::now();
		wp.ns = "astar_path";
		wp.id = 1 + i;
		wp.type = visualization_msgs::Marker::SPHERE;
		wp.pose.position.x = this->currPlan_[i](0);
		wp.pose.position.y = this->currPlan_[i](1);
		wp.pose.position.z = this->currPlan_[i](2);
		wp.scale.x = wp.scale.y = wp.scale.z = 0.2;
		wp.color.a = 0.8;
		wp.color.r = 0.3;
		wp.color.g = 1.0;
		wp.color.b = 0.5;
		wp.lifetime = ros::Duration(0.5);
		markers.push_back(wp);
	}
	msg.markers = markers;
	this->astarVisPub_.publish(msg);
}

}

#pragma once
#include<vector>
#include "Node.h"

class BodyChain
{
public:
	BodyChain();
	void createNodeFromArray(std::vector<std::vector<float>>& array);
	void setDistance(float d);
	void setMinAngle(float a);
	const std::vector<std::vector<float>> getNodeData() const;
private:
	std::vector<Node> nodes;
	float distance;
	float min_angle;
};


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
	void render(sf::RenderWindow& window);
	const Node getNode(int id);
private:
	std::vector<Node> nodes;
	float distance;
	float min_angle;
};
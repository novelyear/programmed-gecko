#pragma once
#include<vector>
#include "Node.h"

struct BodyRenderConfig
{
	int SmoothSegments = 24;
};
struct BodyKinematicsConfig
{
	int constraintIterations = 8;
};
class BodyChain
{
public:
	BodyChain() = default;
	void createNodeFromArray(std::vector<std::vector<float>>& array);
	void setDistance(float d);
	void setMinAngle(float a);
	const std::vector<std::vector<float>> getNodeData() const;
	void render(sf::RenderWindow& window, sf::Color );
	const Node getNode(int id);
	void moveTowards(const sf::Vector2f & target, float speed);
	void applyConstraints(int iterations);
	void renderNodes(sf::RenderWindow& window, sf::Color );

	const float getNodeRadius(int index);
private:
	std::vector<Node> nodes;
	float distance;
	float min_angle;
	BodyRenderConfig m_renderConfig;
	BodyKinematicsConfig m_kinematicsConfig;
};
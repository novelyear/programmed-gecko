#include "pch.h"
#include "Node.h"

Node::Node(float x, float y, float r)
{
	this->position = { x, y };
	this->radius = r;
}

void Node::setData(float x, float y, float r){ position = { x, y }; radius = r; }
void Node::setX(float x) { position.x = x; }
void Node::setY(float y) { position.y = y; }

const std::vector<float> Node::getData() const
{
	return { position.x, position.y, radius };
}
const float Node::getX() const { return position.x; }
const float Node::getY() const { return position.y; }
const float Node::getR() const { return radius; }

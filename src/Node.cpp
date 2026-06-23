#include "pch.h"
#include "Node.h"

Node::Node(float x, float y, float r)
{
	this->position = { x, y };
	this->radius = r;
}

const std::vector<float> Node::getData() const
{
	return { position.x, position.y, radius };
}


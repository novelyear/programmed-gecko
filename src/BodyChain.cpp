#include "pch.h"
#include "BodyChain.h"
#include <iostream>

BodyChain::BodyChain(){}

void BodyChain::createNodeFromArray(std::vector<std::vector<float>>& array)
{
	if (array.size() <= 0 || array[0].size() <= 0) {
		std::cerr << "array is empty!" << std::endl;
		return;
	}
	for (const auto& tuple : array) {
		if (tuple.size() >= 3) {
			nodes.emplace_back(tuple[0], tuple[1], tuple[2]);
		}
	}
}

void BodyChain::setDistance(float d)
{
	this->distance = d;
}

void BodyChain::setMinAngle(float a)
{
	this->min_angle = a;
}

const std::vector<std::vector<float>> BodyChain::getNodeData() const
{
	std::vector<std::vector<float>> nodeData;
	for (const auto& node : nodes) {
		nodeData.push_back(node.getData());
	}
	return nodeData;
}

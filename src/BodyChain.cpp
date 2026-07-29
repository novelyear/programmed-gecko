#include "pch.h"
#include "BodyChain.h"
#include <iostream>
#include "beizer.cpp"
#include <SFML/Graphics.hpp>

void BuildBodySides(
    const std::vector<std::vector<float>> body,
    std::vector<sf::Vector2f>& leftSide,
    std::vector<sf::Vector2f>& rightSide)
{
    leftSide.clear();
    rightSide.clear();
    if (body.size() < 2) return;

    for (size_t i = 0; i < body.size(); ++i)
    {
        const sf::Vector2f p(body[i][0], body[i][1]);
        const float r = body[i][2];

        sf::Vector2f tangent;
        if (i == 0)
        {
            tangent = Sub(sf::Vector2f(body[i + 1][0], body[i + 1][1]), p);
        }
        else if (i == body.size() - 1)
        {
            tangent = Sub(p, sf::Vector2f(body[i - 1][0], body[i - 1][1]));
        }
        else
        {
            tangent = Sub(sf::Vector2f(body[i + 1][0], body[i + 1][1]),
                sf::Vector2f(body[i - 1][0], body[i - 1][1]));
        }

        const sf::Vector2f t = Normalize(tangent);
        const sf::Vector2f n(-t.y, t.x);

        if (i == 0)
        {
            leftSide.push_back(Sub(p, Mul(t, r))); // head
        }
        leftSide.push_back(Add(p, Mul(n, r)));
        rightSide.push_back(Sub(p, Mul(n, r)));
        if (i == body.size() - 1)
        {
            rightSide.push_back(Add(p, Mul(t, r))); // tail
        }
    }
}

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

void BodyChain::render(sf::RenderWindow& window)
{
	std::vector<sf::Vector2f> leftSide;
	std::vector<sf::Vector2f> rightSide;
	BuildBodySides(this->getNodeData(), leftSide, rightSide);

	const std::vector<sf::Vector2f> leftSmooth = SampleSmoothPolyline(leftSide, 24);

	std::reverse(rightSide.begin(), rightSide.end());
	const std::vector<sf::Vector2f> rightSmooth = SampleSmoothPolyline(rightSide, 24);

	std::vector<sf::Vector2f> outline;
	outline.reserve(leftSmooth.size() + rightSmooth.size() + 1);
	outline.insert(outline.end(), leftSmooth.begin(), leftSmooth.end());
	outline.insert(outline.end(), rightSmooth.begin(), rightSmooth.end());
	if (!outline.empty()) outline.push_back(outline.front());

	sf::VertexArray va(sf::LineStrip, outline.size());
	for (size_t i = 0; i < outline.size(); ++i)
	{
		va[i].position = outline[i];
		va[i].color = sf::Color(120, 220, 120);
	}

	window.draw(va);
}

const Node BodyChain::getNode(int id)
{
    return nodes[id];
}
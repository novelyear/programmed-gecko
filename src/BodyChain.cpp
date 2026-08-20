#include "pch.h"
#include "BodyChain.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include "beizer.cpp"


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

const float BodyChain::getNodeRadius(int index)
{
	if (index < 0 || index >= nodes.size()) {
		std::cerr << "Invalid node index!" << std::endl;
		return 0.0f;
	}
	return nodes[index].getR();
}

const std::vector<std::vector<float>> BodyChain::getNodeData() const
{
	std::vector<std::vector<float>> nodeData;
	for (const auto& node : nodes) {
		nodeData.push_back(node.getData());
	}
	return nodeData;
}

void BodyChain::render(sf::RenderWindow& window, sf::Color color)
{
	std::vector<sf::Vector2f> leftSide;
	std::vector<sf::Vector2f> rightSide;
	BuildBodySides(this->getNodeData(), leftSide, rightSide);

	const std::vector<sf::Vector2f> leftSmooth = SampleSmoothPolyline(leftSide, m_renderConfig.SmoothSegments);

	std::reverse(rightSide.begin(), rightSide.end());
	const std::vector<sf::Vector2f> rightSmooth = SampleSmoothPolyline(rightSide, m_renderConfig.SmoothSegments);

	std::vector<sf::Vector2f> outline;
	outline.reserve(leftSmooth.size() + rightSmooth.size() + 1);
	outline.insert(outline.end(), leftSmooth.begin(), leftSmooth.end());
	outline.insert(outline.end(), rightSmooth.begin(), rightSmooth.end());
	if (!outline.empty()) outline.push_back(outline.front());

	sf::VertexArray va(sf::LineStrip, outline.size());
	for (size_t i = 0; i < outline.size(); ++i)
	{
		va[i].position = outline[i];
		va[i].color = color;
	}

	window.draw(va);
}

void BodyChain::renderNodes(sf::RenderWindow& window, sf::Color color)
{
	for (const auto& node : nodes) {
        float r = node.getData()[2];
		sf::CircleShape circle(r);
		circle.setFillColor(color);
		circle.setOrigin(r, r);
		circle.setPosition(node.getX(), node.getY());
		window.draw(circle);
	}
}

const Node BodyChain::getNode(int id)
{
    return nodes[id];
}

void BodyChain::applyConstraints(int iterations)
{
    size_t n = nodes.size();
    if (n < 2) return;

    for (int iter = 0; iter < iterations; ++iter) {
        for (size_t i = 1; i < n; ++i) {
            Node& curr = nodes[i];
            const Node& prev = nodes[i - 1];
            sf::Vector2f prevPos(prev.getX(), prev.getY());
            sf::Vector2f currPos(curr.getX(), curr.getY());
            sf::Vector2f dir = currPos - prevPos;
            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
			constexpr float EPS = 1e-6f;
            if (len > EPS) {
                sf::Vector2f newPos = prevPos + (dir / len) * distance;
                curr.setX(newPos.x);
                curr.setY(newPos.y);
            }
            else {
                curr.setX(prevPos.x + distance);
                curr.setY(prevPos.y);
            }
        }

        for (size_t i = 1; i < n - 1; ++i) {
            const Node& prev = nodes[i - 1];
            Node& curr = nodes[i];
            const Node& next = nodes[i + 1];

            sf::Vector2f v1(prev.getX() - curr.getX(), prev.getY() - curr.getY());
            sf::Vector2f v2(next.getX() - curr.getX(), next.getY() - curr.getY());
            float len1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
            float len2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
            if (len1 < 1e-6f || len2 < 1e-6f) continue;

            float dot = v1.x * v2.x + v1.y * v2.y;
            float cosAngle = dot / (len1 * len2);
            cosAngle = std::clamp(cosAngle, -1.0f, 1.0f);
            float angle = std::acos(cosAngle) / (float)M_PI * 180;

            if (angle < min_angle) {
                sf::Vector2f u1 = -v1 / len1;
                sf::Vector2f u2 = v2 / len2;
                sf::Vector2f bisector = u1 + u2;
                float bisLen = std::sqrt(bisector.x * bisector.x + bisector.y * bisector.y);
                if (bisLen > 1e-6f) {
                    bisector /= bisLen;
                    const float pushDist = distance * 0.1f;
                    curr.setX(curr.getX() + bisector.x * pushDist);
                    curr.setY(curr.getY() + bisector.y * pushDist);
                }
            }
        }
    }
}

void BodyChain::moveTowards(const sf::Vector2f& target, float speed)
{
    if (nodes.empty()) return;

    Node& head = nodes.front();
    sf::Vector2f headPos(head.getX(), head.getY()); 
    sf::Vector2f offset = (target - headPos) * speed;
    head.setX(headPos.x + offset.x);
    head.setY(headPos.y + offset.y);

    applyConstraints(m_kinematicsConfig.constraintIterations);
}
#pragma once
#include <SFML/Graphics.hpp>
class Node
{
public:
	Node(float x, float y, float r);
	const std::vector<float> getData() const;
private:
	sf::Vector2f position;
	float radius;
};


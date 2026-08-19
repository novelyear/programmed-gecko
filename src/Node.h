#pragma once
#include <SFML/Graphics.hpp>
class Node
{
public:
	Node(float x, float y, float r);
	const std::vector<float> getData() const;
	const float getX() const;
	const float getY() const;
	const float getR() const;
	
	void setData(float x, float y, float r);
	void setX(float x);
	void setY(float y);
private:
	sf::Vector2f position;
	float radius;
};


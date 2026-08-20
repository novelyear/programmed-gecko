#pragma once
#include <SFML/Graphics.hpp>
#include "BodyChain.h"
#include "Foot.h"

#define BODY_CONFIG_FILE "src/body-config.json"

struct RenderConfig
{
	sf::Color colorOfInside = sf::Color(200, 150, 100);
	sf::Color colorOfOutline = sf::Color(100, 100, 100);
};

class Application
{
public:
	Application(const sf::String& title);
	void render();
	void update();

private:
	sf::Vector2f nodeTangent(int nodeIndex) const;
	sf::Vector2f footRootAt(int nodeIndex, bool attachLeft) const;
	void setupFoot(Foot& foot, int nodeIndex, bool attachLeft, bool isFront);

	sf::RenderWindow window;
	BodyChain body;
	Foot leftFront;
	Foot rightFront;
	Foot leftHind;
	Foot rightHind;
	int m_frontNodeIndex = 4;
	int m_hindNodeIndex = 8;
	RenderConfig m_renderConfig;
};


#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

class Renderer
{
public:
	Renderer(int width, int height, const sf::String& title);

	void render(const std::vector<std::vector<float>> body,
	            const std::vector<std::vector<float>> foot);

private:
	sf::RenderWindow window;
};


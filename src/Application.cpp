#include "pch.h"
#include "Application.h"
#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"
#include "beizer.cpp"

Application::Application(const sf::String& title) : window(sf::VideoMode(1600, 1200), title)
{
    try
    {
        body = BodyChain();
        std::ifstream file(BODY_CONFIG_FILE);
        if (!file.is_open())
        {
            std::cerr << "Failed to open: " << BODY_CONFIG_FILE << std::endl;
            return;
        }

        nlohmann::json j;
        file >> j;

        if (!j.contains("distance") ||
            !j.contains("min_angle") ||
            !j.contains("nodes") ||
            !j.contains("window_width") ||
            !j.contains("window_height"))
        {
            std::cerr << "Some essential data Miss!" << std::endl;
            return;
        
        }
		unsigned int window_width = j["window_width"];
		unsigned int window_height = j["window_height"];

        std::vector<std::vector<float>> body_nodes_data;
        for (const auto& v : j["nodes"]) {
            body_nodes_data.push_back({ (float)v[0] * window_width, (float)v[1] * window_height, v[2] });
        }
        body.createNodeFromArray(body_nodes_data);
        body.setDistance(j["distance"]);
        body.setMinAngle(j["min_angle"]);
        m_frontNodeIndex = j.value("front", m_frontNodeIndex);
        m_hindNodeIndex = j.value("hind", m_hindNodeIndex);


        window.setSize({ window_width, window_height });
		window.setView(sf::View(sf::FloatRect(0.f, 0.f, static_cast<float>(window_width), static_cast<float>(window_height))));

        setupFoot(leftFront, m_frontNodeIndex, true, true);
        setupFoot(rightFront, m_frontNodeIndex, false, true);
        setupFoot(leftHind, m_hindNodeIndex, true, false);
        setupFoot(rightHind, m_hindNodeIndex, false, false);

        if (j.contains("default_angle1") && j.contains("default_angle2")) {
            leftFront.setDefaultAngle(j["default_angle1"], j["default_angle2"]);
            rightFront.setDefaultAngle(j["default_angle1"], j["default_angle2"]);
            leftHind.setDefaultAngle(j["default_angle1"], j["default_angle2"]);
            rightHind.setDefaultAngle(j["default_angle1"], j["default_angle2"]);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading body data: " << e.what() << std::endl;
        return;
    }
}

sf::Vector2f Application::nodeTangent(int nodeIndex) const
{
    const auto nodeData = body.getNodeData();
    const size_t i = static_cast<size_t>(nodeIndex);
    const sf::Vector2f center(nodeData[i][0], nodeData[i][1]);

    sf::Vector2f tangent;
    if (i == 0)
    {
        tangent = Sub(sf::Vector2f(nodeData[i + 1][0], nodeData[i + 1][1]), center);
    }
    else if (i + 1 == nodeData.size())
    {
        tangent = Sub(center, sf::Vector2f(nodeData[i - 1][0], nodeData[i - 1][1]));
    }
    else
    {
        tangent = Sub(sf::Vector2f(nodeData[i + 1][0], nodeData[i + 1][1]),
            sf::Vector2f(nodeData[i - 1][0], nodeData[i - 1][1]));
    }
    return Normalize(tangent);
}

sf::Vector2f Application::footRootAt(int nodeIndex, bool attachLeft) const
{
    const auto nodeData = body.getNodeData();
    const size_t i = static_cast<size_t>(nodeIndex);
    const sf::Vector2f center(nodeData[i][0], nodeData[i][1]);
    const float radius = nodeData[i][2];

    const sf::Vector2f t = nodeTangent(nodeIndex);
    const sf::Vector2f n(-t.y, t.x);
    return attachLeft ? Add(center, Mul(n, radius)) : Sub(center, Mul(n, radius));
}

void Application::setupFoot(Foot& foot, int nodeIndex, bool attachLeft, bool isFront)
{
    foot = Foot(40.f, 40.f, isFront, attachLeft);
    const sf::Vector2f root = footRootAt(nodeIndex, attachLeft);
    const sf::Vector2f rootDir = nodeTangent(nodeIndex);

    const sf::Vector2f normal(-rootDir.y, rootDir.x);
    const sf::Vector2f outward = attachLeft ? normal : Mul(normal, -1.f);
    const sf::Vector2f faceDir = isFront ? Mul(rootDir, -1.f) : rootDir;
    const sf::Vector2f target = foot.getEnd(root, rootDir);

    foot.update(root, rootDir);
	foot.setTargetPos(target);
}

void Application::render()
{
    window.clear(sf::Color::Black);
    body.render(window);
	body.renderNodes(window);

    leftFront.render(footRootAt(m_frontNodeIndex, true), nodeTangent(m_frontNodeIndex), window);
    rightFront.render(footRootAt(m_frontNodeIndex, false), nodeTangent(m_frontNodeIndex), window);
    leftHind.render(footRootAt(m_hindNodeIndex, true), nodeTangent(m_hindNodeIndex), window);
    rightHind.render(footRootAt(m_hindNodeIndex, false), nodeTangent(m_hindNodeIndex), window);

    window.display();
}

// Application.cpp

void Application::update()
{
    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    sf::Vector2f target(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));

    const float speed = 0.01f;
    body.moveTowards(target, speed);
    
	leftFront.update(footRootAt(m_frontNodeIndex, true), nodeTangent(m_frontNodeIndex));
	rightFront.update(footRootAt(m_frontNodeIndex, false), nodeTangent(m_frontNodeIndex));
	leftHind.update(footRootAt(m_hindNodeIndex, true), nodeTangent(m_hindNodeIndex));
	rightHind.update(footRootAt(m_hindNodeIndex, false), nodeTangent(m_hindNodeIndex));
}
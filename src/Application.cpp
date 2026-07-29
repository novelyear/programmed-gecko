#include "pch.h"
#include "Application.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"
#include "beizer.cpp"

namespace
{
    sf::Vector2f DefaultFootTarget(
        const sf::Vector2f& root,
        const sf::Vector2f& tangent,
        bool attachLeft,
        bool isFront)
    {
        const sf::Vector2f n(-tangent.y, tangent.x);
        const sf::Vector2f outward = attachLeft ? n : Mul(n, -1.f);
        const sf::Vector2f faceDir = isFront ? Mul(tangent, -1.f) : tangent;
        const sf::Vector2f dir = Normalize(
            Add(Add(Mul(outward, 0.45f), Mul(faceDir, 0.35f)), sf::Vector2f(0.f, 0.55f)));
        return Add(root, Mul(dir, 20.f));
    }
}

Application::Application(int width, int height, const sf::String& title)
    : window(sf::VideoMode((unsigned int)width, (unsigned int)height), title)
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
            !j.contains("nodes"))
        {
            std::cerr << "Some essential data Miss!" << std::endl;
            return;
        }

        std::vector<std::vector<float>> body_nodes_data;
        for (const auto& v : j["nodes"]) {
            body_nodes_data.push_back({ v[0], v[1], v[2] });
        }
        body.createNodeFromArray(body_nodes_data);
        body.setDistance(j["distance"]);
        body.setMinAngle(j["min_angle"]);
        m_frontNodeIndex = j.value("front", m_frontNodeIndex);
        m_hindNodeIndex = j.value("hind", m_hindNodeIndex);

        setupFoot(leftFront, m_frontNodeIndex, true, true);
        setupFoot(rightFront, m_frontNodeIndex, false, true);
        setupFoot(leftHind, m_hindNodeIndex, true, false);
        setupFoot(rightHind, m_hindNodeIndex, false, false);
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
    foot = Foot(12.f, 12.f);
    const sf::Vector2f root = footRootAt(nodeIndex, attachLeft);
    const sf::Vector2f tangent = nodeTangent(nodeIndex);
    const sf::Vector2f target = DefaultFootTarget(root, tangent, attachLeft, isFront);
    foot.update(root, tangent, target, 100.f, 0.f);
    foot.captureDefaultPose();
}

void Application::render()
{
    window.clear(sf::Color::Black);
    body.render(window);

    leftFront.render(footRootAt(m_frontNodeIndex, true), nodeTangent(m_frontNodeIndex), window);
    rightFront.render(footRootAt(m_frontNodeIndex, false), nodeTangent(m_frontNodeIndex), window);
    leftHind.render(footRootAt(m_hindNodeIndex, true), nodeTangent(m_hindNodeIndex), window);
    rightHind.render(footRootAt(m_hindNodeIndex, false), nodeTangent(m_hindNodeIndex), window);

    window.display();
}
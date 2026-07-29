#pragma once
#include <SFML/Graphics.hpp>

class Foot
{
private:
    float m_len1;
    float m_len2;

    float m_angle1;
    float m_angle2;

    float m_defaultAngle1;
    float m_defaultAngle2;

    float m_angle1Vel = 0.f;
    float m_angle2Vel = 0.f;

public:
    Foot();
    Foot(float len1, float len2, float defaultAng1 = 1.f, float defaultAng2 = -1.f);

    void update(const sf::Vector2f& rootWorldPos,
        const sf::Vector2f& rootDir,
        const sf::Vector2f& targetWorldPos,
        float maxReach,
        float minReach = 5.0f);

    void render(const sf::Vector2f& root, const sf::Vector2f& rootDir, sf::RenderWindow& window);

    void captureDefaultPose();
};
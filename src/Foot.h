#pragma once
#include <SFML/Graphics.hpp>

class Foot
{
private:
    float m_len1;
    float m_len2;

    float m_angle1;
    float m_angle2;

    float m_defaultAngle1 = 3.1415926 / 6;
    float m_defaultAngle2 = 3.1415926 / 3;

	sf::Vector2f targetPos;
    bool isFront;
    bool attachLeft;

public:
    Foot();
    Foot(float len1, float len2, bool isFront, bool attachLeft);
    void setTargetPos(const sf::Vector2f& pos);
    void setDefaultAngle(const float da1, const float da2);
    void update(const sf::Vector2f& rootWorldPos,
        const sf::Vector2f& rootDir);

    void render(const sf::Vector2f& root, const sf::Vector2f& rootDir, sf::RenderWindow& window);

    void captureDefaultPose();

    sf::Vector2f getJ1Dir(sf::Vector2f outward);
    sf::Vector2f getJ2Dir(const sf::Vector2f rootDir);
    sf::Vector2f getJoint(const sf::Vector2f root, const sf::Vector2f rootDir);
    sf::Vector2f getEnd(const sf::Vector2f root, const sf::Vector2f rootDir);

    static sf::Vector2f rotateCCW(const sf::Vector2f& direction, float angleDegrees);
};
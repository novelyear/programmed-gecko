#pragma once
#include <SFML/Graphics.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

struct FootKinematicsConfig
{
    static constexpr float DEFAULT_LEN1 = 40.f;
    static constexpr float DEFAULT_LEN2 = 40.f;

    static constexpr float DEFAULT_FRONT_MIN_ELBOW_DEG = 20.f;
    static constexpr float DEFAULT_HIND_MAX_ELBOW_DEG = 160.f;

    float len1 = DEFAULT_LEN1;
    float len2 = DEFAULT_LEN2;

    float frontMinElbowDeg = DEFAULT_FRONT_MIN_ELBOW_DEG;
    float hindMaxElbowDeg = DEFAULT_HIND_MAX_ELBOW_DEG;
};
struct FootRenderConfig
{
    float radius = 12.f;
};

class Foot
{
private:
    float m_len1;
    float m_len2;

    float m_angle1;
    float m_angle2;

    float m_defaultAngle1 = (float)M_PI / 6;
    float m_defaultAngle2 = (float)M_PI / 3;

	sf::Vector2f targetPos;
    bool isFront;
    bool attachLeft;
	FootRenderConfig m_renderConfig;
	FootKinematicsConfig m_kinematicsConfig;
public:
	Foot() = default;
    Foot(bool isFront, bool attachLeft, const FootKinematicsConfig& kinematics = {}, const FootRenderConfig& render = {});
    void setTargetPos(const sf::Vector2f& pos);
    void setDefaultAngle(const float da1, const float da2);
    void update(const sf::Vector2f& rootWorldPos,
        const sf::Vector2f& rootDir);

    void render(const sf::Vector2f& root, const sf::Vector2f& rootDir, sf::RenderWindow& window, sf::Color c);

    void captureDefaultPose();

    sf::Vector2f getJ1Dir(sf::Vector2f outward);
    sf::Vector2f getJ2Dir(const sf::Vector2f rootDir);
    sf::Vector2f getJoint(const sf::Vector2f root, const sf::Vector2f rootDir);
    sf::Vector2f getEnd(const sf::Vector2f root, const sf::Vector2f rootDir);

    static sf::Vector2f rotateCCW(const sf::Vector2f& direction, float angleDegrees);
};
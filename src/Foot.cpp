#include "pch.h"
#include "Foot.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "beizer.cpp"

namespace
{
    float BaseAngle(const sf::Vector2f& rootDir)
    {
        return std::atan2(rootDir.y, rootDir.x);
    }

    float WorldToLocal(float worldAngle, const sf::Vector2f& rootDir)
    {
        return worldAngle - BaseAngle(rootDir);
    }

    float LocalToWorld(float localAngle, const sf::Vector2f& rootDir)
    {
        return BaseAngle(rootDir) + localAngle;
    }
}

Foot::Foot()
{
    m_len1 = 12.f;
    m_len2 = 12.f;

    m_defaultAngle1 = 0.f;
    m_defaultAngle2 = -0.8f;
    m_angle1 = m_defaultAngle1;
    m_angle2 = m_defaultAngle2;
}

Foot::Foot(float len1, float len2, float defaultAng1, float defaultAng2)
    : m_len1(len1), m_len2(len2), m_defaultAngle1(defaultAng1), m_defaultAngle2(defaultAng2)
{
    m_angle1 = defaultAng1;
    m_angle2 = defaultAng2;
}

void Foot::captureDefaultPose()
{
    m_defaultAngle1 = m_angle1;
    m_defaultAngle2 = m_angle2;
}

void Foot::update(const sf::Vector2f& rootWorldPos,
    const sf::Vector2f& rootDir,
    const sf::Vector2f& targetWorldPos,
    float maxReach,
    float minReach)
{
    sf::Vector2f delta = targetWorldPos - rootWorldPos;
    float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);

    if (dist > maxReach || dist < minReach) {
        m_angle1 += (m_defaultAngle1 - m_angle1) * 0.1f;
        m_angle2 += (m_defaultAngle2 - m_angle2) * 0.1f;
        return;
    }

    float targetAngleWorld = std::atan2(delta.y, delta.x);

    float cosAngle2 = (m_len1 * m_len1 + m_len2 * m_len2 - dist * dist)
        / (2.0f * m_len1 * m_len2);
    cosAngle2 = std::clamp(cosAngle2, -1.0f, 1.0f);

    float angle2 = std::acos(cosAngle2);

    float angle1 = std::atan2(m_len2 * std::sin(angle2),
        m_len1 + m_len2 * std::cos(angle2));
    float finalAngle1World = targetAngleWorld - angle1;
    float finalAngle2 = -(static_cast<float>(M_PI) - angle2);

    m_angle1 = WorldToLocal(finalAngle1World, rootDir);
    m_angle2 = finalAngle2;
}

void Foot::render(const sf::Vector2f& root, const sf::Vector2f& rootDir, sf::RenderWindow& window)
{
    const float worldAngle1 = LocalToWorld(m_angle1, rootDir);
    const float worldAngle2 = worldAngle1 + m_angle2;

    sf::Vector2f j1 = root + sf::Vector2f(std::cos(worldAngle1), std::sin(worldAngle1)) * m_len1;
    sf::Vector2f j2 = j1 + sf::Vector2f(std::cos(worldAngle2), std::sin(worldAngle2)) * m_len2;

    sf::Vector2f dir = Normalize(j2 - j1);
    sf::Vector2f n(-dir.y, dir.x);

    const float radius = 12.f;
    const float extend = radius * 1.5f;

    std::vector<sf::Vector2f> pts;

    sf::Vector2f start_left = root - n * radius - dir * extend;
    sf::Vector2f left1 = j1 - n * radius;
    sf::Vector2f left2 = j2 - n * radius;
    sf::Vector2f end = j2 + dir * radius;

    sf::Vector2f right2 = j2 + n * radius;
    sf::Vector2f right1 = j1 + n * radius;
    sf::Vector2f start_right = root + n * radius - dir * extend;

    pts.push_back(start_left);
    pts.push_back(left1);
    pts.push_back(left2);
    pts.push_back(end);
    pts.push_back(right2);
    pts.push_back(right1);
    pts.push_back(start_right);
    pts.push_back(start_left);

    const int samplesPerSeg = 12;
    std::vector<sf::Vector2f> smooth = SampleSmoothPolyline(pts, samplesPerSeg);

    sf::VertexArray va(sf::LineStrip, smooth.size());
    for (size_t i = 0; i < smooth.size(); ++i)
    {
        va[i].position = smooth[i];
        va[i].color = sf::Color(200, 150, 100);
    }
    window.draw(va);
}

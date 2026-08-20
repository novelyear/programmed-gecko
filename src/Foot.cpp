#include "pch.h"
#include "Foot.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "beizer.cpp"

namespace
{
    float BaseAngle(const sf::Vector2f& rootDir)
    {
        return std::atan2(-rootDir.y, rootDir.x);
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

Foot::Foot(bool isFront, bool attachLeft, const FootKinematicsConfig& kinematics, const FootRenderConfig& render)
    : isFront(isFront), attachLeft(attachLeft), m_kinematicsConfig(kinematics), m_renderConfig(render)
{
    m_angle1 = m_defaultAngle1;
    m_angle2 = m_defaultAngle2;

	m_len1 = kinematics.len1;
	m_len2 = kinematics.len2;
}

void Foot::setTargetPos(const sf::Vector2f& pos) { targetPos = pos; }
void Foot::setDefaultAngle(const float da1, const float da2)
{
	if (da1 < 0.f || da1 > (float)M_PI || da2 < 0.f || da2 >(float)M_PI) {
		m_defaultAngle1 = da1 / 180.f * (float)M_PI;
		m_defaultAngle2 = da2 / 180.f * (float)M_PI;
		return;
	}
    m_defaultAngle1 = da1;
    m_defaultAngle2 = da2;
}

sf::Vector2f Foot::getJ1Dir(const sf::Vector2f rootDir)
{
    const sf::Vector2f normal(-rootDir.y, rootDir.x);
    const sf::Vector2f outward = attachLeft ? normal : Mul(normal, -1.f);
    return Foot::rotateCCW(outward, m_angle1 * (isFront ^ attachLeft ? 1.f : -1.f));
}

sf::Vector2f Foot::getJ2Dir(const sf::Vector2f rootDir)
{
    sf::Vector2f J1_dir = this->getJ1Dir(rootDir);
    return Foot::rotateCCW(J1_dir, m_angle2 * (isFront ^ attachLeft ? 1.f : -1.f));
}

sf::Vector2f Foot::getJoint(
    const sf::Vector2f root,
    const sf::Vector2f rootDir)
{
    const sf::Vector2f j1Dir = getJ1Dir(rootDir);
    return root + j1Dir * m_len1;
}

sf::Vector2f Foot::getEnd(
    const sf::Vector2f root,
    const sf::Vector2f rootDir)
{
    const sf::Vector2f joint = getJoint(root, rootDir);
    const sf::Vector2f j2Dir = getJ2Dir(rootDir);

    return joint + j2Dir * m_len2;
}

void Foot::captureDefaultPose()
{
    m_defaultAngle1 = m_angle1;
    m_defaultAngle2 = m_angle2;
}

void Foot::update(const sf::Vector2f& rootWorldPos,
    const sf::Vector2f& rootDir)
{
    constexpr float EPS = 1e-6f;
    auto Dot = [](const sf::Vector2f& a, const sf::Vector2f& b) {
        return a.x * b.x + a.y * b.y;
        };
    auto Cross = [](const sf::Vector2f& a, const sf::Vector2f& b) {
        return a.x * b.y - a.y * b.x;
        };
    auto Length = [](const sf::Vector2f& v) {
        return std::sqrt(v.x * v.x + v.y * v.y);
        };
    auto Normalize = [&](const sf::Vector2f& v) {
        float len = Length(v);
        if (len < EPS) {
            return sf::Vector2f(0.f, 0.f);
        }
        return sf::Vector2f(v.x / len, v.y / len);
        };
    auto SignedAngle = [&](const sf::Vector2f& a,
        const sf::Vector2f& b)
        {
            const float dot =
                a.x * b.x + a.y * b.y;
            const float crossCCW =
                a.y * b.x - a.x * b.y;
            return std::atan2(crossCCW, dot);
        };
    auto ResetFoot = [&]()
        {
            m_angle1 = m_defaultAngle1;
            m_angle2 = m_defaultAngle2;
            targetPos = getEnd(rootWorldPos, rootDir);
        };
    sf::Vector2f delta = targetPos - rootWorldPos;
    float dist = Length(delta);

    bool needStep = (dist > m_len1 + m_len2);

    if (needStep) {
        ResetFoot();
        return;
    }

    const sf::Vector2f rootDirN = Normalize(rootDir);
    const sf::Vector2f normal(-rootDirN.y, rootDirN.x);
    const sf::Vector2f outward =
        attachLeft ? normal : sf::Vector2f(-normal.x, -normal.y);

    const sf::Vector2f targetDir(delta.x / dist, delta.y / dist);
    const sf::Vector2f perpendicular(-targetDir.y, targetDir.x);

    const float a = (m_len1 * m_len1 - m_len2 * m_len2 + dist * dist) / (2.f * dist);

    float hSquared = m_len1 * m_len1 - a * a;
    hSquared = std::max(0.f, hSquared);
    const float h = std::sqrt(hSquared);

    const sf::Vector2f projectionPoint(
        rootWorldPos.x + targetDir.x * a,
        rootWorldPos.y + targetDir.y * a
    );

    const sf::Vector2f elbowA(
        projectionPoint.x + perpendicular.x * h,
        projectionPoint.y + perpendicular.y * h
    );

    const sf::Vector2f elbowB(
        projectionPoint.x - perpendicular.x * h,
        projectionPoint.y - perpendicular.y * h
    );

    struct Candidate
    {
        sf::Vector2f j1Dir;
        sf::Vector2f j2Dir;
        float openingScore;
    };
    auto MakeCandidate = [&](const sf::Vector2f& elbow) {
        Candidate candidate;
        candidate.j1Dir = Normalize(sf::Vector2f(elbow.x - rootWorldPos.x, elbow.y - rootWorldPos.y));
        candidate.j2Dir = Normalize(sf::Vector2f(targetPos.x - elbow.x, targetPos.y - elbow.y));
        sf::Vector2f opening(
            -candidate.j1Dir.x + candidate.j2Dir.x,
            -candidate.j1Dir.y + candidate.j2Dir.y
        );
        float openingLength = Length(opening);
        if (openingLength > EPS) {
            opening.x /= openingLength;
            opening.y /= openingLength;
            const sf::Vector2f backward( -rootDirN.x, -rootDirN.y);
            candidate.openingScore = Dot(opening, backward);
        }
        else {
            candidate.openingScore = 0.f;
        }
        return candidate;
    };
    const Candidate candidateA = MakeCandidate(elbowA);
    const Candidate candidateB = MakeCandidate(elbowB);
        const Candidate& chosen =
            candidateA.openingScore >= candidateB.openingScore
            ? (isFront ? candidateA : candidateB)
            : (isFront ? candidateB : candidateA);

        const float PI = 3.14159265358979323846f;
        const float DEG_TO_RAD = PI / 180.0f;
        const float FRONT_MIN_ELBOW = m_kinematicsConfig.frontMinElbowDeg * DEG_TO_RAD;
        const float HIND_MAX_ELBOW = m_kinematicsConfig.hindMaxElbowDeg * DEG_TO_RAD;

        const sf::Vector2f elbowToRoot( -chosen.j1Dir.x, -chosen.j1Dir.y);
        float cosElbow = Dot(elbowToRoot, chosen.j2Dir);
        cosElbow = std::clamp(cosElbow, -1.0f, 1.0f);
        const float elbowAngle = std::acos(cosElbow);
        bool invalidPose = false;
        if (isFront)
        {
            constexpr float OUTWARD_EPS = 1e-4f;
            const float outwardComponent = Dot(chosen.j1Dir, outward);
            if (outwardComponent <= OUTWARD_EPS)
                invalidPose = true;
            if (elbowAngle < FRONT_MIN_ELBOW)
                invalidPose = true;
        }
        else
            if (elbowAngle > HIND_MAX_ELBOW)
                invalidPose = true;

        if (invalidPose)
            ResetFoot();
    const float sign =
        (isFront ^ attachLeft) ? 1.f : -1.f;
    const float actualAngle1 =
        SignedAngle(outward, chosen.j1Dir);
    const float actualAngle2 =
        SignedAngle(chosen.j1Dir, chosen.j2Dir);
    m_angle1 = actualAngle1 * sign;
    m_angle2 = actualAngle2 * sign;
}

void Foot::render(
    const sf::Vector2f& root,
    const sf::Vector2f& rootDir,
    sf::RenderWindow& window, const sf::Color color)
{
    sf::Vector2f dir1 = getJ1Dir(rootDir);
    sf::Vector2f dir2 = getJ2Dir(rootDir);

	sf::Vector2f j1 = getJoint(root, rootDir);
	sf::Vector2f j2 = getEnd(root, rootDir);

    sf::Vector2f n1(-dir1.y, dir1.x);
    sf::Vector2f n2(-dir2.y, dir2.x);

	const float radius = m_renderConfig.radius;
    const float width = radius * 2.f;
    const sf::Color footColor = color;
    sf::RectangleShape arm1(sf::Vector2f(m_len1, width));
    arm1.setOrigin(0.f, radius);
    arm1.setPosition(root);
    const float angle1 = std::atan2(dir1.y, dir1.x) * 180.f / (float)M_PI;
    arm1.setRotation(angle1);
    arm1.setFillColor(footColor);
    window.draw(arm1);

    sf::RectangleShape arm2(sf::Vector2f(m_len2, width));
    arm2.setOrigin(0.f, radius);
    arm2.setPosition(j1);
    const float angle2 = std::atan2(dir2.y, dir2.x) * 180.f / (float)M_PI;
    arm2.setRotation(angle2);
    arm2.setFillColor(footColor);
    window.draw(arm2);

    sf::CircleShape circle(radius);
    circle.setOrigin(radius, radius);
    circle.setFillColor(footColor);
    circle.setPosition(root);
    window.draw(circle);

    circle.setPosition(j1);
    window.draw(circle);
    circle.setPosition(j2);
    window.draw(circle);
}

sf::Vector2f Foot::rotateCCW(
    const sf::Vector2f& direction,
    float angleRadians)
{
    const float cosA = std::cos(angleRadians);
    const float sinA = std::sin(angleRadians);

    return sf::Vector2f(
        direction.x * cosA + direction.y * sinA,
        -direction.x * sinA + direction.y * cosA
    );
}
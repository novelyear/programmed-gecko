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

Foot::Foot() {}

Foot::Foot(float len1, float len2, bool isFront, bool attachLeft)
    : m_len1(len1), m_len2(len2), isFront(isFront), attachLeft(attachLeft)
{
    m_angle1 = m_defaultAngle1;
    m_angle2 = m_defaultAngle2;
}

void Foot::setTargetPos(const sf::Vector2f& pos) { targetPos = pos; }
void Foot::setDefaultAngle(const float da1, const float da2)
{
	if (da1 < 0.f || da1 > M_PI || da2 < 0.f || da2 > M_PI) {
		m_defaultAngle1 = da1 / 180.f * M_PI;
		m_defaultAngle2 = da2 / 180.f * M_PI;
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
    sf::Vector2f delta = targetPos - rootWorldPos;
    float dist = Length(delta);

    bool needStep = (dist > m_len1 + m_len2);

    if (needStep) {
        m_angle1 = m_defaultAngle1;
        m_angle2 = m_defaultAngle2;

        targetPos = this->getEnd(rootWorldPos, rootDir);
        return;
    }

    const sf::Vector2f rootDirN = Normalize(rootDir);

    const sf::Vector2f normal(-rootDirN.y, rootDirN.x);
    const sf::Vector2f outward =
        attachLeft ? normal : sf::Vector2f(-normal.x, -normal.y);

    const sf::Vector2f targetDir(
        delta.x / dist,
        delta.y / dist
    );

    const sf::Vector2f perpendicular(
        -targetDir.y,
        targetDir.x
    );

    const float a =
        (m_len1 * m_len1
            - m_len2 * m_len2
            + dist * dist)
        / (2.f * dist);

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
        candidate.j1Dir = Normalize(
            sf::Vector2f(
                elbow.x - rootWorldPos.x,
                elbow.y - rootWorldPos.y
            )
        );
        candidate.j2Dir = Normalize(
            sf::Vector2f(
                targetPos.x - elbow.x,
                targetPos.y - elbow.y
            )
        );
        sf::Vector2f opening(
            -candidate.j1Dir.x + candidate.j2Dir.x,
            -candidate.j1Dir.y + candidate.j2Dir.y
        );
        float openingLength = Length(opening);
        if (openingLength > EPS) {
            opening.x /= openingLength;
            opening.y /= openingLength;
            const sf::Vector2f backward(
                -rootDirN.x,
                -rootDirN.y
            );
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
    sf::RenderWindow& window)
{
    sf::Vector2f dir1 = getJ1Dir(rootDir);
    sf::Vector2f dir2 = getJ2Dir(rootDir);

	sf::Vector2f j1 = getJoint(root, rootDir);
	sf::Vector2f j2 = getEnd(root, rootDir);

    sf::Vector2f n1(-dir1.y, dir1.x);
    sf::Vector2f n2(-dir2.y, dir2.x);

    const float radius = 12.f;
    const float extend = radius * 1.5f;

    std::vector<sf::Vector2f> pts;

    pts.push_back(root - n1 * radius - dir1 * extend);

    pts.push_back(j1 - n1 * radius);
    pts.push_back(j2 - n2 * radius);

    pts.push_back(j2 + dir2 * radius);

    pts.push_back(j2 + n2 * radius);
    pts.push_back(j1 + n1 * radius);

    pts.push_back(root + n1 * radius - dir1 * extend);

    pts.push_back(root - n1 * radius - dir1 * extend);

    const int samplesPerSeg = 12;
    std::vector<sf::Vector2f> smooth =
        SampleSmoothPolyline(pts, samplesPerSeg);

    sf::VertexArray va(sf::LineStrip, smooth.size());

    for (size_t i = 0; i < smooth.size(); ++i) {
        va[i].position = smooth[i];
        va[i].color = sf::Color(200, 150, 100);
    }

    window.draw(va);

    sf::VertexArray lines(sf::LinesStrip, 3);
    lines[0].position = root;
    lines[0].color = sf::Color::White;
    lines[1].position = j1;
    lines[1].color = sf::Color::White;
    lines[2].position = j2;
    lines[2].color = sf::Color::White;
    window.draw(lines);

    const float pointRadius = 4.f;
    sf::CircleShape dot(pointRadius);
    dot.setOrigin(pointRadius, pointRadius);

    dot.setFillColor(sf::Color::Blue);
    dot.setPosition(root);
    window.draw(dot);

    dot.setFillColor(sf::Color::Green);
    dot.setPosition(j1);
    window.draw(dot);

    dot.setFillColor(sf::Color::Red);
    dot.setPosition(j2);
    window.draw(dot);

	dot.setFillColor(sf::Color::Yellow);
	dot.setPosition(targetPos);
	window.draw(dot);
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
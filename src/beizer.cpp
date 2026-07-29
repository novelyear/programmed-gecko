#include "pch.h"
#include <SFML/Graphics.hpp>
#include <vector>

namespace
{
    sf::Vector2f Add(const sf::Vector2f& a, const sf::Vector2f& b) { return { a.x + b.x, a.y + b.y }; }
    sf::Vector2f Sub(const sf::Vector2f& a, const sf::Vector2f& b) { return { a.x - b.x, a.y - b.y }; }
    sf::Vector2f Mul(const sf::Vector2f& v, float s) { return { v.x * s, v.y * s }; }

    float Length(const sf::Vector2f& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    sf::Vector2f Normalize(const sf::Vector2f& v)
    {
        const float len = Length(v);
        if (len < 1e-5f) return { 1.0f, 0.0f };
        return { v.x / len, v.y / len };
    }

    sf::Vector2f CubicBezier(
        const sf::Vector2f& p0,
        const sf::Vector2f& p1,
        const sf::Vector2f& p2,
        const sf::Vector2f& p3,
        float t)
    {
        const float u = 1.0f - t;
        const float b0 = u * u * u;
        const float b1 = 3.0f * u * u * t;
        const float b2 = 3.0f * u * t * t;
        const float b3 = t * t * t;
        return Add(Add(Mul(p0, b0), Mul(p1, b1)), Add(Mul(p2, b2), Mul(p3, b3)));
    }

    // Catmull-Rom -> Bezier 控制点
    void BuildBezierSegment(
        const sf::Vector2f& pm1,
        const sf::Vector2f& p0,
        const sf::Vector2f& p1,
        const sf::Vector2f& p2,
        sf::Vector2f& b0, sf::Vector2f& b1, sf::Vector2f& b2, sf::Vector2f& b3)
    {
        b0 = p0;
        b1 = Add(p0, Mul(Sub(p1, pm1), 1.0f / 6.0f));
        b2 = Sub(p1, Mul(Sub(p2, p0), 1.0f / 6.0f));
        b3 = p1;
    }

    std::vector<sf::Vector2f> SampleSmoothPolyline(const std::vector<sf::Vector2f>& pts, int samplesPerSeg)
    {
        std::vector<sf::Vector2f> out;
        if (pts.size() < 2) return out;
        if (pts.size() == 2)
        {
            out.push_back(pts[0]);
            out.push_back(pts[1]);
            return out;
        }

        for (size_t i = 0; i + 1 < pts.size(); ++i)
        {
            const sf::Vector2f pm1 = (i == 0) ? pts[i] : pts[i - 1];
            const sf::Vector2f p0 = pts[i];
            const sf::Vector2f p1 = pts[i + 1];
            const sf::Vector2f p2 = (i + 2 < pts.size()) ? pts[i + 2] : pts[i + 1];

            sf::Vector2f b0, b1, b2, b3;
            BuildBezierSegment(pm1, p0, p1, p2, b0, b1, b2, b3);

            for (int s = 0; s < samplesPerSeg; ++s)
            {
                const float t = static_cast<float>(s) / static_cast<float>(samplesPerSeg);
                out.push_back(CubicBezier(b0, b1, b2, b3, t));
            }
        }
        out.push_back(pts.back());
        return out;
    }
}
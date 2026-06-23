#include "pch.h"
#include "Renderer.h"
#include <cmath>
#include <algorithm>

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

    // body: [ [x,y,r], [x,y,r], ... ]
    void BuildBodySides(
        const std::vector<std::vector<float>> body,
        std::vector<sf::Vector2f>& leftSide,
        std::vector<sf::Vector2f>& rightSide)
    {
        leftSide.clear();
        rightSide.clear();
        if (body.size() < 2) return;

        for (size_t i = 0; i < body.size(); ++i)
        {
            const sf::Vector2f p(body[i][0], body[i][1]);
            const float r = body[i][2];

            sf::Vector2f tangent;
            if (i == 0)
            {
                tangent = Sub(sf::Vector2f(body[i + 1][0], body[i + 1][1]), p);
            }
            else if (i == body.size() - 1)
            {
                tangent = Sub(p, sf::Vector2f(body[i - 1][0], body[i - 1][1]));
            }
            else
            {
                tangent = Sub(sf::Vector2f(body[i + 1][0], body[i + 1][1]),
                              sf::Vector2f(body[i - 1][0], body[i - 1][1]));
            }

            const sf::Vector2f t = Normalize(tangent);
            const sf::Vector2f n(-t.y, t.x); // 左法线

            if (i == 0) 
            {
                leftSide.push_back(Sub(p, Mul(t, r))); // head
            }
            leftSide.push_back(Add(p, Mul(n, r)));
            rightSide.push_back(Sub(p, Mul(n, r)));
            if (i == body.size() - 1)
            {
                rightSide.push_back(Add(p, Mul(t, r))); // tail
            }
        }
    }
}

Renderer::Renderer(int width, int height, const sf::String& title)
    : window(sf::VideoMode((unsigned int)width, (unsigned int)height), title)  {}

void Renderer::render(const std::vector<std::vector<float>> body,
                      const std::vector<std::vector<float>> foot)
{
    window.clear(sf::Color::Black);

    std::vector<sf::Vector2f> leftSide;
    std::vector<sf::Vector2f> rightSide;
    BuildBodySides(body, leftSide, rightSide);

    const std::vector<sf::Vector2f> leftSmooth = SampleSmoothPolyline(leftSide, 240);

    std::reverse(rightSide.begin(), rightSide.end());
    const std::vector<sf::Vector2f> rightSmooth = SampleSmoothPolyline(rightSide, 240);

    std::vector<sf::Vector2f> outline;
    outline.reserve(leftSmooth.size() + rightSmooth.size() + 1);
    outline.insert(outline.end(), leftSmooth.begin(), leftSmooth.end());
    outline.insert(outline.end(), rightSmooth.begin(), rightSmooth.end());
    if (!outline.empty()) outline.push_back(outline.front());

    sf::VertexArray va(sf::LineStrip, outline.size());
    for (size_t i = 0; i < outline.size(); ++i)
    {
        va[i].position = outline[i];
        va[i].color = sf::Color(120, 220, 120);
    }
    window.draw(va);

    window.display();
}

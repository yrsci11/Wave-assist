#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace wa {

struct Box {
    float x0, y0, x1, y1;
};

inline bool overlapE(Box const& a, Box const& b, float e) {
    return a.x0 + e < b.x1 && a.x1 - e > b.x0 && a.y0 + e < b.y1 && a.y1 - e > b.y0;
}

struct World {
    std::vector<Box> hazards;
    std::vector<Box> solids;
    void clear() {
        hazards.clear();
        solids.clear();
    }
    bool empty() const { return hazards.empty() && solids.empty(); }
};

struct Params {
    float x = 0.f, y = 0.f;
    float dx = 1.3f;      // x travel per physics step
    float slope = 1.f;    // |dy| / dx
    float half = 5.f;     // wave half hitbox
    float margin = 1.5f;  // extra clearance vs hazards
    float floorY = -1e9f; // learned slide bounds (wave center)
    float ceilY = 1e9f;
    int stride = 2;       // physics steps per planner step
    int horizon = 50;     // planner steps to look ahead
    int upSign = 1;       // +1 if holding moves up, -1 if down
};

struct PlanResult {
    bool full = false;
    int depth = 0;
    std::vector<char> acts; // 1 = hold, 0 = release
};

class Planner {
public:
    Planner(World const& w, Params const& p) : m_w(w), m_p(p) {}

    // forced: first N steps must use forcedHold. prefer: input kept when free.
    PlanResult plan(int forced, bool forcedHold, bool prefer) {
        m_forced = forced;
        m_forcedHold = forcedHold;
        m_budget = 3000;
        m_visited.clear();
        m_cur.assign(static_cast<size_t>(m_p.horizon), 0);
        m_best.clear();
        m_bestDepth = -1;
        PlanResult r;
        r.full = dfs(0, m_p.y, prefer);
        r.depth = m_bestDepth;
        r.acts = m_best;
        return r;
    }

private:
    bool step(int d, float y, bool hold, float& outY) const {
        const float stepU = m_p.dx * static_cast<float>(m_p.stride);
        const float x0 = m_p.x + static_cast<float>(d) * stepU;
        const float nx = x0 + stepU;
        const float h = m_p.half;
        const float dir = static_cast<float>(hold ? m_p.upSign : -m_p.upSign);
        float ny = std::clamp(y + dir * m_p.slope * stepU, m_p.floorY, m_p.ceilY);

        // 1) side hit on a solid is fatal
        Box a{nx - h, y - h, nx + h, y + h};
        for (auto const& s : m_w.solids) {
            if (overlapE(a, s, 0.05f)) return false;
        }

        // 2) vertical move slides along block tops/bottoms
        bool settled = false;
        for (int it = 0; it < 3; ++it) {
            Box b{nx - h, ny - h, nx + h, ny + h};
            Box const* hit = nullptr;
            for (auto const& s : m_w.solids) {
                if (overlapE(b, s, 0.05f)) {
                    hit = &s;
                    break;
                }
            }
            if (!hit) {
                settled = true;
                break;
            }
            ny = (dir > 0.f) ? hit->y0 - h - 0.02f : hit->y1 + h + 0.02f;
        }
        if (!settled) return false;

        // 3) hazards over the swept area
        const float m = m_p.margin;
        Box u{
            std::min(x0, nx) - h - m, std::min(y, ny) - h - m,
            std::max(x0, nx) + h + m, std::max(y, ny) + h + m
        };
        for (auto const& hz : m_w.hazards) {
            if (overlapE(u, hz, 0.f)) return false;
        }

        outY = ny;
        return true;
    }

    bool dfs(int d, float y, bool last) {
        if (d > m_bestDepth) {
            m_bestDepth = d;
            m_best.assign(m_cur.begin(), m_cur.begin() + d);
        }
        if (d >= m_p.horizon) return true;
        if (m_budget-- <= 0) return false;

        int64_t key = (static_cast<int64_t>(d) << 32) |
                      static_cast<uint32_t>(static_cast<int32_t>(std::lround(y * 2.f)));
        if (!m_visited.insert(key).second) return false;

        bool order[2];
        int n = 0;
        if (d < m_forced) {
            order[n++] = m_forcedHold;
        } else {
            order[n++] = last;
            order[n++] = !last;
        }
        for (int i = 0; i < n; ++i) {
            float ny = 0.f;
            if (!step(d, y, order[i], ny)) continue;
            m_cur[static_cast<size_t>(d)] = order[i] ? 1 : 0;
            if (dfs(d + 1, ny, order[i])) return true;
        }
        return false;
    }

    World const& m_w;
    Params m_p;
    int m_forced = 0;
    bool m_forcedHold = false;
    int m_budget = 0;
    int m_bestDepth = -1;
    std::unordered_set<int64_t> m_visited;
    std::vector<char> m_cur;
    std::vector<char> m_best;
};

} // namespace wa

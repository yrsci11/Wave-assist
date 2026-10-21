#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/PlayerObject.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

#include "Planner.hpp"

using namespace geode::prelude;

namespace {

struct Cached {
    float minX;
    GameObject* obj;
    bool hazard;
};

struct PState {
    bool userHold = false; // what the player wants
    bool real = false;     // what the game is currently doing
    bool applied = false;  // hold state used for the previous step
    bool havePrev = false;
    bool prevDart = false;
    float px = 0.f, py = 0.f;
    float dx = 0.f;
    float slope = 1.f;
    int upSign = 1;
    float floorY = -1e9f;
    float ceilY = 1e9f;
};

PState g_st[2];
std::vector<Cached> g_cache;
wa::World g_world;
float g_maxW = 30.f;
bool g_assisted = false;

bool g_enabled = true;
bool g_safe = true;
float g_margin = 1.5f;
float g_horizonUnits = 180.f;
int g_slack = 8;
int g_tick = 0;

void refreshSettings() {
    auto mod = Mod::get();
    g_enabled = mod->getSettingValue<bool>("enabled");
    g_safe = mod->getSettingValue<bool>("safe-mode");
    g_margin = static_cast<float>(mod->getSettingValue<double>("margin"));
    auto s = mod->getSettingValue<std::string>("strength");
    if (s == "Light") {
        g_horizonUnits = 70.f;
        g_slack = 0;
    } else if (s == "Medium") {
        g_horizonUnits = 120.f;
        g_slack = 3;
    } else {
        g_horizonUnits = 180.f;
        g_slack = 8;
    }
}

void buildCache(PlayLayer* pl) {
    g_cache.clear();
    g_maxW = 30.f;
    if (!pl->m_objects) return;
    for (auto obj : CCArrayExt<GameObject*>(pl->m_objects)) {
        if (!obj) continue;
        bool hz = obj->m_objectType == GameObjectType::Hazard;
        bool so = obj->m_objectType == GameObjectType::Solid;
        if (!hz && !so) continue;
        auto r = obj->getObjectRect();
        g_cache.push_back({r.origin.x, obj, hz});
        g_maxW = std::max(g_maxW, std::min(r.size.width, 600.f));
    }
    std::sort(g_cache.begin(), g_cache.end(), [](Cached const& a, Cached const& b) {
        return a.minX < b.minX;
    });
}

void showLabel(PlayLayer* pl, bool show) {
    if (!pl || !pl->m_uiLayer) return;
    auto id = "assisted-label"_spr;
    auto l = typeinfo_cast<CCLabelBMFont*>(pl->m_uiLayer->getChildByID(id));
    if (!l) {
        if (!show) return;
        l = CCLabelBMFont::create("Assisted", "bigFont.fnt");
        l->setID(id);
        l->setScale(0.35f);
        l->setOpacity(140);
        auto ws = CCDirector::get()->getWinSize();
        l->setPosition({ws.width / 2.f, 14.f});
        pl->m_uiLayer->addChild(l, 100);
    }
    l->setVisible(show);
}

void runAssist(PlayLayer* pl, PlayerObject* p, bool isP1) {
    auto& st = g_st[isP1 ? 0 : 1];
    CCPoint pos = p->getPosition();
    bool dart = p->m_isDart;
    bool prevApplied = st.applied;

    if (!g_enabled || !dart || p->m_isDead) {
        st.havePrev = false;
        st.prevDart = dart;
        st.applied = st.real;
        return;
    }

    if (!st.prevDart) {
        st.floorY = -1e9f;
        st.ceilY = 1e9f;
        st.upSign = p->m_isUpsideDown ? -1 : 1;
        st.havePrev = false;
    }
    st.prevDart = true;

    bool had = st.havePrev;
    if (had) {
        float mx = pos.x - st.px;
        float my = pos.y - st.py;
        if (mx > 0.05f && mx < 30.f && std::fabs(my) < 100.f) {
            st.dx = mx;
            if (std::fabs(my) > 0.02f) {
                st.slope = std::clamp(std::fabs(my) / mx, 0.25f, 4.f);
                st.upSign = ((my > 0.f) == prevApplied) ? 1 : -1;
            } else {
                int attempted = prevApplied ? st.upSign : -st.upSign;
                if (attempted > 0) st.ceilY = pos.y;
                else st.floorY = pos.y;
            }
        } else {
            had = false; // teleport / respawn / portal
        }
    }
    if (pos.y < st.floorY - 0.5f) st.floorY = -1e9f;
    if (pos.y > st.ceilY + 0.5f) st.ceilY = 1e9f;

    st.px = pos.x;
    st.py = pos.y;
    st.havePrev = true;
    if (!had || st.dx <= 0.f) {
        st.applied = st.real;
        return;
    }

    wa::Params prm;
    prm.x = pos.x;
    prm.y = pos.y;
    prm.dx = st.dx;
    prm.slope = st.slope;
    prm.half = 5.f * std::clamp(p->m_vehicleSize, 0.3f, 1.5f);
    prm.margin = g_margin;
    prm.floorY = st.floorY;
    prm.ceilY = st.ceilY;
    prm.upSign = st.upSign;
    prm.stride = std::clamp(static_cast<int>(std::lround(3.f / st.dx)), 1, 8);
    float stepU = st.dx * static_cast<float>(prm.stride);
    prm.horizon = std::max(4, static_cast<int>(std::ceil(g_horizonUnits / stepU)));

    float reachX = stepU * static_cast<float>(prm.horizon);
    float reachY = st.slope * reachX + 30.f;

    g_world.clear();
    auto it = std::lower_bound(
        g_cache.begin(), g_cache.end(), pos.x - 20.f - g_maxW,
        [](Cached const& c, float v) { return c.minX < v; }
    );
    for (; it != g_cache.end() && it->minX <= pos.x + reachX + 40.f; ++it) {
        auto r = it->obj->getObjectRect();
        float x0 = r.origin.x, y0 = r.origin.y;
        float x1 = x0 + r.size.width, y1 = y0 + r.size.height;
        if (x1 < pos.x - 20.f || y1 < pos.y - reachY || y0 > pos.y + reachY) continue;
        (it->hazard ? g_world.hazards : g_world.solids).push_back({x0, y0, x1, y1});
        if (g_world.hazards.size() + g_world.solids.size() > 600) break;
    }
    if (g_world.empty()) {
        st.applied = st.real;
        return;
    }

    wa::Planner planner(g_world, prm);
    bool user = st.userHold;
    bool want = user;

    auto now = planner.plan(0, false, user);
    if (now.full && !now.acts.empty()) {
        int k = std::min(g_slack, prm.horizon - 1);
        auto keep = planner.plan(k, user, user);
        if (!keep.full) want = now.acts[0] != 0;
    }

    if (want != st.real) {
        if (want) p->pushButton(PlayerButton::Jump);
        else p->releaseButton(PlayerButton::Jump);
        st.real = want;
    }
    if (want != user && !g_assisted) {
        g_assisted = true;
        showLabel(pl, true);
    }
    st.applied = st.real;
}

} // namespace

$on_mod(Loaded) {
    refreshSettings();
}

class $modify(WAGameLayer, GJBaseGameLayer) {
    void handleButton(bool down, int button, bool isPlayer1) {
        if (button == 1) {
            bool two = m_levelSettings && m_levelSettings->m_twoPlayerMode;
            if (isPlayer1) {
                g_st[0].userHold = down;
                g_st[0].real = down;
                if (!two) {
                    g_st[1].userHold = down;
                    g_st[1].real = down;
                }
            } else {
                g_st[1].userHold = down;
                g_st[1].real = down;
            }
        }
        GJBaseGameLayer::handleButton(down, button, isPlayer1);
    }
};

class $modify(WAPlayer, PlayerObject) {
    void update(float dt) {
        auto pl = PlayLayer::get();
        if (pl && (this == pl->m_player1 || this == pl->m_player2)) {
            if ((g_tick++ & 127) == 0) refreshSettings();
            runAssist(pl, this, this == pl->m_player1);
        }
        PlayerObject::update(dt);
    }
};

class $modify(WAPlayLayer, PlayLayer) {
    void setupHasCompleted() {
        PlayLayer::setupHasCompleted();
        refreshSettings();
        buildCache(this);
        g_assisted = false;
        g_st[0] = PState{};
        g_st[1] = PState{};
    }

    void resetLevel() {
        PlayLayer::resetLevel();
        g_assisted = false;
        g_st[0] = PState{};
        g_st[1] = PState{};
        showLabel(this, false);
    }

    void levelComplete() {
        if (g_assisted && g_safe) {
            m_isTestMode = true; // blocks saving/submitting this completion
        }
        PlayLayer::levelComplete();
    }

    void onQuit() {
        g_cache.clear();
        g_assisted = false;
        PlayLayer::onQuit();
    }
};

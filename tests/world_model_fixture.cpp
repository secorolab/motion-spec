// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
//
// Compiled against the real rendered runtime header. Three modes:
//   verify -- world-model poses against KDL chain FK, plus every startup and freshness failure
//   alloc  -- the valid hot path allocates nothing
//   bench  -- one control cycle's worth of reads, chain FK shape against world-model shape

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <vector>

#include <kdl/chainfksolverpos_recursive.hpp>

#include "runtime.hpp"

namespace {

// Counted only while armed, so construction and sizing do not show up as hot-path allocations.
std::atomic<long> g_allocations{0};
bool g_counting = false;

int g_failures = 0;

void check(bool ok, const char *what) {
    if (!ok) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

using motion_spec::runtime::WorldModel;

constexpr int kJointsPerArm = 3;

// A branched tree shaped like the maintained scenes: a fixed world/table prefix above each arm,
// so no chain root is the tree root, and a fixed offset leaf at each tip.
void add_arm(KDL::Tree &tree, const std::string &prefix, const std::string &parent, double y) {
    tree.addSegment(KDL::Segment(prefix + "base", KDL::Joint(KDL::Joint::None),
                                 KDL::Frame(KDL::Rotation::RotZ(0.3), KDL::Vector(0.1, y, 0.05))),
                    parent);
    tree.addSegment(KDL::Segment(prefix + "1", KDL::Joint(prefix + "j1", KDL::Joint::RotZ),
                                 KDL::Frame(KDL::Vector(0.0, 0.0, 0.28))),
                    prefix + "base");
    tree.addSegment(KDL::Segment(prefix + "2", KDL::Joint(prefix + "j2", KDL::Joint::RotY),
                                 KDL::Frame(KDL::Rotation::RotX(0.2), KDL::Vector(0.0, 0.05, 0.31))),
                    prefix + "1");
    tree.addSegment(KDL::Segment(prefix + "3", KDL::Joint(prefix + "j3", KDL::Joint::RotX),
                                 KDL::Frame(KDL::Vector(0.02, 0.0, 0.21))),
                    prefix + "2");
    // The leaf plan 04 gives a posed frame: a constant offset the tree composes itself.
    tree.addSegment(KDL::Segment(prefix + "site", KDL::Joint(KDL::Joint::None),
                                 KDL::Frame(KDL::Rotation::RotY(0.7), KDL::Vector(0.0, 0.03, 0.07))),
                    prefix + "3");
}

KDL::Tree scene_tree() {
    KDL::Tree tree("w/root");
    tree.addSegment(KDL::Segment("w/table", KDL::Joint(KDL::Joint::None),
                                 KDL::Frame(KDL::Vector(0.4, 0.0, 0.75))),
                    "w/root");
    add_arm(tree, "w/a", "w/table", -0.3);
    add_arm(tree, "w/b", "w/table", 0.3);
    return tree;
}

KDL::Tree second_tree() {
    KDL::Tree tree("s/root");
    add_arm(tree, "s/a", "s/root", 0.0);
    return tree;
}

// Every arm's joints, in the order the sliced chain articulates them.
struct Arm {
    std::string prefix;
    std::string root;
    std::string tip;
    KDL::Chain chain;
    std::vector<double> q = std::vector<double>(kJointsPerArm, 0.0);
    int world_root = -1;
    int world_tip = -1;
    int world_elbow = -1;
};

Arm slice(const KDL::Tree &tree, const std::string &prefix) {
    Arm arm;
    arm.prefix = prefix;
    arm.root = prefix + "base";
    arm.tip = prefix + "site";
    if (!tree.getChain(arm.root, arm.tip, arm.chain)) {
        std::printf("FAIL could not slice the chain %s\n", prefix.c_str());
        ++g_failures;
    }
    return arm;
}

bool bind(WorldModel &world, Arm &arm, std::string &error) {
    for (int i = 0; i < kJointsPerArm; ++i) {
        if (!world.bind_joint(arm.prefix + std::to_string(i + 1), &arm.q[i], error)) return false;
    }
    return true;
}

void resolve(WorldModel &world, Arm &arm) {
    arm.world_root = world.index_of(arm.root);
    arm.world_tip = world.index_of(arm.tip);
    arm.world_elbow = world.index_of(arm.prefix + "2");
}

double frame_error(const KDL::Frame &a, const KDL::Frame &b) {
    const KDL::Twist delta = KDL::diff(a, b);
    return std::max(delta.vel.Norm(), delta.rot.Norm());
}

// What a chain solver would have reported: the pose of one segment relative to the chain's root.
KDL::Frame chain_pose(const Arm &arm, int segment) {
    KDL::ChainFkSolverPos_recursive fk(arm.chain);
    KDL::JntArray q(kJointsPerArm);
    for (int i = 0; i < kJointsPerArm; ++i) q(i) = arm.q[i];
    KDL::Frame frame;
    fk.JntToCart(q, frame, segment);
    return frame;
}

KDL::Frame world_pose(const WorldModel &world, const Arm &arm, int index, std::uint64_t cycle) {
    return world.pose(arm.world_root, cycle).Inverse() * world.pose(index, cycle);
}

constexpr double kTolerance = 1e-10;

void verify_against_chain_fk() {
    KDL::Tree scene = scene_tree();
    KDL::Tree other = second_tree();
    WorldModel world;
    std::string error;
    check(world.add_tree(scene, error), "the scene tree is accepted");
    check(world.add_tree(other, error), "a second, independent tree is accepted");

    Arm a = slice(scene, "w/a");
    Arm b = slice(scene, "w/b");
    Arm s = slice(other, "s/a");
    check(bind(world, a, error) && bind(world, b, error) && bind(world, s, error),
          "every arm's joints bind");
    // Binding the same segment to the same address again is the same fact stated twice: several
    // solver records share one runtime.
    check(world.bind_joint("w/a1", &a.q[0], error), "rebinding the same address succeeds");
    check(!world.bind_joint("w/a1", &b.q[0], error), "rebinding another address fails");

    for (Arm *arm : {&a, &b, &s}) {
        resolve(world, *arm);
        check(arm->world_root >= 0 && arm->world_tip >= 0 && arm->world_elbow >= 0,
              "every required name resolves");
        check(world.require_path(arm->world_root, arm->world_tip, error), "the tip is required");
        check(world.require_path(arm->world_root, arm->world_elbow, error), "the elbow is required");
    }
    check(world.seal(error), "a fully bound model seals");

    // Zero, mixed signs, and the ends of a Kinova-sized joint range.
    const double configurations[][kJointsPerArm] = {
        {0.0, 0.0, 0.0},      {0.4, -1.1, 2.0},     {-2.41, 2.66, -2.23},
        {2.41, -2.66, 2.23},  {3.14159, -0.0001, 1.5707963},
    };
    std::uint64_t cycle = 0;
    for (const auto &configuration : configurations) {
        for (int i = 0; i < kJointsPerArm; ++i) {
            a.q[i] = configuration[i];
            b.q[i] = -0.5 * configuration[i];
            s.q[i] = 0.25 * configuration[i];
        }
        world.update(cycle);
        for (const Arm *arm : {&a, &b, &s}) {
            // The tip is the fixed offset leaf: it has to agree with chain FK, offset and all.
            check(frame_error(chain_pose(*arm, -1), world_pose(world, *arm, arm->world_tip, cycle))
                      <= kTolerance,
                  "the world model agrees with chain FK at the offset leaf");
            check(frame_error(chain_pose(*arm, 2), world_pose(world, *arm, arm->world_elbow, cycle))
                      <= kTolerance,
                  "the world model agrees with chain FK partway along the chain");
        }
        ++cycle;
    }

    // One branch moving must not move the other, in either tree.
    const KDL::Frame b_before = world_pose(world, b, b.world_tip, cycle - 1);
    const KDL::Frame s_before = world_pose(world, s, s.world_tip, cycle - 1);
    a.q[0] += 0.9;
    world.update(cycle);
    check(frame_error(b_before, world_pose(world, b, b.world_tip, cycle)) == 0.0,
          "moving one arm leaves the other branch where it was");
    check(frame_error(s_before, world_pose(world, s, s.world_tip, cycle)) == 0.0,
          "moving one arm leaves the second tree where it was");
    check(frame_error(chain_pose(a, -1), world_pose(world, a, a.world_tip, cycle)) <= kTolerance,
          "the arm that moved still agrees with chain FK");
}

// The stop flag is the one failure channel, so each case clears it and asserts it was raised.
bool raised_stop(void (*body)()) {
    motion_spec::runtime::g_stop_requested = 0;
    body();
    const bool raised = motion_spec::runtime::stop_requested();
    motion_spec::runtime::g_stop_requested = 0;
    return raised;
}

void verify_startup_failures() {
    KDL::Tree scene = scene_tree();
    std::string error;

    WorldModel duplicate;
    check(duplicate.add_tree(scene, error), "the first tree is accepted");
    check(!duplicate.add_tree(scene, error), "a duplicate segment name is rejected");

    WorldModel world;
    check(world.add_tree(scene, error), "the scene tree is accepted");
    check(world.add_tree(second_tree(), error), "the second tree is accepted");
    check(world.index_of("w/nowhere") == -1, "an unknown segment has no index");
    double measurement = 0.0;
    check(!world.bind_joint("w/nowhere", &measurement, error), "an unknown segment cannot bind");
    check(!world.bind_joint("w/table", &measurement, error), "a fixed segment cannot bind");
    check(!world.bind_joint("w/a1", nullptr, error), "a null measurement cannot bind");
    check(!world.require_path(world.index_of("w/a base"), world.index_of("w/asite"), error),
          "a path naming no segment is rejected");
    check(!world.require_path(world.index_of("w/abase"), world.index_of("s/asite"), error),
          "a path across two trees is rejected");
    check(!world.require_path(world.index_of("w/a3"), world.index_of("w/b3"), error),
          "a target that is not below its reference is rejected");

    // One joint of the required path left unbound: reading through it would evaluate at zero.
    Arm a = slice(scene, "w/a");
    check(world.bind_joint("w/a1", &a.q[0], error), "the first joint binds");
    check(world.bind_joint("w/a2", &a.q[1], error), "the second joint binds");
    check(world.require_path(world.index_of("w/abase"), world.index_of("w/asite"), error),
          "the tip path is required");
    check(!world.seal(error), "sealing rejects an unbound joint on a required path");
    check(error.find("w/aj3") != std::string::npos, "the unbound joint is named");
    check(world.bind_joint("w/a3", &a.q[2], error), "the third joint binds");
    check(world.seal(error), "sealing succeeds once every required joint is bound");
    check(!world.add_tree(second_tree(), error), "a sealed model takes no more trees");
    check(!world.bind_joint("w/b1", &measurement, error), "a sealed model binds nothing more");
    check(!world.require_path(0, 1, error), "a sealed model requires no more paths");
}

WorldModel *g_freshness = nullptr;

void verify_freshness() {
    KDL::Tree scene = scene_tree();
    WorldModel world;
    std::string error;
    world.add_tree(scene, error);
    Arm a = slice(scene, "w/a");
    bind(world, a, error);
    resolve(world, a);
    world.require_path(a.world_root, a.world_tip, error);

    g_freshness = &world;
    check(raised_stop([] { g_freshness->update(0); }), "updating before sealing requests stop");
    check(world.seal(error), "the model seals");
    check(raised_stop([] { static_cast<void>(g_freshness->pose(0, 0)); }),
          "reading before the first update requests stop");
    world.update(0);
    check(!raised_stop([] { static_cast<void>(g_freshness->pose(0, 0)); }),
          "reading at the cycle it was updated for is quiet");
    world.update(1);
    check(raised_stop([] { static_cast<void>(g_freshness->pose(0, 0)); }),
          "reading with the previous cycle token requests stop");
    check(raised_stop([] { g_freshness->update(1); }), "a duplicate update token requests stop");
    check(raised_stop([] { g_freshness->update(0); }), "an update that goes backwards requests stop");
    check(raised_stop([] { static_cast<void>(g_freshness->pose(-1, 1)); }),
          "reading a segment it does not hold requests stop");
}

// The valid hot path: one world update and one Cartesian-acceleration resolve per cycle.
struct HotPath {
    KDL::Tree scene = scene_tree();
    WorldModel world;
    Arm a;
    KDL::Jacobian jac{kJointsPerArm};
    KDL::Jacobian directions{kJointsPerArm};
    KDL::JntArray acceleration{kJointsPerArm};
    KDL::JntArray qdd{kJointsPerArm};
    Eigen::VectorXd bias = Eigen::VectorXd::Zero(kJointsPerArm);
    motion_spec::runtime::CartesianAccelerationWorkspace work;

    HotPath() {
        std::string error;
        world.add_tree(scene, error);
        a = slice(scene, "w/a");
        bind(world, a, error);
        resolve(world, a);
        world.require_path(a.world_root, a.world_tip, error);
        world.require_path(a.world_root, a.world_elbow, error);
        world.seal(error);
        work.resize(kJointsPerArm, kJointsPerArm);
        jac.data.setRandom();
        directions.data.setZero();
        for (int i = 0; i < kJointsPerArm; ++i) directions.data(i, i) = 1.0;
    }

    double cycle(std::uint64_t token) {
        for (int i = 0; i < kJointsPerArm; ++i) {
            a.q[i] = 0.3 * std::sin(0.001 * static_cast<double>(token) + i);
            acceleration(i) = 0.01 * static_cast<double>(i + 1);
        }
        world.update(token);
        const KDL::Frame root = world.pose(a.world_root, token).Inverse();
        const KDL::Frame tip = root * world.pose(a.world_tip, token);
        const KDL::Frame elbow = root * world.pose(a.world_elbow, token);
        motion_spec::runtime::resolve_cartesian_acceleration(jac, directions, acceleration, &bias,
                                                             work, qdd);
        return tip.p.x() + elbow.p.y() + qdd(0);
    }
};

void verify_no_allocation() {
    HotPath hot;
    // Warm every lazily sized buffer before arming the counter.
    double consumed = hot.cycle(0);
    constexpr std::uint64_t kCycles = 10000;
    g_allocations.store(0);
    g_counting = true;
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    for (std::uint64_t token = 1; token <= kCycles; ++token) consumed += hot.cycle(token);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    g_counting = false;
    const long allocations = g_allocations.load();
    check(allocations == 0, "the valid hot path allocates nothing");
    std::printf("cycles %llu allocations %ld consumed %.6f\n",
                static_cast<unsigned long long>(kCycles), allocations, consumed);
}

// `real_demo_hold` reads five poses in one cycle: two pose/direction reads and three wrench
// frames. That whole cycle is the honest unit to compare, not one read in isolation.
constexpr int kReadsPerCycle = 5;

double chain_fk_cycle(Arm &arm) {
    double consumed = 0.0;
    KDL::JntArray q(kJointsPerArm);
    for (int i = 0; i < kJointsPerArm; ++i) q(i) = arm.q[i];
    for (int read = 0; read < kReadsPerCycle; ++read) {
        KDL::ChainFkSolverPos_recursive fk(arm.chain);
        KDL::Frame frame;
        fk.JntToCart(q, frame, read < 2 ? 2 : -1);
        consumed += frame.p.x();
    }
    return consumed;
}

double world_cycle(WorldModel &world, Arm &arm, std::uint64_t token) {
    world.update(token);
    const KDL::Frame root = world.pose(arm.world_root, token).Inverse();
    double consumed = 0.0;
    for (int read = 0; read < kReadsPerCycle; ++read) {
        const KDL::Frame frame =
            root * world.pose(read < 2 ? arm.world_elbow : arm.world_tip, token);
        consumed += frame.p.x();
    }
    return consumed;
}

long long median(std::vector<long long> samples) {
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

long long percentile95(std::vector<long long> samples) {
    std::sort(samples.begin(), samples.end());
    return samples[(samples.size() * 95) / 100];
}

void benchmark() {
    HotPath hot;
    constexpr int kBatches = 40;
    constexpr int kCyclesPerBatch = 2000;
    std::vector<long long> before, after;
    double consumed = 0.0;
    std::uint64_t token = 1;

    // Warm both paths before measuring either.
    for (int i = 0; i < kCyclesPerBatch; ++i) {
        consumed += chain_fk_cycle(hot.a);
        consumed += world_cycle(hot.world, hot.a, token++);
    }

    for (int batch = 0; batch < kBatches; ++batch) {
        // Alternate the order, so neither path always pays for the other's cache state.
        for (int order = 0; order < 2; ++order) {
            const bool chain_first = (batch % 2 == 0) ? (order == 0) : (order == 1);
            const auto start = std::chrono::steady_clock::now();
            for (int i = 0; i < kCyclesPerBatch; ++i) {
                consumed += chain_first ? chain_fk_cycle(hot.a) : world_cycle(hot.world, hot.a, token++);
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now() - start)
                                     .count();
            (chain_first ? before : after).push_back(elapsed / kCyclesPerBatch);
        }
    }

    const long long before_median = median(before);
    const long long after_median = median(after);
    std::printf("PLAN05 benchmark: %d batches of %d control cycles, %d reads per cycle\n", kBatches,
                kCyclesPerBatch, kReadsPerCycle);
    std::printf("PLAN05 before(chain FK per cycle) median %lld ns p95 %lld ns\n", before_median,
                percentile95(before));
    std::printf("PLAN05 after(world model per cycle) median %lld ns p95 %lld ns\n", after_median,
                percentile95(after));
    std::printf("PLAN05 ratio before/after %.3f\n",
                static_cast<double>(before_median) / static_cast<double>(after_median));
    std::printf("PLAN05 consumed %.6f\n", consumed);
}

}  // namespace

void *operator new(std::size_t size) {
    if (g_counting) g_allocations.fetch_add(1, std::memory_order_relaxed);
    void *memory = std::malloc(size);
    if (memory == nullptr) throw std::bad_alloc();
    return memory;
}

void *operator new[](std::size_t size) { return ::operator new(size); }

void *operator new(std::size_t size, std::align_val_t alignment) {
    if (g_counting) g_allocations.fetch_add(1, std::memory_order_relaxed);
    void *memory = std::aligned_alloc(static_cast<std::size_t>(alignment), size);
    if (memory == nullptr) throw std::bad_alloc();
    return memory;
}

void *operator new[](std::size_t size, std::align_val_t alignment) {
    return ::operator new(size, alignment);
}

void operator delete(void *memory) noexcept { std::free(memory); }
void operator delete[](void *memory) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void *memory, std::size_t) noexcept { std::free(memory); }
void operator delete(void *memory, std::align_val_t) noexcept { std::free(memory); }
void operator delete[](void *memory, std::align_val_t) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }
void operator delete[](void *memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }

int main(int argc, char **argv) {
    const std::string mode = argc > 1 ? argv[1] : "verify";
    if (mode == "bench") {
        benchmark();
        return 0;
    }
    if (mode == "alloc") {
        verify_no_allocation();
        return g_failures == 0 ? 0 : 1;
    }
    verify_against_chain_fk();
    verify_startup_failures();
    verify_freshness();
    std::printf("%s\n", g_failures == 0 ? "world model verified" : "world model FAILED");
    return g_failures == 0 ? 0 : 1;
}

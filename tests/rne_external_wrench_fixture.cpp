// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
//
// What an RNE solver does with a modelled external wrench, run against KDL itself.
//
// The generated code states the wrench the way the model authors it: in the chain root's
// orientation, about the attached segment's origin, and as a force the arm generates there.
// Vereshchagin takes that as written; RNEA reads f_ext[i] in segment i's own frame and enters it
// as `-J_i^T f_ext`. This runs the exact sequence `solver-assign-f-ext-RNE` emits and checks the
// torque it produces is `+J_i^T F`, and that handing RNEA the wrench as authored is not.

#include <cmath>
#include <cstdio>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace {

constexpr int kForcedSegment = 2;  // 1-based, and a joint moves beyond it
constexpr double kTol = 1e-9;

// Links along X on Y-axis joints: away from the zero pose every segment frame is rotated out of
// the base frame, which is the whole difference the fix is about.
KDL::Chain make_chain() {
    KDL::Chain chain;
    for (int i = 0; i < 3; ++i) {
        chain.addSegment(KDL::Segment(
            "l" + std::to_string(i), KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.4, 0, 0)),
            KDL::RigidBodyInertia(1.5, KDL::Vector(-0.2, 0, 0),
                                  KDL::RotationalInertia(0.02, 0.02, 0.02))));
    }
    return chain;
}

// The torque that generates `wrench` on segment `kForcedSegment`: +J^T F, over the chain sliced
// at that segment, zero on the joints beyond it.
void generating_torque(const KDL::Chain &chain, const KDL::JntArray &q, const KDL::Wrench &wrench,
                       KDL::JntArray &out) {
    KDL::Chain sliced;
    for (int i = 0; i < kForcedSegment; ++i) {
        sliced.addSegment(chain.getSegment(i));
    }
    KDL::ChainJntToJacSolver jac_solver(sliced);
    KDL::JntArray q_sliced(sliced.getNrOfJoints());
    for (unsigned i = 0; i < sliced.getNrOfJoints(); ++i) {
        q_sliced(i) = q(i);
    }
    KDL::Jacobian jac(sliced.getNrOfJoints());
    jac_solver.JntToJac(q_sliced, jac);
    KDL::SetToZero(out);
    for (unsigned j = 0; j < sliced.getNrOfJoints(); ++j) {
        out(j) = jac(0, j) * wrench.force.x() + jac(1, j) * wrench.force.y() +
                 jac(2, j) * wrench.force.z() + jac(3, j) * wrench.torque.x() +
                 jac(4, j) * wrench.torque.y() + jac(5, j) * wrench.torque.z();
    }
}

bool matches(const KDL::JntArray &a, const KDL::JntArray &b) {
    for (unsigned i = 0; i < a.rows(); ++i) {
        if (std::fabs(a(i) - b(i)) > kTol) return false;
    }
    return true;
}

void report(const char *label, const KDL::JntArray &tau) {
    std::printf("%-28s = [", label);
    for (unsigned i = 0; i < tau.rows(); ++i) {
        std::printf("% .6f%s", tau(i), i + 1 == tau.rows() ? "]\n" : ", ");
    }
}

}  // namespace

int main() {
    const KDL::Chain chain = make_chain();
    const unsigned nj = chain.getNrOfJoints();
    const unsigned ns = chain.getNrOfSegments();

    KDL::JntArray q(nj), qd(nj), qdd(nj);
    q(0) = 0.35;
    q(1) = -0.80;
    q(2) = 0.55;
    qd(0) = 0.20;
    qd(1) = -0.10;
    qd(2) = 0.30;
    qdd(0) = 0.50;
    qdd(1) = 0.15;
    qdd(2) = -0.25;

    // The gravity an RNE solver is built with is the negated root acceleration the model states.
    KDL::ChainIdSolver_RNE rnea(chain, KDL::Vector(0.0, 0.0, -9.81));
    KDL::ChainFkSolverPos_recursive fk_pos(chain);

    // A pure +Z push in the chain root's orientation -- an elbow support, in the shape the model
    // writes one.
    const KDL::Wrench wrench(KDL::Vector(0.0, 0.0, 40.0), KDL::Vector::Zero());

    KDL::Wrenches f_ext(ns);
    KDL::JntArray tau_free(nj);
    rnea.CartToJnt(q, qd, qdd, f_ext, tau_free);

    // Exactly what `solver-assign-f-ext-RNE` emits.
    KDL::Frame f_ext_frame;
    fk_pos.JntToCart(q, f_ext_frame, kForcedSegment);
    f_ext[kForcedSegment - 1] -= f_ext_frame.M.Inverse() * wrench;
    KDL::JntArray tau_fixed(nj);
    rnea.CartToJnt(q, qd, qdd, f_ext, tau_fixed);

    // The wrench handed over as authored, which is what Vereshchagin wants and RNEA does not.
    KDL::Wrenches f_ext_authored(ns);
    f_ext_authored[kForcedSegment - 1] += wrench;
    KDL::JntArray tau_authored(nj);
    rnea.CartToJnt(q, qd, qdd, f_ext_authored, tau_authored);

    KDL::JntArray expected(nj), delta_fixed(nj), delta_authored(nj);
    generating_torque(chain, q, wrench, expected);
    for (unsigned i = 0; i < nj; ++i) {
        delta_fixed(i) = tau_fixed(i) - tau_free(i);
        delta_authored(i) = tau_authored(i) - tau_free(i);
    }

    report("+J^T F (generates F)", expected);
    report("rotated and negated", delta_fixed);
    report("as authored", delta_authored);

    if (!matches(delta_fixed, expected)) {
        std::printf("FAIL: the emitted f_ext does not generate the modelled wrench\n");
        return 1;
    }
    if (matches(delta_authored, expected)) {
        std::printf("FAIL: handing RNEA the wrench as authored generated it too, so this "
                    "chain cannot tell the two apart -- the fixture is not testing anything\n");
        return 1;
    }
    std::printf("ok\n");
    return 0;
}

# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the loop/device handoff promises: the loop reads whole measurements, in publication
order, and loses none of them. Those are claims about two threads, so they are checked by
compiling the shipped primitives and running them against each other."""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

TEMPLATES = Path(__file__).resolve().parents[1] / "src" / "motion_spec" / "templates"


def _sampled(state: str = "Reading", sample: str = "reading_sample") -> str:
    """The C++ of the handoff buffer, with the StringTemplate escapes undone and the rule's
    state/sample parameters filled in the way the backend template fills them."""
    text = (TEMPLATES / "backend_robif2b_io.stg").read_text()
    found = re.search(
        r"^sampled-buffer\(state, sample\) ::= <<\n(.*?)\n>>",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert found, "sampled-buffer is no longer in backend_robif2b_io.stg"
    body = found.group(1).replace("\\<", "<").replace("\\>", ">").replace("\\}", "}")
    return body.replace("<state>", state).replace("<sample>", sample)


def _ft_io() -> str:
    text = (TEMPLATES / "backend_robif2b_io.stg").read_text()
    found = re.search(
        r"^device-io-section-RobotiqFT300s\(solver, device\) ::= <<\n(.*?)\n>>",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert found, "the FT worker is no longer in backend_robif2b_io.stg"
    body = found.group(1).replace("\\<", "<").replace("\\>", ">").replace("\\}", "}")
    return body.replace(
        "<sampled-buffer({ft_sensor_state}, {ft_sensor_sample})>",
        _sampled("ft_sensor_state", "ft_sensor_sample"),
    )


HAMMER = """
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <type_traits>

// Every field derives from the counter, so a mixture of two publications is visible, and the
// counter is the sequence the publication must arrive under.
struct Reading {
    std::uint64_t a = 0;
    std::uint32_t b = 0;
    float c = 0.0f;
    bool ok = false;
};

/* SAMPLED_BUFFER */

int main() {
    reading_sample sampled;
    constexpr std::uint64_t kPublications = 1000;
    // Two slots hold because a publication costs a bus transaction while a read costs a copy;
    // pace the writer so the hammer runs at the ratio a serial device actually imposes.
    constexpr auto kTransaction = std::chrono::microseconds(100);
    std::atomic<bool> done{false};

    std::thread writer([&] {
        for (std::uint64_t n = 1; n <= kPublications; ++n) {
            const auto until = std::chrono::steady_clock::now() + kTransaction;
            while (std::chrono::steady_clock::now() < until) {}
            sampled.publish(Reading{n, static_cast<std::uint32_t>(n), static_cast<float>(n),
                                    (n & 1) == 0});
        }
        done.store(true);
    });

    std::uint64_t previous = 0;
    std::uint64_t reads = 0;
    while (!done.load()) {
        const auto snapshot = sampled.read();
        if (snapshot.seq == 0) continue;
        ++reads;
        if (snapshot.seq < previous) { std::puts("sequence went backwards"); return 1; }
        previous = snapshot.seq;
        const Reading &value = snapshot.value;
        if (value.b != static_cast<std::uint32_t>(value.a) ||
            value.c != static_cast<float>(value.a) ||
            value.ok != ((value.a & 1) == 0)) {
            std::puts("torn measurement");
            return 1;
        }
        // The nth publication is the one published under sequence n: a slot indexed the other
        // way round hands back a whole, consistent, wrong reading.
        if (value.a != snapshot.seq) { std::puts("wrong slot"); return 1; }
    }
    writer.join();
    const auto last = sampled.read();
    if (last.seq != kPublications || last.value.a != kPublications) {
        std::puts("lost a publication");
        return 1;
    }
    if (reads == 0) { std::puts("the reader never observed a publication"); return 1; }
    return 0;
}
"""


def test_a_reader_never_sees_a_torn_measurement(tmp_path: Path) -> None:
    """Plain, not -fsanitize=thread: the writer rewrites a slot the reader read earlier, and
    nothing the reader does is ever acquired by the writer, so no happens-before edge exists for
    a detector to find. TSan reports that structure on every run -- it would flag the handoff
    design the loop depends on, not a defect in it. What keeps the design honest is the ratio
    the hammer runs at, so that is what is asserted here."""
    compiler = shutil.which("g++") or shutil.which("c++")
    if compiler is None:
        pytest.skip("no C++ compiler")
    source = tmp_path / "device_io.cpp"
    source.write_text(HAMMER.replace("/* SAMPLED_BUFFER */", _sampled()))
    binary = tmp_path / "device_io"
    subprocess.run(
        [compiler, "-std=c++20", "-O2", "-pthread", str(source), "-o", str(binary)], check=True
    )
    subprocess.run([str(binary)], check=True)


def test_ft_worker_publishes_failure_then_recovers(tmp_path: Path) -> None:
    compiler = shutil.which("g++") or shutil.which("c++")
    if compiler is None:
        pytest.skip("no C++ compiler")
    source = tmp_path / "ft_failure.cpp"
    source.write_text(
        """
#include <atomic>
#include <chrono>
#include <cstdint>
#include <stop_token>
#include <thread>
#include <type_traits>

namespace motion_spec::io { struct Staleness {}; }

struct robif2b_robotiq_ft_sensor_nbx { bool *success = nullptr; };
inline std::atomic<int> calls{0};
inline void robif2b_robotiq_ft_update(robif2b_robotiq_ft_sensor_nbx *driver) {
    *driver->success = ++calls > 1;
}

/* FT_IO */

int main() {
    ft_sensor_io io;
    robif2b_robotiq_ft_sensor_nbx driver{&io.state.success};
    io.state.success = true;
    io.start(driver, 1.0);
    while (calls.load() < 2) std::this_thread::yield();
    io.stop();
    const auto recovered = io.sample.read();
    return calls.load() >= 2 && recovered.seq >= 2 && recovered.value.success ? 0 : 1;
}
""".replace("/* FT_IO */", _ft_io())
    )
    binary = tmp_path / "ft_failure"
    subprocess.run(
        [compiler, "-std=c++20", "-O2", "-pthread", str(source), "-o", str(binary)], check=True
    )
    subprocess.run([str(binary)], check=True)

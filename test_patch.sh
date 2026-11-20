#!/usr/bin/env bash
# Old-vs-new test for the FAST corner detector patch.
# Run from the evnt_work repo root, with the patch applied but NOT committed:
#   bash test_patch.sh                    # old = origin/optimized_optical_flow
#   bash test_patch.sh <git-ref>          # old = any other commit/branch
# Needs: g++, Eigen (/usr/include/eigen3). No camera, libcaer or OpenCV needed.
set -euo pipefail

OLD_REF="${1:-origin/optimized_optical_flow}"
REPO="$(pwd)"
W="$(mktemp -d)"
trap 'rm -rf "$W"' EXIT

if [[ "$(uname -m)" == "aarch64" ]]; then ARCH=-mcpu=cortex-a78ae; else ARCH=-march=native; fi
CXXFLAGS="-std=c++17 -O3 -DNDEBUG $ARCH -I/usr/include/eigen3"

# Two copies of the detector, each with stub headers so libcaer/OpenCV are not needed.
for v in old new; do
  mkdir -p "$W/$v/third_party" "$W/$v/include" "$W/$v/libcaer"
  cp "$REPO/include/Types.hpp" "$W/$v/include/"
  cp "$REPO/third_party/detector.h" "$REPO/third_party/timer.h" "$W/$v/third_party/"
  : > "$W/$v/include/DVX_Drivers.hpp"
  : > "$W/$v/libcaer/libcaer.h"
done
git -C "$REPO" show "$OLD_REF:third_party/fast_detector.cpp" > "$W/old/third_party/fast_detector.cpp"
git -C "$REPO" show "$OLD_REF:third_party/fast_detector.h"   > "$W/old/third_party/fast_detector.h"
cp "$REPO/third_party/fast_detector.cpp" "$REPO/third_party/fast_detector.h" "$W/new/third_party/"

if cmp -s "$W/old/third_party/fast_detector.cpp" "$W/new/third_party/fast_detector.cpp"; then
  echo "Old and new fast_detector.cpp are identical: is the patch applied (and not committed to $OLD_REF)?"; exit 1
fi

for v in old new; do
  cat > "$W/${v}_wrap.cpp" <<EOF
#define corner_event_detector ${v}_ns
#include "fast_detector.cpp"
void* mk_${v}() { return new ${v}_ns::FastDetector(); }
bool ${v}_is(void* d, const Event& e) { return static_cast<${v}_ns::FastDetector*>(d)->isFeature(e); }
EOF
done

cat > "$W/main.cpp" <<'EOF'
#include "Types.hpp"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
void* mk_old(); bool old_is(void*, const Event&);
void* mk_new(); bool new_is(void*, const Event&);

// Synthetic events: moving rectangles fire events along their edges (corners at their corners) + 15% noise.
static std::vector<Event> scene(size_t n, unsigned seed) {
  std::mt19937 g(seed); std::uniform_real_distribution<double> U(0, 1);
  struct R { double x, y, w, h, vx, vy; }; std::vector<R> rs;
  for (int i = 0; i < 8; i++)
    rs.push_back({U(g)*500, U(g)*350, 20+U(g)*80, 20+U(g)*80, (U(g)-.5)*0.004, (U(g)-.5)*0.004});
  std::vector<Event> v; v.reserve(n); uint32_t t = 1000;
  while (v.size() < n) {
    t += 1 + (g() % 3);
    for (auto& r : rs) {
      r.x += r.vx; r.y += r.vy;
      if (r.x < 0 || r.x + r.w > 639) r.vx = -r.vx;
      if (r.y < 0 || r.y + r.h > 479) r.vy = -r.vy;
    }
    Event e;
    if (U(g) < 0.15) { e.position << uint16_t(g() % 640), uint16_t(g() % 480); e.polarity = g() & 1; }
    else {
      auto& r = rs[g() % rs.size()]; double s = U(g); int side = g() % 4; double x, y;
      if (side == 0) { x = r.x + s*r.w; y = r.y; } else if (side == 1) { x = r.x + r.w; y = r.y + s*r.h; }
      else if (side == 2) { x = r.x + s*r.w; y = r.y + r.h; } else { x = r.x; y = r.y + s*r.h; }
      e.position << uint16_t(std::min(639, std::max(0, int(x)))), uint16_t(std::min(479, std::max(0, int(y))));
      e.polarity = side < 2;
    }
    e.timestamp = t; v.push_back(e);
  }
  return v;
}

int main(int argc, char** argv) {
  const char* mode = argc > 1 ? argv[1] : "eq";
  if (!strcmp(mode, "eq")) {          // 1) same output on every event?
    size_t total = 0, corners = 0, mismatch = 0;
    for (unsigned seed = 1; seed <= 6; seed++) {
      auto ev = scene(1000000, seed); void* a = mk_old(); void* b = mk_new();
      for (auto& e : ev) { bool x = old_is(a, e), y = new_is(b, e); corners += x; mismatch += (x != y); total++; }
    }
    printf("events=%zu corners=%zu mismatches=%zu -> %s\n", total, corners, mismatch,
           (mismatch == 0 && corners > 0) ? "PASS" : "FAIL");
    return mismatch == 0 && corners > 0 ? 0 : 1;
  }
  auto ev = scene(3000000, 99);       // 2) run one version only, so perf stat measures just it
  bool use_old = !strcmp(mode, "old");
  void* d = use_old ? mk_old() : mk_new();
  auto t0 = std::chrono::steady_clock::now(); size_t c = 0;
  for (auto& e : ev) c += use_old ? old_is(d, e) : new_is(d, e);
  auto t1 = std::chrono::steady_clock::now();
  printf("%s: %.1f ns/event (%zu corners)\n", mode,
         std::chrono::duration<double, std::nano>(t1 - t0).count() / ev.size(), c);
}
EOF

echo "Building ($ARCH), old = $OLD_REF, new = working tree ..."
g++ $CXXFLAGS -I"$W/old/third_party" -I"$W/old/include" -I"$W/old" -c "$W/old_wrap.cpp" -o "$W/old.o"
g++ $CXXFLAGS -I"$W/new/third_party" -I"$W/new/include" -I"$W/new" -c "$W/new_wrap.cpp" -o "$W/new.o"
g++ $CXXFLAGS -I"$W/new/include" -c "$W/main.cpp" -o "$W/main.o"
g++ "$W/old.o" "$W/new.o" "$W/main.o" -o "$W/fasttest"

echo; echo "== 1. Equivalence (must say PASS)"
"$W/fasttest" eq

echo; echo "== 2. Speed (3 runs each, same 3M events)"
for i in 1 2 3; do "$W/fasttest" old; "$W/fasttest" new; done

if command -v perf >/dev/null 2>&1; then
  echo; echo "== 3. CPU counters: r11 cycles, r08 instructions, r21 branches, r22 branch mispredicts"
  for v in old new; do
    echo "-- $v"
    perf stat -r 5 -e r11,r08,r21,r22 "$W/fasttest" $v 2>&1 | grep -E "r11|r08|r21|r22|elapsed"
  done
else
  echo; echo "(perf not installed: skipping counters)"
fi

#include <algorithm>
#include <array>
#include <list>
#include <memory>
#include <mutex>
#include <numbers>
#include <numeric>
#include <tuple>
#include <utility>

#include "grid_buffer.hpp"
namespace {
constexpr double tau = 2 * std::numbers::pi;
// Per-process immutable templates. Twelve workers may retain at most 3 GiB
// here; live grids, model state and template construction need separate RAM.
constexpr std::size_t cache_limit = 256 * 1024 * 1024;
constexpr std::size_t cache_templates = 16;
struct Cell {
  int x, y;
  bool operator==(const Cell &) const = default;
};
// A node is one simultaneous event and its unique prefix. Sorted beam support
// is stored as contiguous runs; a wrap or an intervening shorter beam starts
// a distinct node, retaining exact support without filling any angular gaps.
// Exact corner contacts have singleton angular support, before diagonal entry.
struct Node {
  Cell a{}, b{};
  int count = 1, parent = -1, child = -1, next = -1, first = 0, last = 0;
};
struct Fan {
  int d, side;
  double radius;
  bool streaming;
  std::vector<Node> nodes;
  std::vector<double> angles;
  std::vector<Cell> beam_tips;
  std::vector<std::vector<int>> reverse;
  std::vector<Cell> offsets;
  explicit Fan(double radius, int width, int height)
      : d(static_cast<int>(std::ceil(radius))), side(2 * d + 1), radius(radius), streaming(radius > 100.) {
    // Explicit resource failure, never silently alter range or beam density.
    if (d > 512)
      throw std::invalid_argument("exact ray template exceeds resource budget");
    // Long ranges retain the exact finite center-tip beams, but generate their
    // events on demand. Materializing every prefix grows far faster than the
    // beam list and exhausts the bounded process cache at ordinary 100 m range.
    if (!streaming)
      reverse.resize(static_cast<std::size_t>(side) * side);
    nodes.emplace_back();
    struct Tip {
      Cell cell;
      double angle;
    };
    std::vector<Tip> tips;
    for (int y = -d; y <= d; ++y)
      for (int x = -d; x <= d; ++x) {
        if (std::hypot(double(x), double(y)) > radius + 1e-10)
          continue;
        offsets.push_back({x, y});
        if (!x && !y)
          continue;
        const int gcd = std::gcd(std::abs(x), std::abs(y));
        // Only the farthest collinear tip is needed; its events contain every
        // shorter tip's prefix, and its angle is identical.
        if (std::hypot(double(x + x / gcd), double(y + y / gcd)) <=
            radius + 1e-10)
          continue;
        tips.push_back({{x, y}, std::atan2(y, x)});
      }
    std::sort(offsets.begin(), offsets.end(), [](Cell a, Cell b) {
      const int aa = a.x * a.x + a.y * a.y, bb = b.x * b.x + b.y * b.y;
      return aa == bb ? std::pair(a.x, a.y) < std::pair(b.x, b.y) : aa < bb;
    });
    std::sort(tips.begin(), tips.end(),
              [](const Tip &a, const Tip &b) { return a.angle < b.angle; });
    if (streaming) {
      angles.reserve(tips.size());
      beam_tips.reserve(tips.size());
    }
    for (const auto &tip : tips) {
      const int beam = angles.size();
      angles.push_back(tip.angle);
      if (streaming) {
        beam_tips.push_back(tip.cell);
        continue;
      }
      int parent = 0;
      bool clipped = false;
      const int cap_x = std::min(d, width - 1), cap_y = std::min(d, height - 1);
      auto append = [&](Cell a, Cell b, int count) {
        const auto outside = [&](Cell c) {
          return std::abs(c.x) > cap_x || std::abs(c.y) > cap_y;
        };
        clipped = outside(a) || (count == 2 && outside(b));
        if (outside(a) && (count == 1 || outside(b)))
          return;
        int n = nodes[parent].child;
        while (n >= 0 &&
               !(nodes[n].a == a && nodes[n].b == b &&
                 nodes[n].count == count && nodes[n].last == beam - 1))
          n = nodes[n].next;
        if (n < 0) {
          n = nodes.size();
          Node node;
          node.a = a;
          node.b = b;
          node.count = count;
          node.parent = parent;
          node.next = nodes[parent].child;
          node.first = beam;
          node.last = beam;
          nodes.push_back(node);
          nodes[parent].child = n;
          reverse[index(a)].push_back(n);
          if (count == 2)
            reverse[index(b)].push_back(n);
          if (nodes.size() * sizeof(Node) > cache_limit / 2)
            throw std::invalid_argument(
                "exact ray prefix template exceeds resource budget");
        } else {
          nodes[n].last = beam;
        }
        parent = n;
      };
      const int dx = std::abs(tip.cell.x), dy = std::abs(tip.cell.y);
      const int sx = tip.cell.x > 0 ? 1 : -1, sy = tip.cell.y > 0 ? 1 : -1;
      int x = 0, y = 0, ix = 0, iy = 0;
      while (!clipped && (ix < dx || iy < dy)) {
        const auto ax = std::int64_t(1 + 2 * ix) * dy,
                   ay = std::int64_t(1 + 2 * iy) * dx;
        if (ax == ay) {
          append({x + sx, y}, {x, y + sy}, 2);
          if (clipped)
            break;
          x += sx;
          y += sy;
          ++ix;
          ++iy;
        } else if (ax < ay) {
          x += sx;
          ++ix;
        } else {
          y += sy;
          ++iy;
        }
        append({x, y}, {}, 1);
      }
    }
    // Prefer prefixes supported by the target's center beam; every reverse
    // entry is still retained and tested when that fast positive path fails.
    if (!streaming)
      for (Cell c : offsets) {
        if (!c.x && !c.y)
          continue;
        const auto angle = std::atan2(c.y, c.x);
        auto &entries = reverse[index(c)];
        std::stable_partition(entries.begin(), entries.end(), [&](int n) {
          return angles[nodes[n].first] <= angle + 1e-14 &&
                 angles[nodes[n].last] >= angle - 1e-14;
        });
      }
    if (bytes() > cache_limit)
      throw std::invalid_argument("exact ray template exceeds cache budget");
  }
  std::size_t index(Cell c) const {
    return static_cast<std::size_t>(c.y + d) * side + c.x + d;
  }
  const std::vector<int> *entries(Cell c) const {
    if (std::abs(c.x) > d || std::abs(c.y) > d)
      return nullptr;
    return &reverse[index(c)];
  }
  std::size_t bytes() const {
    std::size_t n = nodes.capacity() * sizeof(Node) +
                    angles.capacity() * sizeof(double) +
                    beam_tips.capacity() * sizeof(Cell) +
                    reverse.capacity() * sizeof(std::vector<int>) +
                    offsets.capacity() * sizeof(Cell);
    for (const auto &v : reverse)
      n += v.capacity() * sizeof(int);
    return n;
  }
};
// Process-owned immutable LRU, serialized construction, shared ownership keeps
// evicted templates alive only for active calls. Queries run without a lock.
std::mutex fan_mutex;
struct BlockIndex;
std::shared_ptr<const BlockIndex> block_cache;
std::size_t block_cache_bytes = 0;
using FanKey = std::tuple<double, int, int>;
std::list<std::pair<FanKey, std::shared_ptr<const Fan>>> fan_cache;
std::shared_ptr<const Fan> fan(double radius, int width, int height) {
  const int d = static_cast<int>(std::ceil(radius));
  const FanKey key{radius, std::min(width, d + 1), std::min(height, d + 1)};
  std::lock_guard lock(fan_mutex);
  for (auto i = fan_cache.begin(); i != fan_cache.end(); ++i)
    if (i->first == key) {
      auto result = i->second;
      fan_cache.splice(fan_cache.begin(), fan_cache, i);
      return result;
    }
  auto result = std::make_shared<Fan>(radius, width, height);
  std::size_t bytes = result->bytes() + block_cache_bytes;
  for (const auto &item : fan_cache)
    bytes += item.second->bytes();
  while (!fan_cache.empty() &&
         (bytes > cache_limit || fan_cache.size() >= cache_templates)) {
    bytes -= fan_cache.back().second->bytes();
    fan_cache.pop_back();
  }
  fan_cache.emplace_front(key, result);
  return result;
}
void dimensions(const Buffer &b, int &h, int &w) {
  if (b.view.shape[0] <= 0 || b.view.shape[1] <= 0 ||
      b.view.shape[0] > 100000 || b.view.shape[1] > 100000)
    throw std::invalid_argument("invalid visibility grid shape");
  h = b.view.shape[0];
  w = b.view.shape[1];
}
void range_check(double r) {
  if (!std::isfinite(r) || r < 0 || r > 100000)
    throw std::invalid_argument("invalid sensor range");
}
void pose_check(int x, int y, int w, int h) {
  if (x < 0 || y < 0 || x >= w || y >= h)
    throw std::invalid_argument("visibility cell outside grid");
}
void target_check(Buffer &t, int w, int h) {
  static_assert(sizeof(long) == sizeof(std::int64_t));
  if (t.view.shape[1] != 2)
    throw std::invalid_argument("invalid visibility targets");
  for (Py_ssize_t i = 0; i < t.view.shape[0] * 2; i += 2) {
    const auto x = t.data<std::int64_t>()[i], y = t.data<std::int64_t>()[i + 1];
    if (x < 0 || y < 0 || x >= w || y >= h)
      throw std::invalid_argument("visibility cell outside grid");
  }
}
struct Grid {
  const std::uint8_t *b;
  int w, h, x, y;
  std::int64_t index(Cell c) const {
    const int xx = x + c.x, yy = y + c.y;
    return xx < 0 || yy < 0 || xx >= w || yy >= h
               ? -1
               : static_cast<std::int64_t>(yy) * w + xx;
  }
  bool stops(const Node &n) const {
    auto k = index(n.a);
    if (k < 0 || b[k] == 2)
      return true;
    if (n.count == 2) {
      k = index(n.b);
      if (k < 0 || b[k] == 2)
        return true;
    }
    return false;
  }
};
// One exact, immutable blocked-cell index accelerates positive center rays.
// Byte equality, not pointer identity, guards reuse after in-place map edits.
// Its storage participates in the same process cache budget as ray templates.
struct BlockIndex {
  int w, h;
  std::vector<std::uint8_t> grid;
  std::vector<std::uint64_t> sum;
  explicit BlockIndex(const Grid &g) : w(g.w), h(g.h),
      grid(g.b, g.b + static_cast<std::size_t>(w)*h),
      sum(static_cast<std::size_t>(w+1)*(h+1)) {
    for (int y=0; y<h; ++y) {
      std::uint64_t row = 0;
      for (int x=0; x<w; ++x) {
        row += grid[static_cast<std::size_t>(y)*w+x] == 2;
        sum[static_cast<std::size_t>(y+1)*(w+1)+x+1] =
            sum[static_cast<std::size_t>(y)*(w+1)+x+1] + row;
      }
    }
  }
  std::size_t bytes() const {
    return grid.capacity() + sum.capacity()*sizeof(std::uint64_t);
  }
  bool clear(int x, int y, Cell delta) const {
    const int x0=std::min(x,x+delta.x), x1=std::max(x,x+delta.x)+1;
    const int y0=std::min(y,y+delta.y), y1=std::max(y,y+delta.y)+1;
    auto at = [&](int xx,int yy) {return sum[static_cast<std::size_t>(yy)*(w+1)+xx];};
    auto empty = [&](int xa,int ya,int xb,int yb) {
      return at(xb,yb)+at(xa,ya)==at(xb,ya)+at(xa,yb);
    };
    if (empty(x0,y0,x1,y1)) return true;
    // A sparse off-ray obstacle need not force a cell-by-cell walk over the
    // entire long ray. These overlapping boxes conservatively contain every
    // supercover contact of each center-line segment, including corner sides.
    const int pieces=(std::max(std::abs(delta.x),std::abs(delta.y))+15)/16;
    if (pieces<2) return false;
    for (int i=0; i<pieces; ++i) {
      const double a=double(i)/pieces, b=double(i+1)/pieces;
      const double xa=x+delta.x*a, xb=x+delta.x*b;
      const double ya=y+delta.y*a, yb=y+delta.y*b;
      const int low_x=std::max(0,int(std::ceil(std::min(xa,xb)-.5-1e-10)));
      const int high_x=std::min(w,int(std::floor(std::max(xa,xb)+.5+1e-10))+1);
      const int low_y=std::max(0,int(std::ceil(std::min(ya,yb)-.5-1e-10)));
      const int high_y=std::min(h,int(std::floor(std::max(ya,yb)+.5+1e-10))+1);
      if (!empty(low_x,low_y,high_x,high_y)) return false;
    }
    return true;
  }
};
std::shared_ptr<const BlockIndex> block_index(const Grid &g) {
  const auto cells=static_cast<std::size_t>(g.w)*g.h;
  const auto bytes=cells+static_cast<std::size_t>(g.w+1)*(g.h+1)*sizeof(std::uint64_t);
  if (bytes > 64*1024*1024) return nullptr;
  std::lock_guard lock(fan_mutex);
  if (block_cache && block_cache->w==g.w && block_cache->h==g.h &&
      std::equal(block_cache->grid.begin(),block_cache->grid.end(),g.b))
    return block_cache;
  auto result=std::make_shared<BlockIndex>(g);
  block_cache=result;
  block_cache_bytes=result->bytes();
  std::size_t total=block_cache_bytes;
  for (const auto &item:fan_cache) total+=item.second->bytes();
  while (!fan_cache.empty() && total>cache_limit) {
    total-=fan_cache.back().second->bytes();
    fan_cache.pop_back();
  }
  return result;
}
// All STRICT predecessors must transmit; the target event is emitted whole
// even when its sibling blocks. Check backwards for cheap negative rejection.
bool prefix_clear(const Fan &f, const Grid &g, int n) {
  if (g.b[static_cast<std::size_t>(g.y) * g.w + g.x] == 2)
    return false;
  for (n = f.nodes[n].parent; n > 0; n = f.nodes[n].parent)
    if (g.stops(f.nodes[n]))
      return false;
  return true;
}
int stream_witness(const Fan &f, const Grid &g, Cell target);
int witness(const Fan &f, const Grid &g, Cell target) {
  if (f.streaming)
    return stream_witness(f, g, target);
  const Cell offset{target.x - g.x, target.y - g.y};
  if (!offset.x && !offset.y)
    return 0;
  if (const auto *entries = f.entries(offset))
    for (int n : *entries)
      if (prefix_clear(f, g, n))
        return n;
  return -1;
}
struct Selection {
  std::array<std::pair<int, int>, 2> intervals{};
  int count = 0;
  Selection(const Fan &f, double yaw, double fov) {
    if (!std::isfinite(yaw) || !std::isfinite(fov) || fov <= 0 ||
        fov > tau + 1e-10)
      throw std::invalid_argument("invalid sensor yaw/FOV");
    auto add = [&](double low, double high) {
      int a = std::lower_bound(f.angles.begin(), f.angles.end(), low - 1e-12) -
              f.angles.begin();
      int b = std::upper_bound(f.angles.begin(), f.angles.end(), high + 1e-12) -
              f.angles.begin();
      if (a < b)
        intervals[count++] = {a, b - 1};
    };
    if (fov >= tau)
      add(-std::numbers::pi, std::numbers::pi);
    else {
      yaw = std::remainder(yaw, tau);
      const double low = yaw - fov / 2, high = yaw + fov / 2;
      if (low <= -std::numbers::pi + 1e-12) {
        add(low + tau, std::numbers::pi);
        add(-std::numbers::pi, high);
      } else if (high >= std::numbers::pi - 1e-12) {
        add(low, std::numbers::pi);
        add(-std::numbers::pi, high - tau);
      } else
        add(low, high);
    }
  }
  bool includes(const Node &n) const {
    for (int i = 0; i < count; ++i)
      if (n.first <= intervals[i].second && n.last >= intervals[i].first)
        return true;
    return false;
  }
};

// Emit one exact supercover event at a time. Simultaneous side contacts are
// emitted together before either can block diagonal entry, just like Fan.
template <class Visit>
void walk_beam(const Fan &f, const Grid &g, int beam, Visit visit) {
  const Cell tip = f.beam_tips[beam];
  const int dx = std::abs(tip.x), dy = std::abs(tip.y);
  const int sx = tip.x > 0 ? 1 : -1, sy = tip.y > 0 ? 1 : -1;
  int x = 0, y = 0, ix = 0, iy = 0;
  while (ix < dx || iy < dy) {
    const auto ax = std::int64_t(1 + 2 * ix) * dy;
    const auto ay = std::int64_t(1 + 2 * iy) * dx;
    if (ax == ay) {
      Node event;
      event.a = {x + sx, y}; event.b = {x, y + sy}; event.count = 2;
      if (!visit(event) || g.stops(event)) return;
      x += sx; y += sy; ++ix; ++iy;
    } else if (ax < ay) {
      x += sx; ++ix;
    } else {
      y += sy; ++iy;
    }
    Node event;
    event.a = {x, y};
    if (!visit(event) || g.stops(event)) return;
  }
}

bool beam_hits(const Fan &f, const Grid &g, int beam, Cell delta) {
  bool hit = false;
  walk_beam(f, g, beam, [&](const Node &event) {
    hit = event.a == delta || (event.count == 2 && event.b == delta);
    // Coordinates are monotonic along a ray; after this box no later event
    // can touch the target. This only prunes misses, never coarsens the beams.
    return !hit && std::abs(event.a.x) <= std::abs(delta.x) + 1 &&
                   std::abs(event.a.y) <= std::abs(delta.y) + 1;
  });
  return hit;
}

template <class Visit>
void candidate_beams(const Fan &f, Cell delta, Visit visit) {
  // Every event is coordinate-wise between the origin and its finite tip.
  // A target center outside the nominal circle cannot be an emitted event.
  if (std::hypot(double(delta.x), double(delta.y)) > f.radius + 1e-10) return;
  const double angle = std::atan2(delta.y, delta.x);
  const double half = std::asin(std::min(1., std::sqrt(.5) /
                                   std::hypot(double(delta.x), double(delta.y))));
  // The circumscribed target-cell disk is a conservative angular bound.
  // Exact event traversal still decides membership and finite endpoint reach.
  Selection candidates(f, angle, 2 * half + 2e-12);
  const int center = std::lower_bound(f.angles.begin(), f.angles.end(), angle - 1e-14) - f.angles.begin();
  const bool center_exists = center < int(f.angles.size()) && std::abs(f.angles[center] - angle) <= 1e-12;
  if (center_exists && visit(center)) return;
  for (int i = 0; i < candidates.count; ++i)
    for (int beam = candidates.intervals[i].first; beam <= candidates.intervals[i].second; ++beam)
      if ((!center_exists || beam != center) && visit(beam)) return;
}

int stream_witness(const Fan &f, const Grid &g, Cell target) {
  const Cell delta{target.x - g.x, target.y - g.y};
  if (!delta.x && !delta.y) return 0;
  if (g.b[static_cast<std::size_t>(g.y) * g.w + g.x] == 2) return -1;
  int found = -1;
  candidate_beams(f, delta, [&](int beam) {
    if (beam_hits(f, g, beam, delta)) found = beam + 1;
    return found >= 0;
  });
  return found;
}
} // namespace
PyObject *observe(PyObject *, PyObject *args) {
  PyObject *bo, *oo;
  int x, y;
  double r, yaw, fov;
  if (!PyArg_ParseTuple(args, "OiidddO", &bo, &x, &y, &r, &yaw, &fov, &oo))
    return nullptr;
  try {
    Buffer b(bo, "B", 2), out(oo, "B", 2, true);
    int h, w;
    dimensions(b, h, w);
    out.shape(h, w);
    range_check(r);
    pose_check(x, y, w, h);
    {
      ReleasedGIL release;
      auto f = fan(r, w, h);
      Selection selected(*f, yaw, fov);
      Grid g{b.data<std::uint8_t>(), w, h, x, y};
      auto *o = out.data<std::uint8_t>();
      std::fill_n(o, static_cast<std::size_t>(h) * w, 0);
      o[static_cast<std::size_t>(y) * w + x] = 1;
      if (b.data<std::uint8_t>()[static_cast<std::size_t>(y) * w + x] != 2 && f->streaming) {
        for (int i = 0; i < selected.count; ++i)
          for (int beam = selected.intervals[i].first; beam <= selected.intervals[i].second; ++beam)
            walk_beam(*f, g, beam, [&](const Node &event) {
              const auto a = g.index(event.a);
              if (a >= 0) o[a] = 1;
              if (event.count == 2) {
                const auto bb = g.index(event.b);
                if (bb >= 0) o[bb] = 1;
              }
              return true;
            });
      } else if (b.data<std::uint8_t>()[static_cast<std::size_t>(y) * w + x] != 2) {
        std::vector<int> stack;
        for (int n = f->nodes[0].child; n >= 0; n = f->nodes[n].next)
          stack.push_back(n);
        while (!stack.empty()) {
          int i = stack.back();
          stack.pop_back();
          const auto &n = f->nodes[i];
          if (!selected.includes(n))
            continue;
          const auto a = g.index(n.a);
          if (a >= 0)
            o[a] = 1;
          if (n.count == 2) {
            const auto bb = g.index(n.b);
            if (bb >= 0)
              o[bb] = 1;
          }
          if (!g.stops(n))
            for (int child = n.child; child >= 0; child = f->nodes[child].next)
              stack.push_back(child);
        }
      }
    }
    Py_RETURN_NONE;
  } catch (const std::exception &e) {
    return error(e);
  }
}
PyObject *reachable(PyObject *, PyObject *args) {
  PyObject *mo, *oo;
  int x, y;
  if (!PyArg_ParseTuple(args, "OiiO", &mo, &x, &y, &oo))
    return nullptr;
  try {
    Buffer m(mo, "B", 2), o(oo, "B", 2, true);
    int h, w;
    dimensions(m, h, w);
    o.shape(h, w);
    if (x < 0 || x >= w || y < 0 || y >= h)
      throw std::invalid_argument("start outside terrain");
    {
      ReleasedGIL release;
      auto *out = o.data<std::uint8_t>();
      auto *state = m.data<std::uint8_t>();
      std::fill_n(out, static_cast<std::size_t>(h) * w, 0);
      // Frontier deque drops visited entries, avoiding an O(component) queue.
      std::vector<std::size_t> current, next;
      if (state[static_cast<std::size_t>(y) * w + x] == 1) {
        current.push_back(static_cast<std::size_t>(y) * w + x);
        out[current[0]] = 1;
      }
      while (!current.empty()) {
        next.clear();
        for (auto k : current) {
          int cy = k / w, cx = k % w;
          for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
              int nx = cx + dx, ny = cy + dy;
              if (nx < 0 || ny < 0 || nx >= w || ny >= h)
                continue;
              auto q = static_cast<std::size_t>(ny) * w + nx;
              if (out[q] || state[q] != 1 ||
                  state[static_cast<std::size_t>(cy) * w + nx] != 1 ||
                  state[static_cast<std::size_t>(ny) * w + cx] != 1)
                continue;
              out[q] = 1;
              next.push_back(q);
            }
        }
        current.swap(next);
      }
    }
    Py_RETURN_NONE;
  } catch (const std::exception &e) {
    return error(e);
  }
}
PyObject *visible_union(PyObject *, PyObject *args) {
  PyObject *bo, *ro, *co, *oo;
  double r;
  if (!PyArg_ParseTuple(args, "OOOdO", &bo, &ro, &co, &r, &oo))
    return nullptr;
  try {
    Buffer b(bo, "B", 2), rr(ro, "B", 2), c(co, "B", 2), o(oo, "B", 2, true);
    int h, w;
    dimensions(b, h, w);
    rr.shape(h, w);
    c.shape(h, w);
    o.shape(h, w);
    range_check(r);
    {
      ReleasedGIL release;
      auto f = fan(r, w, h);
      auto *out = o.data<std::uint8_t>();
      auto *reach = rr.data<std::uint8_t>();
      std::fill_n(out, static_cast<std::size_t>(h) * w, 0);
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
          auto k = static_cast<std::size_t>(y) * w + x;
          if (!c.data<std::uint8_t>()[k])
            continue;
          if (reach[k]) {
            out[k] = 1;
            continue;
          }
          for (Cell delta : f->offsets) {
            const int sx = x - delta.x, sy = y - delta.y;
            if (sx < 0 || sy < 0 || sx >= w || sy >= h ||
                !reach[static_cast<std::size_t>(sy) * w + sx])
              continue;
            if (witness(*f, {b.data<std::uint8_t>(), w, h, sx, sy}, {x, y}) >=
                0) {
              out[k] = 1;
              break;
            }
          }
        }
    }
    Py_RETURN_NONE;
  } catch (const std::exception &e) {
    return error(e);
  }
}
PyObject *visible_targets(PyObject *, PyObject *args) {
  PyObject *bo, *to, *oo;
  int x, y;
  double r;
  if (!PyArg_ParseTuple(args, "OiidOO", &bo, &x, &y, &r, &to, &oo))
    return nullptr;
  try {
    Buffer b(bo, "B", 2), t(to, "l", 2), o(oo, "B", 2, true);
    int h, w;
    dimensions(b, h, w);
    pose_check(x, y, w, h);
    target_check(t, w, h);
    o.shape(t.view.shape[0], 1);
    range_check(r);
    {
      ReleasedGIL release;
      auto f = fan(r, w, h);
      Grid g{b.data<std::uint8_t>(), w, h, x, y};
      for (Py_ssize_t i = 0; i < t.view.shape[0]; ++i)
        o.data<std::uint8_t>()[i] =
            witness(*f, g,
                    {int(t.data<std::int64_t>()[2 * i]),
                     int(t.data<std::int64_t>()[2 * i + 1])}) >= 0;
    }
    Py_RETURN_NONE;
  } catch (const std::exception &e) {
    return error(e);
  }
}
// FORWARD source-array -> fixed target relation; beam visibility is directed.
PyObject *visible_sources(PyObject *, PyObject *args) {
  PyObject *bo, *so, *oo;
  int x, y;
  double r;
  if (!PyArg_ParseTuple(args, "OiidOO", &bo, &x, &y, &r, &so, &oo))
    return nullptr;
  try {
    Buffer b(bo, "B", 2), s(so, "l", 2), o(oo, "B", 2, true);
    int h, w;
    dimensions(b, h, w);
    pose_check(x, y, w, h);
    target_check(s, w, h);
    o.shape(s.view.shape[0], 1);
    range_check(r);
    {
      ReleasedGIL release;
      auto f = fan(r, w, h);
      for (Py_ssize_t i = 0; i < s.view.shape[0]; ++i) {
        Grid g{b.data<std::uint8_t>(), w, h, int(s.data<std::int64_t>()[2 * i]),
               int(s.data<std::int64_t>()[2 * i + 1])};
        o.data<std::uint8_t>()[i] = witness(*f, g, {x, y}) >= 0;
      }
    }
    Py_RETURN_NONE;
  } catch (const std::exception &e) {
    return error(e);
  }
}
PyObject *directional_targets(PyObject *, PyObject *args) {
  PyObject *bo, *to, *ho, *oo;
  int x, y;
  double r, fov;
  if (!PyArg_ParseTuple(args, "OiidOOdO", &bo, &x, &y, &r, &to, &ho, &fov, &oo))
    return nullptr;
  try {
    Buffer b(bo, "B", 2), t(to, "l", 2), headings(ho, "d", 2),
        o(oo, "B", 2, true);
    int h, w;
    dimensions(b, h, w);
    pose_check(x, y, w, h);
    target_check(t, w, h);
    range_check(r);
    headings.shape(8, 1);
    o.shape(t.view.shape[0], 8);
    {
      ReleasedGIL release;
      auto f = fan(r, w, h);
      Grid g{b.data<std::uint8_t>(), w, h, x, y};
      std::vector<Selection> selected;
      for (int j = 0; j < 8; ++j)
        selected.emplace_back(*f, headings.data<double>()[j], fov);
      std::fill_n(o.data<std::uint8_t>(), t.view.shape[0] * 8, 0);
      // Dense frontier batches share one exact traversal per beam. Re-walking
      // thousands of possible beams separately for every occluded target can
      // exceed a worker's reset budget at 100 m. The raster stores heading bits
      // for this call only; no occupancy result survives a map update.
      std::vector<std::uint8_t> directional;
      if (f->streaming && t.view.shape[0] >= 64) {
        directional.resize(static_cast<std::size_t>(w) * h);
        const auto source = static_cast<std::size_t>(y) * w + x;
        directional[source] = 255;
        if (g.b[source] != 2) {
          const auto blockers = block_index(g);
          // Targets often occupy a narrow sector of the map. Only beams that
          // intersect a target's circumscribed cell disk can contribute; retain
          // their union as angular intervals, then use exact events below.
          std::vector<int> needed(f->angles.size() + 1);
          for (Py_ssize_t i = 0; i < t.view.shape[0]; ++i) {
            const Cell delta{int(t.data<std::int64_t>()[2*i]) - x,
                             int(t.data<std::int64_t>()[2*i+1]) - y};
            if ((!delta.x && !delta.y) ||
                std::hypot(double(delta.x), double(delta.y)) > f->radius + 1e-10) continue;
            // Every in-range integer center has its finite collinear beam.
            // Full-heading clear positives need no angular support search.
            if (fov >= tau && blockers && blockers->clear(x,y,delta)) {
              std::fill_n(o.data<std::uint8_t>() + 8*i,8,1);
              continue;
            }
            const double angle = std::atan2(delta.y, delta.x);
            const double half = std::asin(std::min(1., std::sqrt(.5) /
                                       std::hypot(double(delta.x), double(delta.y))));
            Selection candidates(*f, angle, 2 * half + 2e-12);
            std::uint8_t possible = 0, proven = 0;
            for (int j = 0; j < 8; ++j)
              for (int k = 0; k < candidates.count; ++k) {
                Node support;
                support.first = candidates.intervals[k].first;
                support.last = candidates.intervals[k].second;
                if (selected[j].includes(support)) possible |= 1 << j;
              }
            const int center = std::lower_bound(f->angles.begin(), f->angles.end(), angle - 1e-14) - f->angles.begin();
            if (center < int(f->angles.size()) &&
                std::abs(f->angles[center] - angle) <= 1e-12 &&
                ((blockers && blockers->clear(x,y,delta)) ||
                 beam_hits(*f, g, center, delta))) {
              Node support;
              support.first = support.last = center;
              for (int j = 0; j < 8; ++j)
                if (selected[j].includes(support)) proven |= 1 << j;
            }
            auto *row = o.data<std::uint8_t>() + 8 * i;
            for (int j = 0; j < 8; ++j) row[j] = bool(proven & (1 << j));
            if (!(possible & ~proven)) continue;
            for (int j = 0; j < candidates.count; ++j) {
              ++needed[candidates.intervals[j].first];
              --needed[candidates.intervals[j].second + 1];
            }
          }
          int pending = 0;
          for (int beam = 0; beam < int(f->angles.size()); ++beam) {
            pending += needed[beam];
            if (!pending) continue;
            Node support;
            support.first = support.last = beam;
            std::uint8_t mask = 0;
            for (int j = 0; j < 8; ++j)
              if (selected[j].includes(support)) mask |= 1 << j;
            if (!mask) continue;
            walk_beam(*f, g, beam, [&](const Node &event) {
              const auto a = g.index(event.a);
              if (a >= 0) directional[a] |= mask;
              if (event.count == 2) {
                const auto bb = g.index(event.b);
                if (bb >= 0) directional[bb] |= mask;
              }
              return true;
            });
          }
        }
      }
      for (Py_ssize_t i = 0; i < t.view.shape[0]; ++i) {
        Cell delta{int(t.data<std::int64_t>()[2 * i]) - x,
                   int(t.data<std::int64_t>()[2 * i + 1]) - y};
        auto *row = o.data<std::uint8_t>() + 8 * i;
        if (!directional.empty()) {
          const auto mask = directional[g.index(delta)];
          for (int j = 0; j < 8; ++j) row[j] |= bool(mask & (1 << j));
          continue;
        }
        if (!delta.x && !delta.y) {
          std::fill_n(row, 8, 1);
          continue;
        }
        if (f->streaming) {
          if (g.b[static_cast<std::size_t>(y) * w + x] != 2)
            candidate_beams(*f, delta, [&](int beam) {
              Node support;
              support.first = support.last = beam;
              std::uint8_t mask = 0;
              for (int j = 0; j < 8; ++j)
                if (!row[j] && selected[j].includes(support)) mask |= 1 << j;
              if (mask && beam_hits(*f, g, beam, delta))
                for (int j = 0; j < 8; ++j)
                  if (mask & (1 << j)) row[j] = 1;
              return std::all_of(row, row + 8, [](auto value) { return value != 0; });
            });
          continue;
        }
        if (const auto *entries = f->entries(delta))
          for (int n : *entries) {
            std::uint8_t mask = 0;
            for (int j = 0; j < 8; ++j)
              if (!row[j] && selected[j].includes(f->nodes[n]))
                mask |= 1 << j;
            if (mask && prefix_clear(*f, g, n))
              for (int j = 0; j < 8; ++j)
                if (mask & (1 << j))
                  row[j] = 1;
            if (std::all_of(row, row + 8,
                            [](auto value) { return value != 0; }))
              break;
          }
      }
    }
    Py_RETURN_NONE;
  } catch (const std::exception &e) {
    return error(e);
  }
}
PyObject *first_pending(PyObject *, PyObject *args) {
  PyObject *bo, *ko, *to, *oo;
  int x, y;
  double r;
  if (!PyArg_ParseTuple(args, "OOiidOO", &bo, &ko, &x, &y, &r, &to, &oo))
    return nullptr;
  try {
    Buffer b(bo, "B", 2), known(ko, "B", 2), t(to, "l", 2), o(oo, "l", 2, true);
    int h, w;
    dimensions(b, h, w);
    pose_check(x, y, w, h);
    target_check(t, w, h);
    range_check(r);
    known.shape(h, w);
    o.shape(t.view.shape[0], 1);
    {
      ReleasedGIL release;
      auto f = fan(r, w, h);
      Grid g{b.data<std::uint8_t>(), w, h, x, y};
      std::vector<int> path;
      for (Py_ssize_t i = 0; i < t.view.shape[0]; ++i) {
        auto &out = o.data<std::int64_t>()[i];
        out = -1;
        int n = witness(*f, g,
                        {int(t.data<std::int64_t>()[2 * i]),
                         int(t.data<std::int64_t>()[2 * i + 1])});
        if (n < 0)
          continue;
        auto source = static_cast<std::int64_t>(y) * w + x;
        if (!known.data<std::uint8_t>()[source]) {
          out = source;
          continue;
        }
        if (f->streaming) {
          const Cell delta{int(t.data<std::int64_t>()[2 * i]) - x,
                           int(t.data<std::int64_t>()[2 * i + 1]) - y};
          if (n > 0)
            walk_beam(*f, g, n - 1, [&](const Node &event) {
              const auto a = g.index(event.a), bb = event.count == 2 ? g.index(event.b) : -1;
              if (a >= 0 && !known.data<std::uint8_t>()[a]) out = a;
              else if (bb >= 0 && !known.data<std::uint8_t>()[bb]) out = bb;
              return out < 0 && !(event.a == delta || (event.count == 2 && event.b == delta));
            });
          continue;
        }
        path.clear();
        for (; n > 0; n = f->nodes[n].parent)
          path.push_back(n);
        for (auto it = path.rbegin(); it != path.rend() && out < 0; ++it) {
          const auto &node = f->nodes[*it];
          const auto a = g.index(node.a),
                     bb = node.count == 2 ? g.index(node.b) : -1;
          if (a >= 0 && !known.data<std::uint8_t>()[a])
            out = a;
          else if (bb >= 0 && !known.data<std::uint8_t>()[bb])
            out = bb;
        }
      }
    }
    Py_RETURN_NONE;
  } catch (const std::exception &e) {
    return error(e);
  }
}
PyObject *visible_witnesses(PyObject *, PyObject *args) {
  PyObject *bo, *ro, *to, *oo;
  double r;
  if (!PyArg_ParseTuple(args, "OOOdO", &bo, &ro, &to, &r, &oo))
    return nullptr;
  try {
    Buffer b(bo, "B", 2), rr(ro, "B", 2), t(to, "l", 2), o(oo, "l", 2, true);
    int h, w;
    dimensions(b, h, w);
    rr.shape(h, w);
    target_check(t, w, h);
    o.shape(t.view.shape[0], 2);
    range_check(r);
    {
      ReleasedGIL release;
      auto f = fan(r, w, h);
      std::fill_n(o.data<std::int64_t>(), t.view.shape[0] * 2, -1);
      for (Py_ssize_t i = 0; i < t.view.shape[0]; ++i) {
        const int x = t.data<std::int64_t>()[2 * i],
                  y = t.data<std::int64_t>()[2 * i + 1];
        for (Cell delta : f->offsets) {
          const int sx = x - delta.x, sy = y - delta.y;
          if (sx < 0 || sy < 0 || sx >= w || sy >= h ||
              !rr.data<std::uint8_t>()[static_cast<std::size_t>(sy) * w + sx])
            continue;
          if (witness(*f, {b.data<std::uint8_t>(), w, h, sx, sy}, {x, y}) >=
              0) {
            o.data<std::int64_t>()[2 * i] = sx;
            o.data<std::int64_t>()[2 * i + 1] = sy;
            break;
          }
        }
      }
    }
    Py_RETURN_NONE;
  } catch (const std::exception &e) {
    return error(e);
  }
}
PyObject *visibility_cache_info(PyObject *, PyObject *) {
  std::lock_guard lock(fan_mutex);
  std::size_t bytes = block_cache_bytes, nodes = 0, beams = 0;
  for (const auto &item : fan_cache) {
    bytes += item.second->bytes();
    nodes += item.second->nodes.size();
    beams += item.second->angles.size();
  }
  return Py_BuildValue("{s:k,s:k,s:k,s:k,s:k,s:k}", "bytes", bytes, "limit_bytes",
                       cache_limit, "templates", fan_cache.size(),
                       "prefix_nodes", nodes, "beams", beams,
                       "blocked_index_bytes", block_cache_bytes);
}

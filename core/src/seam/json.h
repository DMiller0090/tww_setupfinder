/* Just enough JSON for the seam. Never throws or crashes: a parse failure is a `Bad` value. */
#pragma once

#include <map>
#include <string>
#include <vector>

namespace seam {

class Json {
 public:
  enum class Type { Bad, Null, Bool, Num, Str, Arr, Obj, Raw };

  Json() = default;
  static Json null_() { return Json(Type::Null); }
  static Json boolean(bool v) { Json j(Type::Bool); j.b_ = v; return j; }
  static Json num(double v) { Json j(Type::Num); j.n_ = v; return j; }
  /** Written as the shortest text that reads back as the same float. */
  static Json f32(double v);
  static Json str(std::string v) { Json j(Type::Str); j.s_ = std::move(v); return j; }
  static Json arr() { return Json(Type::Arr); }
  static Json obj() { return Json(Type::Obj); }
  static Json bad(std::string why) { Json j(Type::Bad); j.s_ = std::move(why); return j; }
  /** Already-written JSON, dumped verbatim. Only `floats()` output goes in here. */
  static Json raw(std::string json) { Json j(Type::Raw); j.s_ = std::move(json); return j; }

  Type type() const { return k_; }
  bool ok() const { return k_ != Type::Bad; }
  /** Empty unless `Bad`. */
  const std::string& why() const { return s_; }

  bool as_bool(bool fallback = false) const { return k_ == Type::Bool ? b_ : fallback; }
  double as_num(double fallback = 0) const { return k_ == Type::Num ? n_ : fallback; }
  std::string as_str(const std::string& fallback = {}) const {
    return k_ == Type::Str ? s_ : fallback;
  }
  /** A `Bad` naming the key when it is missing. */
  const Json& at(const std::string& key) const;
  bool has(const std::string& key) const;
  const std::vector<Json>& items() const { return a_; }

  void push(Json v) { a_.push_back(std::move(v)); }
  void set(const std::string& key, Json v);

  /** One line, no newline, no spaces. */
  std::string dump() const;

 private:
  explicit Json(Type k) : k_(k) {}

  Type k_ = Type::Bad;
  bool b_ = false;
  double n_ = 0;
  std::string s_;
  std::vector<Json> a_;
  std::map<std::string, Json> o_;
};

/** Parse one whole value; trailing non-blank text is a failure. */
Json parse(const std::string& text);

/** A JSON array of floats, each shortest-round-trip as a float, not a double. */
std::string floats(const std::vector<float>& values);

}  // namespace seam

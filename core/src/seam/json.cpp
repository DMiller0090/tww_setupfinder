#include "json.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

namespace seam {
namespace {

const Json& bad_field(const std::string& key) {
  /* Kept for the process because `at` returns a reference. Locked: the reader and answering
     threads both reach it. */
  static std::mutex lock;
  static std::map<std::string, Json> held;
  const std::lock_guard<std::mutex> holding(lock);
  auto it = held.find(key);
  if (it == held.end()) it = held.emplace(key, Json::bad("no field `" + key + "`")).first;
  return it->second;
}

/* Shortest text that reads back as the same double. */
std::string number(double v) {
  char buf[40];
  for (int digits = 15; digits <= 17; ++digits) {
    std::snprintf(buf, sizeof buf, "%.*g", digits, v);
    if (std::strtod(buf, nullptr) == v) break;
  }
  return buf;
}

/* Shortest text that reads back as the same float. */
std::string float_number(float v) {
  char buf[32];
  for (int digits = 6; digits <= 9; ++digits) {
    std::snprintf(buf, sizeof buf, "%.*g", digits, static_cast<double>(v));
    if (std::strtof(buf, nullptr) == v) break;
  }
  return buf;
}

void escape(const std::string& s, std::string& out) {
  out += '"';
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char u[8];
          std::snprintf(u, sizeof u, "\\u%04x", c);
          out += u;
        } else {
          // Bytes >= 0x80 pass through: the charset is whatever the disc uses.
          out += static_cast<char>(c);
        }
    }
  }
  out += '"';
}

struct Reader {
  const std::string& text;
  size_t i = 0;

  void blanks() {
    while (i < text.size() && (text[i] == ' ' || text[i] == '\t' || text[i] == '\n' ||
                               text[i] == '\r')) ++i;
  }
  bool word(const char* w) {
    size_t n = std::strlen(w);
    if (text.compare(i, n, w) != 0) return false;
    i += n;
    return true;
  }

  Json value(int depth) {
    if (depth > 32) return Json::bad("nested too deep");
    blanks();
    if (i >= text.size()) return Json::bad("ended early");
    char c = text[i];
    if (c == '{') return object(depth);
    if (c == '[') return array(depth);
    if (c == '"') return string();
    if (word("true")) return Json::boolean(true);
    if (word("false")) return Json::boolean(false);
    if (word("null")) return Json::null_();
    return number();
  }

  Json object(int depth) {
    ++i;  // '{'
    Json out = Json::obj();
    blanks();
    if (i < text.size() && text[i] == '}') { ++i; return out; }
    for (;;) {
      blanks();
      if (i >= text.size() || text[i] != '"') return Json::bad("a key must be a string");
      Json key = string();
      if (!key.ok()) return key;
      blanks();
      if (i >= text.size() || text[i] != ':') return Json::bad("a key needs a `:`");
      ++i;
      Json val = value(depth + 1);
      if (!val.ok()) return val;
      out.set(key.as_str(), std::move(val));
      blanks();
      if (i < text.size() && text[i] == ',') { ++i; continue; }
      if (i < text.size() && text[i] == '}') { ++i; return out; }
      return Json::bad("an object needs `,` or `}`");
    }
  }

  Json array(int depth) {
    ++i;  // '['
    Json out = Json::arr();
    blanks();
    if (i < text.size() && text[i] == ']') { ++i; return out; }
    for (;;) {
      Json val = value(depth + 1);
      if (!val.ok()) return val;
      out.push(std::move(val));
      blanks();
      if (i < text.size() && text[i] == ',') { ++i; continue; }
      if (i < text.size() && text[i] == ']') { ++i; return out; }
      return Json::bad("an array needs `,` or `]`");
    }
  }

  Json string() {
    ++i;  // '"'
    std::string out;
    while (i < text.size()) {
      char c = text[i++];
      if (c == '"') return Json::str(std::move(out));
      if (c != '\\') { out += c; continue; }
      if (i >= text.size()) break;
      char e = text[i++];
      switch (e) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case '/': out += '/'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        case 'u': {
          if (i + 4 > text.size()) return Json::bad("a `\\u` needs four digits");
          unsigned code = 0;
          for (int k = 0; k < 4; ++k) {
            char h = text[i + k];
            unsigned d;
            if (h >= '0' && h <= '9') d = static_cast<unsigned>(h - '0');
            else if (h >= 'a' && h <= 'f') d = static_cast<unsigned>(h - 'a' + 10);
            else if (h >= 'A' && h <= 'F') d = static_cast<unsigned>(h - 'A' + 10);
            else return Json::bad("a `\\u` needs four digits");
            code = code * 16 + d;
          }
          i += 4;
          // A lone surrogate becomes U+FFFD rather than an error.
          if (code >= 0xD800 && code <= 0xDFFF) code = 0xFFFD;
          if (code < 0x80) {
            out += static_cast<char>(code);
          } else if (code < 0x800) {
            out += static_cast<char>(0xC0 | (code >> 6));
            out += static_cast<char>(0x80 | (code & 0x3F));
          } else {
            out += static_cast<char>(0xE0 | (code >> 12));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
          }
          break;
        }
        default: return Json::bad("unknown escape");
      }
    }
    return Json::bad("a string never closed");
  }

  Json number() {
    size_t from = i;
    if (i < text.size() && (text[i] == '-' || text[i] == '+')) ++i;
    bool any = false;
    while (i < text.size() && text[i] >= '0' && text[i] <= '9') { ++i; any = true; }
    if (i < text.size() && text[i] == '.') {
      ++i;
      while (i < text.size() && text[i] >= '0' && text[i] <= '9') { ++i; any = true; }
    }
    if (any && i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
      size_t save = i;
      ++i;
      if (i < text.size() && (text[i] == '-' || text[i] == '+')) ++i;
      bool digits = false;
      while (i < text.size() && text[i] >= '0' && text[i] <= '9') { ++i; digits = true; }
      if (!digits) i = save;
    }
    if (!any) return Json::bad("not a value");
    return Json::num(std::strtod(text.substr(from, i - from).c_str(), nullptr));
  }
};

}  // namespace

const Json& Json::at(const std::string& key) const {
  if (k_ == Type::Obj) {
    auto it = o_.find(key);
    if (it != o_.end()) return it->second;
  }
  return bad_field(key);
}

bool Json::has(const std::string& key) const {
  return k_ == Type::Obj && o_.find(key) != o_.end();
}

void Json::set(const std::string& key, Json v) {
  if (k_ != Type::Obj) *this = obj();
  o_[key] = std::move(v);
}

std::string Json::dump() const {
  std::string out;
  switch (k_) {
    // A `Bad` should never be dumped, but still writes legal JSON.
    case Type::Bad:
    case Type::Null: out = "null"; break;
    case Type::Bool: out = b_ ? "true" : "false"; break;
    case Type::Num: out = number(n_); break;
    case Type::Raw: out = s_; break;
    case Type::Str: escape(s_, out); break;
    case Type::Arr: {
      out += '[';
      for (size_t k = 0; k < a_.size(); ++k) {
        if (k) out += ',';
        out += a_[k].dump();
      }
      out += ']';
      break;
    }
    case Type::Obj: {
      out += '{';
      bool first = true;
      for (const auto& kv : o_) {
        if (!first) out += ',';
        first = false;
        escape(kv.first, out);
        out += ':';
        out += kv.second.dump();
      }
      out += '}';
      break;
    }
  }
  return out;
}

Json Json::f32(double v) { return raw(float_number(static_cast<float>(v))); }

std::string floats(const std::vector<float>& values) {
  std::string out;
  out.reserve(values.size() * 10 + 2);
  out += '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i) out += ',';
    out += float_number(values[i]);
  }
  out += ']';
  return out;
}

Json parse(const std::string& text) {
  Reader r{text};
  Json out = r.value(0);
  if (!out.ok()) return out;
  r.blanks();
  if (r.i != text.size()) return Json::bad("more than one value on the line");
  return out;
}

}  // namespace seam

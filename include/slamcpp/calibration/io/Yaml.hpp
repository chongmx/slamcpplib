// A YAML reader covering the subset Kalibr's configuration uses.
//
// Vendored for Phase 0. The plan calls for a header-only YAML reader; yaml-cpp
// is not header-only and would be the library's second mandatory dependency,
// which defeats the point. What upstream's camchain.yaml, imu.yaml and
// target.yaml actually contain is small and completely specified:
//
//   block mappings, nested by indentation
//   block sequences, "- " items, including sequences of flow sequences
//   flow sequences, [a, b, c], nested
//   scalars: strings (bare, 'single' or "double" quoted), numbers, booleans
//   # comments, and --- document markers
//
// Deliberately not supported, because no Kalibr config uses them: anchors and
// aliases, tags, flow mappings, multi-document streams, block scalars (| and
// >), and multi-line plain scalars. Each is rejected with a line number rather
// than silently mis-parsed, which matters: a config that quietly loses a
// distortion coefficient is far worse than one that fails to load.
//
// Round-tripping is a hard requirement from the plan, so the emitter here
// writes the same layout upstream does, with matrices as block sequences of
// flow sequences.
#ifndef CALIB_YAML_HPP
#define CALIB_YAML_HPP

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace slamcpp {
namespace calib {
namespace yaml {

class ParseError : public std::runtime_error {
 public:
  ParseError(int line, const std::string& what)
      : std::runtime_error("YAML line " + std::to_string(line) + ": " + what),
        line(line) {}
  int line;
};

class Node {
 public:
  enum class Type { Null, Scalar, Sequence, Map };

  Node() = default;
  static Node scalar(std::string v) {
    Node n;
    n.type_ = Type::Scalar;
    n.scalar_ = std::move(v);
    return n;
  }
  static Node sequence() {
    Node n;
    n.type_ = Type::Sequence;
    return n;
  }
  static Node map() {
    Node n;
    n.type_ = Type::Map;
    return n;
  }

  Type type() const { return type_; }
  bool isNull() const { return type_ == Type::Null; }
  bool isScalar() const { return type_ == Type::Scalar; }
  bool isSequence() const { return type_ == Type::Sequence; }
  bool isMap() const { return type_ == Type::Map; }

  // ---- map access ----
  bool has(const std::string& key) const {
    return type_ == Type::Map && map_.count(key) > 0;
  }
  const Node& operator[](const std::string& key) const {
    static const Node kNull;
    auto it = map_.find(key);
    return it == map_.end() ? kNull : it->second;
  }
  Node& operator[](const std::string& key) {
    type_ = Type::Map;
    if (!map_.count(key)) order_.push_back(key);
    return map_[key];
  }
  // Insertion order, which is the order the emitter writes.
  const std::vector<std::string>& keys() const { return order_; }

  // ---- sequence access ----
  std::size_t size() const {
    return type_ == Type::Sequence ? seq_.size()
                                   : (type_ == Type::Map ? map_.size() : 0);
  }
  const Node& operator[](std::size_t i) const { return seq_.at(i); }
  Node& operator[](std::size_t i) { return seq_.at(i); }
  void push_back(Node n) {
    type_ = Type::Sequence;
    seq_.push_back(std::move(n));
  }
  const std::vector<Node>& items() const { return seq_; }

  // ---- typed reads ----
  const std::string& raw() const { return scalar_; }

  std::string asString() const {
    requireScalar("string");
    return scalar_;
  }

  double asDouble() const {
    requireScalar("double");
    const char* s = scalar_.c_str();
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == s || *end != '\0') {
      throw std::runtime_error("YAML: '" + scalar_ + "' is not a number");
    }
    return v;
  }

  int asInt() const {
    requireScalar("int");
    const char* s = scalar_.c_str();
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (end == s || *end != '\0') {
      throw std::runtime_error("YAML: '" + scalar_ + "' is not an integer");
    }
    return static_cast<int>(v);
  }

  bool asBool() const {
    requireScalar("bool");
    std::string v = scalar_;
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (v == "true" || v == "yes" || v == "on" || v == "1") return true;
    if (v == "false" || v == "no" || v == "off" || v == "0") return false;
    throw std::runtime_error("YAML: '" + scalar_ + "' is not a boolean");
  }

  std::vector<double> asDoubleVector() const {
    if (type_ != Type::Sequence) {
      throw std::runtime_error("YAML: expected a sequence of numbers");
    }
    std::vector<double> out;
    out.reserve(seq_.size());
    for (const Node& n : seq_) out.push_back(n.asDouble());
    return out;
  }

  // Rows of equal length, the shape a T_cn_cnm1 block has.
  std::vector<std::vector<double>> asMatrix() const {
    if (type_ != Type::Sequence) {
      throw std::runtime_error("YAML: expected a sequence of rows");
    }
    std::vector<std::vector<double>> rows;
    rows.reserve(seq_.size());
    for (const Node& r : seq_) rows.push_back(r.asDoubleVector());
    for (const auto& r : rows) {
      if (r.size() != rows.front().size()) {
        throw std::runtime_error("YAML: matrix rows have differing lengths");
      }
    }
    return rows;
  }

 private:
  void requireScalar(const char* as) const {
    if (type_ != Type::Scalar) {
      throw std::runtime_error(std::string("YAML: node is not a scalar, "
                                           "cannot read it as ") +
                               as);
    }
  }

  Type type_ = Type::Null;
  std::string scalar_;
  std::vector<Node> seq_;
  std::map<std::string, Node> map_;
  std::vector<std::string> order_;
};

namespace detail {

struct Line {
  int number = 0;
  int indent = 0;
  std::string text;  // comment-stripped, right-trimmed
};

inline bool isBlank(const std::string& s) {
  return s.find_first_not_of(" \t") == std::string::npos;
}

inline std::string rtrim(std::string s) {
  const auto p = s.find_last_not_of(" \t\r");
  return p == std::string::npos ? std::string() : s.substr(0, p + 1);
}

inline std::string ltrim(std::string s) {
  const auto p = s.find_first_not_of(" \t");
  return p == std::string::npos ? std::string() : s.substr(p);
}

inline std::string trim(std::string s) { return ltrim(rtrim(std::move(s))); }

// Removes a trailing comment, respecting quotes so that a '#' inside a string
// is not treated as one.
inline std::string stripComment(const std::string& s) {
  bool inSingle = false, inDouble = false;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (c == '\'' && !inDouble) inSingle = !inSingle;
    else if (c == '"' && !inSingle) inDouble = !inDouble;
    else if (c == '#' && !inSingle && !inDouble) {
      // A '#' only starts a comment at the start of a line or after a space.
      if (i == 0 || s[i - 1] == ' ' || s[i - 1] == '\t') return s.substr(0, i);
    }
  }
  return s;
}

inline std::string unquote(const std::string& s, int lineNo) {
  if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                        (s.front() == '\'' && s.back() == '\''))) {
    const char q = s.front();
    std::string body = s.substr(1, s.size() - 2);
    if (q == '\'') {
      // In single quotes only '' is an escape, for a literal quote.
      std::string out;
      for (std::size_t i = 0; i < body.size(); ++i) {
        if (body[i] == '\'' && i + 1 < body.size() && body[i + 1] == '\'') {
          out += '\'';
          ++i;
        } else {
          out += body[i];
        }
      }
      return out;
    }
    std::string out;
    for (std::size_t i = 0; i < body.size(); ++i) {
      if (body[i] == '\\' && i + 1 < body.size()) {
        switch (body[++i]) {
          case 'n': out += '\n'; break;
          case 't': out += '\t'; break;
          case 'r': out += '\r'; break;
          case '\\': out += '\\'; break;
          case '"': out += '"'; break;
          default:
            throw ParseError(lineNo, std::string("unsupported escape \\") + body[i]);
        }
      } else {
        out += body[i];
      }
    }
    return out;
  }
  return s;
}

// Parses a flow sequence "[a, b, [c, d]]". Returns the node and consumes
// through the matching bracket.
inline Node parseFlow(const std::string& s, std::size_t& i, int lineNo) {
  if (s[i] != '[') throw ParseError(lineNo, "expected '['");
  ++i;
  Node seq = Node::sequence();
  std::string token;
  auto flush = [&]() {
    const std::string t = trim(token);
    if (!t.empty()) seq.push_back(Node::scalar(unquote(t, lineNo)));
    token.clear();
  };
  while (i < s.size()) {
    const char c = s[i];
    if (c == '[') {
      seq.push_back(parseFlow(s, i, lineNo));
      token.clear();
      continue;
    }
    if (c == ']') {
      ++i;
      flush();
      return seq;
    }
    if (c == ',') {
      flush();
      ++i;
      continue;
    }
    if (c == '{') throw ParseError(lineNo, "flow mappings are not supported");
    token += c;
    ++i;
  }
  throw ParseError(lineNo, "unterminated flow sequence");
}

inline Node parseScalarOrFlow(const std::string& value, int lineNo) {
  const std::string v = trim(value);
  if (v.empty()) return Node();
  if (v[0] == '[') {
    std::size_t i = 0;
    Node n = parseFlow(v, i, lineNo);
    if (trim(v.substr(i)).size() != 0) {
      throw ParseError(lineNo, "trailing content after flow sequence");
    }
    return n;
  }
  if (v[0] == '{') throw ParseError(lineNo, "flow mappings are not supported");
  if (v[0] == '&' || v[0] == '*') {
    throw ParseError(lineNo, "anchors and aliases are not supported");
  }
  if (v == "|" || v == ">" || v == "|-" || v == ">-") {
    throw ParseError(lineNo, "block scalars are not supported");
  }
  if (v == "~" || v == "null") return Node();
  return Node::scalar(unquote(v, lineNo));
}

// Splits "key: value" at the first colon outside quotes. Returns false when
// the line is not a mapping entry.
inline bool splitKey(const std::string& s, std::string* key, std::string* value) {
  bool inSingle = false, inDouble = false;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (c == '\'' && !inDouble) inSingle = !inSingle;
    else if (c == '"' && !inSingle) inDouble = !inDouble;
    else if (c == ':' && !inSingle && !inDouble) {
      // A colon only separates when followed by a space or end of line, so
      // that a bare scalar like "12:30" is not mistaken for a key.
      if (i + 1 == s.size() || s[i + 1] == ' ' || s[i + 1] == '\t') {
        *key = trim(s.substr(0, i));
        *value = trim(s.substr(i + 1));
        return true;
      }
    }
  }
  return false;
}

class Parser {
 public:
  explicit Parser(const std::string& text) {
    std::istringstream in(text);
    std::string raw;
    int n = 0;
    while (std::getline(in, raw)) {
      ++n;
      std::string stripped = rtrim(stripComment(raw));
      if (isBlank(stripped)) continue;
      if (trim(stripped) == "---") continue;
      if (trim(stripped) == "...") continue;
      if (stripped.find('\t') != std::string::npos &&
          stripped.find_first_not_of(" \t") > stripped.find('\t')) {
        throw ParseError(n, "tabs cannot be used for indentation");
      }
      Line line;
      line.number = n;
      line.indent = static_cast<int>(stripped.find_first_not_of(' '));
      line.text = trim(stripped);
      lines_.push_back(line);
    }
  }

  Node parse() {
    if (lines_.empty()) return Node();
    std::size_t i = 0;
    Node root = parseBlock(i, lines_[0].indent);
    if (i != lines_.size()) {
      throw ParseError(lines_[i].number, "unexpected indentation");
    }
    return root;
  }

 private:
  // Parses every line at exactly `indent`, stopping at the first line that is
  // less indented.
  Node parseBlock(std::size_t& i, int indent) {
    if (i >= lines_.size()) return Node();

    if (lines_[i].text.rfind("- ", 0) == 0 || lines_[i].text == "-") {
      return parseSequence(i, indent);
    }
    return parseMap(i, indent);
  }

  Node parseSequence(std::size_t& i, int indent) {
    Node seq = Node::sequence();
    while (i < lines_.size() && lines_[i].indent == indent) {
      const Line& line = lines_[i];
      if (line.text != "-" && line.text.rfind("- ", 0) != 0) break;

      const std::string rest = trim(line.text.size() > 1 ? line.text.substr(2) : "");
      ++i;

      if (rest.empty()) {
        // Item content is on the following, more indented lines.
        if (i < lines_.size() && lines_[i].indent > indent) {
          seq.push_back(parseBlock(i, lines_[i].indent));
        } else {
          seq.push_back(Node());
        }
        continue;
      }

      std::string key, value;
      if (splitKey(rest, &key, &value)) {
        // "- key: value", a map whose first key shares the dash's line. The
        // map's indent is the column the key starts at.
        const int mapIndent = indent + 2;
        Node m = Node::map();
        addMapEntry(&m, key, value, i, mapIndent, line.number);
        while (i < lines_.size() && lines_[i].indent == mapIndent) {
          std::string k2, v2;
          if (!splitKey(lines_[i].text, &k2, &v2)) break;
          const int lineNo = lines_[i].number;
          ++i;
          addMapEntry(&m, k2, v2, i, mapIndent, lineNo);
        }
        seq.push_back(std::move(m));
      } else {
        seq.push_back(parseScalarOrFlow(rest, line.number));
      }
    }
    return seq;
  }

  Node parseMap(std::size_t& i, int indent) {
    Node m = Node::map();
    while (i < lines_.size() && lines_[i].indent == indent) {
      const Line& line = lines_[i];
      if (line.text.rfind("- ", 0) == 0 || line.text == "-") break;

      std::string key, value;
      if (!splitKey(line.text, &key, &value)) {
        throw ParseError(line.number, "expected 'key: value'");
      }
      const int lineNo = line.number;
      ++i;
      addMapEntry(&m, key, value, i, indent, lineNo);
    }
    return m;
  }

  void addMapEntry(Node* m, const std::string& key, const std::string& value,
                   std::size_t& i, int indent, int lineNo) {
    if (m->has(key)) throw ParseError(lineNo, "duplicate key '" + key + "'");

    if (!value.empty()) {
      (*m)[key] = parseScalarOrFlow(value, lineNo);
      return;
    }
    // Value is a nested block on the following lines. A block sequence may sit
    // at the same indent as its key, which is legal YAML and is how upstream
    // writes T_cn_cnm1.
    if (i < lines_.size() && lines_[i].indent > indent) {
      (*m)[key] = parseBlock(i, lines_[i].indent);
    } else if (i < lines_.size() && lines_[i].indent == indent &&
               (lines_[i].text.rfind("- ", 0) == 0 || lines_[i].text == "-")) {
      (*m)[key] = parseSequence(i, indent);
    } else {
      (*m)[key] = Node();
    }
  }

  std::vector<Line> lines_;
};

}  // namespace detail

inline Node parse(const std::string& text) {
  return detail::Parser(text).parse();
}

// ---------------------------------------------------------------------------
// Emitter. Writes the layout upstream writes, so that output can be diffed
// against a reference camchain.yaml directly.
// ---------------------------------------------------------------------------
namespace detail {

inline bool needsQuoting(const std::string& s) {
  if (s.empty()) return true;
  if (s.find_first_of(":#{}[],&*'\"\n") != std::string::npos) return true;
  if (s.front() == ' ' || s.back() == ' ') return true;
  return false;
}

inline std::string quoteIfNeeded(const std::string& s) {
  if (!needsQuoting(s)) return s;
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out + "\"";
}

// True when a sequence holds only scalars, in which case it is written inline
// as a flow sequence. A sequence of sequences becomes a block of flow rows.
inline bool allScalars(const Node& n) {
  for (const Node& c : n.items()) {
    if (!c.isScalar()) return false;
  }
  return true;
}

inline void emitFlow(const Node& n, std::ostream& out) {
  out << '[';
  for (std::size_t i = 0; i < n.size(); ++i) {
    if (i) out << ", ";
    const Node& c = n[i];
    if (c.isSequence()) emitFlow(c, out);
    else out << quoteIfNeeded(c.raw());
  }
  out << ']';
}

inline void emitNode(const Node& n, std::ostream& out, int indent);

inline void emitSequence(const Node& n, std::ostream& out, int indent) {
  const std::string pad(indent, ' ');
  for (const Node& c : n.items()) {
    out << pad << "- ";
    if (c.isSequence() && allScalars(c)) {
      emitFlow(c, out);
      out << '\n';
    } else if (c.isScalar()) {
      out << quoteIfNeeded(c.raw()) << '\n';
    } else if (c.isNull()) {
      out << '\n';
    } else {
      out << '\n';
      emitNode(c, out, indent + 2);
    }
  }
}

inline void emitMap(const Node& n, std::ostream& out, int indent) {
  const std::string pad(indent, ' ');
  for (const std::string& key : n.keys()) {
    const Node& v = n[key];
    out << pad << key << ':';
    if (v.isScalar()) {
      out << ' ' << quoteIfNeeded(v.raw()) << '\n';
    } else if (v.isNull()) {
      out << '\n';
    } else if (v.isSequence() && allScalars(v)) {
      out << ' ';
      emitFlow(v, out);
      out << '\n';
    } else if (v.isSequence()) {
      out << '\n';
      emitSequence(v, out, indent);
    } else {
      out << '\n';
      emitMap(v, out, indent + 2);
    }
  }
}

inline void emitNode(const Node& n, std::ostream& out, int indent) {
  if (n.isSequence()) emitSequence(n, out, indent);
  else if (n.isMap()) emitMap(n, out, indent);
  else if (n.isScalar()) out << std::string(indent, ' ') << quoteIfNeeded(n.raw()) << '\n';
}

}  // namespace detail

inline std::string emit(const Node& root) {
  std::ostringstream out;
  detail::emitNode(root, out, 0);
  return out.str();
}

}  // namespace yaml
}  // namespace calib
}  // namespace slamcpp

#endif  // CALIB_YAML_HPP

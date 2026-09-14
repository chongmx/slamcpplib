// A JSON writer for calibration reports.
//
// Vendored for Phase 0. Write-only by design: the plan uses JSON for reports
// and YAML for configuration, and nothing in the library reads JSON back. A
// reader would be code with no caller.
//
// Number formatting is the part that matters. Reports are compared against
// reference results, so doubles are written with enough precision to round
// trip exactly (max_digits10), and non-finite values are written as null
// rather than the bare NaN some writers emit, which is not valid JSON and
// which downstream parsers reject.
#ifndef CALIB_JSON_HPP
#define CALIB_JSON_HPP

#include <cmath>
#include <cstdio>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace slamcpp {
namespace calib {
namespace json {

class Writer {
 public:
  explicit Writer(std::ostream& out, int indentWidth = 2)
      : out_(out), indentWidth_(indentWidth) {}

  void beginObject() {
    comma();
    out_ << '{';
    push(true);
  }
  void endObject() { pop('}'); }

  void beginArray() {
    comma();
    out_ << '[';
    push(false);
  }
  void endArray() { pop(']'); }

  // Names the next value. Valid only directly inside an object.
  void key(const std::string& k) {
    comma();
    newline();
    writeString(k);
    out_ << ": ";
    pendingComma_ = false;
    suppressNewline_ = true;
  }

  void value(const std::string& v) {
    comma();
    newline();
    writeString(v);
  }
  void value(const char* v) { value(std::string(v)); }
  void value(bool v) {
    comma();
    newline();
    out_ << (v ? "true" : "false");
  }
  void value(int v) {
    comma();
    newline();
    out_ << v;
  }
  void value(long long v) {
    comma();
    newline();
    out_ << v;
  }
  void value(double v) {
    comma();
    newline();
    writeNumber(v);
  }
  void null() {
    comma();
    newline();
    out_ << "null";
  }

  // Convenience for the shapes reports actually contain.
  void keyValue(const std::string& k, double v) { key(k); value(v); }
  void keyValue(const std::string& k, int v) { key(k); value(v); }
  void keyValue(const std::string& k, bool v) { key(k); value(v); }
  void keyValue(const std::string& k, const std::string& v) { key(k); value(v); }

  void keyValue(const std::string& k, const std::vector<double>& v) {
    key(k);
    beginArray();
    for (double d : v) value(d);
    endArray();
  }

  void keyValue(const std::string& k,
                const std::vector<std::vector<double>>& rows) {
    key(k);
    beginArray();
    for (const auto& row : rows) {
      beginArray();
      for (double d : row) value(d);
      endArray();
    }
    endArray();
  }

 private:
  void push(bool isObject) {
    stack_.push_back(isObject);
    pendingComma_ = false;
    suppressNewline_ = false;
  }

  void pop(char close) {
    const bool wasEmpty = !pendingComma_;
    stack_.pop_back();
    if (!wasEmpty) {
      out_ << '\n' << std::string(stack_.size() * indentWidth_, ' ');
    }
    out_ << close;
    pendingComma_ = true;
    suppressNewline_ = false;
  }

  void comma() {
    if (pendingComma_) out_ << ',';
  }

  void newline() {
    if (suppressNewline_) {
      suppressNewline_ = false;
    } else if (!stack_.empty()) {
      out_ << '\n' << std::string(stack_.size() * indentWidth_, ' ');
    }
    pendingComma_ = true;
  }

  void writeNumber(double v) {
    // NaN and infinity are not representable in JSON. Emitting them bare
    // produces a document no parser accepts, so they become null.
    if (!std::isfinite(v)) {
      out_ << "null";
      return;
    }
    std::ostringstream ss;
    ss.precision(std::numeric_limits<double>::max_digits10);
    ss << v;
    out_ << ss.str();
  }

  void writeString(const std::string& s) {
    out_ << '"';
    for (unsigned char c : s) {
      switch (c) {
        case '"': out_ << "\\\""; break;
        case '\\': out_ << "\\\\"; break;
        case '\n': out_ << "\\n"; break;
        case '\r': out_ << "\\r"; break;
        case '\t': out_ << "\\t"; break;
        case '\b': out_ << "\\b"; break;
        case '\f': out_ << "\\f"; break;
        default:
          if (c < 0x20) {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\u%04x", c);
            out_ << buf;
          } else {
            out_ << static_cast<char>(c);
          }
      }
    }
    out_ << '"';
  }

  std::ostream& out_;
  int indentWidth_;
  std::vector<bool> stack_;
  bool pendingComma_ = false;
  bool suppressNewline_ = false;
};

}  // namespace json
}  // namespace calib
}  // namespace slamcpp

#endif  // CALIB_JSON_HPP

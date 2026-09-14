// Source location for assertion and exception messages.
//
// Replaces sm_common's sm/source_file_pos.hpp. Behaviour is unchanged; the
// class is renamed to the port's convention and the implicit conversion to
// std::string is dropped, because it made every ostream insertion ambiguous
// with the free operator<< below.
#ifndef CALIB_CORE_SOURCE_FILE_POS_HPP
#define CALIB_CORE_SOURCE_FILE_POS_HPP

#include <ostream>
#include <sstream>
#include <string>

namespace slamcpp {
namespace calib {

class SourceFilePos {
 public:
  SourceFilePos(std::string functionName, std::string fileName, int lineNumber)
      : function(std::move(functionName)),
        file(std::move(fileName)),
        line(lineNumber) {}

  std::string toString() const {
    std::stringstream s;
    s << file << ":" << line << ": " << function << "()";
    return s.str();
  }

  std::string function;
  std::string file;
  int line;
};

inline std::ostream& operator<<(std::ostream& out, const SourceFilePos& sfp) {
  out << sfp.file << ":" << sfp.line << ": " << sfp.function << "()";
  return out;
}

}  // namespace calib
}  // namespace slamcpp

#define CALIB_SOURCE_FILE_POS \
  ::slamcpp::calib::SourceFilePos(__FUNCTION__, __FILE__, __LINE__)

#endif  // CALIB_CORE_SOURCE_FILE_POS_HPP

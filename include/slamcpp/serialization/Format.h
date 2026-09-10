/**
 * Archive formats: binary, text and XML.
 *
 * The archive front-end in Archive.h knows how to walk a C++ object graph. It
 * knows nothing about how the result is spelled on disk. That is this file's
 * job: three backends behind one pair of interfaces, so adding a format costs
 * a backend rather than a second traversal.
 *
 * The readers are far simpler than a general parser has any right to be, and
 * the reason is worth stating. Saving and loading run the *same* serialize()
 * bodies in the same order, so the reader already knows what comes next. It
 * never has to interpret the document, only to pull the next value out of it
 * in order. Structural markers exist to make the text and XML readable, and
 * the readers step over them rather than dispatching on them.
 *
 * Binary is the compact default. Text is meant to be read and diffed. XML is
 * for feeding the state to something else; it is written by hand here rather
 * than by pulling in a parser, because what is needed is a scanner, not a
 * document model.
 */

#ifndef SLAMCPP_SERIALIZATION_FORMAT_H
#define SLAMCPP_SERIALIZATION_FORMAT_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <istream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace slamcpp
{
namespace serialization
{

enum class Format
{
    Binary,
    Text,
    Xml
};

constexpr std::uint16_t kArchiveVersion = 1;

// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------
class OutputBackend
{
public:
    virtual ~OutputBackend() = default;

    virtual void prologue() = 0;
    virtual void epilogue() {}

    virtual void beginObject() {}
    virtual void endObject() {}
    virtual void beginSequence(std::uint64_t count) = 0;
    virtual void endSequence() {}

    virtual void writeBool(bool v) = 0;
    virtual void writeInt(std::int64_t v, unsigned width, bool isSigned) = 0;
    virtual void writeFloat(double v, unsigned width) = 0;
    virtual void writeString(const std::string& v) = 0;
    virtual void writeBlob(const void* data, std::size_t bytes) = 0;
};

class InputBackend
{
public:
    virtual ~InputBackend() = default;

    virtual void prologue() = 0;

    virtual void beginObject() {}
    virtual void endObject() {}
    virtual std::uint64_t beginSequence() = 0;
    virtual void endSequence() {}

    virtual bool         readBool() = 0;
    virtual std::int64_t readInt(unsigned width, bool isSigned) = 0;
    virtual double       readFloat(unsigned width) = 0;
    virtual std::string  readString() = 0;
    virtual void         readBlob(void* data, std::size_t bytes) = 0;
};

namespace detail
{

inline void fail(const std::string& what)
{
    throw std::runtime_error("slamcpp archive: " + what);
}

inline char hexDigit(unsigned v)
{
    return static_cast<char>(v < 10 ? '0' + v : 'a' + (v - 10));
}

inline unsigned hexValue(char c)
{
    if (c >= '0' && c <= '9') return static_cast<unsigned>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<unsigned>(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return static_cast<unsigned>(c - 'A' + 10);
    fail("expected a hex digit");
    return 0;
}

}  // namespace detail

// ===========================================================================
// Binary
// ===========================================================================

// Records the widths the writer used, so a file from an incompatible build is
// refused rather than misread. Text and XML need no such guard: they carry
// values, not the machine's representation of them.
struct BinaryHeader
{
    char          magic[8];
    std::uint16_t version;
    std::uint8_t  sizeOfLong;
    std::uint8_t  sizeOfDouble;
    std::uint8_t  reserved[4];
};

class BinaryOutput : public OutputBackend
{
public:
    explicit BinaryOutput(std::ostream& os) : os_(os) {}

    void prologue() override
    {
        BinaryHeader h{};
        std::memcpy(h.magic, "SLAMCPPA", 8);
        h.version = kArchiveVersion;
        h.sizeOfLong = sizeof(long);
        h.sizeOfDouble = sizeof(double);
        raw(&h, sizeof(h));
    }

    void beginSequence(std::uint64_t count) override { raw(&count, sizeof(count)); }

    void writeBool(bool v) override { raw(&v, sizeof(v)); }

    void writeInt(std::int64_t v, unsigned width, bool) override
    {
        // Written at its natural width, so the file stays the size it was.
        switch (width)
        {
            case 1: { std::int8_t t = static_cast<std::int8_t>(v); raw(&t, 1); break; }
            case 2: { std::int16_t t = static_cast<std::int16_t>(v); raw(&t, 2); break; }
            case 4: { std::int32_t t = static_cast<std::int32_t>(v); raw(&t, 4); break; }
            default: raw(&v, 8); break;
        }
    }

    void writeFloat(double v, unsigned width) override
    {
        if (width == 4)
        {
            float t = static_cast<float>(v);
            raw(&t, 4);
        }
        else
        {
            raw(&v, 8);
        }
    }

    void writeString(const std::string& v) override
    {
        const std::uint64_t n = v.size();
        raw(&n, sizeof(n));
        if (n != 0)
            raw(v.data(), v.size());
    }

    void writeBlob(const void* data, std::size_t bytes) override { raw(data, bytes); }

private:
    void raw(const void* p, std::size_t n)
    {
        os_.write(static_cast<const char*>(p), static_cast<std::streamsize>(n));
        if (!os_)
            detail::fail("write failed");
    }

    std::ostream& os_;
};

class BinaryInput : public InputBackend
{
public:
    explicit BinaryInput(std::istream& is) : is_(is) {}

    void prologue() override
    {
        BinaryHeader h{};
        raw(&h, sizeof(h));
        if (std::memcmp(h.magic, "SLAMCPPA", 8) != 0)
            detail::fail("not a slamcpp binary archive");
        if (h.version != kArchiveVersion)
            detail::fail("version mismatch");
        if (h.sizeOfLong != sizeof(long) || h.sizeOfDouble != sizeof(double))
            detail::fail("written by an incompatible build");
    }

    std::uint64_t beginSequence() override
    {
        std::uint64_t n = 0;
        raw(&n, sizeof(n));
        return n;
    }

    bool readBool() override
    {
        bool v = false;
        raw(&v, sizeof(v));
        return v;
    }

    std::int64_t readInt(unsigned width, bool isSigned) override
    {
        switch (width)
        {
            case 1:
            {
                std::int8_t t = 0;
                raw(&t, 1);
                return isSigned ? t : static_cast<std::int64_t>(static_cast<std::uint8_t>(t));
            }
            case 2:
            {
                std::int16_t t = 0;
                raw(&t, 2);
                return isSigned ? t : static_cast<std::int64_t>(static_cast<std::uint16_t>(t));
            }
            case 4:
            {
                std::int32_t t = 0;
                raw(&t, 4);
                return isSigned ? t : static_cast<std::int64_t>(static_cast<std::uint32_t>(t));
            }
            default:
            {
                std::int64_t t = 0;
                raw(&t, 8);
                return t;
            }
        }
    }

    double readFloat(unsigned width) override
    {
        if (width == 4)
        {
            float t = 0.f;
            raw(&t, 4);
            return t;
        }
        double t = 0.0;
        raw(&t, 8);
        return t;
    }

    std::string readString() override
    {
        std::uint64_t n = 0;
        raw(&n, sizeof(n));
        std::string s;
        s.resize(static_cast<std::size_t>(n));
        if (n != 0)
            raw(&s[0], static_cast<std::size_t>(n));
        return s;
    }

    void readBlob(void* data, std::size_t bytes) override { raw(data, bytes); }

private:
    void raw(void* p, std::size_t n)
    {
        is_.read(static_cast<char*>(p), static_cast<std::streamsize>(n));
        if (is_.gcount() != static_cast<std::streamsize>(n))
            detail::fail("read failed or file truncated");
    }

    std::istream& is_;
};

// ===========================================================================
// Text
// ===========================================================================

class TextOutput : public OutputBackend
{
public:
    explicit TextOutput(std::ostream& os) : os_(os) {}

    void prologue() override { os_ << "slamcpp text archive " << kArchiveVersion << "\n"; }

    void beginObject() override { line("{"); ++depth_; }
    void endObject() override { --depth_; line("}"); }

    void beginSequence(std::uint64_t count) override
    {
        line("[ " + std::to_string(count));
        ++depth_;
    }
    void endSequence() override { --depth_; line("]"); }

    void writeBool(bool v) override { line(v ? "1" : "0"); }

    void writeInt(std::int64_t v, unsigned, bool isSigned) override
    {
        line(isSigned ? std::to_string(v)
                      : std::to_string(static_cast<std::uint64_t>(v)));
    }

    // Printed with enough digits to come back bit-for-bit.
    void writeFloat(double v, unsigned width) override
    {
        char buf[64];
        if (width == 4)
            std::snprintf(buf, sizeof(buf), "%.*g",
                          std::numeric_limits<float>::max_digits10,
                          static_cast<double>(static_cast<float>(v)));
        else
            std::snprintf(buf, sizeof(buf), "%.*g",
                          std::numeric_limits<double>::max_digits10, v);
        line(buf);
    }

    void writeString(const std::string& v) override
    {
        std::string out = "\"";
        for (char c : v)
        {
            switch (c)
            {
                case '"':  out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                default:   out += c; break;
            }
        }
        out += '"';
        line(out);
    }

    void writeBlob(const void* data, std::size_t bytes) override
    {
        const unsigned char* p = static_cast<const unsigned char*>(data);
        std::string out = "#" + std::to_string(bytes) + ":";
        out.reserve(out.size() + bytes * 2 + 1);
        for (std::size_t i = 0; i < bytes; ++i)
        {
            out += detail::hexDigit(p[i] >> 4);
            out += detail::hexDigit(p[i] & 0x0F);
        }
        line(out);
    }

private:
    void line(const std::string& s)
    {
        os_ << std::string(static_cast<std::size_t>(depth_) * 2, ' ') << s << "\n";
        if (!os_)
            detail::fail("write failed");
    }

    std::ostream& os_;
    int           depth_ = 0;
};

class TextInput : public InputBackend
{
public:
    explicit TextInput(std::istream& is) : is_(is) {}

    void prologue() override
    {
        std::string line;
        if (!std::getline(is_, line) || line.rfind("slamcpp text archive", 0) != 0)
            detail::fail("not a slamcpp text archive");
    }

    void beginObject() override { expect('{'); }
    void endObject() override { expect('}'); }

    std::uint64_t beginSequence() override
    {
        expect('[');
        return static_cast<std::uint64_t>(std::stoull(token()));
    }
    void endSequence() override { expect(']'); }

    bool readBool() override { return token() != "0"; }

    std::int64_t readInt(unsigned, bool isSigned) override
    {
        const std::string t = token();
        return isSigned ? std::stoll(t)
                        : static_cast<std::int64_t>(std::stoull(t));
    }

    double readFloat(unsigned) override { return std::stod(token()); }

    std::string readString() override
    {
        skipSpace();
        if (get() != '"')
            detail::fail("expected a quoted string");
        std::string out;
        for (;;)
        {
            const char c = get();
            if (c == '"')
                break;
            if (c != '\\')
            {
                out += c;
                continue;
            }
            const char e = get();
            switch (e)
            {
                case 'n':  out += '\n'; break;
                case 'r':  out += '\r'; break;
                case 't':  out += '\t'; break;
                case '"':  out += '"'; break;
                case '\\': out += '\\'; break;
                default:   detail::fail("unknown string escape"); break;
            }
        }
        return out;
    }

    void readBlob(void* data, std::size_t bytes) override
    {
        skipSpace();
        if (get() != '#')
            detail::fail("expected a blob");
        std::string digits;
        for (;;)
        {
            const char c = get();
            if (c == ':')
                break;
            digits += c;
        }
        if (std::stoull(digits) != bytes)
            detail::fail("blob length does not match the expected size");

        unsigned char* p = static_cast<unsigned char*>(data);
        for (std::size_t i = 0; i < bytes; ++i)
        {
            const unsigned hi = detail::hexValue(get());
            const unsigned lo = detail::hexValue(get());
            p[i] = static_cast<unsigned char>((hi << 4) | lo);
        }
    }

private:
    char get()
    {
        const int c = is_.get();
        if (c == std::char_traits<char>::eof())
            detail::fail("unexpected end of file");
        return static_cast<char>(c);
    }

    void skipSpace()
    {
        int c = is_.peek();
        while (c == ' ' || c == '\n' || c == '\r' || c == '\t')
        {
            is_.get();
            c = is_.peek();
        }
    }

    void expect(char want)
    {
        skipSpace();
        const char c = get();
        if (c != want)
            detail::fail(std::string("expected '") + want + "'");
    }

    std::string token()
    {
        skipSpace();
        std::string out;
        int         c = is_.peek();
        while (c != std::char_traits<char>::eof() && c != ' ' && c != '\n' && c != '\r' &&
               c != '\t')
        {
            out += static_cast<char>(is_.get());
            c = is_.peek();
        }
        if (out.empty())
            detail::fail("expected a value");
        return out;
    }

    std::istream& is_;
};

// ===========================================================================
// XML
// ===========================================================================

class XmlOutput : public OutputBackend
{
public:
    explicit XmlOutput(std::ostream& os) : os_(os) {}

    void prologue() override
    {
        os_ << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        os_ << "<slamcpp version=\"" << kArchiveVersion << "\">\n";
        ++depth_;
    }
    void epilogue() override
    {
        --depth_;
        os_ << "</slamcpp>\n";
    }

    void beginObject() override { line("<o>"); ++depth_; }
    void endObject() override { --depth_; line("</o>"); }

    void beginSequence(std::uint64_t count) override
    {
        line("<s n=\"" + std::to_string(count) + "\">");
        ++depth_;
    }
    void endSequence() override { --depth_; line("</s>"); }

    void writeBool(bool v) override { value(v ? "1" : "0"); }

    void writeInt(std::int64_t v, unsigned, bool isSigned) override
    {
        value(isSigned ? std::to_string(v) : std::to_string(static_cast<std::uint64_t>(v)));
    }

    void writeFloat(double v, unsigned width) override
    {
        char buf[64];
        if (width == 4)
            std::snprintf(buf, sizeof(buf), "%.*g",
                          std::numeric_limits<float>::max_digits10,
                          static_cast<double>(static_cast<float>(v)));
        else
            std::snprintf(buf, sizeof(buf), "%.*g",
                          std::numeric_limits<double>::max_digits10, v);
        value(buf);
    }

    void writeString(const std::string& v) override { value(escape(v)); }

    void writeBlob(const void* data, std::size_t bytes) override
    {
        const unsigned char* p = static_cast<const unsigned char*>(data);
        std::string out;
        out.reserve(bytes * 2);
        for (std::size_t i = 0; i < bytes; ++i)
        {
            out += detail::hexDigit(p[i] >> 4);
            out += detail::hexDigit(p[i] & 0x0F);
        }
        line("<b n=\"" + std::to_string(bytes) + "\">" + out + "</b>");
    }

private:
    // Only the three characters that can end an element early, plus the
    // control characters XML does not allow at all.
    static std::string escape(const std::string& v)
    {
        std::string out;
        out.reserve(v.size());
        for (unsigned char c : v)
        {
            switch (c)
            {
                case '&': out += "&amp;"; break;
                case '<': out += "&lt;"; break;
                case '>': out += "&gt;"; break;
                default:
                    if (c < 0x20 || c == 0x7F)
                    {
                        char buf[16];
                        std::snprintf(buf, sizeof(buf), "&#x%02X;", c);
                        out += buf;
                    }
                    else
                    {
                        out += static_cast<char>(c);
                    }
                    break;
            }
        }
        return out;
    }

    void value(const std::string& text) { line("<v>" + text + "</v>"); }

    void line(const std::string& s)
    {
        os_ << std::string(static_cast<std::size_t>(depth_) * 2, ' ') << s << "\n";
        if (!os_)
            detail::fail("write failed");
    }

    std::ostream& os_;
    int           depth_ = 0;
};

class XmlInput : public InputBackend
{
public:
    explicit XmlInput(std::istream& is) : is_(is) {}

    void prologue() override
    {
        // Skip the declaration if one is there, then the document element.
        skipSpace();
        if (peekIs("<?"))
            scanPast("?>");
        skipSpace();
        if (!consumeTag("<slamcpp"))
            detail::fail("not a slamcpp XML archive");
        scanPast(">");
    }

    void beginObject() override { expectTag("<o>"); }
    void endObject() override { expectTag("</o>"); }

    std::uint64_t beginSequence() override
    {
        skipSpace();
        if (!consumeTag("<s"))
            detail::fail("expected a sequence");
        const std::uint64_t n = attributeN();
        scanPast(">");
        return n;
    }
    void endSequence() override { expectTag("</s>"); }

    bool readBool() override { return valueText() != "0"; }

    std::int64_t readInt(unsigned, bool isSigned) override
    {
        const std::string t = valueText();
        return isSigned ? std::stoll(t) : static_cast<std::int64_t>(std::stoull(t));
    }

    double readFloat(unsigned) override { return std::stod(valueText()); }

    std::string readString() override { return valueText(); }

    void readBlob(void* data, std::size_t bytes) override
    {
        skipSpace();
        if (!consumeTag("<b"))
            detail::fail("expected a blob");
        const std::uint64_t n = attributeN();
        scanPast(">");
        if (n != bytes)
            detail::fail("blob length does not match the expected size");

        unsigned char* p = static_cast<unsigned char*>(data);
        for (std::size_t i = 0; i < bytes; ++i)
        {
            const unsigned hi = detail::hexValue(get());
            const unsigned lo = detail::hexValue(get());
            p[i] = static_cast<unsigned char>((hi << 4) | lo);
        }
        expectTag("</b>");
    }

private:
    char get()
    {
        const int c = is_.get();
        if (c == std::char_traits<char>::eof())
            detail::fail("unexpected end of file");
        return static_cast<char>(c);
    }

    void skipSpace()
    {
        int c = is_.peek();
        while (c == ' ' || c == '\n' || c == '\r' || c == '\t')
        {
            is_.get();
            c = is_.peek();
        }
    }

    bool peekIs(const char* s)
    {
        return is_.peek() == static_cast<unsigned char>(s[0]);
    }

    // Consumes the literal if it is next; leaves the stream untouched if not.
    bool consumeTag(const char* tag)
    {
        const std::size_t n = std::strlen(tag);
        std::vector<char> buf(n);
        is_.read(buf.data(), static_cast<std::streamsize>(n));
        if (is_.gcount() != static_cast<std::streamsize>(n) ||
            std::memcmp(buf.data(), tag, n) != 0)
        {
            for (std::streamsize i = is_.gcount(); i > 0; --i)
                is_.putback(buf[static_cast<std::size_t>(i - 1)]);
            is_.clear();
            return false;
        }
        return true;
    }

    void expectTag(const char* tag)
    {
        skipSpace();
        if (!consumeTag(tag))
            detail::fail(std::string("expected ") + tag);
    }

    void scanPast(const char* end)
    {
        const std::size_t n = std::strlen(end);
        std::string       window;
        for (;;)
        {
            window += get();
            if (window.size() > n)
                window.erase(0, window.size() - n);
            if (window.size() == n && window == end)
                return;
        }
    }

    // Reads the n="..." attribute of the tag just consumed.
    std::uint64_t attributeN()
    {
        scanPast("n=\"");
        std::string digits;
        for (;;)
        {
            const char c = get();
            if (c == '"')
                break;
            digits += c;
        }
        return static_cast<std::uint64_t>(std::stoull(digits));
    }

    std::string valueText()
    {
        expectTag("<v>");
        std::string raw;
        for (;;)
        {
            const char c = get();
            if (c == '<')
                break;
            raw += c;
        }
        // The '<' just consumed opens "/v>".
        if (!consumeTag("/v>"))
            detail::fail("expected </v>");
        return unescape(raw);
    }

    static std::string unescape(const std::string& v)
    {
        std::string out;
        out.reserve(v.size());
        for (std::size_t i = 0; i < v.size(); ++i)
        {
            if (v[i] != '&')
            {
                out += v[i];
                continue;
            }
            const std::size_t semi = v.find(';', i);
            if (semi == std::string::npos)
                detail::fail("malformed entity");
            const std::string ent = v.substr(i + 1, semi - i - 1);
            if (ent == "amp")       out += '&';
            else if (ent == "lt")   out += '<';
            else if (ent == "gt")   out += '>';
            else if (ent == "quot") out += '"';
            else if (ent == "apos") out += '\'';
            else if (ent.size() > 2 && ent[0] == '#' && (ent[1] == 'x' || ent[1] == 'X'))
                out += static_cast<char>(std::stoul(ent.substr(2), nullptr, 16));
            else if (ent.size() > 1 && ent[0] == '#')
                out += static_cast<char>(std::stoul(ent.substr(1)));
            else
                detail::fail("unknown entity &" + ent + ";");
            i = semi;
        }
        return out;
    }

    std::istream& is_;
};

}  // namespace serialization
}  // namespace slamcpp

#endif  // SLAMCPP_SERIALIZATION_FORMAT_H

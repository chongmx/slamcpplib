/**
 * A small serialization archive, replacing Boost.Serialization.
 *
 * Why this exists
 * ---------------
 * Boost.Serialization was the library's last Boost dependency, pulled in for
 * one feature: saving and loading the Atlas. It is a heavyweight, and most of
 * what makes it heavyweight is machinery this code never asked for.
 *
 * In particular it tracks pointers, so that an object reachable twice is
 * written once and restored as one shared object. The SLAM core does not need
 * that. It flattens its own graph first: PreSave() rewrites every
 * cross-reference as an integer id, and PostLoad() turns the ids back into
 * pointers. What remains is a strict ownership tree, where Atlas owns its maps
 * and cameras and each Map owns its key frames and map points, and every heap
 * object is reached exactly once. Serializing a tree needs no tracking table.
 *
 * So this archive deliberately does not track pointers. Serializing the same
 * object through two different owners would duplicate it, which is a bug in
 * the caller's ownership model rather than something to paper over here.
 *
 * The API deliberately mirrors the small part of Boost's that the core used:
 * `ar & x` chains, `Archive::is_saving::value` picks a direction inside a
 * shared serialize() body, make_array() handles raw buffers, and a friend
 * declaration of `access` reaches private members. That kept the eighteen
 * existing serialize() bodies almost entirely unchanged.
 *
 * Formats
 * -------
 * This file walks the object graph; it does not decide how the result is
 * spelled. Format.h holds the three backends: Binary, Text and Xml. Binary is
 * compact and is what the Atlas uses by default. Text is meant to be read and
 * diffed. Xml is for handing the state to something else.
 *
 * Text and XML carry values rather than the machine's representation of them,
 * so they move between builds and architectures. Binary does not, and says so:
 * it records the type widths the writer used and refuses a file that disagrees.
 * Floating point is printed with enough digits to come back bit-for-bit.
 */

#ifndef SLAMCPP_SERIALIZATION_ARCHIVE_H
#define SLAMCPP_SERIALIZATION_ARCHIVE_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <istream>
#include <list>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include "slamcpp/serialization/Format.h"

namespace slamcpp
{
namespace serialization
{

class OutputArchive;
class InputArchive;

// ---------------------------------------------------------------------------
// Reaches the private serialize() members. The classes being serialized name
// this as a friend, exactly as they used to name boost::serialization::access.
// ---------------------------------------------------------------------------
class access
{
public:
    template <class Archive, class T>
    static void serialize(Archive& ar, T& t, const unsigned int version)
    {
        t.serialize(ar, version);
    }
};

// ---------------------------------------------------------------------------
// A raw buffer of fixed length, the stand-in for boost's make_array. Used for
// Eigen and Sophus coefficient blocks and for cv::Mat pixel rows, where the
// length is known from elsewhere.
// ---------------------------------------------------------------------------
template <class T>
struct ArrayRef
{
    T*          data;
    std::size_t size;
};

template <class T>
inline ArrayRef<T> make_array(T* data, std::size_t size)
{
    return ArrayRef<T>{data, size};
}

// Serializes the base class part of a derived object. The stand-in for
// boost::serialization::base_object.
template <class Base, class Derived>
inline Base& base_object(Derived& d)
{
    static_assert(std::is_base_of<Base, Derived>::value, "not a base class");
    return static_cast<Base&>(d);
}

// ---------------------------------------------------------------------------
// Runtime type registry for polymorphic owned pointers.
//
// The core has exactly one polymorphic hierarchy in its saved state:
// GeometricCamera, with Pinhole and KannalaBrandt8 under it. A saved pointer
// carries a short string tag so the loader knows which one to build. This is
// the replacement for BOOST_CLASS_EXPORT_GUID and for register_type<>().
// ---------------------------------------------------------------------------
template <class Base>
class PolymorphicRegistry
{
public:
    struct Entry
    {
        std::string                                guid;
        std::function<void(OutputArchive&, Base*)> save;
        std::function<Base*(InputArchive&)>        load;
    };

    template <class Derived>
    static void add(const std::string& guid);

    static const Entry* find(const std::type_index& type)
    {
        auto it = byType().find(type);
        return it == byType().end() ? nullptr : &it->second;
    }

    static const Entry* find(const std::string& guid)
    {
        auto it = byGuid().find(guid);
        return it == byGuid().end() ? nullptr : &it->second;
    }

    // Function-local statics, so registration from any translation unit works
    // regardless of static initialisation order.
    static std::unordered_map<std::type_index, Entry>& byType()
    {
        static std::unordered_map<std::type_index, Entry> m;
        return m;
    }

    static std::unordered_map<std::string, Entry>& byGuid()
    {
        static std::unordered_map<std::string, Entry> m;
        return m;
    }
};

// ---------------------------------------------------------------------------
// Type dispatch
// ---------------------------------------------------------------------------
namespace detail
{

template <class T>          struct is_array_ref              : std::false_type {};
template <class T>          struct is_array_ref<ArrayRef<T>> : std::true_type {};

template <class T>          struct is_std_vector             : std::false_type {};
template <class T, class A> struct is_std_vector<std::vector<T, A>> : std::true_type {};

template <class T>          struct is_std_list               : std::false_type {};
template <class T, class A> struct is_std_list<std::list<T, A>> : std::true_type {};

template <class T>                   struct is_std_set       : std::false_type {};
template <class T, class C, class A> struct is_std_set<std::set<T, C, A>> : std::true_type {};

template <class T>                            struct is_std_map : std::false_type {};
template <class K, class V, class C, class A> struct is_std_map<std::map<K, V, C, A>> : std::true_type {};

template <class T>          struct is_std_pair               : std::false_type {};
template <class A, class B> struct is_std_pair<std::pair<A, B>> : std::true_type {};

// A free serialize() found by argument-dependent lookup takes precedence over
// a member one. That is what lets a type we do not own, such as DBoW3's
// BowVector, be supported without touching the vendored tree.
template <class Archive, class T, class = void>
struct has_free_serialize : std::false_type {};

template <class Archive, class T>
struct has_free_serialize<
    Archive, T,
    std::void_t<decltype(serialize(std::declval<Archive&>(), std::declval<T&>(), 0u))>>
    : std::true_type {};

template <class T>
constexpr bool is_scalar_v = (std::is_arithmetic<T>::value || std::is_enum<T>::value);

// Byte-wide elements go out as one blob, which keeps a key frame's descriptor
// matrix from becoming a few hundred thousand separate values.
template <class T>
constexpr bool is_byte_v = (std::is_integral<T>::value && sizeof(T) == 1);

}  // namespace detail

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------
class OutputArchive
{
public:
    using is_saving  = std::true_type;
    using is_loading = std::false_type;

    explicit OutputArchive(std::ostream& os, Format format = Format::Binary)
    {
        switch (format)
        {
            case Format::Text: backend_.reset(new TextOutput(os)); break;
            case Format::Xml:  backend_.reset(new XmlOutput(os)); break;
            default:           backend_.reset(new BinaryOutput(os)); break;
        }
        backend_->prologue();
    }

    ~OutputArchive() { backend_->epilogue(); }

    // Present so that serialize() bodies written against Boost still compile.
    // Registration happens through SLAMCPP_REGISTER_POLYMORPHIC instead.
    template <class T>
    void register_type()
    {
    }

    template <class T>
    OutputArchive& operator&(const T& t)
    {
        save(const_cast<T&>(t));
        return *this;
    }

    // make_array() returns a temporary, so this takes it by value. ArrayRef is
    // a pointer and a length; copying it does not copy the buffer.
    template <class T>
    OutputArchive& operator&(ArrayRef<T> a)
    {
        save(a);
        return *this;
    }

    template <class T>
    OutputArchive& operator<<(const T& t)
    {
        return *this & t;
    }

private:
    template <class T>
    void save(T& t)
    {
        if constexpr (std::is_same<T, bool>::value)
        {
            backend_->writeBool(t);
        }
        else if constexpr (detail::is_scalar_v<T>)
        {
            saveScalar(t);
        }
        else if constexpr (detail::is_array_ref<T>::value)
        {
            using E = std::remove_cv_t<std::remove_reference_t<decltype(*t.data)>>;
            if constexpr (detail::is_byte_v<E>)
            {
                backend_->writeBlob(t.data, t.size * sizeof(E));
            }
            else
            {
                backend_->beginSequence(t.size);
                for (std::size_t i = 0; i < t.size; ++i)
                    save(t.data[i]);
                backend_->endSequence();
            }
        }
        else if constexpr (std::is_same<T, std::string>::value)
        {
            backend_->writeString(t);
        }
        else if constexpr (std::is_pointer<T>::value)
        {
            savePointer(t);
        }
        else if constexpr (detail::is_std_pair<T>::value)
        {
            backend_->beginObject();
            save(const_cast<std::remove_const_t<typename T::first_type>&>(t.first));
            save(t.second);
            backend_->endObject();
        }
        else if constexpr (detail::is_std_vector<T>::value || detail::is_std_list<T>::value ||
                           detail::is_std_set<T>::value || detail::is_std_map<T>::value)
        {
            backend_->beginSequence(t.size());
            for (auto& e : t)
            {
                // A set's elements and a map's keys are const in the container.
                using E = std::remove_const_t<std::remove_reference_t<decltype(e)>>;
                save(const_cast<E&>(e));
            }
            backend_->endSequence();
        }
        else if constexpr (detail::has_free_serialize<OutputArchive, T>::value)
        {
            backend_->beginObject();
            serialize(*this, t, kArchiveVersion);
            backend_->endObject();
        }
        else
        {
            backend_->beginObject();
            access::serialize(*this, t, kArchiveVersion);
            backend_->endObject();
        }
    }

    template <class T>
    void saveScalar(T& t)
    {
        if constexpr (std::is_floating_point<T>::value)
        {
            static_assert(sizeof(T) <= 8, "long double is not supported");
            backend_->writeFloat(static_cast<double>(t), sizeof(T));
        }
        else if constexpr (std::is_enum<T>::value)
        {
            using U = std::underlying_type_t<T>;
            backend_->writeInt(static_cast<std::int64_t>(t), sizeof(U),
                               std::is_signed<U>::value);
        }
        else
        {
            backend_->writeInt(static_cast<std::int64_t>(t), sizeof(T),
                               std::is_signed<T>::value);
        }
    }

    // std::vector<bool> packs its bits, so iterating it yields a proxy rather
    // than a bool&. Round-tripping through a real bool keeps that case honest.
    void save(std::vector<bool>::reference t)
    {
        const bool b = t;
        backend_->writeBool(b);
    }

    template <class T>
    void savePointer(T*& p)
    {
        backend_->writeBool(p != nullptr);
        if (p == nullptr)
            return;

        using Elem = std::remove_cv_t<T>;
        if constexpr (std::is_polymorphic<Elem>::value)
        {
            const std::type_index dynamic(typeid(*p));
            if (dynamic == std::type_index(typeid(Elem)))
            {
                // The static type is the whole story; tag it as such.
                backend_->writeString(std::string());
                save(*p);
                return;
            }
            const auto* entry = PolymorphicRegistry<Elem>::find(dynamic);
            if (entry == nullptr)
                throw std::runtime_error(
                    std::string("slamcpp: unregistered polymorphic type '") + dynamic.name() +
                    "'; add SLAMCPP_REGISTER_POLYMORPHIC for it");
            backend_->writeString(entry->guid);
            entry->save(*this, p);
        }
        else
        {
            save(*p);
        }
    }

    std::unique_ptr<OutputBackend> backend_;

    template <class Base>
    friend class PolymorphicRegistry;
    friend class access;
};

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------
class InputArchive
{
public:
    using is_saving  = std::false_type;
    using is_loading = std::true_type;

    explicit InputArchive(std::istream& is, Format format = Format::Binary)
    {
        switch (format)
        {
            case Format::Text: backend_.reset(new TextInput(is)); break;
            case Format::Xml:  backend_.reset(new XmlInput(is)); break;
            default:           backend_.reset(new BinaryInput(is)); break;
        }
        backend_->prologue();
    }

    template <class T>
    void register_type()
    {
    }

    template <class T>
    InputArchive& operator&(T& t)
    {
        load(t);
        return *this;
    }

    // See the matching overload on OutputArchive.
    template <class T>
    InputArchive& operator&(ArrayRef<T> a)
    {
        load(a);
        return *this;
    }

    template <class T>
    InputArchive& operator>>(T& t)
    {
        return *this & t;
    }

private:
    template <class T>
    void load(T& t)
    {
        if constexpr (std::is_same<T, bool>::value)
        {
            t = backend_->readBool();
        }
        else if constexpr (detail::is_scalar_v<T>)
        {
            loadScalar(t);
        }
        else if constexpr (detail::is_array_ref<T>::value)
        {
            using E = std::remove_cv_t<std::remove_reference_t<decltype(*t.data)>>;
            if constexpr (detail::is_byte_v<E>)
            {
                backend_->readBlob(t.data, t.size * sizeof(E));
            }
            else
            {
                const std::uint64_t n = backend_->beginSequence();
                if (n != t.size)
                    throw std::runtime_error("slamcpp: fixed array length mismatch");
                for (std::size_t i = 0; i < t.size; ++i)
                    load(t.data[i]);
                backend_->endSequence();
            }
        }
        else if constexpr (std::is_same<T, std::string>::value)
        {
            t = backend_->readString();
        }
        else if constexpr (std::is_pointer<T>::value)
        {
            loadPointer(t);
        }
        else if constexpr (detail::is_std_pair<T>::value)
        {
            backend_->beginObject();
            load(const_cast<std::remove_const_t<typename T::first_type>&>(t.first));
            load(t.second);
            backend_->endObject();
        }
        else if constexpr (detail::is_std_vector<T>::value || detail::is_std_list<T>::value)
        {
            const std::uint64_t n = backend_->beginSequence();
            t.resize(static_cast<std::size_t>(n));
            for (auto& e : t)
                load(e);
            backend_->endSequence();
        }
        else if constexpr (detail::is_std_set<T>::value)
        {
            const std::uint64_t n = backend_->beginSequence();
            t.clear();
            for (std::uint64_t i = 0; i < n; ++i)
            {
                typename T::value_type e{};
                load(e);
                t.insert(std::move(e));
            }
            backend_->endSequence();
        }
        else if constexpr (detail::is_std_map<T>::value)
        {
            const std::uint64_t n = backend_->beginSequence();
            t.clear();
            for (std::uint64_t i = 0; i < n; ++i)
            {
                // Read as the pair the writer emitted, structure and all.
                backend_->beginObject();
                typename T::key_type    k{};
                typename T::mapped_type v{};
                load(k);
                load(v);
                backend_->endObject();
                t.emplace(std::move(k), std::move(v));
            }
            backend_->endSequence();
        }
        else if constexpr (detail::has_free_serialize<InputArchive, T>::value)
        {
            backend_->beginObject();
            serialize(*this, t, kArchiveVersion);
            backend_->endObject();
        }
        else
        {
            backend_->beginObject();
            access::serialize(*this, t, kArchiveVersion);
            backend_->endObject();
        }
    }

    template <class T>
    void loadScalar(T& t)
    {
        if constexpr (std::is_floating_point<T>::value)
        {
            static_assert(sizeof(T) <= 8, "long double is not supported");
            t = static_cast<T>(backend_->readFloat(sizeof(T)));
        }
        else if constexpr (std::is_enum<T>::value)
        {
            using U = std::underlying_type_t<T>;
            t = static_cast<T>(backend_->readInt(sizeof(U), std::is_signed<U>::value));
        }
        else
        {
            t = static_cast<T>(backend_->readInt(sizeof(T), std::is_signed<T>::value));
        }
    }

    void load(std::vector<bool>::reference t) { t = backend_->readBool(); }

    template <class T>
    void loadPointer(T*& p)
    {
        if (!backend_->readBool())
        {
            p = nullptr;
            return;
        }

        using Elem = std::remove_cv_t<T>;
        if constexpr (std::is_polymorphic<Elem>::value)
        {
            const std::string guid = backend_->readString();
            if (!guid.empty())
            {
                const auto* entry = PolymorphicRegistry<Elem>::find(guid);
                if (entry == nullptr)
                    throw std::runtime_error("slamcpp: archive names unregistered type '" +
                                             guid + "'");
                p = entry->load(*this);
                return;
            }
            // An empty tag means the object was written as its static type.
            if constexpr (std::is_abstract<Elem>::value)
            {
                throw std::runtime_error(
                    "slamcpp: archive holds an untagged object of abstract type");
            }
            else
            {
                p = new Elem();
                load(*p);
            }
        }
        else
        {
            p = new Elem();
            load(*p);
        }
    }

    std::unique_ptr<InputBackend> backend_;

    template <class Base>
    friend class PolymorphicRegistry;
    friend class access;
};

// Defined here because it needs both archive types complete.
template <class Base>
template <class Derived>
void PolymorphicRegistry<Base>::add(const std::string& guid)
{
    Entry e;
    e.guid = guid;
    e.save = [](OutputArchive& ar, Base* p) { ar& *static_cast<Derived*>(p); };
    e.load = [](InputArchive& ar) -> Base* {
        Derived* d = new Derived();
        ar& *d;
        return d;
    };
    byType().emplace(std::type_index(typeid(Derived)), e);
    byGuid().emplace(guid, e);
}

}  // namespace serialization
}  // namespace slamcpp

// Registers a derived class so that a pointer to its base can be saved and
// restored as the concrete type it really is. Put this in the derived class's
// .cpp file. Replaces the register_type<>() calls that Boost archives took.
//
// The generated name is keyed on the line number rather than on the class
// name, because the class name is usually qualified and ## cannot paste a "::".
#define SLAMCPP_DETAIL_CAT2(a, b) a##b
#define SLAMCPP_DETAIL_CAT(a, b) SLAMCPP_DETAIL_CAT2(a, b)

#define SLAMCPP_REGISTER_POLYMORPHIC(Base, Derived, Guid)                        \
    namespace                                                                    \
    {                                                                            \
    const bool SLAMCPP_DETAIL_CAT(g_slamcppRegistered_, __LINE__) = [] {         \
        ::slamcpp::serialization::PolymorphicRegistry<Base>::add<Derived>(Guid); \
        return true;                                                             \
    }();                                                                         \
    }

#endif  // SLAMCPP_SERIALIZATION_ARCHIVE_H

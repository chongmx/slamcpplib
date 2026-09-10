/**
 * Bag-of-words vocabulary for slamcpplib.
 *
 * DBoW3 is the only bag-of-words backend. It was chosen over DBoW2 because it
 * carries the descriptor type inside the vocabulary file and dispatches on it
 * at run time, so a binary ORB vocabulary (CV_8U) and a float SuperPoint one
 * (CV_32F) are the same C++ type. DBoW2 fixed the descriptor as a 32-byte
 * binary word in the template arguments, which forced a whole second build of
 * the core to support a second front-end. That is the reason the runtime
 * front-end switch is possible at all, and why DBoW2 is gone for good.
 *
 * Note the one asymmetry: DBoW3's text loader hardcodes CV_8U, so the classic
 * ORBvoc.txt still loads as it always did, while a float vocabulary has to be
 * supplied in DBoW3's own binary or YAML format.
 */

#ifndef SLAMCPP_VOCABULARY_H
#define SLAMCPP_VOCABULARY_H

#include "slamcpp/serialization/Archive.h"

#include <map>
#include <string>
#include <vector>


#include "3rdparty/DBoW3/src/Vocabulary.h"
#include "3rdparty/DBoW3/src/BowVector.h"
#include "3rdparty/DBoW3/src/FeatureVector.h"

namespace ORB_SLAM3
{

// One vocabulary type for every front-end.
typedef DBoW3::Vocabulary Vocabulary;

// Transitional alias. The core still spells its member mpORBvocabulary in a
// few hundred places; that rename belongs to the front-end abstraction pass,
// not to the DBoW3 migration.
typedef Vocabulary ORBVocabulary;

// DBoW2's loadFromTextFile() returned a bool; DBoW3's load() reports failure
// by throwing, and picks the format from the file itself. This keeps the
// bool-returning shape the core was written against.
//
// DBoW3 throws a bare std::string on one path rather than an exception type,
// hence the second catch.
inline bool LoadVocabulary(Vocabulary& voc, const std::string& path)
{
    try
    {
        voc.load(path);
    }
    catch (const std::exception&)
    {
        return false;
    }
    catch (const std::string&)
    {
        return false;
    }
    return !voc.empty();
}

}  // namespace ORB_SLAM3

// ---------------------------------------------------------------------------
// Serialization for the DBoW3 vector types.
//
// KeyFrame writes mBowVec and mFeatVec straight into its archive. The fork of
// DBoW2 this code grew up with had a serialize() member bolted onto both
// classes; stock DBoW3 has no such member and is not ours to patch. The
// support goes here as free functions instead, so the vendored tree stays
// pristine and updatable.
// ---------------------------------------------------------------------------
namespace DBoW3
{

// Found by argument-dependent lookup, so the archive picks these up without
// the vendored DBoW3 tree needing a serialize() member of its own. Both types
// derive from a std::map the archive already handles.
template <class Archive>
void serialize(Archive& ar, BowVector& bow, const unsigned int)
{
    ar& slamcpp::serialization::base_object<std::map<WordId, WordValue> >(bow);
}

template <class Archive>
void serialize(Archive& ar, FeatureVector& feat, const unsigned int)
{
    ar& slamcpp::serialization::base_object<std::map<NodeId, std::vector<unsigned int> > >(feat);
}

}  // namespace DBoW3

#endif  // SLAMCPP_VOCABULARY_H

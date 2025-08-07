

#ifndef ST_PLANNING_UTILS_MAP_UTIL
#define ST_PLANNING_UTILS_MAP_UTIL

#include <iterator>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

namespace e2e_noa {

template <class Collection>
const typename Collection::value_type::second_type& FindOrDie(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  CHECK(it != collection.end()) << " Collection key not found: " << key;
  return it->second;
}

template <class Collection>
typename Collection::value_type::second_type& FindOrDie(
    Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection.find(key);
  CHECK(it != collection.end()) << " Collection key not found: " << key;
  return it->second;
}

template <class Collection>
const typename Collection::value_type::second_type& FindOrDieNoPrint(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  CHECK(it != collection.end()) << " Collection key not found";
  return it->second;
}

template <class Collection>
typename Collection::value_type::second_type& FindOrDieNoPrint(
    Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection.find(key);
  CHECK(it != collection.end()) << " Collection key not found";
  return it->second;
}

template <class Collection>
const typename Collection::value_type::second_type& FindWithDefault(
    const Collection& collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& value) {
  typename Collection::const_iterator it = collection.find(key);
  if (it == collection.end()) {
    return value;
  }
  return it->second;
}

template <class Collection, class Key>
auto FindOrNull(const Collection& collection, const Key& key) -> const
    typename Collection::value_type::second_type* {
  typename Collection::const_iterator it = collection.find(key);
  if (it == collection.end()) {
    return 0;
  }
  return &it->second;
}

template <class Collection>
typename Collection::value_type::second_type* FindOrNull(
    Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection.find(key);
  if (it == collection.end()) {
    return 0;
  }
  return &it->second;
}

template <class Collection>
typename Collection::value_type::second_type FindPtrOrNull(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  if (it == collection.end()) {
    return typename Collection::value_type::second_type();
  }
  return it->second;
}

template <class Collection>
typename Collection::value_type::second_type FindPtrOrNull(
    Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection.find(key);
  if (it == collection.end()) {
    return typename Collection::value_type::second_type();
  }
  return it->second;
}

template <class Collection>
typename Collection::value_type::second_type::element_type* FindLinkedPtrOrNull(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  if (it == collection.end()) {
    return 0;
  }

  return it->second.get();
}

template <class Collection>
typename Collection::value_type::second_type::element_type& FindLinkedPtrOrDie(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  CHECK(it != collection.end()) << " key not found: " << key;

  return *it->second;
}

template <class Collection, class Key, class Value>
bool FindCopy(const Collection& collection, const Key& key,
              Value* const value) {
  typename Collection::const_iterator it = collection.find(key);
  if (it == collection.end()) {
    return false;
  }
  if (value) {
    *value = it->second;
  }
  return true;
}

template <class Collection, class Key>
bool ContainsKey(const Collection& collection, const Key& key) {
  return collection.find(key) != collection.end();
}

template <class Collection, class Key, class Value>
bool ContainsKeyValuePair(const Collection& collection, const Key& key,
                          const Value& value) {
  typedef typename Collection::const_iterator const_iterator;
  std::pair<const_iterator, const_iterator> range = collection.equal_range(key);
  for (const_iterator it = range.first; it != range.second; ++it) {
    if (it->second == value) {
      return true;
    }
  }
  return false;
}

template <class Collection>
bool InsertOrUpdate(Collection* const collection,
                    const typename Collection::value_type& vt) {
  std::pair<typename Collection::iterator, bool> ret = collection->insert(vt);
  if (!ret.second) {
    ret.first->second = vt.second;
    return false;
  }
  return true;
}

template <class Collection>
bool InsertOrUpdate(Collection* const collection,
                    const typename Collection::value_type::first_type& key,
                    const typename Collection::value_type::second_type& value) {
  return InsertOrUpdate(collection,
                        typename Collection::value_type(key, value));
}

template <class Collection, class InputIterator>
void InsertOrUpdateMany(Collection* const collection, InputIterator first,
                        InputIterator last) {
  for (; first != last; ++first) {
    InsertOrUpdate(collection, *first);
  }
}

template <class Collection>
bool InsertAndDeleteExisting(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& value) {
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(key, value));
  if (!ret.second) {
    delete ret.first->second;
    ret.first->second = value;
    return false;
  }
  return true;
}

template <class Collection>
bool InsertIfNotPresent(Collection* const collection,
                        const typename Collection::value_type& vt) {
  return collection->insert(vt).second;
}

template <class Collection>
bool InsertIfNotPresent(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& value) {
  return InsertIfNotPresent(collection,
                            typename Collection::value_type(key, value));
}

template <class Collection>
void InsertOrDie(Collection* const collection,
                 const typename Collection::value_type& value) {
  CHECK(InsertIfNotPresent(collection, value)) << " duplicate value: " << value;
}

template <class Collection>
void InsertOrDieNoPrint(Collection* const collection,
                        const typename Collection::value_type& value) {
  CHECK(InsertIfNotPresent(collection, value)) << " duplicate value.";
}

template <class Collection>
void InsertOrDie(Collection* const collection,
                 const typename Collection::value_type::first_type& key,
                 const typename Collection::value_type::second_type& data) {
  CHECK(InsertIfNotPresent(collection, key, data)) << " duplicate key: " << key;
}

template <class Collection>
void InsertOrDieNoPrint(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& data) {
  CHECK(InsertIfNotPresent(collection, key, data)) << " duplicate key.";
}

template <class Collection>
typename Collection::value_type::second_type& InsertKeyOrDie(
    Collection* const collection,
    const typename Collection::value_type::first_type& key) {
  typedef typename Collection::value_type value_type;
  std::pair<typename Collection::iterator, bool> res =
      collection->insert(value_type(key, typename value_type::second_type()));
  CHECK(res.second) << " duplicate key: " << key;
  return res.first->second;
}

template <class Collection>
typename Collection::value_type::second_type& LookupOrInsert(
    Collection* const collection, const typename Collection::value_type& vt) {
  return collection->insert(vt).first->second;
}

template <class Collection>
typename Collection::value_type::second_type& LookupOrInsert(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& value) {
  return LookupOrInsert(collection,
                        typename Collection::value_type(key, value));
}

template <typename Sequence, typename Collection>
void AddTokenCounts(
    const Sequence& sequence,
    const typename Collection::value_type::second_type& increment,
    Collection* const count_map) {
  for (typename Sequence::const_iterator it = sequence.begin();
       it != sequence.end(); ++it) {
    typename Collection::value_type::second_type& value = LookupOrInsert(
        count_map, *it, typename Collection::value_type::second_type());
    value += increment;
  }
}

template <class Collection>
typename Collection::value_type::second_type& LookupOrInsertNew(
    Collection* const collection,
    const typename Collection::value_type::first_type& key) {
  typedef typename std::iterator_traits<
      typename Collection::value_type::second_type>::value_type Element;
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(
          key,
          static_cast<typename Collection::value_type::second_type>(nullptr)));
  if (ret.second) {
    ret.first->second = new Element();
  }
  return ret.first->second;
}

template <class Collection, class Arg>
typename Collection::value_type::second_type& LookupOrInsertNew(
    Collection* const collection,
    const typename Collection::value_type::first_type& key, const Arg& arg) {
  typedef typename std::iterator_traits<
      typename Collection::value_type::second_type>::value_type Element;
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(
          key,
          static_cast<typename Collection::value_type::second_type>(nullptr)));
  if (ret.second) {
    ret.first->second = new Element(arg);
  }
  return ret.first->second;
}

template <class Collection>
typename Collection::value_type::second_type::element_type*
LookupOrInsertNewLinkedPtr(
    Collection* const collection,
    const typename Collection::value_type::first_type& key) {
  typedef typename Collection::value_type::second_type Value;
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(key, Value()));
  if (ret.second) {
    ret.first->second.reset(new typename Value::element_type);
  }
  return ret.first->second.get();
}

template <class Collection, class Arg>
typename Collection::value_type::second_type::element_type*
LookupOrInsertNewLinkedPtr(
    Collection* const collection,
    const typename Collection::value_type::first_type& key, const Arg& arg) {
  typedef typename Collection::value_type::second_type Value;
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(key, Value()));
  if (ret.second) {
    ret.first->second.reset(new typename Value::element_type(arg));
  }
  return ret.first->second.get();
}

template <class Collection>
typename Collection::value_type::second_type& LookupOrInsertNewSharedPtr(
    Collection* const collection,
    const typename Collection::value_type::first_type& key) {
  typedef typename Collection::value_type::second_type SharedPtr;
  typedef typename Collection::value_type::second_type::element_type Element;
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(key, SharedPtr()));
  if (ret.second) {
    ret.first->second.reset(new Element());
  }
  return ret.first->second;
}

template <class Collection, class Arg>
typename Collection::value_type::second_type& LookupOrInsertNewSharedPtr(
    Collection* const collection,
    const typename Collection::value_type::first_type& key, const Arg& arg) {
  typedef typename Collection::value_type::second_type SharedPtr;
  typedef typename Collection::value_type::second_type::element_type Element;
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(key, SharedPtr()));
  if (ret.second) {
    ret.first->second.reset(new Element(arg));
  }
  return ret.first->second;
}

template <class Collection>
bool UpdateReturnCopy(Collection* const collection,
                      const typename Collection::value_type::first_type& key,
                      const typename Collection::value_type::second_type& value,
                      typename Collection::value_type::second_type* previous) {
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(key, value));
  if (!ret.second) {
    if (previous) {
      *previous = ret.first->second;
    }
    ret.first->second = value;
    return true;
  }
  return false;
}

template <class Collection>
bool UpdateReturnCopy(Collection* const collection,
                      const typename Collection::value_type& vt,
                      typename Collection::value_type::second_type* previous) {
  std::pair<typename Collection::iterator, bool> ret = collection->insert(vt);
  if (!ret.second) {
    if (previous) {
      *previous = ret.first->second;
    }
    ret.first->second = vt.second;
    return true;
  }
  return false;
}

template <class Collection>
typename Collection::value_type::second_type* const InsertOrReturnExisting(
    Collection* const collection, const typename Collection::value_type& vt) {
  std::pair<typename Collection::iterator, bool> ret = collection->insert(vt);
  if (ret.second) {
    return nullptr;
  } else {
    return &ret.first->second;
  }
}

template <class Collection>
typename Collection::value_type::second_type* const InsertOrReturnExisting(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& data) {
  return InsertOrReturnExisting(collection,
                                typename Collection::value_type(key, data));
}

template <class Collection>
typename Collection::value_type::second_type* const InsertOrReturnNull(
    Collection* const collection, const typename Collection::value_type& vt) {
  std::pair<typename Collection::iterator, bool> ret = collection->insert(vt);
  if (ret.second) {
    return &ret.first->second;
  } else {
    return nullptr;
  }
}

template <class Collection>
typename Collection::value_type::second_type* const InsertOrReturnNull(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& data) {
  return InsertOrReturnNull(collection,
                            typename Collection::value_type(key, data));
}

template <class Collection>
typename Collection::value_type::second_type EraseKeyReturnValuePtr(
    Collection* const collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection->find(key);
  if (it == collection->end()) {
    return nullptr;
  }
  typename Collection::value_type::second_type v = it->second;
  collection->erase(it);
  return v;
}

template <class MapContainer, class KeyContainer>
void InsertKeysFromMap(const MapContainer& map_container,
                       KeyContainer* key_container) {
  CHECK(key_container != nullptr);
  for (typename MapContainer::const_iterator it = map_container.begin();
       it != map_container.end(); ++it) {
    key_container->insert(it->first);
  }
}

template <class MapContainer, class KeyContainer>
void AppendKeysFromMapping(const MapContainer& map_container,
                           KeyContainer* key_container) {
  CHECK(key_container != nullptr);
  for (typename MapContainer::const_iterator it = map_container.begin();
       it != map_container.end(); ++it) {
    key_container->push_back(it->first);
  }
}

template <class MapContainer, class KeyType>
void AppendKeysFromMapping(const MapContainer& map_container,
                           std::vector<KeyType>* key_container) {
  CHECK(key_container != nullptr);

  if (key_container->empty()) {
    key_container->reserve(map_container.size());
  }
  for (typename MapContainer::const_iterator it = map_container.begin();
       it != map_container.end(); ++it) {
    key_container->push_back(it->first);
  }
}

template <class MapContainer, class ValueContainer>
void AppendValuesFromMap(const MapContainer& map_container,
                         ValueContainer* value_container) {
  CHECK(value_container != nullptr);
  for (typename MapContainer::const_iterator it = map_container.begin();
       it != map_container.end(); ++it) {
    value_container->push_back(it->second);
  }
}

template <class MapContainer, class ValueType>
void AppendValuesFromMap(const MapContainer& map_container,
                         std::vector<ValueType>* value_container) {
  CHECK(value_container != nullptr);

  if (value_container->empty()) {
    value_container->reserve(map_container.size());
  }
  for (typename MapContainer::const_iterator it = map_container.begin();
       it != map_container.end(); ++it) {
    value_container->push_back(it->second);
  }
}

template <typename M, typename V>
void MapVToVec(const M& m, V& v) {
  for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
    v.push_back(it->second);
  }
}
}  // namespace e2e_noa

#endif

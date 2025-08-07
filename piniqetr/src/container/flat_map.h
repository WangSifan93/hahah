
#ifndef ST_PLANNING_CONTAINER_FLAT_MAP
#define ST_PLANNING_CONTAINER_FLAT_MAP

#include <functional>

#include "boost/container/flat_map.hpp"

namespace e2e_noa {

template <typename K, typename V, typename Compare = std::less<K>>
using FlatMap = boost::container::flat_map<K, V, Compare>;

}

#endif

#include <boost/numeric/conversion/cast.hpp>
#include <boost/test/unit_test.hpp>

#include "util/multi_level_partition.hpp"

#define CHECK_SIZE_RANGE(range, ref) BOOST_CHECK_EQUAL(range.second - range.first, ref)
#define CHECK_EQUAL_RANGE(range, ref)                                                              \
    do                                                                                             \
    {                                                                                              \
        const auto &lhs = range;                                                                   \
        const auto &rhs = ref;                                                                     \
        BOOST_CHECK_EQUAL_COLLECTIONS(lhs.first, lhs.second, rhs.begin(), rhs.end());              \
    } while (0)

using namespace osrm;
using namespace osrm::util;

BOOST_AUTO_TEST_SUITE(multi_level_partition_tests)

BOOST_AUTO_TEST_CASE(mutable_cell_storage)
{
    // node:                0  1  2  3  4  5  6  7  8  9 10 11
    std::vector<CellID> l1{{0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5}};
    std::vector<CellID> l2{{0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3}};
    std::vector<CellID> l3{{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1}};
    std::vector<CellID> l4{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    PackedMultiLevelPartition mlp{{l1, l2, l3, l4}, {6, 4, 2, 1}};

}

BOOST_AUTO_TEST_SUITE_END()

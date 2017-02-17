#ifndef OSRM_UTIL_MULTI_LEVEL_PARTITION_HPP
#define OSRM_UTIL_MULTI_LEVEL_PARTITION_HPP

#include "util/typedefs.hpp"
#include "util/for_each_pair.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

#include <boost/range/adaptor/reversed.hpp>

namespace osrm
{
namespace util
{
namespace detail
{
// get the msb of an integer
// return 0 for integers without msb
template <typename T> std::size_t highestMSB(T value)
{
    static_assert(std::is_integral<T>::value, "Integer required.");
    std::size_t msb = 0;
    while (value > 0)
    {
        value >>= 1u;
        msb++;
    }
    return msb;
}
}


using LevelID = std::uint8_t;
using CellID = std::uint32_t;

static constexpr CellID INVALID_CELL_ID = std::numeric_limits<CellID>::max();

// Mock interface, can be removed when we have an actual implementation
class MultiLevelPartition
{
  public:
    // Returns the cell id of `node` at `level`
    virtual CellID GetCell(LevelID level, NodeID node) const = 0;

    // Returns the highest level in which `first` and `second` are still in different cells
    virtual LevelID GetHighestDifferentLevel(NodeID first, NodeID second) const = 0;

    // Returns the level at which a `node` is relevant for a query from start to target
    virtual LevelID GetQueryLevel(NodeID start, NodeID target, NodeID node) const = 0;

    virtual std::size_t GetNumberOfLevels() const = 0;

    virtual std::size_t GetNumberOfCells(LevelID level) const = 0;

};

class PackedMultiLevelPartition final : public MultiLevelPartition
{
    using PartitionID = std::uint64_t;
    static const constexpr std::size_t NUM_PARTITION_BITS = sizeof(PartitionID) * 8;

  public:
    // cell_sizes is index by level (starting at 0, the base graph).
    // However level 0 always needs to have cell size 1, since it is the
    // basegraph.
    PackedMultiLevelPartition(const std::vector<std::vector<CellID>> &partitions,
                              const std::vector<std::size_t> &level_to_num_cells)
        : level_offsets(makeLevelOffsets(level_to_num_cells)), level_masks(makeLevelMasks()),
          bit_to_level(makeBitToLevel())
    {
        initializePartitionIDs(partitions);
    }

    // returns the index of the cell the vertex is contained at level l
    CellID GetCell(LevelID l, NodeID node) const final override
    {
        auto p = partition[node];
        auto idx = LevelIDToIndex(l);
        auto masked = p & level_masks[idx];
        return masked >> level_offsets[idx];
    }

    LevelID GetQueryLevel(NodeID start, NodeID target, NodeID node) const final override
    {
        return std::min(GetHighestDifferentLevel(start, node),
                        GetHighestDifferentLevel(target, node));
    }

    LevelID GetHighestDifferentLevel(NodeID first, NodeID second) const final override
    {
        if (partition[first] == partition[second])
            return 0;

        auto msb = detail::highestMSB(partition[first] ^ partition[second]);
        return bit_to_level[msb];
    }

    std::size_t GetNumberOfLevels() const
    {
        return level_offsets.size();
    }

    std::size_t GetNumberOfCells(LevelID level) const
    {
        auto max_id = GetCell(level, GetSenitileNode());
    }

  private:
    inline std::size_t LevelIDToIndex(LevelID l) const { return l - 1; }

    // We save the senitile as last node in the partition information.
    // It has the highest cell id in each level so we can derived the range
    // of cell ids efficiently.
    inline NodeID GetSenitileNode() const { return partition.size() - 1; }

    void SetCellID(LevelID l, NodeID node, std::size_t cell_id)
    {
        auto lidx = LevelIDToIndex(l);

        auto shifted_id = cell_id << level_offsets[lidx];
        auto cleared_cell = partition[node] & ~level_masks[lidx];
        partition[node] = cleared_cell | shifted_id;
    }

    // If we have N cells per level we need log_2 bits for every cell ID
    std::vector<std::size_t>
    makeLevelOffsets(const std::vector<std::size_t> &level_to_num_cells) const
    {
        std::vector<std::size_t> offsets;
        offsets.reserve(level_to_num_cells.size());

        auto sum_bits = 0;
        for (auto num_cells : level_to_num_cells)
        {
            // bits needed to number all contained vertexes
            auto bits = static_cast<std::uint64_t>(std::ceil(std::log2(num_cells)));
            offsets.push_back(sum_bits);
            sum_bits += bits;
            BOOST_ASSERT(sum_bits < 64);
        }
        // senitile
        offsets.push_back(sum_bits);

        return offsets;
    }

    std::vector<PartitionID> makeLevelMasks() const
    {
        std::vector<PartitionID> masks;
        masks.reserve(level_offsets.size());

        util::for_each_pair(level_offsets.begin(), level_offsets.end(), [&](const auto offset, const auto next_offset) {
            // create mask that has `bits` ones at its LSBs.
            // 000011
            BOOST_ASSERT(offset < sizeof(PartitionID)*8);
            PartitionID mask = (1UL << offset) - 1UL;
            // 001111
            BOOST_ASSERT(next_offset < sizeof(PartitionID)*8);
            PartitionID next_mask = (1UL << next_offset) - 1UL;
            // 001100
            masks.push_back(next_mask ^ mask);
        });

        return masks;
    }

    std::array<LevelID, NUM_PARTITION_BITS> makeBitToLevel() const
    {
        std::array<LevelID, NUM_PARTITION_BITS> bit_to_level;

        LevelID l = 0;
        for (auto bits : level_offsets)
        {
            // set all bits to point to the correct level.
            for (auto idx = bits; idx < NUM_PARTITION_BITS; ++idx)
            {
                bit_to_level[idx] = l;
            }
            l++;
        }

        return bit_to_level;
    }

    void initializePartitionIDs(const std::vector<std::vector<CellID>> &partitions)
    {
        auto num_nodes = partitions.front().size();
        std::vector<NodeID> permutation(num_nodes);
        std::iota(permutation.begin(), permutation.end(), 0);
        // We include a senitile element at the end of the partition
        partition.resize(num_nodes + 1);

        // Sort nodes bottum-up by cell id.
        // This ensures that we get a nice grouping from parent to child cells:
        //
        // intitial:
        // level 0: 0 1 2 3 4 5
        // level 1: 2 1 3 4 3 4
        // level 2: 2 2 0 1 0 1
        //
        // first round:
        // level 0: 1 0 2 4 3 5
        // level 1: 1 2 3 3 4 4 (< sorted)
        // level 2: 2 2 0 0 1 1
        //
        // second round:
        // level 0: 2 4 3 5 1 0
        // level 1: 3 3 4 4 1 2
        // level 2: 0 0 1 1 2 2 (< sorted)
        for (const auto &partition : partitions)
        {
            std::stable_sort(permutation.begin(),
                             permutation.end(),
                             [partition](const auto lhs, const auto rhs) {
                                 return partition[lhs] < partition[rhs];
                             });
        }

        // top down assign new cell ids
        LevelID level = partitions.size() + 1;
        for (const auto &partition : boost::adaptors::reverse(partitions))
        {
            BOOST_ASSERT(permutation.size() > 0);
            CellID last_cell_id = partition[permutation.front()];
            CellID cell_id = 0;
            for (const auto node : permutation)
            {
                if (last_cell_id != partition[node])
                {
                    cell_id++;
                }
                SetCellID(level, node, cell_id);
            }
            level--;
        }
    }

    std::vector<PartitionID> partition;
    std::vector<std::size_t> level_offsets;
    std::vector<PartitionID> level_masks;
    std::array<LevelID, NUM_PARTITION_BITS> bit_to_level;
};
}
}

#endif

#pragma once

#include <longeron/containers/intarray_multimap.hpp>
#include <longeron/id_management/registry.hpp>

#include <Magnum/Math/Vector3.h>
#include <Magnum/Magnum.h>

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayViewStl.h>

#include <map>

namespace testapp::redstone
{

using Magnum::Vector3i;
using Magnum::Vector3;


/* Stuff */

using HierBitset_t = lgrn::HierarchicalBitset<uint64_t>;

// probably better to generalize this into parallel vectors
template <typename KEY_T, typename VALUE_T>
class SmallMap
{
    using VecKey_t = std::vector<KEY_T>;
    using VecValue_t = std::vector<VALUE_T>;
public:

    VALUE_T operator[](KEY_T const& key)
    {
        std::size_t idx = find(key);
        if (idx == m_keys.size())
        {
            m_keys.emplace_back(key);
            return m_values.emplace_back();
        }
        return m_values[idx];
    }

    std::size_t find_assure(KEY_T const& key)
    {
        std::size_t idx = find(key);
        if (idx == m_keys.size())
        {
            m_keys.emplace_back(key);
            m_values.emplace_back();
        }
        return idx;
    }

    std::size_t find(KEY_T const& key) const
    {
        return std::distance(m_keys.begin(), std::find_if(m_keys.begin(), m_keys.end(), [&key] (KEY_T const& lhs) { return lhs == key; } ));
    }

    void reserve(std::size_t n)
    {
        m_keys.reserve(n);
        m_values.reserve(n);
    }

    std::size_t size() const { return m_keys.size(); }
    KEY_T& key(std::size_t i) noexcept { return m_keys[i]; }
    VALUE_T& value(std::size_t i) noexcept { return m_values[i]; }
    KEY_T const& key(std::size_t i) const noexcept { return m_keys[i]; }
    VALUE_T const& value(std::size_t i) const noexcept { return m_values[i]; }

private:
    VecKey_t m_keys;
    VecValue_t m_values;
};

template <typename CMP_T, typename FUNC_T, typename PAIR_IT_T>
void funnel_each(PAIR_IT_T first, PAIR_IT_T last, CMP_T&& cmp, FUNC_T&& func) noexcept
{
    if (first == last)
    {
        return;
    }

    // initially sort first..last
    std::sort(first, last, cmp);

    while (true)
    {
        if (first->first != first->second) [[likely]]
        {
            // still has value

            func(first->first);
            ++(first->first);

            // swap towards end until first..last is sorted
            PAIR_IT_T lhs = first;
            PAIR_IT_T rhs = std::next(first);
            while (rhs != last && !cmp(*lhs, *rhs))
            {
                std::iter_swap(lhs, rhs);
                ++lhs;
                ++rhs;
            }
        }
        else [[unlikely]]
        {
            // this pair is done iterating, forget about it
            ++first;
            if (first == last)
            {
                break;
            }
        }
    }
}

//-----------------------------------------------------------------------------

/* Types and fundementals */

enum class BlkTypeId : uint8_t { };       // IDs for each different block type
enum class ChkBlkId : uint16_t { };       // IDs for blocks within a chunk
enum class ChunkId : uint16_t { };          // IDs for currently loaded chunks

constexpr Vector3i const gc_chunkDim{16, 16, 16};
constexpr Vector3i::Type const gc_chunkSize
        = gc_chunkDim.x() * gc_chunkDim.y() * gc_chunkDim.z();

template <typename VEC_T>
constexpr typename VEC_T::Type pos_index(VEC_T chunkSize, VEC_T pos) noexcept
{
    return   pos.x()
           + pos.z() * chunkSize.x()
           + pos.y() * chunkSize.x() * chunkSize.z();
}

constexpr Vector3i index_to_pos(int index) noexcept
{
    int const y     = index / (gc_chunkDim.x() * gc_chunkDim.z());
    int const yrem  = index % (gc_chunkDim.x() * gc_chunkDim.z());
    int const z     = yrem / gc_chunkDim.x();
    int const x     = yrem % gc_chunkDim.x();
    return {x, y, z};
}

constexpr ChkBlkId chunk_block_id(Vector3i pos) noexcept
{
    return ChkBlkId(pos_index(gc_chunkDim, pos));
}

using ChunkCoord_t = std::array<int, 3>;

//-----------------------------------------------------------------------------

/* Sides and Directions */

enum class ESide : uint8_t
{
    POS_X = 0,
    NEG_X = 1,
    POS_Y = 2,
    NEG_Y = 3,
    POS_Z = 4,
    NEG_Z = 5,
    ERROR = 6
};

inline constexpr std::array<Vector3i, 6> gc_sideToVec =
{{
    { 1,  0,  0},
    {-1,  0,  0},
    { 0,  1,  0},
    { 0, -1,  0},
    { 0,  0,  1},
    { 0,  0, -1}
}};

constexpr Vector3i side_to_vec(ESide side) noexcept
{
    return gc_sideToVec[uint8_t(side)];
}

constexpr ESide vec_to_side(Vector3i const vec) noexcept
{
    // convert vector3 to some kind of unique int that can be LUTed
    int const signiture = vec.x() * 1 + vec.y() * 2 + vec.z() * 4 + 7;

    std::array<ESide, 15> const sc_table =
    {
        ESide::ERROR, ESide::ERROR, ESide::ERROR, ESide::NEG_Z, ESide::ERROR,
        ESide::NEG_Y, ESide::NEG_X, ESide::ERROR, ESide::POS_X, ESide::POS_Y,
        ESide::ERROR, ESide::POS_Z, ESide::ERROR, ESide::ERROR, ESide::ERROR
    };

    return sc_table[signiture];
}

enum class EMultiDir : uint8_t
{
    POS_X = 1 << 0,
    NEG_X = 1 << 1,
    POS_Y = 1 << 2,
    NEG_Y = 1 << 3,
    POS_Z = 1 << 4,
    NEG_Z = 1 << 5
};

using EMultiDirs = Corrade::Containers::EnumSet<EMultiDir>;
CORRADE_ENUMSET_OPERATORS(EMultiDirs)

constexpr int clamp_overflow(int in, int bounds, int& rOverflows) noexcept
{
    rOverflows = in / bounds - (in < 0);
    return in - rOverflows * bounds;
}

struct DiffBlk
{
    Vector3i m_pos;
    Vector3i m_chunkDelta;
};

inline DiffBlk inter_chunk_pos(Vector3i const pos)
{
    Vector3i chunkDelta{0};
    Vector3i const posClamped{
        clamp_overflow(pos.x(), gc_chunkDim.x(), chunkDelta.x()),
        clamp_overflow(pos.y(), gc_chunkDim.y(), chunkDelta.y()),
        clamp_overflow(pos.z(), gc_chunkDim.z(), chunkDelta.z())
    };
    return {posClamped, chunkDelta};
}

//-----------------------------------------------------------------------------

/* Block place commands */

/**
 * @brief Command to place a block in a chunk at a certain position and direction
 */
struct ChkBlkPlace
{
    Vector3i m_pos;
    Vector3 m_lookDir;
};

using ChkBlkPlacements_t = std::unordered_map< BlkTypeId, std::vector<ChkBlkPlace> >;

//-----------------------------------------------------------------------------

/* Block change updates */

/**
 * @brief Dirty flags for added and removed blocks within a chunk
 */
struct ChkBlkChanges
{
    HierBitset_t m_added;
    HierBitset_t m_removed;
};

struct TmpChkBlkTypeUpd
{
    std::unordered_map< BlkTypeId, ChkBlkChanges > m_perType;
    HierBitset_t m_changed;
};

ChkBlkChanges& assure_chunk_update(TmpChkBlkTypeUpd &rBlkTypeUpd, BlkTypeId blkTypeId);

//-----------------------------------------------------------------------------

/* Connection updates and subscribers */


/**
 * @brief Blocks within a chunk marked for connection update
 */
struct ChkBlkConnect
{
    ChkBlkConnect() = default;
    ChkBlkConnect(ChkBlkConnect const& copy) = delete;
    ChkBlkConnect(ChkBlkConnect&& move)
     : m_connect{std::move(move.m_connect)}
    {};

    HierBitset_t m_connect;
    std::atomic_flag m_lock = ATOMIC_FLAG_INIT;
};

using ChkBlkTypeConnect_t = std::unordered_map<BlkTypeId, ChkBlkConnect>;

/**
 * @brief Updates publishers of a chunk
 */
struct TmpChkUpdPublish
{
    struct PublishTo
    {
        ChkBlkId m_pubBlk;
        ChkBlkId m_subBlk;
        ChunkId m_subChunk;
    };

    using Vec_t = std::vector<PublishTo>;

    //Vector3i m_publisherCoords;
    Vec_t m_changes;
};

using TmpUpdPublish_t = SmallMap<Vector3i, TmpChkUpdPublish>;

struct TmpChkUpdSubscribe
{
    struct SubscribeToItn
    {
        ChkBlkId m_subBlk;
        ChkBlkId m_pubBlk;
    };

    struct SubscribeToExt
    {
        ChkBlkId m_subBlk;
        ChkBlkId m_pubBlk;
        ChunkId m_pubChunk;
    };

    using VecItn_t = std::vector<SubscribeToItn>;
    using VecExt_t = std::vector<SubscribeToExt>;

    //Vector3i m_publisherCoords;
    VecItn_t m_changesItn;
    VecExt_t m_changesExt;
};

using TmpUpdSubscribe_t = std::vector<TmpChkUpdSubscribe>;


/**
 * @brief Assigned to a block if another block is subscribed to it
 */
struct BlkConnect
{
    ChunkId m_chunk;
    ChkBlkId m_block;
};

struct ChkConnect
{
    using MultiMap_t = lgrn::IntArrayMultiMap<uint32_t, BlkConnect>;

    MultiMap_t      m_blkPublish;
    MultiMap_t      m_blkSubscribe;
};

struct TmpChkNotify
{
    struct NotifyPair
    {
        BlkTypeId m_blkTypeId;
        uint8_t m_slot;
    };

    // Slots are associated with a set of block IDs to notify
    std::array<HierBitset_t, 1> m_notifySlots;
    std::vector<NotifyPair> m_notifyTypes;
};

// make one of these for each chunk-updating thread
struct TmpExtNotify
{
    // [chunkId][blockID]
    std::vector<HierBitset_t> m_chkNotify;
};

//-----------------------------------------------------------------------------

/* Common Loaded Chunk data */

using Corrade::Containers::ArrayView;
using Corrade::Containers::Array;

struct ACtxVxLoadedChunks
{
    lgrn::IdRegistry<ChunkId> m_ids;
    HierBitset_t m_dirty;
    std::map< ChunkCoord_t, ChunkId > m_coordToChunk;
    std::vector<ChunkCoord_t> m_chunkToCoord;

    // [ChunkId][ChkBlkId]
    std::vector< Array<BlkTypeId> > m_blkTypes;
    std::vector< Array<Magnum::BoolVector3> > m_blkDirection;

    std::vector<ChkConnect> m_connect;
    //std::vector< std::vector<bool> > m_blkSensitive;

    // [ChunkId][BlkTypeId].m_something[ChkBlkId]
    //std::vector<TmpChkBlkTypeUpd_t> m_blkTypeChanges;
    //std::vector<ChkBlkTypeConnect_t> m_blkTypeConnect;

    //std::vector<ChkBlkSubscribers> m_blkSubscribers;
};

void chunk_assign_block_ids(
        BlkTypeId newBlkTypeId,
        std::vector<ChkBlkPlace> const& rPlace,
        ArrayView<BlkTypeId> blkTypeIds,
        TmpChkBlkTypeUpd& rBlkTypeUpd);

/**
 * @brief chunk_notify_subscribers
 *
 * @param rConnect [in]
 * @param changes [in]
 * @param rChkNotify [out]
 * @param rExtNotify [out]
 */
void chunk_notify_subscribers(ChkConnect const& rConnect, HierBitset_t const& changes, Array<BlkTypeId> const& blkTypes, TmpChkNotify& rChkNotify, TmpExtNotify& rExtNotify);

} // namespace testapp::redstone

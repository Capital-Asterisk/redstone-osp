#include "redstone_osp.h"

#include <osp/logging.h>

namespace testapp::redstone
{

/**
 * @brief Main block-related update function
 *
 * @param rScene
 * @param blkChanges
 */
void update_blocks(RedstoneScene& rScene, ArrayView< std::pair<ChunkId, ChkBlkPlacements_t> > blkChanges)
{
    // clear previous frame's block type updates
    for (TmpChkBlkTypeUpd &rBlkTypeUpd : rScene.m_blkTypeUpdates)
    {
        rBlkTypeUpd.m_changed.reset();
        rBlkTypeUpd.m_perType.clear();
    }

    // Update block IDs
    // * Modifies each chunk's BlkTypeId buffers
    // * Writes modified chunks
    // * Writes per-blocktype added/removed changes: rScene.m_blkTypeUpdates
    world_update_block_ids(rScene, blkChanges);

    // clear notify slots
    for (TmpChkNotify &rNotify : rScene.m_chkNotify)
    {
        rNotify.m_notifyTypes.clear();

        // clear bitsets
        for (HierBitset_t &rBitset : rNotify.m_notifySlots)
        {
            rBitset.reset();
        }
    }

    // Notify subscribers
    for (std::size_t chkIdI = 0; chkIdI < rScene.m_blkTypeUpdates.size(); ++ chkIdI)
    {
        if (rScene.m_blkTypeUpdates[chkIdI].m_changed.count() != 0)
        {
            TmpExtNotify dummy; // not yet implemented
            chunk_notify_subscribers(
                        rScene.m_chunks.m_connect[chkIdI],
                                     rScene.m_blkTypeUpdates[chkIdI].m_changed, rScene.m_chunks.m_blkTypes[chkIdI], rScene.m_chkNotify[chkIdI], dummy);
        }
    }

    // Log blocks to notify
    for (TmpChkNotify const &rNotify : rScene.m_chkNotify)
    {
        for (HierBitset_t const &rBitset : rNotify.m_notifySlots)
        {
            if (rBitset.size() == 0)
            {
                continue;
            }
            for (int const blkSubscriber : rBitset)
            {
                OSP_LOG_TRACE("Notify: {}", blkSubscriber);
            }
        }
    }

    // Accumolate subscription changes
    TmpUpdPublish_t updPub;
    TmpUpdSubscribe_t updSub(rScene.m_chunks.m_ids.capacity());

    for (std::size_t chunkId : rScene.m_chunks.m_dirty)
    {
        TmpChkBlkTypeUpd const& blkTypeUpd = rScene.m_blkTypeUpdates[chunkId];
        if (auto it = blkTypeUpd.m_perType.find(g_blkTypeIds.id_of("dust"));
            it != blkTypeUpd.m_perType.end())
        {
            chunk_subscribe_dusts(it->second, ChunkId(chunkId), index_to_pos(chunkId), updPub, updSub[chunkId]);
        }
    }

    // TODO: allocate chunks allowed to subscribe to not-yet-loaded chunks

    // Apply publish changes
    // 1 element for each chunk changed
    for (std::size_t i = 0; i < updPub.size(); ++i)
    {
        Vector3i const pos = updPub.key(i);
        TmpChkUpdPublish &rChkUpdConnect = updPub.value(i);
        auto found = rScene.m_chunks.m_coordToChunk.find(ChunkCoord_t{pos.x(), pos.y(), pos.z()});

        if (found == rScene.m_chunks.m_coordToChunk.end())
        {
            continue; // just skip non existent chunks for now
        }

        using PublishTo = TmpChkUpdPublish::PublishTo;
        std::sort(rChkUpdConnect.m_changes.begin(), rChkUpdConnect.m_changes.end(),
                  [] (PublishTo const& lhs, PublishTo const& rhs)
        {
            return lhs.m_pubBlk < rhs.m_pubBlk;
        });

        ChkConnect::MultiMap_t &rBlkPublish = rScene.m_chunks.m_connect.at(std::size_t(found->second)).m_blkPublish;

        // Intention is to have a thread for each chunk do this.
        // Multiple TmpChkUpdPublish can be passed to this thread and
        // iterated all at once
        using it_t = TmpChkUpdPublish::Vec_t::iterator;
        using pair_t = std::pair<it_t, it_t>;
        auto its = std::array
        {
                pair_t(rChkUpdConnect.m_changes.begin(),
                       rChkUpdConnect.m_changes.end())
        };
        std::size_t const totalChanges = rChkUpdConnect.m_changes.size();

        if (std::size_t const sizeReq
                = lgrn::div_ceil(rBlkPublish.data_size() + totalChanges, gc_chunkSize) * gc_chunkSize;
            rBlkPublish.data_capacity() < sizeReq)
        {
            rBlkPublish.data_reserve(sizeReq);
        }

        std::vector<BlkConnect> tmpConnect;
        ChkBlkId currBlk = lgrn::id_null<ChkBlkId>();

        auto flush = [&tmpConnect, &currBlk, &rBlkPublish] () -> void
        {
            if (tmpConnect.empty()) [[unlikely]]
            {
                return;
            }

            uint16_t const currBlkI = uint16_t(currBlk);

            if (rBlkPublish.contains(currBlkI))
            {
                // erase existing subscribers
                auto span = rBlkPublish[currBlkI];
                tmpConnect.insert(tmpConnect.end(), span.begin(), span.end());
                rBlkPublish.erase(currBlkI);
            }

            rBlkPublish.emplace(currBlkI, tmpConnect.begin(), tmpConnect.end());

            OSP_LOG_TRACE("Block {} publishes to:", currBlk);

            for (auto const fish : tmpConnect)
            {
                OSP_LOG_TRACE("* {}", fish.m_block);
            }
            tmpConnect.clear();

        };

        funnel_each(its.begin(), its.end(),
                    [] (pair_t const& lhs, pair_t const& rhs) -> bool
        {
            return lhs.first->m_pubBlk < rhs.first->m_pubBlk;
        },
                    [&tmpConnect, &currBlk, flush] (it_t pubIt)
        {
            if (currBlk != pubIt->m_pubBlk)
            {
                flush();
                currBlk = pubIt->m_pubBlk;
            }

            if (currBlk == pubIt->m_pubBlk)
            {
                tmpConnect.emplace_back(BlkConnect{pubIt->m_subChunk, pubIt->m_subBlk});
            }
        });

        flush();

        rBlkPublish.pack();
    }
}

//-----------------------------------------------------------------------------

void chunk_subscribe_dusts(ChkBlkChanges const& dustBlkUpd, ChunkId chunkId, Vector3i chunkPos, TmpUpdPublish_t& rUpdPublish, TmpChkUpdSubscribe& rUpdChkSubscribe)
{
    if (dustBlkUpd.m_added.count() == 0)
    {
        return;
    }

    using PublishTo         = TmpChkUpdPublish::PublishTo;
    using SubscribeToItn    = TmpChkUpdSubscribe::SubscribeToItn;
    using SubscribeToExt    = TmpChkUpdSubscribe::SubscribeToExt;

    std::array<Vector3i, 12> sc_subToOffsets
    {{
       { 1, -1,  0}, { 1,  0,  0}, { 1,  1,  0},
       {-1, -1,  0}, {-1,  0,  0}, {-1,  1,  0},
       { 0, -1,  1}, { 0,  0,  1}, { 0,  1,  1},
       { 0, -1, -1}, { 0,  0, -1}, { 0,  1, -1}
    }};

    std::size_t const ownChunkPubIdx = rUpdPublish.find_assure(chunkPos);

    for (std::size_t subBlkId : dustBlkUpd.m_added)
    {
        Vector3i const subBlkPos = index_to_pos(subBlkId);
        for (Vector3i const offset : sc_subToOffsets)
        {
            DiffBlk const diff = inter_chunk_pos(subBlkPos + offset);
            ChkBlkId const pubBlkId = chunk_block_id(diff.m_pos);
            PublishTo const pub{pubBlkId, ChkBlkId(subBlkId), chunkId};
            if (diff.m_chunkDelta.isZero())
            {
                // subscribe to own chunk (common case)
                rUpdPublish.value(ownChunkPubIdx).m_changes.emplace_back(pub);
                rUpdChkSubscribe.m_changesItn.emplace_back(SubscribeToItn{ChkBlkId(subBlkId), pubBlkId});
            }
            else
            {
                // subscribe to neighbouring chunk
                Vector3i const neighbourPos = chunkPos + diff.m_chunkDelta;
                rUpdPublish[neighbourPos].m_changes.emplace_back(pub);
            }
        }
    }
}

void chunk_connect_dusts(
        BlkTypeId type,
        ChunkId chkId,
        HierBitset_t const& notify,
        ACtxVxLoadedChunks const& chunks,
        ACtxVxRsDusts const& dusts,
        ACtxVxRsDusts::ChkDust_t &rChkDust)
{

}

//-----------------------------------------------------------------------------

void world_update_block_ids(
        RedstoneScene& rScene,
        ArrayView< std::pair<ChunkId, ChkBlkPlacements_t> > chunkUpd)
{
    using namespace osp::active;

    for (auto const& [chunkId, blkPlace] : chunkUpd)
    {
        // Clear block added and removed queues
        for (auto & [blkTypeId, blkTypeUpd]
             : rScene.m_blkTypeUpdates.at(size_t(chunkId)).m_perType)
        {
            blkTypeUpd.m_added.reset();
            blkTypeUpd.m_removed.reset();
        }

        // Update all block IDs and write type changes
        for (auto const& [newBlkTypeId, rPlace] : blkPlace)
        {
            chunk_assign_block_ids(
                    newBlkTypeId, rPlace,
                    rScene.m_chunks.m_blkTypes.at(size_t(chunkId)),
                    rScene.m_blkTypeUpdates.at(size_t(chunkId)));
        }

        HierBitset_t& rNotify = rScene.m_chkNotify.at(size_t(chunkId)).m_notifySlots.at(0);
        rNotify.reset(); // Clear notify queues too

        // just set all chunks dirty for now lol
        rScene.m_chunks.m_dirty.set(size_t(chunkId));
    }
}

} // namespace testapp::redstone


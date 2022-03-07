#include "redstone_osp.h"

namespace testapp::redstone
{


void chunk_connect_dusts(ChkBlkChanges const& dustBlkUpd, ChunkId chunkId, Vector3i chunkPos, TmpUpdPublish_t& rUpdPublish, TmpChkUpdSubscribe& rUpdChkSubscribe)
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
             : rScene.m_blkTypeUpdates[size_t(chunkId)])
        {
            blkTypeUpd.m_added.reset();
            blkTypeUpd.m_removed.reset();
        }

        // Update all block IDs and write type changes
        chunk_assign_block_ids(
                rScene.m_chunks.m_blkTypes[size_t(chunkId)],
                rScene.m_blkTypeUpdates[size_t(chunkId)],
                blkPlace);

        // just set all chunks dirty for now lol
        rScene.m_chunks.m_dirty.set(size_t(chunkId));
    }
}

} // namespace testapp::redstone


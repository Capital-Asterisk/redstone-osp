#include "voxels.h"

#include <osp/logging.h>

namespace testapp::redstone
{

ChkBlkChanges& assure_chunk_update(TmpChkBlkTypeUpd_t &rBlkTypeUpd, BlkTypeId blkTypeId)
{
    auto const& [it, success] = rBlkTypeUpd.try_emplace(blkTypeId);
    ChkBlkChanges &rChkUpd = it->second;

    if (success)
    {
        rChkUpd.m_added.resize(gc_chunkSize);
        rChkUpd.m_removed.resize(gc_chunkSize);
    }

    return rChkUpd;
}

void chunk_assign_block_ids(
        ArrayView<BlkTypeId> chkBlkTypeIds,
        TmpChkBlkTypeUpd_t& rBlkTypeUpd,
        ChkBlkPlacements_t const& blkPlacements)
{
    // Update Block IDs
    for (auto const& [newBlkTypeId, rPlace] : blkPlacements)
    {
        for (ChkBlkPlace const& place : rPlace)
        {
            assert(place.m_pos.x() < gc_chunkDim.x());
            assert(place.m_pos.y() < gc_chunkDim.y());
            assert(place.m_pos.z() < gc_chunkDim.z());

            ChkBlkId const block = chunk_block_id(place.m_pos);

            OSP_LOG_INFO("block ID: {}", size_t(block));

            BlkTypeId &rCurrBlkTypeId = chkBlkTypeIds[size_t(block)];

            // Block type that was currently there got removed
            assure_chunk_update(rBlkTypeUpd, rCurrBlkTypeId).m_removed.set(size_t(block));
            // New block type got added
            assure_chunk_update(rBlkTypeUpd, newBlkTypeId).m_added.set(size_t(block));

            // assign new block type
            rCurrBlkTypeId = newBlkTypeId;
        }
    }
}

} // namespace testapp::redstone

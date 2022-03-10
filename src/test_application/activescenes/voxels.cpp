#include "voxels.h"

#include <osp/logging.h>

namespace testapp::redstone
{

ChkBlkChanges& assure_chunk_update(TmpChkBlkTypeUpd &rBlkTypeUpd, BlkTypeId blkTypeId)
{
    auto const& [it, success] = rBlkTypeUpd.m_perType.try_emplace(blkTypeId);
    ChkBlkChanges &rChkUpd = it->second;

    if (success)
    {
        rChkUpd.m_added.resize(gc_chunkSize);
        rChkUpd.m_removed.resize(gc_chunkSize);
    }

    return rChkUpd;
}

void chunk_assign_block_ids(
        BlkTypeId newBlkTypeId,
        std::vector<ChkBlkPlace> const& rPlace,
        ArrayView<BlkTypeId> blkTypeIds,
        TmpChkBlkTypeUpd& rBlkTypeUpd)
{
    // Update Block IDs
    for (ChkBlkPlace const& place : rPlace)
    {
        assert(place.m_pos.x() < gc_chunkDim.x());
        assert(place.m_pos.y() < gc_chunkDim.y());
        assert(place.m_pos.z() < gc_chunkDim.z());

        ChkBlkId const blkId = chunk_block_id(place.m_pos);
        auto const blkIdI = std::size_t(blkId);

        OSP_LOG_INFO("block ID: {}", blkIdI);

        BlkTypeId &rCurrBlkTypeId = blkTypeIds[blkIdI];

        // Block type that was currently there got removed
        assure_chunk_update(rBlkTypeUpd, rCurrBlkTypeId).m_removed.set(blkIdI);
        // New block type got added
        assure_chunk_update(rBlkTypeUpd, newBlkTypeId).m_added.set(blkIdI);

        // Notify any block change
        rBlkTypeUpd.m_changed.set(blkIdI);

        // assign new block type
        rCurrBlkTypeId = newBlkTypeId;
    }
}

void chunk_notify_subscribers(
        ChkConnect const& rConnect,
        HierBitset_t const& changes,
        Array<BlkTypeId> const& blkTypes,
        TmpChkNotify& rChkNotify,
        TmpExtNotify& rExtNotify)
{
    using NotifyPair = TmpChkNotify::NotifyPair;

    for (int blkIdI : changes)
    {
        if (auto span = rConnect.m_blkPublish[blkIdI];
            span.begin() != nullptr)
        {
            for (BlkConnect const& publish : span)
            {
                int const blkIdISubscriber = int(publish.m_block);
                BlkTypeId const type = blkTypes[blkIdISubscriber];

                auto itBegin = rChkNotify.m_notifyTypes.begin();
                auto itEnd = rChkNotify.m_notifyTypes.end();
                auto found = std::find_if(
                        itBegin, itEnd, [type] (NotifyPair const pair)
                {
                    return pair.m_blkTypeId == type;
                });

                uint8_t slot;

                if (found != itEnd)
                {
                    slot = found->m_slot;
                }
                else
                {
                    // TODO: find bitset with smallest count()
                    slot = 0;
                    rChkNotify.m_notifyTypes.emplace_back(NotifyPair{type, slot});
                }

                rChkNotify.m_notifySlots[slot].set(blkIdISubscriber);

                //OSP_LOG_INFO("AAAAAAAAAA {}", connect.m_block);
            }
        }
    }
}

} // namespace testapp::redstone

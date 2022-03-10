#pragma once

#include "voxels.h"

#include <osp/Active/activetypes.h>

#include <osp/id_set.h>

#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/ImageData.h>

namespace testapp::redstone
{


// Block Rendering (just make an entity for each visible block lol)

struct ChkEntModels
{
    std::vector< Array<osp::active::ActiveEnt> > m_chunkEnts;
};

//-----------------------------------------------------------------------------

// Dust

struct BlkRsDust
{
    EMultiDirs m_connected{EMultiDir::NEG_X | EMultiDir::POS_X};
    EMultiDirs m_rises;
    //ElemLocIdSto_t m_elem;
};

struct ACtxVxRsDusts
{
    using ChkDust_t = Array<BlkRsDust>;
    // m_dusts[ChunkId][VxSubChunkId][VxSubChkBlkId]
    std::vector<ChkDust_t> m_dusts;
};


void chunk_subscribe_dusts(ChkBlkChanges const& dustBlkUpd, ChunkId chunkId, Vector3i chunkPos, TmpUpdPublish_t& rUpdPublish, TmpChkUpdSubscribe& rUpdChkSubscribe);

void chunk_connect_dusts(BlkTypeId type, ChunkId chkId, HierBitset_t const& notify, ACtxVxLoadedChunks const& chunks, ACtxVxRsDusts const& dusts, ACtxVxRsDusts::ChkDust_t &rChkDust);

//-----------------------------------------------------------------------------

inline osp::IdSet<std::string_view, BlkTypeId> g_blkTypeIds;

struct RedstoneScene
{
    ACtxVxLoadedChunks m_chunks;
    ACtxVxRsDusts m_chunkDusts;
    ChunkId m_singleChunk;

    // temporary states
    std::vector<TmpChkBlkTypeUpd> m_blkTypeUpdates;
    std::vector<TmpChkNotify> m_chkNotify;
};

void update_blocks(RedstoneScene& rScene, ArrayView< std::pair<ChunkId, ChkBlkPlacements_t> > blkChanges);

void world_update_block_ids(
        RedstoneScene& rScene,
        ArrayView< std::pair<ChunkId, ChkBlkPlacements_t> > chunkUpd);

} // namespace testapp::redstone

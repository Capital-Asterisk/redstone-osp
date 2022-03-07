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
    EMultiDir m_connected;
    EMultiDir m_rises;
    //ElemLocIdSto_t m_elem;
};

struct ACtxVxRsDusts
{
    using ChunkDust_t = Array<BlkRsDust>;
    // m_dusts[ChunkId][VxSubChunkId][VxSubChkBlkId]
    std::vector<ChunkDust_t> m_dusts;
};


void chunk_connect_dusts(ChkBlkChanges const& dustBlkUpd, ChunkId chunkId, Vector3i chunkPos, TmpUpdPublish_t& rUpdPublish, TmpChkUpdSubscribe& rUpdChkSubscribe);

//-----------------------------------------------------------------------------

inline osp::IdSet<std::string_view, BlkTypeId> g_blkTypeIds;

struct RedstoneScene
{
    ACtxVxLoadedChunks m_chunks;
    ACtxVxRsDusts m_chunkDusts;
    ChunkId m_singleChunk;

    // temporary states
    std::vector<TmpChkBlkTypeUpd_t> m_blkTypeUpdates;
};

void world_update_block_ids(
        RedstoneScene& rScene,
        ArrayView< std::pair<ChunkId, ChkBlkPlacements_t> > chunkUpd);

} // namespace testapp::redstone

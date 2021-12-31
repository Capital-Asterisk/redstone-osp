/**
 * Open Space Program
 * Copyright © 2019-2021 Open Space Program Project
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "scenarios.h"
#include "CameraController.h"

#include "../ActiveApplication.h"

#include <osp/Active/basic.h>
#include <osp/Active/drawing.h>

#include <osp/Active/SysHierarchy.h>

#include <osp/Active/SysRender.h>
#include <osp/Active/opengl/SysRenderGL.h>

#include <osp/Shaders/Flat.h>
#include <osp/Shaders/Phong.h>
#include <osp/Shaders/MeshVisualizer.h>

#include <osp/id_registry.h>
#include <osp/id_set.h>
#include <osp/Resource/Package.h>

#include <osp/logging.h>

#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/ImageData.h>

#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/DefaultFramebuffer.h>

#include <Magnum/Math/Color.h>

#include <Corrade/Containers/ArrayViewStl.h>

#include <atomic>
#include <tuple>
#include <unordered_map>

using Magnum::Trade::MeshData;
using Magnum::Trade::ImageData2D;

using osp::Vector3;
using Magnum::Vector3i;
using osp::Vector3l;
using osp::Matrix3;
using osp::Matrix4;

using osp::IdRegistry;
using osp::HierarchicalBitset;
using osp::IdRefCount;

// for the 0xrrggbb_rgbf and angle literals
using namespace Magnum::Math::Literals;

using Corrade::Containers::ArrayView;
using Corrade::Containers::Array;

using HierBitset_t = osp::HierarchicalBitset<uint64_t>;


namespace testapp::redstone
{

enum class BlkTypeId : uint8_t { };       // IDs for each different block type
enum class ChkBlkId : uint32_t { };       // IDs for blocks within a chunk
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

/**
 * @brief Dirty flags for added and removed blocks within a chunk
 */
struct ChkBlkChanges
{
    HierBitset_t m_added;
    HierBitset_t m_removed;
};

using ChkBlkTypeChanges_t = std::unordered_map< BlkTypeId, ChkBlkChanges >;

ChkBlkChanges& assure_chunk_update(ChkBlkTypeChanges_t &rBlkUpd, BlkTypeId blkTypeId)
{
    auto const& [it, success] = rBlkUpd.try_emplace(blkTypeId);
    ChkBlkChanges &rChkUpd = it->second;

    if (success)
    {
        rChkUpd.m_added.resize(gc_chunkSize);
        rChkUpd.m_removed.resize(gc_chunkSize);
    }

    return rChkUpd;
}

//-----------------------------------------------------------------------------

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

DiffBlk inter_chunk_pos(Vector3i pos)
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

/**
 * @brief Blocks within a chunk marked for connection update
 */
struct ChkBlkConnect
{
    HierBitset_t m_connect;
    std::atomic_flag m_lock = ATOMIC_FLAG_INIT;
};

using ChkBlkTypeConnect_t = std::unordered_map<BlkTypeId, ChkBlkConnect>;

//-----------------------------------------------------------------------------

struct ACtxVxLoadedChunks
{
    IdRegistry<ChunkId> m_ids;
    std::map< ChunkCoord_t, ChunkId > m_coordToChunk;
    std::vector<ChunkCoord_t> m_chunkToCoord;

    std::vector< Array<BlkTypeId> > m_blkTypes;
    std::vector< Array<Magnum::BoolVector3> > m_blkDirection;
    std::vector< std::vector<bool> > m_blkSensitive;

    std::vector<ChkBlkTypeChanges_t> m_blkTypeChanges;
    std::vector<ChkBlkTypeConnect_t> m_blkTypeConnect;
};

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------


void update_chunk_block_ids(
        ArrayView<BlkTypeId> chkBlkTypeIds,
        ChkBlkTypeChanges_t& rBlkUpd,
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
            assure_chunk_update(rBlkUpd, rCurrBlkTypeId).m_removed.set(size_t(block));
            // New block type got added
            assure_chunk_update(rBlkUpd, newBlkTypeId).m_added.set(size_t(block));

            // assign new block type
            rCurrBlkTypeId = newBlkTypeId;
        }
    }
}

bool try_update_chunk_connections(
        HierBitset_t const& added,
        std::vector< Array<BlkTypeId> > const& blkTypes,
        std::vector< std::vector<bool> > const& blkSensitive,
        ChunkId ownChunk,
        BlkTypeId ownType,
        std::array<ChunkId, 6> sideChunks,
        std::vector<ChkBlkTypeConnect_t>& rBlkConnect,
        std::array< std::vector< std::pair<BlkTypeId,ChkBlkId> >, 6 >& rSideTmp)
{
    ChkBlkConnect &rConnect = rBlkConnect.at(size_t(ownChunk)).at(ownType);

    if (rConnect.m_lock.test_and_set(std::memory_order_acquire))
    {
        return false;
    }

    std::vector<bool> const &ownSensitive = blkSensitive[size_t(ownChunk)];

    // Loop through each block
    for (size_t blkIdx : added)
    {
        // mark block itself as needing connection update
        if (ownSensitive[blkIdx])
        {
            rConnect.m_connect.set(blkIdx);
        }

        // Loop through 6 neighbouring blocks
        for (int i = 0; i < 6; i ++)
        {
            DiffBlk const diff
                    = inter_chunk_pos(index_to_pos(blkIdx) + gc_sideToVec[i]);
            ChkBlkId const sideBlkId = chunk_block_id(diff.m_pos);


            if (diff.m_chunkDelta.isZero())
            {
                // Side block not on different chunk
                if (ownSensitive[size_t(sideBlkId)])
                {
                    OSP_LOG_INFO("BESIDE 2!");
                    rConnect.m_connect.set(size_t(sideBlkId));
                }
            }
            else if (sideChunks[i] != osp::id_null<ChunkId>())
            {
                ESide const side = vec_to_side(diff.m_chunkDelta);
                ChunkId const chkId = sideChunks.at(size_t(side));

                if (blkSensitive.at(size_t(chkId)).at(size_t(sideBlkId)))
                {
                    BlkTypeId const sideType = blkTypes.at(size_t(chkId))[size_t(sideBlkId)];

                    std::vector<std::pair<BlkTypeId,ChkBlkId>> &rSideLater
                            = rSideTmp.at(size_t(side));

                    auto it = std::find_if(
                            std::begin(rSideLater), std::end(rSideLater),
                            [sideType] (std::pair<BlkTypeId,ChkBlkId> const& pearLol)
                    {
                        return sideType == pearLol.first;
                    });

                    rSideLater.emplace(it, sideType, sideBlkId);
                }
            }
        }
    }

    rConnect.m_lock.clear(std::memory_order_release);
    return true;
}


//-----------------------------------------------------------------------------

// Block Rendering (just make an entity for each visible block lol)

struct ChkEntModels
{
    std::vector< Array<osp::active::ActiveEnt> > m_chunkEnts;
};

//-----------------------------------------------------------------------------

// Materials used by the test scene. A more general application may want to
// generate IDs at runtime, and map them to named identifiers.
constexpr int const gc_mat_common      = 0;
constexpr int const gc_mat_flat        = 1;
constexpr int const gc_mat_wireframe   = 2;
constexpr int const gc_mat_visualizer  = 3;

constexpr int const gc_maxMaterials = 4;

auto const gc_meshsUsed = std::array{
    "Redstone/redstone:Block",
    "Redstone/redstone:TorchLit",
    "Redstone/redstone:Repeater",
    "Redstone/redstone:Torch",
    "Redstone/redstone:ShortTorch",
    "Redstone/redstone:RepeaterLit",
    "Redstone/redstone:ShortTorchLit",
    "Redstone/redstone:Lamp",
    "Redstone/redstone:LampLit",
    "Redstone/redstone:Comparator",
    "Redstone/redstone:ComparatorLit",
    "Redstone/redstone:DustStraight",
    "Redstone/redstone:Dust",
    "Redstone/redstone:DustRt",
    "Redstone/redstone:DustBk",
    "Redstone/redstone:DustLf",
    "Redstone/redstone:DustFd",
    "Redstone/redstone:LeverBlock",
    "Redstone/redstone:Lever",
    "Redstone/redstone:LeverLit"
};

/**
 * @brief State of the entire engine test scene
 */
struct RedstoneScene
{
    // ID registry generates entity IDs, and keeps track of which ones exist
    osp::IdRegistry<osp::active::ActiveEnt> m_activeIds;

    // Components and supporting data structures
    osp::active::ACtxBasic          m_basic;
    osp::active::ACtxDrawing        m_drawing;

    // Hierarchy root, needs to exist so all hierarchy entities are connected
    osp::active::ActiveEnt          m_hierRoot;

    // Crosshair
    osp::active::ActiveEnt          m_cube;

    osp::IdSet<std::string_view, BlkTypeId> m_blkTypeIds;

    ACtxVxLoadedChunks m_chunks;


    ACtxVxRsDusts m_chunkDusts;

    ChunkId m_singleChunk;

    osp::DependRes<ImageData2D> m_blockTexture;
    std::unordered_map< std::string_view, osp::DependRes<MeshData> > m_meshs;

};

entt::any setup_scene(osp::Package &rPkg)
{
    using namespace osp::active;

    entt::any sceneAny = entt::make_any<RedstoneScene>();
    RedstoneScene &rScene = entt::any_cast<RedstoneScene&>(sceneAny);


    // Allocate space to fit all materials
    rScene.m_drawing.m_materials.resize(gc_maxMaterials);

    // Create hierarchy root entity
    rScene.m_hierRoot = rScene.m_activeIds.create();
    rScene.m_basic.m_hierarchy.emplace(rScene.m_hierRoot);

    // Create camera entity
    ActiveEnt camEnt = rScene.m_activeIds.create();

    // Create camera transform and draw transform
    ACompTransform &rCamTf = rScene.m_basic.m_transform.emplace(camEnt);
    rCamTf.m_transform.translation().z() = 25;
    rScene.m_drawing.m_drawTransform.emplace(camEnt);

    // Create camera component
    ACompCamera &rCamComp = rScene.m_basic.m_camera.emplace(camEnt);
    rCamComp.m_far = 1u << 24;
    rCamComp.m_near = 1.0f;
    rCamComp.m_fov = 45.0_degf;

    // Add camera to hierarchy
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, rScene.m_hierRoot, camEnt);

    // Make a cube
    rScene.m_cube = rScene.m_activeIds.create();

    // Add cube mesh to cube
    rScene.m_drawing.m_mesh.emplace(
            rScene.m_cube, ACompMesh{ rPkg.get<MeshData>("cube_wireframe") });
    rScene.m_drawing.m_meshDirty.push_back(rScene.m_cube);

    // Add flat material to cube
    {
        MaterialData &rMatFlat = rScene.m_drawing.m_materials[gc_mat_flat];
        rMatFlat.m_comp.emplace(rScene.m_cube);
        rMatFlat.m_added.push_back(rScene.m_cube);
    }

    // Add transform and draw transform
    rScene.m_basic.m_transform.emplace(rScene.m_cube, ACompTransform{ Matrix4::scaling(Vector3{0.5}) });
    rScene.m_drawing.m_drawTransform.emplace(rScene.m_cube);

    // Make cube green
    rScene.m_drawing.m_color.emplace(rScene.m_cube, 0x67ff00ff_rgbaf);

    // Add opaque and visible component
    rScene.m_drawing.m_opaque.emplace(rScene.m_cube);
    rScene.m_drawing.m_visible.emplace(rScene.m_cube);

    // Add cube to hierarchy, parented to root
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, rScene.m_hierRoot, rScene.m_cube);

    // Create floor mesh entity
    ActiveEnt floorMesh = rScene.m_activeIds.create();

    // Add grid mesh to floor mesh
    rScene.m_drawing.m_mesh.emplace(
            floorMesh, ACompMesh{ rPkg.get<MeshData>("grid64") });
    rScene.m_drawing.m_meshDirty.push_back(floorMesh);

    // Add mesh flat material to floor mesh
    {
        MaterialData &rMatFlat = rScene.m_drawing.m_materials[gc_mat_flat];
        rMatFlat.m_comp.emplace(floorMesh);
        rMatFlat.m_added.push_back(floorMesh);
    }

    // Make floor grey
    rScene.m_drawing.m_color.emplace(floorMesh, 0x808080ff_rgbaf);

    // Add transform, draw transform, opaque, and visible
    rScene.m_basic.m_transform.emplace(
            floorMesh, ACompTransform{Matrix4::translation({32.0f, 0.0f, 32.0f}) * Matrix4::rotationX(-90.0_degf) * Matrix4::scaling({32.0f, 32.0f, 0.0f})});
    rScene.m_drawing.m_drawTransform.emplace(floorMesh);
    rScene.m_drawing.m_opaque.emplace(floorMesh);
    rScene.m_drawing.m_visible.emplace(floorMesh);

    // Add floor root to hierarchy root
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, rScene.m_hierRoot, floorMesh);

    // Create single chunk
    rScene.m_singleChunk = rScene.m_chunks.m_ids.create();
    rScene.m_chunks.m_blkTypes.resize(1);
    rScene.m_chunks.m_blkSensitive.resize(1);
    rScene.m_chunkDusts.m_dusts.resize(1);
    rScene.m_chunkDusts.m_dusts[0]
            = Array<BlkRsDust>{Corrade::DirectInit, gc_chunkSize};

    // Allocate block type array, fill with air
    rScene.m_chunks.m_blkTypes[0]
            = Array<BlkTypeId>{Corrade::DirectInit, gc_chunkSize,
                                 rScene.m_blkTypeIds.id_of("air")};
    rScene.m_chunks.m_blkSensitive[0].resize(gc_chunkSize, false);

    // Keep track of meshes
    for (std::string_view const str : gc_meshsUsed)
    {
        rScene.m_meshs.emplace(str, rPkg.get<MeshData>(str));
    }

    rScene.m_blockTexture = rPkg.get<ImageData2D>("textures");

    return std::move(sceneAny);
}

osp::active::ActiveEnt add_mesh_quick(RedstoneScene& rScene, Matrix4 const& tf, osp::DependRes<MeshData> mesh)
{
    using namespace osp::active;

    ActiveEnt ent = rScene.m_activeIds.create();

    // Add cube mesh to cube
    rScene.m_drawing.m_mesh.emplace( ent, ACompMesh{std::move(mesh)} );
    rScene.m_drawing.m_meshDirty.push_back(ent);

    // Add block texture
    rScene.m_drawing.m_diffuseTex.emplace( ent, ACompTexture{rScene.m_blockTexture} );
    rScene.m_drawing.m_diffuseDirty.push_back(ent);

    // Add common material to cube
    MaterialData &rMatCommon = rScene.m_drawing.m_materials[gc_mat_flat];
    rMatCommon.m_comp.emplace(ent);
    rMatCommon.m_added.push_back(ent);

    // Add transform and draw transform
    rScene.m_basic.m_transform.emplace( ent, ACompTransform{tf} );
    rScene.m_drawing.m_drawTransform.emplace(ent);

    // Add opaque and visible component
    rScene.m_drawing.m_opaque.emplace(ent);
    rScene.m_drawing.m_visible.emplace(ent);

    // Add cube to hierarchy, parented to root
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, rScene.m_hierRoot, ent);

    return ent;
}

void update_chunk_various(
        RedstoneScene& rScene, ChunkId chkId,
        ChkBlkTypeChanges_t const& blkUpdates)
{
    using namespace osp::active;

    ACtxVxRsDusts::ChunkDust_t &rDusts = rScene.m_chunkDusts.m_dusts.at(size_t(chkId));

    BlkTypeId const dustId = rScene.m_blkTypeIds.id_of("dust");

    // System: Redstone Dust connector

    if (auto it = blkUpdates.find(dustId);
        it != blkUpdates.end())
    {
        HierBitset_t const& updDust = it->second.m_added;

        for (size_t blockId : updDust)
        {
            Vector3i const pos = index_to_pos(blockId);

            constexpr std::array< std::pair<Vector3i, EMultiDir>, 4 > const c_sides
            {{
                { { 1,  0,  0}, EMultiDir::POS_X },
                { {-1,  0,  0}, EMultiDir::NEG_X },
                { { 0,  0,  1}, EMultiDir::POS_Z },
                { { 0,  0, -1}, EMultiDir::NEG_Z }
            }};

            BlkRsDust &rDust = rDusts[blockId];
            rDust.m_connected = EMultiDir(0);
            rDust.m_rises = EMultiDir(0);

            for (std::pair<Vector3i, EMultiDir> const& pair : c_sides)
            {
                DiffBlk const interpos = inter_chunk_pos(pos + pair.first);
                ChkBlkId const sideBlkId = chunk_block_id(interpos.m_pos);

                if (rScene.m_chunks.m_blkTypes[size_t(chkId)][size_t(sideBlkId)] == dustId)
                {
                    OSP_LOG_INFO("BESIDE!");
                }
            }

            //DiffBlk const below = inter_chunk_pos(pos + Vector3i{1, 0, 0});
            //OSP_LOG_INFO("pos: ({}, {}, {})", below.pos.x(), below.pos.y(), below.pos.z());
            //OSP_LOG_INFO("chunk: ({}, {}, {})", below.chunkDelta.x(), below.chunkDelta.y(), below.chunkDelta.z());
        }
    }


    // Create models

    Vector3 chunkOffset{0}; // TODO

    // Create redstone dust models
    if (auto it = blkUpdates.find(rScene.m_blkTypeIds.id_of("dust"));
        it != blkUpdates.end())
    {
        for (size_t blockId : it->second.m_added)
        {
            Vector3 const pos = Vector3(index_to_pos(blockId)) + Vector3{0.5, 0.5, 0.5};
            ActiveEnt meshEnt = add_mesh_quick(rScene, Matrix4::translation(pos), rScene.m_meshs.at("Redstone/redstone:Dust"));

            rScene.m_drawing.m_color.emplace(meshEnt, 0x880000ff_rgbaf);
        }
    }

    // Create redstone torch models
    if (auto it = blkUpdates.find(rScene.m_blkTypeIds.id_of("torch"));
        it != blkUpdates.end())
    {
        for (size_t blockId : it->second.m_added)
        {
            Vector3 const pos = Vector3(index_to_pos(blockId)) + Vector3{0.5, 0.5, 0.5};
            add_mesh_quick(rScene, Matrix4::translation(pos), rScene.m_meshs.at("Redstone/redstone:Torch"));
        }
    }

}

void set_sensitive(RedstoneScene& rScene, ChunkId chunk, BlkTypeId type, bool sensitive)
{
    ChkBlkTypeChanges_t &rTypeChanges = rScene.m_chunks.m_blkTypeChanges[size_t(chunk)];
    if (auto it = rTypeChanges.find(type);
        it != rTypeChanges.end())
    {
        for (size_t blkId : it->second.m_added)
        {
            rScene.m_chunks.m_blkSensitive[size_t(chunk)].at(blkId) = sensitive;
        }
    }
}

void update_chunks(
        RedstoneScene& rScene,
        ArrayView< std::pair<ChunkId, ChkBlkPlacements_t> > chunkUpd)
{
    using namespace osp::active;

    rScene.m_chunks.m_blkTypeChanges.resize(rScene.m_chunks.m_ids.capacity());
    rScene.m_chunks.m_blkTypeConnect.resize(rScene.m_chunks.m_ids.capacity());

    for (auto const& [chunkId, blkChanges] : chunkUpd)
    {
        // Clear block added and removed queues
        for (auto & [blkTypeId, blkUpd]
             : rScene.m_chunks.m_blkTypeChanges[size_t(chunkId)])
        {
            blkUpd.m_added.reset();
            blkUpd.m_removed.reset();
        }

        // Update all block IDs
        update_chunk_block_ids(
                rScene.m_chunks.m_blkTypes[size_t(chunkId)],
                rScene.m_chunks.m_blkTypeChanges[size_t(chunkId)],
                blkChanges);

        // mark dusts as sensitive
        set_sensitive(rScene, chunkId, rScene.m_blkTypeIds.id_of("dust"), true);
    }


    for (auto const& [chunkId, blkChanges] : chunkUpd)
    {
        for (auto & [blkTypeId, chkBlkChanges] : rScene.m_chunks.m_blkTypeChanges[size_t(chunkId)])
        {
            std::array< std::vector< std::pair<BlkTypeId,ChkBlkId> >, 6 > sideTmp;

            rScene.m_chunks.m_blkTypeConnect[size_t(chunkId)][blkTypeId].m_connect.resize(gc_chunkSize);

            try_update_chunk_connections(
                    chkBlkChanges.m_added,
                    rScene.m_chunks.m_blkTypes,
                    rScene.m_chunks.m_blkSensitive,
                    chunkId,
                    blkTypeId,
                    {osp::id_null<ChunkId>(), osp::id_null<ChunkId>(), osp::id_null<ChunkId>(), osp::id_null<ChunkId>(), osp::id_null<ChunkId>(), osp::id_null<ChunkId>()},
                    rScene.m_chunks.m_blkTypeConnect,
                    sideTmp);
        }

        update_chunk_various(rScene, chunkId, rScene.m_chunks.m_blkTypeChanges[size_t(chunkId)]);
    }
}

/**
 * @brief Update an EngineTestScene, this just rotates the cube
 *
 * @param rScene [ref] scene to update
 */
void update_test_scene(RedstoneScene& rScene, float delta)
{
    using namespace osp::active;



}

//-----------------------------------------------------------------------------

// Everything below here is for rendering

/**
 * @brief Data needed to render the EngineTestScene
 */
struct RedstoneRenderer
{
    RedstoneRenderer(ActiveApplication &rApp)
     : m_camCtrl(rApp.get_input_handler())
     , m_controls(&rApp.get_input_handler())
     , m_btnPlaceDust(m_controls.button_subscribe("rs_place_dust"))
     , m_btnPlaceTorch(m_controls.button_subscribe("rs_place_torch"))
    { }

    osp::active::ACtxRenderGroups m_renderGroups{};

    osp::active::ACtxRenderGL m_renderGl{};

    osp::active::ActiveEnt m_camera;
    ACtxCameraController m_camCtrl;

    osp::shader::ACtxDrawFlat m_flat;
    osp::shader::ACtxDrawPhong m_phong;
    osp::shader::ACtxDrawMeshVisualizer m_visualizer;
    osp::shader::ACtxDrawMeshVisualizer m_wireframe;

    osp::input::ControlSubscriber m_controls;
    osp::input::EButtonControlIndex m_btnPlaceDust;
    osp::input::EButtonControlIndex m_btnPlaceTorch;
};

/**
 * @brief Render an EngineTestScene
 *
 * @param rApp      [ref] Application with GL context and resources
 * @param rScene    [ref] Test scene to render
 * @param rRenderer [ref] Renderer data for test scene
 */
void render_test_scene(
        ActiveApplication& rApp, RedstoneScene& rScene,
        RedstoneRenderer& rRenderer)
{
    using namespace osp::active;
    using namespace osp::shader;
    using Magnum::GL::Renderer;
    using Magnum::GL::Framebuffer;
    using Magnum::GL::FramebufferClear;
    using Magnum::GL::Texture2D;

    osp::Package &rGlResources = rApp.get_gl_resources();

    // Load any required meshes
    SysRenderGL::compile_meshes(
            rScene.m_drawing.m_mesh, rScene.m_drawing.m_meshDirty,
            rRenderer.m_renderGl.m_meshGl, rApp.get_gl_resources());

    // Load any required textures
    SysRenderGL::compile_textures(
            rScene.m_drawing.m_diffuseTex, rScene.m_drawing.m_diffuseDirty,
            rRenderer.m_renderGl.m_diffuseTexGl, rApp.get_gl_resources());

    RenderGroup &rGroupFwdOpaque
            = rRenderer.m_renderGroups.m_groups["fwd_opaque"];
    RenderGroup &rGroupFwdTransparent
            = rRenderer.m_renderGroups.m_groups["fwd_transparent"];

    // Assign Flat shader to entities with the gc_mat_flat material, and put
    // results into the fwd_opaque render group
    {
        MaterialData &rMatFlat = rScene.m_drawing.m_materials[gc_mat_flat];
        assign_flat(
                rMatFlat.m_added,
                &rGroupFwdOpaque.m_entities,
                &rGroupFwdTransparent.m_entities,
                rScene.m_drawing.m_opaque,
                rRenderer.m_renderGl.m_diffuseTexGl,
                rRenderer.m_flat);
        rMatFlat.m_added.clear();
    }

    // Assign Phong shader to entities with the gc_mat_common material, and put
    // results into the fwd_opaque render group
    {
        MaterialData &rMatCommon = rScene.m_drawing.m_materials[gc_mat_common];
        assign_phong(
                rMatCommon.m_added,
                &rGroupFwdOpaque.m_entities,
                &rGroupFwdTransparent.m_entities,
                rScene.m_drawing.m_opaque,
                rRenderer.m_renderGl.m_diffuseTexGl,
                rRenderer.m_phong);
        rMatCommon.m_added.clear();
    }

    // Same thing but with MeshVisualizer and gc_mat_visualizer
    {
        MaterialData &rMatVisualizer
                = rScene.m_drawing.m_materials[gc_mat_visualizer];
        assign_visualizer(
                rMatVisualizer.m_added,
                rGroupFwdTransparent.m_entities,
                rRenderer.m_visualizer);
        rMatVisualizer.m_added.clear();
    }

    // Same thing but with MeshVisualizer and gc_mat_visualizer
    {
        MaterialData &rMatWireframe
                = rScene.m_drawing.m_materials[gc_mat_wireframe];
        assign_visualizer(
                rMatWireframe.m_added,
                rGroupFwdTransparent.m_entities,
                rRenderer.m_wireframe);
        rMatWireframe.m_added.clear();
    }

    // Calculate hierarchy transforms
    SysHierarchy::sort(rScene.m_basic.m_hierarchy);
    SysRender::update_draw_transforms(
            rScene.m_basic.m_hierarchy,
            rScene.m_basic.m_transform,
            rScene.m_drawing.m_drawTransform);

    // Get camera, and calculate projection matrix and inverse transformation
    ACompCamera &rCamera = rScene.m_basic.m_camera.get(rRenderer.m_camera);
    ACompDrawTransform const &cameraDrawTf
            = rScene.m_drawing.m_drawTransform.get(rRenderer.m_camera);
    rCamera.m_viewport
            = osp::Vector2(Magnum::GL::defaultFramebuffer.viewport().size());
    rCamera.calculate_projection();
    rCamera.m_inverse = cameraDrawTf.m_transformWorld.inverted();

    // Bind offscreen FBO
    Framebuffer &rFbo = *rGlResources.get<Framebuffer>("offscreen_fbo");
    rFbo.bind();

    // Clear it
    rFbo.clear( FramebufferClear::Color | FramebufferClear::Depth
                | FramebufferClear::Stencil);

    // Forward Render fwd_opaque group to FBO
    Renderer::enable(Renderer::Feature::DepthTest);
    Renderer::disable(Renderer::Feature::FaceCulling);
    Renderer::disable(Renderer::Feature::Blending);
    Renderer::setDepthMask(true);

    SysRenderGL::draw_group(
            rRenderer.m_renderGroups.m_groups.at("fwd_opaque"),
            rScene.m_drawing.m_visible, rCamera);

    // Forward Render fwd_transparent group to FBO
    SysRenderGL::render_transparent(
            rRenderer.m_renderGroups.m_groups.at("fwd_transparent"),
            rScene.m_drawing.m_visible, rCamera);

    // Display FBO
    Texture2D &rFboColor = *rGlResources.get<Texture2D>("offscreen_fbo_color");
    SysRenderGL::display_texture(rGlResources, rFboColor);
}

void load_gl_resources(ActiveApplication& rApp)
{
    using osp::shader::Flat;
    using osp::shader::Phong;
    using osp::shader::MeshVisualizer;

    osp::Package &rGlResources = rApp.get_gl_resources();

    // Create Flat shaders
    rGlResources.add<Flat>("textured", Flat{Flat::Flag::Textured | Flat::Flag::AlphaMask});
    rGlResources.add<Flat>("notexture", Flat{});

    // Create Phong shaders
    auto texturedFlags = Phong::Flag::DiffuseTexture
                       | Phong::Flag::AlphaMask
                       | Phong::Flag::AmbientTexture;
    rGlResources.add<Phong>("textured", Phong{texturedFlags, 2});
    rGlResources.add<Phong>("notexture", Phong{{}, 2});

    // Create MeshVisualizer shader
    osp::DependRes<MeshVisualizer> visualizer = rGlResources.add<MeshVisualizer>(
            "mesh_vis_shader",
            MeshVisualizer{ MeshVisualizer::Flag::Wireframe });
    (*visualizer)
        .setColor(0x888888ff_rgbaf)
        .setWireframeColor(0xffffffff_rgbaf);

    // Create MeshVisualizer shader intended for wireframe
    osp::DependRes<MeshVisualizer> wireframe = rGlResources.add<MeshVisualizer>(
            "wireframe",
            MeshVisualizer{ MeshVisualizer::Flag::Wireframe });
    (*wireframe)
        .setColor(0x00000000_rgbaf)
        .setWireframeColor(0xffffffffff_rgbaf);
}

on_draw_t gen_draw(RedstoneScene& rScene, ActiveApplication& rApp)
{
    using namespace osp::active;
    using namespace osp::shader;

    // Create renderer data. This uses a shared_ptr to allow being stored
    // inside an std::function, which require copyable types
    std::shared_ptr<RedstoneRenderer> pRenderer
            = std::make_shared<RedstoneRenderer>(rApp);

    osp::Package &rGlResources = rApp.get_gl_resources();

    // Acquire data needed to draw Flat materials
    {
        ACtxDrawFlat &rFlat = pRenderer->m_flat;
        rFlat.m_shaderUntextured
                = rGlResources.get_or_reserve<Flat>("notexture");
        rFlat.m_shaderDiffuse
                = rGlResources.get_or_reserve<Flat>("textured");
        rFlat.m_pDrawTf       = &rScene.m_drawing.m_drawTransform;
        rFlat.m_pColor        = &rScene.m_drawing.m_color;
        rFlat.m_pDiffuseTexGl = &pRenderer->m_renderGl.m_diffuseTexGl;
        rFlat.m_pMeshGl       = &pRenderer->m_renderGl.m_meshGl;
    }

    // Acquire data needed to draw Phong materials
    {
        ACtxDrawPhong &rPhong = pRenderer->m_phong;
        rPhong.m_shaderUntextured
                = rGlResources.get_or_reserve<Phong>("notexture");
        rPhong.m_shaderDiffuse
                = rGlResources.get_or_reserve<Phong>("textured");
        rPhong.m_pDrawTf       = &rScene.m_drawing.m_drawTransform;
        rPhong.m_pColor        = &rScene.m_drawing.m_color;
        rPhong.m_pDiffuseTexGl = &pRenderer->m_renderGl.m_diffuseTexGl;
        rPhong.m_pMeshGl       = &pRenderer->m_renderGl.m_meshGl;
    }

    // Acquire data needed to draw MeshVisualizer materials
    {
        ACtxDrawMeshVisualizer &rVisualizer = pRenderer->m_visualizer;
        rVisualizer.m_shader
                = rGlResources.get_or_reserve<MeshVisualizer>("mesh_vis_shader");
        rVisualizer.m_pDrawTf = &rScene.m_drawing.m_drawTransform;
        rVisualizer.m_pMeshGl = &pRenderer->m_renderGl.m_meshGl;
    }

    // Acquire data needed to draw MeshVisualizer materials, but for wireframes
    // only
    {
        ACtxDrawMeshVisualizer &rWireframe = pRenderer->m_wireframe;
        pRenderer->m_wireframe.m_shader
                = rGlResources.get_or_reserve<MeshVisualizer>("wireframe");
        pRenderer->m_wireframe.m_pDrawTf = &rScene.m_drawing.m_drawTransform;
        pRenderer->m_wireframe.m_pMeshGl = &pRenderer->m_renderGl.m_meshGl;
        pRenderer->m_wireframe.m_wireframeOnly = true;

    }

    // Select first camera for rendering
    pRenderer->m_camera = rScene.m_basic.m_camera.at(0);

    pRenderer->m_camCtrl.m_target = osp::Vector3{};
    pRenderer->m_camCtrl.m_up = osp::Vector3{0.0f, 1.0f, 0.0f};

    // Create render group for forward transparent pass
    pRenderer->m_renderGroups.m_groups.emplace("fwd_opaque", RenderGroup{});
    pRenderer->m_renderGroups.m_groups.emplace("fwd_transparent", RenderGroup{});

    // Set everything dirty, required for re-opening an existing scene

    // Set all materials dirty
    for (MaterialData &rMat : rScene.m_drawing.m_materials)
    {
        rMat.m_added.assign(std::begin(rMat.m_comp), std::end(rMat.m_comp));
    }

    // Set all meshs dirty
    auto &rMeshSet = static_cast<active_sparse_set_t&>(rScene.m_drawing.m_mesh);
    rScene.m_drawing.m_meshDirty.assign(std::begin(rMeshSet), std::end(rMeshSet));

    // Set all textures dirty
    auto &rDiffSet = static_cast<active_sparse_set_t&>(rScene.m_drawing.m_diffuseTex);
    rScene.m_drawing.m_diffuseDirty.assign(std::begin(rMeshSet), std::end(rMeshSet));

    return [&rScene, pRenderer = std::move(pRenderer)] (
            ActiveApplication& rApp, float delta)
    {

        ACompTransform &rCamTf = rScene.m_basic.m_transform.get(pRenderer->m_camera);

        SysCameraController::update_view(pRenderer->m_camCtrl, rCamTf, delta);
        SysCameraController::update_move(
                pRenderer->m_camCtrl, rCamTf, delta, true);


        // Move cube to camera target
        Magnum::Vector3i const target{pRenderer->m_camCtrl.m_target.value()};
        ACompTransform &rCubeTf = rScene.m_basic.m_transform.get(rScene.m_cube);

        if (   target.x() >= 0
            && target.y() >= 0
            && target.z() >= 0)
        {

            rCubeTf.m_transform.translation() = Vector3(target) + Vector3{0.5f, 0.5f, 0.5f};
        }
        else
        {
            rCubeTf.m_transform.translation() = Vector3(-2000);
        }


        Vector3 const lookDir = -rCamTf.m_transform.backward();

        ChkBlkPlacements_t blkPlacements;

        for (osp::input::ButtonControlEvent const& event : pRenderer->m_controls.button_events())
        {
            using osp::input::EButtonControlEvent;

            if (event.m_index == pRenderer->m_btnPlaceDust
                && event.m_event == EButtonControlEvent::Triggered)
            {
                // Place dust
                blkPlacements[rScene.m_blkTypeIds.id_of("dust")].push_back(
                    ChkBlkPlace{ target, lookDir }
                );

                OSP_LOG_INFO("Place Dust: ({}, {}, {})", target.x(), target.y(), target.z());
            }

            if (event.m_index == pRenderer->m_btnPlaceTorch
                && event.m_event == EButtonControlEvent::Triggered)
            {
                // Place torch
                blkPlacements[rScene.m_blkTypeIds.id_of("torch")].push_back(
                    ChkBlkPlace{ target, lookDir }
                );

                OSP_LOG_INFO("Place Torch: ({}, {}, {})", target.x(), target.y(), target.z());
            }
        }

        std::array< std::pair<ChunkId, ChkBlkPlacements_t>, 1> changesArray
        {{
            {rScene.m_singleChunk, blkPlacements}
        }};
        update_chunks(rScene, changesArray);

        update_test_scene(rScene, delta);
        render_test_scene(rApp, rScene, *pRenderer);
    };
}

} // namespace testapp::redstone

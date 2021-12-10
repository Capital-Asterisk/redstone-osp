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


namespace testapp::redstone
{

enum class RsNodeId : uint32_t { };
enum class RsElemId : uint32_t { };
enum class RsElemLocalId : uint32_t { };
enum class RsElemTypeId : uint8_t { };

using HierBitset_t = HierarchicalBitset<uint64_t>;

using RsNodeRefCount_t = IdRefCount<RsNodeId>;
using RsNodeSto_t = RsNodeRefCount_t::Storage_t;

template<typename T>
using element_storage_t = entt::basic_storage<RsElemId, T>;

using power_level_t = uint8_t; // 0 to 15

using Corrade::Containers::ArrayView;
using Corrade::Containers::Array;

struct RsNodeConnect
{
    std::array<RsElemId, 6> m_connections;
    uint8_t m_count;
};

struct RsElementUpdate
{
    // m_toUpdate[RsElemTypeId][RsElemLocalId]
    std::vector<HierBitset_t> m_toUpdate;
};

struct RsDust
{
    RsNodeSto_t m_own;
    //std::array<RsNodeSto_t, 5> m_connections;
    //uint8_t m_connectionCount;
};

size_t advance_multiple_max(ArrayView<HierBitset_t::Iterator> iterators) noexcept
{
    // Select largest iterator in localUpd, store in pMax
    HierBitset_t::Iterator *pMaxIt = &iterators[0];
    size_t value = **pMaxIt;

    for (int i = 1; i < iterators.size(); i ++)
    {
        auto &rTestIt = iterators[i];
        size_t const testValue = *rTestIt;

        if (testValue < value)
        {
            // rIt's value beats current max, it is now the new max
            pMaxIt = &rTestIt;
            value = testValue;
        }
        else if (testValue == value)
        {
            // Duplicate found, skip it
            std::advance(rTestIt, 1);
        }
    }

    std::advance(pMaxIt, 1);

    return value;
}

void write_nodes(
        Array<HierBitset_t::Iterator>                       writes,
        ArrayView< ArrayView<power_level_t const> const >   newPowers,
        uint64_t                                            endValue,
        ArrayView<power_level_t>                            powers,
        ArrayView<RsNodeConnect const>                      connections,
        ArrayView<RsElemTypeId const>                       elemTypes,
        ArrayView<RsElemLocalId const>                      elemLocals,
        RsElementUpdate&                                    rElemUpd)
{
    while (true)
    {
        size_t value = advance_multiple_max(writes);

        if (value >= endValue)
        {
            break; // Passed required elements to iterate
        }


    }

    /*
    for (uint32_t i = 0; i < nodePowers.size(); i ++)
    {
        bool hasValue = false;
        power_level_t power = 0;

        for (RsNodeWrites const& write : rWrites)
        {
            hasValue = hasValue || write.m_changes.test(i);
            power = std::max(power, write.m_power[i]);
        }

        if (nodePowers[i] != power)
        {
            nodePowers[i] = power;

            // Value changed, add elements to rElemUpd
            RsNodeConnect const &rConnect = nodeConnections[i];
            for (int j = 0; j < rConnect.m_count; j ++)
            {
                RsElemId const id           = rConnect.m_connections[j];
                RsElemTypeId const type     = elemTypes[size_t(id)];
                RsElemLocalId const local   = elemLocals[size_t(id)];

                rElemUpd.m_toUpdate[size_t(type)].set(size_t(local));
            }
        }
    }*/
}

void calc_dust(
        Array<HierBitset_t::Iterator>       localUpd,
        uint64_t                            endValue,
        RsElemTypeId                        myType,
        ArrayView<power_level_t const>      nodePowers,
        ArrayView<RsDust const>             dustData)
{

    while (true)
    {
        size_t value = advance_multiple_max(localUpd);

        if (value >= endValue)
        {
            break; // Passed required elements to iterate
        }

        // actually calculate the dust stuff

        RsDust const& data = dustData[value];

        RsNodeId const myNode = data.m_own;

        power_level_t const currentPower = nodePowers[size_t(myNode)];

        //for (int i = 0; i < data.m_connectionCount; i ++)
        //{
        //    RsNodeId const connectedNode = data.m_connections[i];


        //}

    }
}

struct RsRepeater
{
    RsNodeSto_t m_in;
    RsNodeSto_t m_out;
    std::array<RsNodeSto_t, 2> m_lock;

    std::bitset<4> m_bits;
    uint8_t m_delay;
};

using ElemIdReg_t = osp::UniqueIdRegistry<RsElemId>;
using ElemIdSto_t = ElemIdReg_t::Storage_t;

using ElemLocIdReg_t = osp::UniqueIdRegistry<RsElemLocalId>;
using ElemLocIdSto_t = ElemLocIdReg_t::Storage_t;

struct RsWorld
{
    IdRegistry<RsNodeId> m_nodeIds;
    IdRefCount<RsNodeId> m_nodeRefCounts;

    std::vector<power_level_t> m_nodePowers;
    std::vector<RsNodeConnect> m_nodeConnections;

    int m_maxTypes;
    ElemIdReg_t m_elementIds;
    std::vector<RsElemTypeId> m_elementTypes;
    std::vector<RsElemLocalId> m_elementLocals;

    ElemLocIdReg_t m_dustIds;
    std::vector<RsDust> m_dustData;
};

//-----------------------------------------------------------------------------

enum class VxBlkTypeId : uint8_t { };       // IDs for each different block type

enum class VxChkBlkId : uint32_t { };       // IDs for blocks within a chunk

enum class VxChkId : uint16_t { };          // IDs for currently loaded chunks

constexpr Vector3i const gc_vxChunkDim{8, 8, 8};
constexpr Vector3i::Type const gc_vxChunkSize
        = gc_vxChunkDim.x() * gc_vxChunkDim.y() * gc_vxChunkDim.z();

template <typename VEC_T>
constexpr typename VEC_T::Type pos_index(VEC_T chunkSize, VEC_T pos) noexcept
{
    return   pos.x()
           + pos.z() * chunkSize.x()
           + pos.y() * chunkSize.x() * chunkSize.z();
}

constexpr VxChkBlkId chunk_block_id(Vector3i pos) noexcept
{
    return VxChkBlkId(pos_index(gc_vxChunkDim, pos));
}

using ChunkCoord_t = std::array<int, 3>;

struct ACtxVxLoadedChunks
{
    IdRegistry<VxChkId> m_chunkId;
    std::map< ChunkCoord_t, VxChkId > m_chunkMap;
    std::vector< Array<VxBlkTypeId> > m_chunkBlockTypes;
};



struct BlkPlace
{
    Vector3i m_pos;
    Vector3 m_lookDir;
};

struct BlkRemove
{

};

struct ChunkBlockChanges
{
    std::unordered_map< VxBlkTypeId, std::vector<BlkPlace> > m_place;
    // streams of blocks to place
    // streams of blocks directly assigned m_chunkBlockTypes
    // scan through stream, and see which block types it contains, add to set
    // pass it to a main block placer system, which dispatches all requireds
};

struct BlkRsDust
{
    ElemLocIdSto_t m_elem;
};

struct ACtxVxRsDusts
{
    using ChunkDust_t = Array<BlkRsDust>;
    // m_dusts[VxChkId][VxSubChkId][VxSubChkBlkId]
    std::vector<ChunkDust_t> m_dusts;
};

void place_block_ids(VxBlkTypeId id, ArrayView<BlkPlace const> placements, ArrayView<VxBlkTypeId> blockIds)
{
    for (BlkPlace const& place : placements)
    {
        assert(place.m_pos.x() < gc_vxChunkDim.x());
        assert(place.m_pos.y() < gc_vxChunkDim.y());
        assert(place.m_pos.z() < gc_vxChunkDim.z());

        VxChkBlkId const block = chunk_block_id(place.m_pos);

        OSP_LOG_INFO("block ID: {}", size_t(block));


        blockIds[size_t(block)] = id;
    }
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

    osp::IdSet<std::string_view, VxBlkTypeId> m_blkTypeIds;

    ACtxVxLoadedChunks m_chunks;
    ACtxVxRsDusts m_chunkDusts;

    VxChkId m_singleChunk;


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
    rScene.m_singleChunk = rScene.m_chunks.m_chunkId.create();
    rScene.m_chunks.m_chunkBlockTypes.resize(1);
    rScene.m_chunkDusts.m_dusts.resize(1);

    // Allocate block type array, fill with air
    rScene.m_chunks.m_chunkBlockTypes[0]
            = Array<VxBlkTypeId>{Corrade::DirectInit, gc_vxChunkSize,
                                 rScene.m_blkTypeIds.id_of("air")};

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
    rScene.m_drawing.m_color.emplace(ent, 0x880000ff_rgbaf);
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


void update_blocks(RedstoneScene& rScene, ChunkBlockChanges &rBlkChange)
{
    using namespace osp::active;


    // Update Block IDs
    for (auto & [id, rChanges] : rBlkChange.m_place)
    {
        place_block_ids(
                id, rChanges,
                rScene.m_chunks.m_chunkBlockTypes[size_t(rScene.m_singleChunk)]);
        OSP_LOG_INFO("Wren!");
    }

    // Place redstone dusts
    if (auto it = rBlkChange.m_place.find(rScene.m_blkTypeIds.id_of("dust"));
        it != rBlkChange.m_place.end())
    {
        for (BlkPlace const& place : it->second)
        {
            Vector3 const pos = Vector3(place.m_pos) + Vector3{0.5, 0.5, 0.5};
            add_mesh_quick(rScene, Matrix4::translation(pos), rScene.m_meshs.at("Redstone/redstone:Dust"));
        }
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

        ChunkBlockChanges changes;

        for (osp::input::ButtonControlEvent const& event : pRenderer->m_controls.button_events())
        {
            using osp::input::EButtonControlEvent;

            if (event.m_index == pRenderer->m_btnPlaceDust
                && event.m_event == EButtonControlEvent::Triggered)
            {
                // Place dust
                changes.m_place[rScene.m_blkTypeIds.id_of("dust")].push_back(
                    BlkPlace{ target, lookDir }
                );

                OSP_LOG_INFO("Place! ({}, {}, {})", target.x(), target.y(), target.z());
            }
        }

        update_blocks(rScene, changes);

        update_test_scene(rScene, delta);
        render_test_scene(rApp, rScene, *pRenderer);
    };
}

} // namespace testapp::redstone

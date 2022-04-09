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
#include "redstone_osp.h"
#include "scene_physics.h" // also happens to include mesh owning
#include "common_scene.h"
#include "common_renderer_gl.h"

#include "../ActiveApplication.h"

#include <osp/Active/basic.h>
#include <osp/Active/drawing.h>

#include <osp/Active/SysHierarchy.h>

#include <osp/Active/SysRender.h>
#include <osp/Active/opengl/SysRenderGL.h>

#include <osp/Shaders/Flat.h>
#include <osp/Shaders/Phong.h>
#include <osp/Shaders/MeshVisualizer.h>

#include <osp/Resource/resources.h>

#include <longeron/id_management/refcount.hpp>

#include <osp/logging.h>

#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/DefaultFramebuffer.h>

#include <Magnum/Math/Color.h>

#include <atomic>
#include <tuple>
#include <unordered_map>


using Magnum::Trade::MeshData;
using Magnum::Trade::ImageData2D;

using osp::Vector3l;
using osp::Matrix3;
using osp::Matrix4;

// for the 0xrrggbb_rgbf and angle literals
using namespace Magnum::Math::Literals;

namespace testapp::redstone
{


auto const gc_meshsUsed = std::array{
    "Redstone/redstone.gltf:Block",
    "Redstone/redstone.gltf:TorchLit",
    "Redstone/redstone.gltf:Repeater",
    "Redstone/redstone.gltf:Torch",
    "Redstone/redstone.gltf:ShortTorch",
    "Redstone/redstone.gltf:RepeaterLit",
    "Redstone/redstone.gltf:ShortTorchLit",
    "Redstone/redstone.gltf:Lamp",
    "Redstone/redstone.gltf:LampLit",
    "Redstone/redstone.gltf:Comparator",
    "Redstone/redstone.gltf:ComparatorLit",
    "Redstone/redstone.gltf:DustStraight",
    "Redstone/redstone.gltf:Dust",
    "Redstone/redstone.gltf:DustRt",
    "Redstone/redstone.gltf:DustBk",
    "Redstone/redstone.gltf:DustLf",
    "Redstone/redstone.gltf:DustFd",
    "Redstone/redstone.gltf:LeverBlock",
    "Redstone/redstone.gltf:Lever",
    "Redstone/redstone.gltf:LeverLit"
};


/**
 * @brief State of the entire engine test scene
 */
struct RedstoneTestScene : RedstoneScene
{
    // Crosshair
    osp::active::ActiveEnt      m_cube;
    osp::active::TexId          m_blockTexture;

    int m_matFlat{0};
    int m_matWireframe{0};

};



void Redstone::setup_scene(CommonTestScene &rScene, osp::PkgId pkg)
{
    using namespace osp::active;

    auto &rScnPhys = rScene.emplace<scenes::PhysicsData>();
    auto &rScnRs = rScene.emplace<RedstoneTestScene>();

    rScene.m_onCleanup.push_back(&scenes::PhysicsData::cleanup);

    osp::Resources &rResources = *rScene.m_pResources;

    // Convenient function to get a reference-counted mesh owner
    auto const quick_add_mesh = [&rScene, &rResources, pkg] (std::string_view name) -> MeshIdOwner_t
    {
        osp::ResId const res = rResources.find(osp::restypes::gc_mesh, pkg, name);
        assert(res != lgrn::id_null<osp::ResId>());
        MeshId const meshId = SysRender::own_mesh_resource(rScene.m_drawing, rScene.m_drawingRes, rResources, res);
        return rScene.m_drawing.m_meshRefCounts.ref_add(meshId);
    };



    // Allocate space to fit all materials
    rScnRs.m_matFlat        = rScene.m_materialCount++;
    rScnRs.m_matWireframe   = rScene.m_materialCount++;
    rScene.m_drawing.m_materials.resize(rScene.m_materialCount);

    // Create hierarchy root entity
    rScene.m_hierRoot = rScene.m_activeIds.create();
    rScene.m_basic.m_hierarchy.emplace(rScene.m_hierRoot);

    // Create camera entity
    ActiveEnt camEnt = rScene.m_activeIds.create();

    // Create camera transform and draw transform
    ACompTransform &rCamTf = rScene.m_basic.m_transform.emplace(camEnt);
    rCamTf.m_transform.translation().z() = 25;

    // Create camera component
    ACompCamera &rCamComp = rScene.m_basic.m_camera.emplace(camEnt);
    rCamComp.m_far = 1u << 24;
    rCamComp.m_near = 1.0f;
    rCamComp.m_fov = 45.0_degf;

    // Add camera to hierarchy
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, rScene.m_hierRoot, camEnt);

    // Make a cube
    rScnRs.m_cube = rScene.m_activeIds.create();

    // Add cube mesh to cube
    rScene.m_drawing.m_mesh.emplace(
            rScnRs.m_cube, quick_add_mesh("cubewire") );
    rScene.m_drawing.m_meshDirty.push_back(rScnRs.m_cube);

    // Add flat material to cube
    {
        MaterialData &rMatFlat = rScene.m_drawing.m_materials[rScnRs.m_matFlat];
        rMatFlat.m_comp.emplace(rScnRs.m_cube);
        rMatFlat.m_added.push_back(rScnRs.m_cube);
    }

    // Add transform and draw transform
    rScene.m_basic.m_transform.emplace(rScnRs.m_cube, ACompTransform{ Matrix4::scaling(Vector3{0.5}) });

    // Make cube green
    rScene.m_drawing.m_color.emplace(rScnRs.m_cube, 0x67ff00ff_rgbaf);

    // Add opaque and visible component
    rScene.m_drawing.m_opaque.emplace(rScnRs.m_cube);
    rScene.m_drawing.m_visible.emplace(rScnRs.m_cube);

    // Add cube to hierarchy, parented to root
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, rScene.m_hierRoot, rScnRs.m_cube);

    // Create floor mesh entity
    ActiveEnt floorMesh = rScene.m_activeIds.create();

    // Add grid mesh to floor mesh
    rScene.m_drawing.m_mesh.emplace(
            floorMesh, quick_add_mesh("grid64wire"));
    rScene.m_drawing.m_meshDirty.push_back(floorMesh);

    // Add mesh flat material to floor mesh
    {
        MaterialData &rMatFlat = rScene.m_drawing.m_materials[rScnRs.m_matFlat];
        rMatFlat.m_comp.emplace(floorMesh);
        rMatFlat.m_added.push_back(floorMesh);
    }

    // Make floor grey
    rScene.m_drawing.m_color.emplace(floorMesh, 0x808080ff_rgbaf);

    // Add transform, draw transform, opaque, and visible
    rScene.m_basic.m_transform.emplace(
            floorMesh, ACompTransform{Matrix4::translation({32.0f, 0.0f, 32.0f}) * Matrix4::rotationX(-90.0_degf) * Matrix4::scaling({32.0f, 32.0f, 0.0f})});
    rScene.m_drawing.m_opaque.emplace(floorMesh);
    rScene.m_drawing.m_visible.emplace(floorMesh);

    // Add floor root to hierarchy root
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, rScene.m_hierRoot, floorMesh);

    // Allocate spaces for a few chunks
    rScnRs.m_chunks.m_ids.reserve(32);
    size_t const capacity = rScnRs.m_chunks.m_ids.capacity();
    rScnRs.m_chunks.m_dirty.resize(capacity);
    rScnRs.m_blkTypeUpdates.resize(capacity);
    //rScene.m_chunks.m_blkTypeConnect.resize(capacity);
    rScnRs.m_chunks.m_blkTypes.resize(capacity);
    rScnRs.m_chunks.m_connect.resize(capacity);
    rScnRs.m_chunkDusts.m_dusts.resize(capacity);
    rScnRs.m_chkNotify.resize(capacity);

    // Create single chunk
    rScnRs.m_singleChunk = rScnRs.m_chunks.m_ids.create();
    rScnRs.m_chunks.m_coordToChunk.emplace(ChunkCoord_t{0, 0, 0}, rScnRs.m_singleChunk);
    rScnRs.m_chunks.m_connect.at(0).m_blkPublish.ids_reserve(gc_chunkSize);
    rScnRs.m_chkNotify.at(0).m_notifySlots.at(0).resize(gc_chunkSize);
    rScnRs.m_blkTypeUpdates.at(0).m_changed.resize(gc_chunkSize);

    // Allocate buffers used by the single chunk
    //rScene.m_chunks.m_blkTypeConnect[0][rScene.m_blkTypeIds.id_of("dust")]
    //        .m_connect.resize(gc_chunkSize);
    rScnRs.m_chunkDusts.m_dusts[0]
            = Array<BlkRsDust>{Corrade::DirectInit, gc_chunkSize};

    // Fill single chunk with air
    rScnRs.m_chunks.m_blkTypes[0]
            = Array<BlkTypeId>{Corrade::DirectInit, gc_chunkSize,
                                 g_blkTypeIds.id_of("air")};

    // Keep track of meshes
    for (std::string_view const str : gc_meshsUsed)
    {
        rScnPhys.m_namedMeshs.emplace(str, quick_add_mesh(str));
    }

    osp::ResId const texRes = rResources.find(osp::restypes::gc_texture, pkg, "Redstone/redstone.gltf:unnamed-0");

    rScnRs.m_blockTexture = SysRender::own_texture_resource(rScene.m_drawing, rScene.m_drawingRes, *rScene.m_pResources, texRes);
}

osp::active::ActiveEnt add_mesh_quick(CommonTestScene &rScene, osp::active::ActiveEnt parent, Matrix4 const& tf, osp::active::MeshId mesh)
{
    using namespace osp::active;

    auto &rScnRs = rScene.get<RedstoneTestScene>();

    ActiveEnt ent = rScene.m_activeIds.create();

    // Add cube mesh to cube
    rScene.m_drawing.m_mesh.emplace( ent, rScene.m_drawing.m_meshRefCounts.ref_add(mesh) );
    rScene.m_drawing.m_meshDirty.push_back(ent);

    // Add block texture
    rScene.m_drawing.m_diffuseTex.emplace( ent, rScene.m_drawing.m_texRefCounts.ref_add(rScnRs.m_blockTexture) );
    rScene.m_drawing.m_diffuseDirty.push_back(ent);

    // Add common material to cube
    MaterialData &rMatCommon = rScene.m_drawing.m_materials[rScnRs.m_matFlat];
    rMatCommon.m_comp.emplace(ent);
    rMatCommon.m_added.push_back(ent);

    // Add transform and draw transform
    rScene.m_basic.m_transform.emplace( ent, ACompTransform{tf} );

    // Add opaque and visible component
    rScene.m_drawing.m_opaque.emplace(ent);
    rScene.m_drawing.m_visible.emplace(ent);

    // Add cube to hierarchy, parented to root
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, parent, ent);

    return ent;
}

//-----------------------------------------------------------------------------

void chunk_update_visuals(CommonTestScene &rScene, ChunkId chkId)
{
    using namespace osp::active;

    auto &rScnRs = rScene.get<RedstoneTestScene>();
    auto &rScnPhys = rScene.get<scenes::PhysicsData>();

    TmpChkBlkTypeUpd const& blkTypeUpd = rScnRs.m_blkTypeUpdates[size_t(chkId)];

    ACtxVxRsDusts::ChkDust_t const &rDusts = rScnRs.m_chunkDusts.m_dusts.at(size_t(chkId));

    // Create models

    Vector3 chunkOffset{0}; // TODO

    // Create redstone dust models
    if (auto it = blkTypeUpd.m_perType.find(g_blkTypeIds.id_of("dust"));
        it != blkTypeUpd.m_perType.end())
    {
        for (size_t blockId : it->second.m_added)
        {
            BlkRsDust const dust = rDusts[blockId];
            EMultiDirs const dir = dust.m_connected;

            //int const dirCount = std::bitset<8>(int(dust.m_connected)).count();

            Vector3 const pos = Vector3(index_to_pos(blockId)) + Vector3{0.5, 0.5, 0.5};


            Matrix4 tf = Matrix4::translation(pos);
            bool line = false;

            if (dir == (EMultiDir::POS_X | EMultiDir::NEG_X))
            {
                // X-aligned line
                tf = tf * Matrix4::rotationY(90.0_degf);
                line = true;
            }
            else if (dir == (EMultiDir::POS_Z | EMultiDir::NEG_Z))
            {
                // Z-aligned line
                line = true;
            }

            MeshId const rootMesh = line
                    ? rScnPhys.m_namedMeshs.at("Redstone/redstone.gltf:DustStraight")
                    : rScnPhys.m_namedMeshs.at("Redstone/redstone.gltf:Dust");

            ActiveEnt const meshEnt = add_mesh_quick(rScene, rScene.m_hierRoot, tf, rootMesh);

            if (!line)
            {
                // put meshes for 4 sides
                using side_t = std::pair<EMultiDir, std::string_view>;
                auto const sides =
                {
                    side_t{EMultiDir::POS_X, "Redstone/redstone.gltf:DustLf"},
                    side_t{EMultiDir::NEG_X, "Redstone/redstone.gltf:DustRt"},
                    side_t{EMultiDir::POS_Z, "Redstone/redstone.gltf:DustFd"},
                    side_t{EMultiDir::NEG_Z, "Redstone/redstone.gltf:DustBk"},
                };

                for (side_t const side : sides)
                {
                    if (dir & side.first)
                    {
                        add_mesh_quick(rScene, meshEnt, Matrix4{}, rScnPhys.m_namedMeshs.at(side.second));
                    }
                }
            }

            rScene.m_drawing.m_color.emplace(meshEnt, 0x880000ff_rgbaf);
        }
    }

    // Create redstone torch models
    if (auto it = blkTypeUpd.m_perType.find(g_blkTypeIds.id_of("torch"));
        it != blkTypeUpd.m_perType.end())
    {
        for (size_t blockId : it->second.m_added)
        {
            Vector3 const pos = Vector3(index_to_pos(blockId)) + Vector3{0.5, 0.5, 0.5};
            add_mesh_quick(rScene, rScene.m_hierRoot, Matrix4::translation(pos), rScnPhys.m_namedMeshs.at("Redstone/redstone.gltf:Torch"));
        }
    }
}

void world_update_visuals(CommonTestScene& rScene)
{
    auto &rScnRs = rScene.get<RedstoneTestScene>();

    // Clear all drawing-related dirty flags
    osp::active::SysRender::clear_dirty_all(rScene.m_drawing);

    for (std::size_t chunkId : rScnRs.m_chunks.m_dirty)
    {
        chunk_update_visuals(rScene, ChunkId(chunkId));
    }
}



//-----------------------------------------------------------------------------

/**
 * @brief Update an EngineTestScene, this just rotates the cube
 *
 * @param rScene [ref] scene to update
 */
void update_test_scene(CommonTestScene& rScene, float delta)
{
    using namespace osp::active;

    // Sort hierarchy, required by renderer
    SysHierarchy::sort(rScene.m_basic.m_hierarchy);
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

    ACtxCameraController m_camCtrl;

    osp::shader::ACtxDrawFlat m_flat;
    osp::shader::ACtxDrawMeshVisualizer m_wireframe;

    osp::input::ControlSubscriber m_controls;
    osp::input::EButtonControlIndex m_btnPlaceDust;
    osp::input::EButtonControlIndex m_btnPlaceTorch;
};


ChkBlkPlacements_t place_user_blocks(CommonTestScene& rScene, CommonSceneRendererGL& rRenderer)
{
    auto &rRenderRs = rRenderer.get<RedstoneRenderer>();

    Magnum::Vector3i const tgt{rRenderRs.m_camCtrl.m_target.value()};
    Magnum::Matrix4 const &camTf = rScene.m_basic.m_transform.get(rRenderer.m_camera).m_transform;
    Vector3 const lookDir = -camTf.backward();
    ChkBlkPlacements_t blkPlacements;

    for (osp::input::ButtonControlEvent const& event : rRenderRs.m_controls.button_events())
    {
        using osp::input::EButtonControlEvent;

        if (event.m_index == rRenderRs.m_btnPlaceDust
            && event.m_event == EButtonControlEvent::Triggered)
        {
            // Place dust
            blkPlacements[g_blkTypeIds.id_of("dust")].push_back(
                ChkBlkPlace{ tgt, lookDir }
            );

            OSP_LOG_INFO("Place Dust: ({}, {}, {})", tgt.x(), tgt.y(), tgt.z());
        }

        if (event.m_index == rRenderRs.m_btnPlaceTorch
            && event.m_event == EButtonControlEvent::Triggered)
        {
            // Place torch
            blkPlacements[g_blkTypeIds.id_of("torch")].push_back(
                ChkBlkPlace{ tgt, lookDir }
            );

            OSP_LOG_INFO("Place Torch: ({}, {}, {})", tgt.x(), tgt.y(), tgt.z());
        }
    }
    return blkPlacements;
}

static void sync(CommonSceneRendererGL& rRenderer, CommonTestScene& rScene)
{
    using namespace osp::active;
    using namespace osp::shader;

    auto &rRenderRs = rRenderer.get<RedstoneRenderer>();
    auto &rScnRs = rScene.get<RedstoneTestScene>();

    RenderGroup &rGroupFwdOpaque
            = rRenderer.m_renderGroups.m_groups["fwd_opaque"];

    // Assign Phong shader to entities with the gc_mat_common material, and put
    // results into the fwd_opaque render group
    {
        MaterialData const &rMatFlat = rScene.m_drawing.m_materials[rScnRs.m_matFlat];
        assign_flat(
                rMatFlat.m_added, &rGroupFwdOpaque.m_entities, nullptr,
                rScene.m_drawing.m_opaque, rRenderer.m_renderGl.m_diffuseTexId,
                rRenderRs.m_flat);
        SysRender::assure_draw_transforms(
                    rScene.m_basic.m_hierarchy,
                    rRenderer.m_renderGl.m_drawTransform,
                    std::cbegin(rMatFlat.m_added),
                    std::cend(rMatFlat.m_added));
    }

    // Same thing but with MeshVisualizer and gc_mat_visualizer
    {
        MaterialData const &rMatVisualizer
                = rScene.m_drawing.m_materials[rScene.m_matVisualizer];

        assign_visualizer(
                rMatVisualizer.m_added, rGroupFwdOpaque.m_entities,
                rRenderRs.m_wireframe);
        SysRender::assure_draw_transforms(
                    rScene.m_basic.m_hierarchy,
                    rRenderer.m_renderGl.m_drawTransform,
                    std::cbegin(rMatVisualizer.m_added),
                    std::cend(rMatVisualizer.m_added));
    }
}

void Redstone::setup_renderer_gl(CommonSceneRendererGL& rRenderer, CommonTestScene& rScene, ActiveApplication& rApp) noexcept
{
    using namespace osp::active;
    using namespace osp::shader;

    auto &rRenderRs = rRenderer.emplace<RedstoneRenderer>(rApp);

    // Setup Flat shaders
    rRenderRs.m_flat.m_shaderDiffuse      = Flat{Flat::Flag::Textured | Flat::Flag::AlphaMask};
    rRenderRs.m_flat.m_shaderUntextured   = Flat{};
    rRenderRs.m_flat.assign_pointers(rRenderer.m_renderGl, rApp.get_render_gl());
    rRenderRs.m_flat.m_pColor = &rScene.m_drawing.m_color;

    // Create MeshVisualizer shader intended for wireframe
    rRenderRs.m_wireframe.m_shader
            = MeshVisualizer{ MeshVisualizer::Flag::Wireframe };
    rRenderRs.m_wireframe.m_shader
        .setColor(0x00000000_rgbaf)
        .setWireframeColor(0xffffffffff_rgbaf);
    rRenderRs.m_wireframe.m_wireframeOnly = true;
    rRenderRs.m_wireframe.assign_pointers(
        rRenderer.m_renderGl, rApp.get_render_gl());

    // Select first camera for rendering
    ActiveEnt const camEnt = rScene.m_basic.m_camera.at(0);
    rRenderer.m_camera = camEnt;
    rScene.m_basic.m_camera.get(camEnt).set_aspect_ratio(
            osp::Vector2(Magnum::GL::defaultFramebuffer.viewport().size()));
    SysRender::add_draw_transforms_recurse(
            rScene.m_basic.m_hierarchy,
            rRenderer.m_renderGl.m_drawTransform,
            camEnt);

    rRenderRs.m_camCtrl.m_target = osp::Vector3{};
    rRenderRs.m_camCtrl.m_up = osp::Vector3{0.0f, 1.0f, 0.0f};

    rRenderer.m_onCustomDraw = [] (
            CommonSceneRendererGL& rRenderer, CommonTestScene& rScene,
            ActiveApplication& rApp, float delta) noexcept
    {
        auto &rRenderRs = rRenderer.get<RedstoneRenderer>();
        auto &rScnRs = rScene.get<RedstoneTestScene>();

        sync(rRenderer, rScene);

        ACompTransform &rCamTf = rScene.m_basic.m_transform.get(rRenderer.m_camera);

        // Move camera
        SysCameraController::update_view(rRenderRs.m_camCtrl, rCamTf, delta);
        SysCameraController::update_move(
                rRenderRs.m_camCtrl, rCamTf, delta, true);

        // Move cursor cube to camera target
        Magnum::Vector3i const tgt{rRenderRs.m_camCtrl.m_target.value()};
        ACompTransform &rCubeTf = rScene.m_basic.m_transform.get(rScnRs.m_cube);
        bool const targetValid = (tgt.x() >= 0 && tgt.y() >= 0 && tgt.z() >= 0);
        rCubeTf.m_transform.translation()
                = targetValid
                ? Vector3{tgt} + Vector3{0.5f, 0.5f, 0.5f}
                : Vector3{-2000};

        // Get blocks to place this frame
        ChkBlkPlacements_t blkPlacements = place_user_blocks(rScene, rRenderer);

        // Accumolated list of block changes this frame, so far only 1
        std::array< std::pair<ChunkId, ChkBlkPlacements_t>, 1> changesArray
        {{
            {rScnRs.m_singleChunk, std::move(blkPlacements)}
        }};

        update_blocks(rScnRs, changesArray);

        // # Update models
        // * Syncs voxel data with OSP entities
        world_update_visuals(rScene);

        update_test_scene(rScene, delta);
    };
}

} // namespace testapp::redstone

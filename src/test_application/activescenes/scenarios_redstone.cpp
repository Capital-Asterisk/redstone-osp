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

#include "../ActiveApplication.h"

#include <osp/Active/basic.h>
#include <osp/Active/drawing.h>

#include <osp/Active/SysHierarchy.h>

#include <osp/Active/SysRender.h>
#include <osp/Active/opengl/SysRenderGL.h>

#include <osp/Shaders/Flat.h>
#include <osp/Shaders/Phong.h>
#include <osp/Shaders/MeshVisualizer.h>


#include <longeron/id_management/refcount.hpp>
#include <osp/Resource/Package.h>

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
struct RedstoneTestScene : RedstoneScene
{
    // ID registry generates entity IDs, and keeps track of which ones exist
    lgrn::IdRegistry<osp::active::ActiveEnt> m_activeIds;

    // Components and supporting data structures
    osp::active::ACtxBasic          m_basic;
    osp::active::ACtxDrawing        m_drawing;

    // Hierarchy root, needs to exist so all hierarchy entities are connected
    osp::active::ActiveEnt          m_hierRoot;

    // resources
    std::unordered_map< std::string_view, osp::DependRes<Magnum::Trade::MeshData> > m_meshs;
    osp::DependRes<Magnum::Trade::ImageData2D> m_blockTexture;

    // Crosshair
    osp::active::ActiveEnt          m_cube;
};



entt::any setup_scene(osp::Package &rPkg)
{
    using namespace osp::active;

    entt::any sceneAny = entt::make_any<RedstoneTestScene>();
    RedstoneTestScene &rScene = entt::any_cast<RedstoneTestScene&>(sceneAny);


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
    rScene.m_drawing.m_opaque.emplace(floorMesh);
    rScene.m_drawing.m_visible.emplace(floorMesh);

    // Add floor root to hierarchy root
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, rScene.m_hierRoot, floorMesh);

    // Allocate spaces for a few chunks
    rScene.m_chunks.m_ids.reserve(32);
    size_t const capacity = rScene.m_chunks.m_ids.capacity();
    rScene.m_chunks.m_dirty.resize(capacity);
    rScene.m_blkTypeUpdates.resize(capacity);
    //rScene.m_chunks.m_blkTypeConnect.resize(capacity);
    rScene.m_chunks.m_blkTypes.resize(capacity);
    rScene.m_chunks.m_connect.resize(capacity);
    rScene.m_chunkDusts.m_dusts.resize(capacity);
    rScene.m_chkNotify.resize(capacity);

    // Create single chunk
    rScene.m_singleChunk = rScene.m_chunks.m_ids.create();
    rScene.m_chunks.m_coordToChunk.emplace(ChunkCoord_t{0, 0, 0}, rScene.m_singleChunk);
    rScene.m_chunks.m_connect.at(0).m_blkPublish.ids_reserve(gc_chunkSize);
    rScene.m_chkNotify.at(0).m_notifySlots.at(0).resize(gc_chunkSize);
    rScene.m_blkTypeUpdates.at(0).m_changed.resize(gc_chunkSize);

    // Allocate buffers used by the single chunk
    //rScene.m_chunks.m_blkTypeConnect[0][rScene.m_blkTypeIds.id_of("dust")]
    //        .m_connect.resize(gc_chunkSize);
    rScene.m_chunkDusts.m_dusts[0]
            = Array<BlkRsDust>{Corrade::DirectInit, gc_chunkSize};

    // Fill single chunk with air
    rScene.m_chunks.m_blkTypes[0]
            = Array<BlkTypeId>{Corrade::DirectInit, gc_chunkSize,
                                 g_blkTypeIds.id_of("air")};

    // Keep track of meshes
    for (std::string_view const str : gc_meshsUsed)
    {
        rScene.m_meshs.emplace(str, rPkg.get<MeshData>(str));
    }

    rScene.m_blockTexture = rPkg.get<ImageData2D>("textures");

    return std::move(sceneAny);
}

osp::active::ActiveEnt add_mesh_quick(RedstoneTestScene& rScene, osp::active::ActiveEnt parent, Matrix4 const& tf, osp::DependRes<MeshData> mesh)
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

    // Add opaque and visible component
    rScene.m_drawing.m_opaque.emplace(ent);
    rScene.m_drawing.m_visible.emplace(ent);

    // Add cube to hierarchy, parented to root
    SysHierarchy::add_child(
            rScene.m_basic.m_hierarchy, parent, ent);

    return ent;
}

//-----------------------------------------------------------------------------

void chunk_update_visuals(RedstoneTestScene& rScene, ChunkId chkId)
{
    using namespace osp::active;

    TmpChkBlkTypeUpd const& blkTypeUpd = rScene.m_blkTypeUpdates[size_t(chkId)];

    ACtxVxRsDusts::ChkDust_t const &rDusts = rScene.m_chunkDusts.m_dusts.at(size_t(chkId));

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

            osp::DependRes<MeshData> rootMesh = line
                    ? rScene.m_meshs.at("Redstone/redstone:DustStraight")
                    : rScene.m_meshs.at("Redstone/redstone:Dust");

            ActiveEnt const meshEnt = add_mesh_quick(rScene, rScene.m_hierRoot, tf, rootMesh);

            if (!line)
            {
                // put meshes for 4 sides
                using side_t = std::pair<EMultiDir, std::string_view>;
                auto const sides =
                {
                    side_t{EMultiDir::POS_X, "Redstone/redstone:DustLf"},
                    side_t{EMultiDir::NEG_X, "Redstone/redstone:DustRt"},
                    side_t{EMultiDir::POS_Z, "Redstone/redstone:DustFd"},
                    side_t{EMultiDir::NEG_Z, "Redstone/redstone:DustBk"},
                };

                for (side_t const side : sides)
                {
                    if (dir & side.first)
                    {
                        add_mesh_quick(rScene, meshEnt, Matrix4{}, rScene.m_meshs.at(side.second));
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
            add_mesh_quick(rScene, rScene.m_hierRoot, Matrix4::translation(pos), rScene.m_meshs.at("Redstone/redstone:Torch"));
        }
    }
}

void world_update_visuals(RedstoneTestScene& rScene)
{
    for (std::size_t chunkId : rScene.m_chunks.m_dirty)
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
void update_test_scene(RedstoneTestScene& rScene, float delta)
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

    osp::active::ACtxRenderGroups m_renderGroups{};

    osp::active::ACtxSceneRenderGL m_renderGl;

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
        ActiveApplication& rApp, RedstoneTestScene const& rScene,
        RedstoneRenderer& rRenderer)
{
    using namespace osp::active;
    using namespace osp::shader;
    using Magnum::GL::Renderer;
    using Magnum::GL::Framebuffer;
    using Magnum::GL::FramebufferClear;
    using Magnum::GL::Texture2D;


    // Load any required meshes
    SysRenderGL::compile_meshes(
            rScene.m_drawing.m_mesh, rScene.m_drawing.m_meshDirty,
            rRenderer.m_renderGl.m_meshId, rApp.get_render_gl());

    // Load any required textures
    SysRenderGL::compile_textures(
            rScene.m_drawing.m_diffuseTex, rScene.m_drawing.m_diffuseDirty,
            rRenderer.m_renderGl.m_diffuseTexId, rApp.get_render_gl());

    RenderGroup &rGroupFwdOpaque
            = rRenderer.m_renderGroups.m_groups["fwd_opaque"];
    RenderGroup &rGroupFwdTransparent
            = rRenderer.m_renderGroups.m_groups["fwd_transparent"];

    // Assign Flat shader to entities with the gc_mat_flat material, and put
    // results into the fwd_opaque render group
    {
        MaterialData const &rMatFlat = rScene.m_drawing.m_materials[gc_mat_flat];
        assign_flat(
                rMatFlat.m_added,
                &rGroupFwdOpaque.m_entities,
                &rGroupFwdTransparent.m_entities,
                rScene.m_drawing.m_opaque,
                rRenderer.m_renderGl.m_diffuseTexId,
                rRenderer.m_flat);
        SysRender::assure_draw_transforms(
                    rScene.m_basic.m_hierarchy,
                    rRenderer.m_renderGl.m_drawTransform,
                    std::cbegin(rMatFlat.m_added),
                    std::cend(rMatFlat.m_added));
    }

    // Assign Phong shader to entities with the gc_mat_common material, and put
    // results into the fwd_opaque render group
    {
        MaterialData const &rMatCommon = rScene.m_drawing.m_materials[gc_mat_common];
        assign_phong(
                rMatCommon.m_added,
                &rGroupFwdOpaque.m_entities,
                &rGroupFwdTransparent.m_entities,
                rScene.m_drawing.m_opaque,
                rRenderer.m_renderGl.m_diffuseTexId,
                rRenderer.m_phong);
        SysRender::assure_draw_transforms(
                    rScene.m_basic.m_hierarchy,
                    rRenderer.m_renderGl.m_drawTransform,
                    std::cbegin(rMatCommon.m_added),
                    std::cend(rMatCommon.m_added));
    }

    // Same thing but with MeshVisualizer and gc_mat_visualizer
    {
        MaterialData const &rMatVisualizer
                = rScene.m_drawing.m_materials[gc_mat_visualizer];
        assign_visualizer(
                rMatVisualizer.m_added,
                rGroupFwdTransparent.m_entities,
                rRenderer.m_visualizer);
        SysRender::assure_draw_transforms(
                    rScene.m_basic.m_hierarchy,
                    rRenderer.m_renderGl.m_drawTransform,
                    std::cbegin(rMatVisualizer.m_added),
                    std::cend(rMatVisualizer.m_added));
    }

    // Same thing but with MeshVisualizer and gc_mat_visualizer
    {
        MaterialData const &rMatWireframe
                = rScene.m_drawing.m_materials[gc_mat_wireframe];
        assign_visualizer(
                rMatWireframe.m_added,
                rGroupFwdTransparent.m_entities,
                rRenderer.m_wireframe);
        SysRender::assure_draw_transforms(
                    rScene.m_basic.m_hierarchy,
                    rRenderer.m_renderGl.m_drawTransform,
                    std::cbegin(rMatWireframe.m_added),
                    std::cend(rMatWireframe.m_added));
    }

    // Calculate hierarchy transforms
    SysRender::update_draw_transforms(
            rScene.m_basic.m_hierarchy,
            rScene.m_basic.m_transform,
            rRenderer.m_renderGl.m_drawTransform);

    // Get camera, and calculate projection matrix and inverse transformation
    ACompCamera const &rCamera = rScene.m_basic.m_camera.get(rRenderer.m_camera);
    ACompDrawTransform const &cameraDrawTf
            = rRenderer.m_renderGl.m_drawTransform.get(rRenderer.m_camera);
    ViewProjMatrix viewProj{
            cameraDrawTf.m_transformWorld.inverted(),
            rCamera.calculate_projection()};

    // Bind offscreen FBO
    Framebuffer &rFbo = rApp.get_render_gl().m_fbo;
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
            rScene.m_drawing.m_visible, viewProj);

    // Forward Render fwd_transparent group to FBO
    SysRenderGL::render_transparent(
            rRenderer.m_renderGroups.m_groups.at("fwd_transparent"),
            rScene.m_drawing.m_visible, viewProj);

    // Display FBO
    Texture2D &rFboColor = rApp.get_render_gl().m_texGl.get(rApp.get_render_gl().m_fboColor);
    SysRenderGL::display_texture(rApp.get_render_gl(), rFboColor);
}

ChkBlkPlacements_t place_user_blocks(RedstoneTestScene &rScene, RedstoneRenderer& rRenderer)
{
    Magnum::Vector3i const tgt{rRenderer.m_camCtrl.m_target.value()};
    Magnum::Matrix4 const &camTf = rScene.m_basic.m_transform.get(rRenderer.m_camera).m_transform;
    Vector3 const lookDir = -camTf.backward();
    ChkBlkPlacements_t blkPlacements;

    for (osp::input::ButtonControlEvent const& event : rRenderer.m_controls.button_events())
    {
        using osp::input::EButtonControlEvent;

        if (event.m_index == rRenderer.m_btnPlaceDust
            && event.m_event == EButtonControlEvent::Triggered)
        {
            // Place dust
            blkPlacements[g_blkTypeIds.id_of("dust")].push_back(
                ChkBlkPlace{ tgt, lookDir }
            );

            OSP_LOG_INFO("Place Dust: ({}, {}, {})", tgt.x(), tgt.y(), tgt.z());
        }

        if (event.m_index == rRenderer.m_btnPlaceTorch
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


on_draw_t gen_draw(RedstoneTestScene& rScene, ActiveApplication& rApp)
{
    using namespace osp::active;
    using namespace osp::shader;


    // Create renderer data. This uses a shared_ptr to allow being stored
    // inside an std::function, which require copyable types
    std::shared_ptr<RedstoneRenderer> pRenderer
            = std::make_shared<RedstoneRenderer>(rApp);

    // Setup Flat shaders
    pRenderer->m_flat.m_shaderDiffuse      = Flat{Flat::Flag::Textured | Flat::Flag::AlphaMask};
    pRenderer->m_flat.m_shaderUntextured   = Flat{};
    pRenderer->m_flat.assign_pointers(pRenderer->m_renderGl, rApp.get_render_gl());
    pRenderer->m_flat.m_pColor = &rScene.m_drawing.m_color;

    // Create Phong shaders
    auto const texturedFlags
            = Phong::Flag::DiffuseTexture | Phong::Flag::AlphaMask
            | Phong::Flag::AmbientTexture;
    pRenderer->m_phong.m_shaderDiffuse      = Phong{texturedFlags, 2};
    pRenderer->m_phong.m_shaderUntextured   = Phong{{}, 2};
    pRenderer->m_phong.assign_pointers(
            pRenderer->m_renderGl, rApp.get_render_gl());

    // Create MeshVisualizer shader
    pRenderer->m_visualizer.m_shader
            = MeshVisualizer{ MeshVisualizer::Flag::Wireframe };
    pRenderer->m_visualizer.m_shader
        .setColor(0x888888ff_rgbaf)
        .setWireframeColor(0xffffffff_rgbaf);
    pRenderer->m_visualizer.assign_pointers(
            pRenderer->m_renderGl, rApp.get_render_gl());

    // Create MeshVisualizer shader intended for wireframe
    pRenderer->m_wireframe.m_shader
            = MeshVisualizer{ MeshVisualizer::Flag::Wireframe };
    pRenderer->m_wireframe.m_shader
        .setColor(0x00000000_rgbaf)
        .setWireframeColor(0xffffffffff_rgbaf);
    pRenderer->m_wireframe.m_wireframeOnly = true;
    pRenderer->m_visualizer.assign_pointers(
            pRenderer->m_renderGl, rApp.get_render_gl());

    // Select first camera for rendering
    ActiveEnt const camEnt = rScene.m_basic.m_camera.at(0);
    pRenderer->m_camera = camEnt;
    rScene.m_basic.m_camera.get(camEnt).set_aspect_ratio(
            osp::Vector2(Magnum::GL::defaultFramebuffer.viewport().size()));
    SysRender::add_draw_transforms_recurse(
            rScene.m_basic.m_hierarchy,
            pRenderer->m_renderGl.m_drawTransform,
            camEnt);

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

        // Move camera
        SysCameraController::update_view(pRenderer->m_camCtrl, rCamTf, delta);
        SysCameraController::update_move(
                pRenderer->m_camCtrl, rCamTf, delta, true);

        // Move cursor cube to camera target
        Magnum::Vector3i const tgt{pRenderer->m_camCtrl.m_target.value()};
        ACompTransform &rCubeTf = rScene.m_basic.m_transform.get(rScene.m_cube);
        bool const targetValid = (tgt.x() >= 0 && tgt.y() >= 0 && tgt.z() >= 0);
        rCubeTf.m_transform.translation()
                = targetValid
                ? Vector3{tgt} + Vector3{0.5f, 0.5f, 0.5f}
                : Vector3{-2000};

        // Get blocks to place this frame
        ChkBlkPlacements_t blkPlacements = place_user_blocks(rScene, *pRenderer);

        // Accumolated list of block changes this frame, so far only 1
        std::array< std::pair<ChunkId, ChkBlkPlacements_t>, 1> changesArray
        {{
            {rScene.m_singleChunk, std::move(blkPlacements)}
        }};

        update_blocks(rScene, changesArray);

        // # Update models
        // * Syncs voxel data with OSP entities
        world_update_visuals(rScene);

        update_test_scene(rScene, delta);

        render_test_scene(rApp, rScene, *pRenderer);

        SysRender::clear_dirty_materials(rScene.m_drawing.m_materials);
        rScene.m_drawing.m_diffuseDirty.clear();
        rScene.m_drawing.m_meshDirty.clear();
    };
}

} // namespace testapp::redstone

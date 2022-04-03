/**
 * Open Space Program
 * Copyright © 2019-2022 Open Space Program Project
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

#include "load_tinygltf.h"

#include "ImporterData.h"
#include "resources.h"

#include "../logging.h"
#include "../string_concat.h"

#include <MagnumPlugins/TinyGltfImporter/TinyGltfImporter.h>
#include <MagnumExternal/TinyGltf/tiny_gltf.h>

#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/TextureData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/SceneData.h>

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Containers/StringStlView.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/PairStl.h>

using namespace osp;

using Magnum::Trade::TinyGltfImporter;

using Magnum::Trade::ImageData2D;
using Magnum::Trade::TextureData;
using Magnum::Trade::MeshData;
using Magnum::Trade::MaterialData;
using Magnum::Trade::SceneData;
using Magnum::Trade::SceneField;

using Magnum::Int;
using Magnum::UnsignedInt;

using PluginManager = Corrade::PluginManager::Manager<Magnum::Trade::AbstractImporter>;
using Corrade::Containers::Optional;
using Corrade::Containers::Pointer;
using Corrade::Containers::Array;
using Corrade::Containers::StridedArrayView1D;
using Corrade::Containers::Pair;

using TinyGltfNodeExtras_t = std::vector<tinygltf::Value>;

void osp::register_tinygltf_resources(Resources &rResources)
{
    rResources.data_register<TinyGltfNodeExtras_t>(restypes::gc_importer);
}

static void load_gltf(TinyGltfImporter &rImporter, ResId res, std::string_view name, Resources &rResources, PkgId pkg)
{
    ImporterData &rImportData = rResources.data_add<ImporterData>(restypes::gc_importer, res);

    // Combine resource names. Maybe make this customizable
    // ie: name = "dir/file.gltf" and resName = "mytexture"
    // "dir/file.gltf:mytexture"
    // "unnamed-[id]" is used as the resource name if it's empty
    auto format_name = [tempStr = std::string{}, name]
            (std::string_view resName, UnsignedInt id) mutable -> std::string const&
    {
        tempStr.clear();
        if ( ! resName.empty())
        {
            string_append(tempStr, name, ":", resName);
        }
        else
        {
            // i don't like std::to_string but screw it, fix it later
            string_append(tempStr, name, ":unnamed-", std::to_string(id));
        }

        return tempStr;
    };

    using namespace restypes;

    // Store images
    rImportData.m_images.resize(rImporter.image2DCount());
    for (UnsignedInt i = 0; i < rImporter.image2DCount(); i ++)
    {
        Optional<ImageData2D> img = rImporter.image2D(i);

        if ( ! bool(img) )
        {
            continue;
        }

        // Create and keep track of resource Id
        ResId const imgRes = rResources.create(gc_image, pkg, format_name(rImporter.image2DName(i), i));
        rImportData.m_images[i] = rResources.owner_create(gc_image, imgRes);

        // Add image data to resource
        rResources.data_add<ImageData2D>(gc_image, imgRes, std::move(*img));
    }

    // Store textures
    rImportData.m_textures.resize(rImporter.textureCount());
    for (UnsignedInt i = 0; i < rImporter.textureCount(); i ++)
    {
        Optional<TextureData> tex = rImporter.texture(i);

        if ( ! bool(tex) )
        {
            continue;
        }

        // Create and keep track of resource Id
        ResId const texRes = rResources.create(gc_texture, pkg, format_name(rImporter.textureName(i), i));
        rImportData.m_textures[i] = rResources.owner_create(gc_texture, texRes);

        // Add data to resource
        rResources.data_add<TextureData>(gc_texture, texRes, std::move(*tex));

        // Keep track of which image this texture uses
        if (ResIdOwner_t const& imgRes = rImportData.m_images.at(tex->image());
            imgRes.has_value())
        {
            ResIdOwner_t imgOwner = rResources.owner_create(gc_image, imgRes);
            rResources.data_add<TextureImgSource>(gc_texture, texRes, TextureImgSource{std::move(imgOwner)} );
        }
    }

    // Store meshes
    rImportData.m_meshes.resize(rImporter.meshCount());
    for (UnsignedInt i = 0; i < rImporter.meshCount(); i ++)
    {
        Optional<MeshData> mesh = rImporter.mesh(i);

        if ( ! bool(mesh) )
        {
            continue;
        }

        ResId const meshRes = rResources.create(gc_mesh, pkg, format_name(rImporter.meshName(i), i));
        rResources.data_add<MeshData>(gc_mesh, meshRes, std::move(*mesh));
        rImportData.m_meshes[i] = rResources.owner_create(gc_mesh, meshRes);
    }

    // Store materials
    rImportData.m_materials.reserve(rImporter.materialCount());
    for (UnsignedInt i = 0; i < rImporter.materialCount(); i ++)
    {
        rImportData.m_materials.emplace_back(rImporter.material(i));
    }

    // Store various node data

    auto &rNodeExtras = rResources.data_add<TinyGltfNodeExtras_t>(
                                restypes::gc_importer, res);
    UnsignedInt parentsSet = 0;

    UnsignedInt const objCount = rImporter.objectCount();
    rNodeExtras                 .resize(objCount);
    rImportData.m_objNames      .resize(objCount);
    rImportData.m_objMeshes     .resize(objCount, -1);
    rImportData.m_objMaterials  .resize(objCount, -1);
    rImportData.m_objTransforms .resize(objCount);
    rImportData.m_objParents    .resize(objCount, -1);

    // Keep track of children. temporary as rImportData.m_objChildren stores
    // child count
    Array<int> objChildCount(Corrade::ValueInit, objCount);

    // Allocate for storing top-level nodes
    rImportData.m_scnTopLevel.ids_reserve(rImporter.sceneCount());
    rImportData.m_scnTopLevel.data_reserve(rImporter.objectCount());
    std::vector<int> topLevel;
    topLevel.reserve(rImporter.objectCount());

    // Iterate all objects
    for (UnsignedInt obj = 0; obj < rImporter.objectCount(); obj ++)
    {
        rImportData.m_objNames[obj] = rImporter.objectName(obj);
    }

    // Iterate scenes and their objects
    for (UnsignedInt scn = 0; scn < rImporter.sceneCount(); scn ++)
    {
        Optional<SceneData> const scene = rImporter.scene(scn);

        if ( ! bool(scene))
        {
            rImportData.m_scnTopLevel.emplace(scn, {});
            continue;
        }

        // Iterate scene objects with parents

        {
            StridedArrayView1D<UnsignedInt const> const parentsMap
                    = scene->mapping<UnsignedInt>(SceneField::Parent);
            StridedArrayView1D<Int const> const parents
                    = scene->field<Int>(SceneField::Parent);

            for (UnsignedInt j = 0; j < parentsMap.size(); j ++)
            {
                UnsignedInt const   obj         = parentsMap[j];
                Int const           objParent   = parents[j];

                // Store object parents
                rImportData.m_objParents[obj] = objParent;

                if (objParent != -1)
                {
                    objChildCount[objParent] ++;
                }
                else
                {
                    topLevel.push_back(obj);
                }

                // Also store transforms here
                if (Optional<Matrix4> objTf = scene->transformation3DFor(obj);
                    bool(objTf))
                {
                    rImportData.m_objTransforms[obj] = *objTf;
                }
            }

            // Store top-level objects
            rImportData.m_scnTopLevel.emplace(
                    scn, std::begin(topLevel), std::end(topLevel));
            topLevel.clear();
        }

        // Iterate scene objects with meshes and materials
        {

            StridedArrayView1D<UnsignedInt const> const meshMap
                    = scene->mapping<UnsignedInt>(SceneField::Mesh);

            // Assign meshes if present
            if (Optional<UnsignedInt> const meshesFieldId
                        = scene->findFieldId(SceneField::Mesh);
                bool(meshesFieldId))
            {
                StridedArrayView1D<UnsignedInt const> const meshes
                        = scene->field<UnsignedInt>(SceneField::Mesh);
                for (UnsignedInt j = 0; j < meshMap.size(); j ++)
                {
                    rImportData.m_objMeshes[meshMap[j]] = meshes[j];
                }
            }

            // Assign materials if present
            if (Optional<UnsignedInt> const matsFieldId
                        = scene->findFieldId(SceneField::MeshMaterial);
                bool(matsFieldId))
            {
                StridedArrayView1D<Int const> const materials
                        = scene->field<Int>(SceneField::MeshMaterial);

                for (UnsignedInt j = 0; j < meshMap.size(); j ++)
                {
                    rImportData.m_objMaterials[meshMap[j]] = materials[j];
                }
            }

        }

    }

    // Store parent to children multimap
    rImportData.m_objChildren.ids_reserve(objCount);
    rImportData.m_objChildren.data_reserve(objCount);

    // Allocate partitions for all objects with children, initialize to -1
    for (UnsignedInt obj = 0; obj < objCount; obj ++)
    {
        if (int childCount = objChildCount[obj];
            childCount != 0)
        {
            int *pChildren = rImportData.m_objChildren.emplace(obj, childCount);
            std::fill_n(pChildren, childCount, -1);
        }
    }

    // Add children to their parent's list of children
    for (UnsignedInt obj = 0; obj < objCount; obj ++)
    {
        if (int objParent = rImportData.m_objParents[obj];
            objParent != -1)
        {
            // Get parent's span of children
            auto siblings = rImportData.m_objChildren[objParent];

            // Linear search for an empty spot (-1)
            auto pSpot = std::find(std::begin(siblings), std::end(siblings), -1);
            assert(pSpot != std::end(siblings));

            *pSpot = obj; // add self to parent's children
        }
    }
}


ResId osp::load_tinygltf_file(std::string_view filepath, Resources &rResources, PkgId pkg)
{
    PluginManager pluginManager;


    // Create Importer resource
    ResId const res = rResources.create(restypes::gc_importer, pkg, filepath);
    TinyGltfImporter importer{pluginManager};

    importer.openFile(filepath);

    if (!importer.isOpened() || importer.defaultScene() == -1)
    {
        // TODO: delete resource (not yet implemented)
        OSP_LOG_ERROR("Could not open file {}", filepath);
        return lgrn::id_null<ResId>();
    }

    load_gltf(importer, res, filepath, rResources, pkg);

    importer.close();

    return res;
}


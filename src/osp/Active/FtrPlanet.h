#pragma once


#include "../scene.h"
#include "../Resource/PlanetData.h"

#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/AbstractFeature.h>
#include <Magnum/GL/Mesh.h>

#include <Newton.h>

using Magnum::SceneGraph::AbstractFeature3D;

namespace osp
{

//template <class PlanetMeshGen, class PlanetColliderGen>
class FtrPlanet: public AbstractFeature3D
{
public:
    explicit FtrPlanet(Object3D& object, PlanetData& data);

    ~FtrPlanet();
};

}

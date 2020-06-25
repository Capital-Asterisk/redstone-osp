#pragma once

#include <vector>

#include <Magnum/Math/Color.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Shaders/Phong.h>

#include <Newton.h>

#include "../UserInputHandler.h"

#include "../../types.h"
#include "activetypes.h"

#include "SysNewton.h"
#include "SysMachine.h"
#include "SysWire.h"
#include "SysDebugObject.h"


namespace osp
{

using MapSysMachine = std::map<std::string,
                               std::unique_ptr<AbstractSysMachine>>;

/**
 * Component for transformation (in meters)
 */
struct CompTransform
{
    //Matrix4 m_transformPrev;
    Matrix4 m_transform;
    Matrix4 m_transformWorld;
    bool m_enableFloatingOrigin;
};

struct CompHierarchy
{
    std::string m_name;

    //unsigned m_childIndex;
    unsigned m_level{0};
    ActiveEnt m_parent{entt::null};
    ActiveEnt m_siblingNext{entt::null};
    ActiveEnt m_siblingPrev{entt::null};

    // as a parent
    unsigned m_childCount{0};
    ActiveEnt m_childFirst{entt::null};

    // transform relative to parent

};

struct CompCollider
{
    ColliderType m_type;
    // TODO: mesh data
};

/**
 * Component that represents a camera
 */
struct CompCamera
{
    float m_near, m_far;
    Magnum::Deg m_fov;
    Vector2 m_viewport;

    Matrix4 m_projection;

    void calculate_projection();
};

struct CompDrawableDebug
{
    Magnum::GL::Mesh* m_mesh;
    Magnum::Shaders::Phong* m_shader;
    Magnum::Color4 m_color;
};

/**
 * The 3D Game Engine scene. ActiveScene implements a scene graph hierarchy and
 * contains a bunch of other systems.
 */
class ActiveScene
{

public:
    ActiveScene(UserInputHandler &userInput);
    ~ActiveScene();

    /**
     * @return Root entity of the entire scene graph
     */
    constexpr ActiveEnt hier_get_root() { return m_root; }

    /**
     * Create a new entity, and add a CompHierarchy to it
     * @param parent [in] Entity to assign as parent
     * @param name   [in] Name of entity
     * @return New entity created
     */
    ActiveEnt hier_create_child(ActiveEnt parent,
                                std::string const& name = "Innocent Entity");

    /**
     * (NOT YET IMPLEMENTED) set parent-child relationship between two nodes
     * containing a CompHierarchy
     * @param parent
     * @param child
     */
    void hier_set_parent_child(ActiveEnt parent, ActiveEnt child);

    /**
     * @return Internal entt::registry
     */
    constexpr entt::registry& get_registry() { return m_registry; }

    /**
     * Shorthand for get_registry().get<T>()
     * @tparam T Component to get
     * @return Reference to component
     */
    template<class T>
    constexpr T& reg_get(ActiveEnt ent) { return m_registry.get<T>(ent); };

    /**
     * Shorthand for get_registry().emplace<T>()
     * @tparam T Component to emplace
     */
    template<class T, typename... Args>
    constexpr T& reg_emplace(ActiveEnt ent, Args &&... args)
    {
        return m_registry.emplace<T>(ent, args...);
    }

    /**
     * Update everything in the update order, including all systems and stuff
     */
    void update();

    /**
     * Update the m_transformWorld of entities with CompTransform and
     * CompHierarchy. Intended for physics interpolation
     */
    void update_hierarchy_transforms();

    /**
     * Request a floating origin mass translation. Multiple calls to this are
     * accumulated and are applied on the next physics update
     * @param pAmount Meters to translate
     */
    void floating_origin_translate(Vector3 const& pAmount);

    /**
     * @return Accumulated total of floating_origin_translate
     */
    constexpr Vector3 const& floating_origin_get_total() { return m_floatingOriginTranslate; }

    /**
     * Attempt ro perform translations on this frame. Will do nothing if the
     * floating origin total (m_floatingOriginTranslate) is 0
     */
    void floating_origin_translate_begin();

    /**
     * @return True if a floating origin translation is being performed this
     *         frame
     */
    constexpr bool floating_origin_in_progress() { return m_floatingOriginInProgress; }

    /**
     * Calculate transformations relative to camera, and draw every
     * CompDrawableDebug
     * @param camera [in] Entity containing a CompCamera
     */
    void draw(ActiveEnt camera);

    constexpr UpdateOrder& get_update_order() { return m_updateOrder; }

    /**
     * Get one of the system members
     * @tparam T either SysPhysics, SysWire, or SysDebugObject
     */
    template<class T>
    constexpr T& get_system();

    /**
     * Get a registered SysMachine by name. This accesses a map.
     * @param name [in] Name used as a key
     * @return Pointer to specified SysMachine, nullptr if not found
     */
    AbstractSysMachine* get_system_machine(std::string const& name);

private:

    void on_hierarchy_construct(entt::registry& reg, ActiveEnt ent);
    void on_hierarchy_destruct(entt::registry& reg, ActiveEnt ent);


    //std::vector<std::vector<ActiveEnt> > m_hierLevels;
    entt::basic_registry<ActiveEnt> m_registry;
    ActiveEnt m_root;
    bool m_hierarchyDirty;

    Vector3 m_floatingOriginTranslate;
    bool m_floatingOriginInProgress;

    //UserInputHandler &m_userInput;
    //std::vector<std::reference_wrapper<AbstractSysMachine>> m_update_sensor;
    //std::vector<std::reference_wrapper<AbstractSysMachine>> m_update_physics;

    UpdateOrder m_updateOrder;

    MapSysMachine m_sysMachines;

    // TODO: base class and a list for Systems (or not)
    SysPhysics m_physics;
    SysWire m_wire;
    SysDebugObject m_debugObj;
    //std::tuple<SysPhysics, SysWire, SysDebugObject> m_systems;

};

// TODO: There's probably a better way to do these:

template<>
constexpr SysPhysics& ActiveScene::get_system<SysPhysics>()
{
    return m_physics;
}

template<>
constexpr SysWire& ActiveScene::get_system<SysWire>()
{
    return m_wire;
}

template<>
constexpr SysDebugObject& ActiveScene::get_system<SysDebugObject>()
{
    return m_debugObj;
}

}

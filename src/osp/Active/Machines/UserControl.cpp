#include <iostream>

#include "../ActiveScene.h"
#include "UserControl.h"

namespace osp
{

MachineUserControl::MachineUserControl(ActiveEnt &ent) :
    Machine(ent),
    //m_woTestPropagate(this, "TestOut", &MachineUserControl::propagate_test),
    m_woTestPropagate(this, "TestOut", m_wiTest),
    m_woThrottle(this, "Throttle"),
    m_wiTest(this, "Test")
{
    //m_woTestPropagate.propagate();
    //m_enable = true;
    m_woThrottle.value() = wiretype::Percent{0.0f};
}

MachineUserControl::MachineUserControl(MachineUserControl&& move) :
    Machine(std::move(move)),
    m_woTestPropagate(this, std::move(move.m_woTestPropagate)),
    m_woThrottle(this, std::move(move.m_woThrottle)),
    m_wiTest(this, std::move(move.m_wiTest))
{
    //m_enable = true;
    m_woThrottle.value() = wiretype::Percent{0.0f};
}

void MachineUserControl::propagate_output(WireOutput* output)
{
    std::cout << "propagate test: " << output->get_name() << "\n";
}

WireInput* MachineUserControl::request_input(WireInPort port)
{
    return existing_inputs()[port];
}

WireOutput* MachineUserControl::request_output(WireOutPort port)
{
    return existing_outputs()[port];
}

std::vector<WireInput*> MachineUserControl::existing_inputs()
{
    return {&m_wiTest};
}

std::vector<WireOutput*> MachineUserControl::existing_outputs()
{
    return {&m_woThrottle, &m_woTestPropagate};
}

SysMachineUserControl::SysMachineUserControl(ActiveScene &scene, UserInputHandler& userControl) :
    SysMachine<SysMachineUserControl, MachineUserControl>(scene),
    m_throttleMax(userControl.config_get("game_thr_max")),
    m_throttleMin(userControl.config_get("game_thr_min")),
    m_selfDestruct(userControl.config_get("game_self_destruct")),
    m_updateSensor(scene.get_update_order(), "mach_usercontrol", "", "wire",
                   std::bind(&SysMachineUserControl::update_sensor, this))
{

}

void SysMachineUserControl::update_sensor()
{
    //std::cout << "updating all MachineUserControls\n";
    // InputDevice.IsActivated()
    // Combination
    

    if (m_selfDestruct.triggered())
    {
        std::cout << "EXPLOSION BOOM!!!!\n";
    }


    for (MachineUserControl& machine : m_machines)
    {
        //if (!machine.m_enable)
        //{
        //    continue;
        //}


        if (m_throttleMin.triggered())
        {
            //std::cout << "throttle min\n";
            std::get<wiretype::Percent>(machine.m_woThrottle.value()).m_value = 0.0f;
        }

        if (m_throttleMax.triggered())
        {
            //std::cout << "throttle max\n";
            std::get<wiretype::Percent>(machine.m_woThrottle.value()).m_value = 1.0f;
        }
        //std::cout << "updating control\n";
    }
}

Machine& SysMachineUserControl::instantiate(ActiveEnt ent)
{
    return emplace(ent);
}

}

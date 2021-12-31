#pragma once

#include <osp/id_registry.h>
#include <entt/entity/storage.hpp>

#include <Corrade/Containers/ArrayViewStl.h>


namespace redstonesim
{

using Corrade::Containers::ArrayView;
using HierBitset_t = osp::HierarchicalBitset<uint64_t>;

enum class RsNodeId : uint32_t { };
enum class RsElemId : uint32_t { };
enum class RsElemLocalId : uint32_t { };
enum class RsElemTypeId : uint8_t { };

using RsNodeRefCount_t = osp::IdRefCount<RsNodeId>;
using RsNodeSto_t = RsNodeRefCount_t::Storage_t;

template<typename T>
using element_storage_t = entt::basic_storage<RsElemId, T>;

using power_level_t = uint8_t; // 0 to 15



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
        ArrayView<HierBitset_t::Iterator>                       writes,
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
        ArrayView<HierBitset_t::Iterator>   localUpd,
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
    osp::IdRegistry<RsNodeId> m_nodeIds;
    osp::IdRefCount<RsNodeId> m_nodeRefCounts;

    std::vector<power_level_t> m_nodePowers;
    std::vector<RsNodeConnect> m_nodeConnections;

    int m_maxTypes;
    ElemIdReg_t m_elementIds;
    std::vector<RsElemTypeId> m_elementTypes;
    std::vector<RsElemLocalId> m_elementLocals;

    ElemLocIdReg_t m_dustIds;
    std::vector<RsDust> m_dustData;
};

}

/**
 * Open Space Program
 * Copyright © 2019-2020 Open Space Program Project
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
#pragma once

#include <Magnum/Math/Color.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Shaders/Phong.h>

#include "../types.h"
#include "activetypes.h"

namespace osp::active
{

struct CompDrawableDebug
{
    Magnum::GL::Mesh* m_mesh;
    std::vector<Magnum::GL::Texture2D*> m_textures;
    Magnum::Shaders::Phong* m_shader;
    Magnum::Color4 m_color;
};

class SysDebugRender : public IDynamicSystem
{
public:

    static const std::string smc_name;

    SysDebugRender(ActiveScene &rScene);
    ~SysDebugRender() = default;

    void draw(ACompCamera const& camera);

private:
    ActiveScene &m_scene;

    RenderOrderHandle m_renderDebugDraw;
};

}

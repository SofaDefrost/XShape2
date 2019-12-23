/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
/*****************************************************************************
 * Contributors:
 *      erwan.douaille@inria.fr
 *      damien.marchal@univ-lille1.fr
 *****************************************************************************/
#pragma once

#include <SofaImplicitField3/config.h>

#include <sofa/core/visual/VisualModel.h>
#include <sofa/helper/gl/GLSLShader.h>

#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/DataFileName.h>

#include <SofaImplicitField3/components/geometry/CustomField.h>

#include <sofa/core/DataEngine.h>

#include <SofaBaseVisual/InteractiveCamera.h>

namespace sofa
{

namespace _implicitfieldshadervisualization_
{
using sofa::core::objectmodel::BaseLink;
using sofa::core::objectmodel::SingleLink;
using sofa::core::visual::Shader ;
using sofa::core::visual::VisualModel ;
using sofa::helper::gl::GLSLShader ;
using  sofa::component::geometry::CustomField;
using sofa::component::geometry::GLSLCodeFragment;

using sofa::core::DataEngine ;
using sofa::core::DataTracker ;
using sofa::core::objectmodel::Data;
using sofa::core::objectmodel::BaseData;
using sofa::core::objectmodel::Event ;
using sofa::core::objectmodel::BaseNode ;
using sofa::core::visual::VisualParams;
using sofa::core::objectmodel::DataFileName;
using sofa::defaulttype::Vec4;

class SOFA_SOFAIMPLICITFIELD3_API ImplicitFieldShaderVisualization : public VisualModel
{
public:
    SOFA_CLASS(ImplicitFieldShaderVisualization, VisualModel);

    ImplicitFieldShaderVisualization() ;
    ~ImplicitFieldShaderVisualization() override ;

    /// Inherited from VisualModel
    void init() override ;
    void reinit() override ;
    void initVisual() override ;
    void drawVisual(const VisualParams* vp) override ;

    SingleLink<ImplicitFieldShaderVisualization, CustomField, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_field ;
    void shaderGenerationCodeHasChanged();

    Data<Vec4>   d_viewportCoordinate;

    DataFileName d_vertFilename;
    DataFileName d_fragFilename;
    std::string  m_templateShaderFileName;
    std::string  m_templateFileName;
protected:
    bool changedFromDataField;
    std::vector<DataTracker*> m_datatrackerList;
    GLSLShader* shader;

    void uploadUniformsValues();
    std::ostream& generateFragmentShader(std::ostream&, const std::string &uniforms, const std::string &implicit);
    std::ostream& generateVertexShader(std::ostream&, const std::string& uniforms, const std::string& implicit);
    BaseData* fetchData(const std::string& argumentName, const std::string& type);
    void initComponentShaderValue();

    std::string uniformsAndConst(std::map<std::string, std::vector<GLSLCodeFragment>> glslMap);
    std::string implicitFunction(std::map<std::string, std::vector<GLSLCodeFragment>> glslMap);
    std::string rayMarchingFunction();
    std::string renderFunction();
    std::string viewFunction();
    std::string mainFragmenShaderFunction();

    std::string m_prevCode;
    std::string m_prevUniforms;

private:
    DataTracker m_datatracker ;

};

} /// private namespace
} /// namespace sofa


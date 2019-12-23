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
#include <SofaImplicitField3/config.h>

#include <boost/algorithm/string.hpp>
#include <sofa/core/ObjectFactory.h>
#include <Python.h>
#include <SofaPython3/PythonEnvironment.h>
using sofapython3::PythonEnvironment;

#include <SofaPython3/PythonFactory.h>
using sofapython3::PythonFactory;

#include <sofa/core/ObjectFactory.h>
using sofa::core::RegisterObject ;

#include <sofa/helper/system/FileMonitor.h>
using sofa::helper::system::FileMonitor ;

#include <sofa/core/objectmodel/IdleEvent.h>
using sofa::core::objectmodel::IdleEvent ;

#include <sofa/simulation/Node.h>
using sofa::simulation::Node ;

#include <cmath>
using std::max;
using sofa::core::objectmodel::ComponentState ;

#include <SofaImplicitField/components/geometry/ScalarField.h>
using sofa::core::objectmodel::CStatus ;

#include <SofaBaseLinearSolver/FullMatrix.h>

#include <sofa/helper/system/FileRepository.h>
using sofa::helper::system::DataRepository;


#include "CustomField.h"



/////////////////////////////////////////////
namespace sofa::core::objectmodel
{

namespace py = pybind11;

/// Specialization for reading strings
template<>
bool TData<py::object>::read( const std::string& str ){
    return false;
}
template<>
void TData<py::object>::printValue( std::ostream& out) const {}
template<> std::string TData<py::object>::getValueString() const
{
    PythonEnvironment::gil acquire{"TData<py::object>::getValueString()"};
    py::object o = virtualGetValue();
    std::string st=py::str(o);
    return st;
}
} ///


namespace sofa
{

namespace component
{

namespace geometry
{

namespace _customfield_
{

using sofapython3::PythonEnvironment ;

CustomField::CustomField() :
    d_pyEvalFunction(initData(&d_pyEvalFunction, "pyEvalFunction", "A callable python function taking a point in R^3 and returning a scalar value.")),
    d_pyGradFunction(initData(&d_pyGradFunction, "pyGradFunction", "A callable python function taking a point in R^3 and returning the scalar field gradient value.")),
    d_pyGLSLFunction (initData(&d_pyGLSLFunction, "pyGlslFunction", "Use a python function to return glsl implicit field description.")),
    d_fieldFunction(initData(&d_fieldFunction, ScalarFieldR3(), "evalFunction", "A function taking a point in R^3 and returning a scalar value."))
{
    d_pyEvalFunction.setValue(py::none());
    d_pyEvalFunction.setReadOnly(true);

    d_pyGradFunction.setValue(py::none());
    d_pyGradFunction.setReadOnly(true);

    d_pyGLSLFunction.setValue(py::none());
    d_pyGLSLFunction.setReadOnly(true);

    addUpdateCallback("field", {&d_pyEvalFunction, &d_pyGradFunction, &d_pyGLSLFunction}, [this](){
        clearLoggedMessages();
        if(d_pyEvalFunction.getValue().is_none())
            return sofa::core::objectmodel::ComponentState::Invalid;

        py::object pythonFunction = d_pyEvalFunction.getValue();
        d_fieldFunction.setValue(ScalarFieldR3{
                                     [pythonFunction](double x, double y, double z) -> double
                                     {
                                         PythonEnvironment::gil acquire;
                                         return py::cast<double>(pythonFunction(x,y,z));
                                     },
                                     [pythonFunction](double x, double y, double z) -> Vec3d {
                                         Vec3d p{x,y,z};
                                         double epsilon=0.0001;
                                         Vec3d Result;
                                         p[0] += epsilon;
                                         PythonEnvironment::gil acquire;
                                         Result[0] = py::cast<double>(pythonFunction(p.x(),p.y(),p.z()));
                                         p[0] -= epsilon;
                                         p[1] += epsilon;
                                         Result[1] = py::cast<double>(pythonFunction(p.x(),p.y(),p.z()));
                                         p[1] -= epsilon;
                                         p[2] += epsilon;
                                         Result[2] = py::cast<double>(pythonFunction(p.x(),p.y(),p.z()));
                                         p[2] -= epsilon;
                                         double v = py::cast<double>(pythonFunction(p.x(),p.y(),p.z()));
                                         Result[0] = (Result[0]-v)/epsilon;
                                         Result[1] = (Result[1]-v)/epsilon;
                                         Result[2] = (Result[2]-v)/epsilon;
                                         return Result;
                                     }
                                 });

        updateGLSLCodeCacheFromPython();

        return sofa::core::objectmodel::ComponentState::Valid;
    }, {&d_fieldFunction});
}

CustomField::~CustomField()
{
}

void CustomField::updateGLSLCodeCacheFromPython()
{
    PythonEnvironment::gil lock ;

    msg_info() << "Search for glsl rendering map" ;
    py::object glslFunction = d_pyGLSLFunction.getValue();
    if(glslFunction.is_none())
    {
        m_glslcodes.clear() ;
        return ;
    }

    py::tuple result = glslFunction();
    if(result.is_none())
    {
        PyErr_Print() ;
    }

    py::list items = result[0];
    py::str glslcode = result[1];

    /// Convert the list of instruction into a list of string to pass for GLSL.
    m_glslcodes.clear();
    std::vector<GLSLCodeFragment> evalList;
    GLSLCodeFragment frag;
    frag.m_value = py::cast<std::string>(glslcode);
    evalList.push_back(frag);
    m_glslcodes["eval"] = evalList;

    /// Convert the list of variable into a GLSL string.
    std::vector<GLSLCodeFragment> variableList;
    for(unsigned int i=0;i<items.size();i++)
    {
        py::tuple titem = items[i];
        py::str glslname =  titem[0];
        py::str glsltype = titem[1];
        py::str glslcode = titem[2];
        py::str glslvalue = titem[3];

        GLSLCodeFragment frag;
        frag.m_dataname = glslname;
        frag.m_name = glslname;
        frag.m_type = glsltype;
        frag.m_value = glslvalue;
        variableList.push_back(frag);
    }
    m_glslcodes["variable"] = variableList;
}

const std::map<std::string, std::vector<GLSLCodeFragment> > &CustomField::getGLSLCode()
{
    return m_glslcodes ;
}

void CustomField::init()
{
    m_componentstate = ComponentState::Loading;
    PythonEnvironment::gil lock ;

    if(d_pyEvalFunction.getValue().is_none())
    {
        m_componentstate = ComponentState::Loading;
        return;
    }

    m_componentstate = ComponentState::Valid ;
}

double CustomField::getValue(const defaulttype::Vec3d &pos, int &domain)
{
    SOFA_UNUSED(domain);
    if(m_componentstate!=ComponentState::Valid)
        return std::nan("") ;

    /// A dedicated python function has been set
    PythonEnvironment::gil scope;
    if(!d_pyEvalFunction.getValue().is_none())
    {
        return py::cast<double>(d_pyEvalFunction.getValue()(pos.x(), pos.y(), pos.z()));
    }

    return std::nan("");
}


Vec3d CustomField::getGradient(const Vec3d& pos, int &domain)
{
    SOFA_UNUSED(domain);
    Vec3d tmp(std::nan(""),std::nan(""),std::nan("")) ;

    /// The component is not valid. We return nan.
    if(m_componentstate!=ComponentState::Valid)
        return tmp ;

    /// The component is valid but we have no gradient function.
    /// Thus we use the finite difference version.
    if(d_pyGradFunction.getValue().is_none())
        return ScalarField::getGradient(pos, domain) ;

    msg_error() << "Missing code to compute gradient from provided function...";
    return tmp ;
}

///factory register
int CustomFieldClass = RegisterObject("A custom scalar field. The scalar function is implemented with python.")
        .add< CustomField >() ;

SOFA_DECL_CLASS(CustomField)

} /// namespace _customfield_

} /// namespace geometry

} /// namespace component

} /// namespace sofa

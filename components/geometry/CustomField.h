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
#pragma once
#include <SofaImplicitField3/config.h>

#include <SofaImplicitField/components/geometry/ScalarField.h>
#include <SofaPython3/PythonEnvironment.h>
#include <pybind11/pybind11.h>
#include <map>

///////////////////////////// FORWARD DEFINITIONS //////////////////////////////////////////////////
namespace sofa {
namespace helper {
namespace system {
    class FileEventListener ;
}
}
}




////////////////////////////////////////////////////////////////////////////////////////////////////
namespace sofa
{

namespace core
{

namespace objectmodel
{
/// Specialization for reading strings
template<>
bool TData<pybind11::object>::read( const std::string& str );
template<>
void TData<pybind11::object>::printValue( std::ostream& out) const;
template<>
std::string TData<pybind11::object>::getValueString() const ;
}
}


namespace component
{

namespace geometry
{

namespace _customfield_
{

using sofa::component::geometry::ScalarField ;
using sofa::defaulttype::Vec3d ;
using sofa::core::objectmodel::Data ;

namespace py = pybind11;

class GLSLCodeFragment
{
public:
    std::string m_dataname ;  ///@/root/object1/sphere1.center
    std::string m_name ;      ///"evale"
    std::string m_type ;      ///"function"
    std::string m_value ;     ///"return ... Vec4(0,0,0,0)"
};


//////////////////////////// CLASS DEFINITION //////////////////////////////////////////////////////
typedef  double (*FieldFunction)(void*, double x, double y, double z) ;

class SOFA_SOFAIMPLICITFIELD3_API CustomField : public ScalarField
{
public:
    SOFA_CLASS(CustomField, BaseObject);

public:
    virtual void init() override ;

    using ScalarField::getValue ;
    using ScalarField::getGradient ;

    virtual double getValue(const Vec3d& pos, int &domain) override ;
    virtual Vec3d getGradient(const Vec3d& pos, int& domain) override ;

    /// Returns a map with "id" and "glsl" textual code of the fragment.
    /// The idea is that we can tune what we take from python and how
    /// it is included in the glsl shader. Some convention have to be defined
    /// on what python should return.
    /// eg:
    ///    map["eval"] = "return min(p.x, p.y)"
    ///    map["eval"] = "return vec3(1.0,1.0,1.0)"
    /// when the map is empty means the component cannot makes a GLSL version of the
    /// distance function (eg no python function found)
    const std::map<std::string, std::vector<GLSLCodeFragment>>& getGLSLCode() ;

    /// C++ version of the field.
    Data<ScalarFieldR3> d_fieldFunction;

    /// python version of the field. (IN)
    Data<py::object> d_pyEvalFunction;
    Data<py::object> d_pyGradFunction;
    Data<py::object> d_pyGLSLFunction;

protected:
    CustomField( ) ;
    ~CustomField() override ;

private:
    CustomField(const CustomField& n) ;
    CustomField& operator=(const CustomField& n) ;
    void updateGLSLCodeCacheFromPython() ;

    std::map<std::string, std::vector<GLSLCodeFragment> > m_glslcodes ;
};

} /// namespace _scalarfield_

using _customfield_::CustomField ;
using _customfield_::GLSLCodeFragment ;

} /// namespace geometry

} /// namespace component

} /// namespace sofa




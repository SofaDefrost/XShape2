cmake_minimum_required(VERSION 3.1)
project(SofaImplicitField3)

set(SOFAIMPLICITFIELD3_VERSION 1.0)
set(SOFA_HAVE_SOFAIMPLICITFIELD3 1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SofaImplicitField REQUIRED)
find_package(SofaPython3 REQUIRED)

set(HEADER_FILES
    config.h
    initSofaImplicitField.h
    components/geometry/CustomField.h
    components/visual/ImplicitFieldShaderVisualization.h
)

set(SOURCE_FILES
    initSofaImplicitField.cpp
    components/geometry/CustomField.cpp
    components/visual/ImplicitFieldShaderVisualization.cpp
)

add_definitions("-DDO_SOFAPYTHON_FEATURES=1")

set(EXTRA_FILES
    README.md
    examples/README.md
    examples/CustomField.scn
    examples/CustomFieldVisualization.scn
    examples/CustomShaderField.scn
    examples/customfield.py
    examples/customshape.py
    examples/ShapeTreeDemo1.psl
    examples/dfgeom.psl
    examples/dfgeom.pyx
    examples/cython/CustomField.scn
    examples/cython/customfield.py
    examples/cython/cythonfield.pyx
    python/difigeometry/Makefile
    python/difigeometry/__init__.py
    python/difigeometry/expression.py
    python/difigeometry/expression2D.py
    python/difigeometry/expressionToString.py
    python/difigeometry/generate_expression.py
    python/difigeometry/pyxsrc/primitives.pxd
    python/difigeometry/pyxsrc/primitives.pyx
    python/difigeometry/pyxsrc/primitives2D.pxd
    python/difigeometry/pyxsrc/primitives2D.pyx

    python/SofaImplicitField/__init__.py
    )


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${EXTRA_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_HAVE_SOFAIMPLICITFIELD3")
target_link_libraries(${PROJECT_NAME} PUBLIC SofaImplicitField SofaPython3)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

## Install rules for the library and headers; CMake package configurations files
sofa_create_package(SofaImplicitField3 ${SOFAIMPLICITFIELD3_VERSION} ${PROJECT_NAME} SofaImplicitField3)

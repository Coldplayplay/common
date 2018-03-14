# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# compile CXX with /usr/bin/c++
CXX_FLAGS =      -march=native -msse4.2 -mfpmath=sse  -fPIC -std=gnu++11

CXX_DEFINES = -DDISABLE_DAVIDSDK -DDISABLE_DSSDK -DDISABLE_ENSENSO -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DDISABLE_RSSDK -DFLANN_STATIC -DQT_CORE_LIB -DQT_GUI_LIB -DQT_NO_DEBUG -DQT_WIDGETS_LIB -Dqh_QHpointer -DvtkDomainsChemistry_AUTOINIT="1(vtkDomainsChemistryOpenGL2)" -DvtkRenderingContext2D_AUTOINIT="1(vtkRenderingContextOpenGL2)" -DvtkRenderingCore_AUTOINIT="3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)" -DvtkRenderingOpenGL2_AUTOINIT="1(vtkRenderingGL2PSOpenGL2)" -DvtkRenderingVolume_AUTOINIT="1(vtkRenderingVolumeOpenGL2)"

CXX_INCLUDES = -I/home/cbc/VTK7-build/Interaction/Style -I/home/cbc/VTK-7.1.1/Interaction/Style -I/home/cbc/VTK7-build/Common/Core -I/home/cbc/VTK-7.1.1/Common/Core -I/home/cbc/VTK7-build/Utilities/KWIML -I/home/cbc/VTK-7.1.1/Utilities/KWIML -I/home/cbc/VTK7-build/Utilities/KWSys -I/home/cbc/VTK-7.1.1/Utilities/KWSys -I/home/cbc/VTK7-build/Common/DataModel -I/home/cbc/VTK-7.1.1/Common/DataModel -I/home/cbc/VTK7-build/Common/Math -I/home/cbc/VTK-7.1.1/Common/Math -I/home/cbc/VTK7-build/Common/Misc -I/home/cbc/VTK-7.1.1/Common/Misc -I/home/cbc/VTK7-build/Common/System -I/home/cbc/VTK-7.1.1/Common/System -I/home/cbc/VTK7-build/Common/Transforms -I/home/cbc/VTK-7.1.1/Common/Transforms -I/home/cbc/VTK7-build/Filters/Extraction -I/home/cbc/VTK-7.1.1/Filters/Extraction -I/home/cbc/VTK7-build/Common/ExecutionModel -I/home/cbc/VTK-7.1.1/Common/ExecutionModel -I/home/cbc/VTK7-build/Filters/Core -I/home/cbc/VTK-7.1.1/Filters/Core -I/home/cbc/VTK7-build/Filters/General -I/home/cbc/VTK-7.1.1/Filters/General -I/home/cbc/VTK7-build/Common/ComputationalGeometry -I/home/cbc/VTK-7.1.1/Common/ComputationalGeometry -I/home/cbc/VTK7-build/Filters/Statistics -I/home/cbc/VTK-7.1.1/Filters/Statistics -I/home/cbc/VTK7-build/Imaging/Fourier -I/home/cbc/VTK-7.1.1/Imaging/Fourier -I/home/cbc/VTK7-build/Imaging/Core -I/home/cbc/VTK-7.1.1/Imaging/Core -I/home/cbc/VTK7-build/ThirdParty/alglib -I/home/cbc/VTK-7.1.1/ThirdParty/alglib -I/home/cbc/VTK7-build/Filters/Sources -I/home/cbc/VTK-7.1.1/Filters/Sources -I/home/cbc/VTK7-build/Rendering/Core -I/home/cbc/VTK-7.1.1/Rendering/Core -I/home/cbc/VTK7-build/Common/Color -I/home/cbc/VTK-7.1.1/Common/Color -I/home/cbc/VTK7-build/Filters/Geometry -I/home/cbc/VTK-7.1.1/Filters/Geometry -I/home/cbc/VTK7-build/Views/Qt -I/home/cbc/VTK-7.1.1/Views/Qt -I/home/cbc/VTK7-build/GUISupport/Qt -I/home/cbc/VTK-7.1.1/GUISupport/Qt -I/home/cbc/VTK7-build/Rendering/OpenGL2 -I/home/cbc/VTK-7.1.1/Rendering/OpenGL2 -I/home/cbc/VTK7-build/IO/Image -I/home/cbc/VTK-7.1.1/IO/Image -I/home/cbc/VTK7-build/Utilities/DICOMParser -I/home/cbc/VTK-7.1.1/Utilities/DICOMParser -I/home/cbc/VTK7-build/Utilities/MetaIO/vtkmetaio -I/home/cbc/VTK7-build/Utilities/MetaIO -I/home/cbc/VTK-7.1.1/Utilities/MetaIO -I/home/cbc/VTK7-build/ThirdParty/zlib -I/home/cbc/VTK-7.1.1/ThirdParty/zlib -I/home/cbc/VTK7-build/ThirdParty/jpeg -I/home/cbc/VTK-7.1.1/ThirdParty/jpeg -I/home/cbc/VTK7-build/ThirdParty/png -I/home/cbc/VTK-7.1.1/ThirdParty/png -I/home/cbc/VTK7-build/ThirdParty/tiff/vtktiff/libtiff -I/home/cbc/VTK7-build/ThirdParty/tiff -I/home/cbc/VTK-7.1.1/ThirdParty/tiff -I/home/cbc/VTK7-build/Utilities/EncodeString -I/home/cbc/VTK-7.1.1/Utilities/EncodeString -I/home/cbc/VTK7-build/ThirdParty/glew -I/home/cbc/VTK-7.1.1/ThirdParty/glew -I/home/cbc/VTK7-build/Infovis/Core -I/home/cbc/VTK-7.1.1/Infovis/Core -I/home/cbc/VTK7-build/Views/Core -I/home/cbc/VTK-7.1.1/Views/Core -I/home/cbc/VTK7-build/Interaction/Widgets -I/home/cbc/VTK-7.1.1/Interaction/Widgets -I/home/cbc/VTK7-build/Filters/Hybrid -I/home/cbc/VTK-7.1.1/Filters/Hybrid -I/home/cbc/VTK7-build/Imaging/Sources -I/home/cbc/VTK-7.1.1/Imaging/Sources -I/home/cbc/VTK7-build/Filters/Modeling -I/home/cbc/VTK-7.1.1/Filters/Modeling -I/home/cbc/VTK7-build/Imaging/Color -I/home/cbc/VTK-7.1.1/Imaging/Color -I/home/cbc/VTK7-build/Imaging/General -I/home/cbc/VTK-7.1.1/Imaging/General -I/home/cbc/VTK7-build/Imaging/Hybrid -I/home/cbc/VTK-7.1.1/Imaging/Hybrid -I/home/cbc/VTK7-build/Rendering/Annotation -I/home/cbc/VTK-7.1.1/Rendering/Annotation -I/home/cbc/VTK7-build/Rendering/FreeType -I/home/cbc/VTK-7.1.1/Rendering/FreeType -I/home/cbc/VTK7-build/ThirdParty/freetype -I/home/cbc/VTK-7.1.1/ThirdParty/freetype -I/home/cbc/VTK7-build/Rendering/Volume -I/home/cbc/VTK-7.1.1/Rendering/Volume -I/home/cbc/VTK7-build/IO/XML -I/home/cbc/VTK-7.1.1/IO/XML -I/home/cbc/VTK7-build/IO/Core -I/home/cbc/VTK-7.1.1/IO/Core -I/home/cbc/VTK7-build/IO/XMLParser -I/home/cbc/VTK-7.1.1/IO/XMLParser -I/home/cbc/VTK7-build/ThirdParty/expat -I/home/cbc/VTK-7.1.1/ThirdParty/expat -I/home/cbc/VTK7-build/Views/Infovis -I/home/cbc/VTK-7.1.1/Views/Infovis -I/home/cbc/VTK7-build/Charts/Core -I/home/cbc/VTK-7.1.1/Charts/Core -I/home/cbc/VTK7-build/Rendering/Context2D -I/home/cbc/VTK-7.1.1/Rendering/Context2D -I/home/cbc/VTK7-build/Filters/Imaging -I/home/cbc/VTK-7.1.1/Filters/Imaging -I/home/cbc/VTK7-build/Infovis/Layout -I/home/cbc/VTK-7.1.1/Infovis/Layout -I/home/cbc/VTK7-build/Rendering/Label -I/home/cbc/VTK-7.1.1/Rendering/Label -I/home/cbc/VTK7-build/IO/LSDyna -I/home/cbc/VTK-7.1.1/IO/LSDyna -I/home/cbc/VTK7-build/IO/Infovis -I/home/cbc/VTK-7.1.1/IO/Infovis -I/home/cbc/VTK7-build/IO/Legacy -I/home/cbc/VTK-7.1.1/IO/Legacy -I/home/cbc/VTK7-build/ThirdParty/libxml2/vtklibxml2 -I/home/cbc/VTK7-build/ThirdParty/libxml2 -I/home/cbc/VTK-7.1.1/ThirdParty/libxml2 -I/home/cbc/VTK7-build/Imaging/Statistics -I/home/cbc/VTK-7.1.1/Imaging/Statistics -I/home/cbc/VTK7-build/Imaging/Morphological -I/home/cbc/VTK-7.1.1/Imaging/Morphological -I/home/cbc/VTK7-build/Views/Context2D -I/home/cbc/VTK-7.1.1/Views/Context2D -I/home/cbc/VTK7-build/Utilities/HashSource -I/home/cbc/VTK-7.1.1/Utilities/HashSource -I/home/cbc/VTK7-build/IO/Movie -I/home/cbc/VTK-7.1.1/IO/Movie -I/home/cbc/VTK7-build/ThirdParty/oggtheora -I/home/cbc/VTK-7.1.1/ThirdParty/oggtheora -I/home/cbc/VTK7-build/Filters/Selection -I/home/cbc/VTK-7.1.1/Filters/Selection -I/home/cbc/VTK7-build/Filters/Generic -I/home/cbc/VTK-7.1.1/Filters/Generic -I/home/cbc/VTK7-build/Filters/Verdict -I/home/cbc/VTK-7.1.1/Filters/Verdict -I/home/cbc/VTK7-build/ThirdParty/verdict -I/home/cbc/VTK-7.1.1/ThirdParty/verdict -I/home/cbc/VTK7-build/ThirdParty/sqlite -I/home/cbc/VTK-7.1.1/ThirdParty/sqlite -I/home/cbc/VTK7-build/Rendering/LOD -I/home/cbc/VTK-7.1.1/Rendering/LOD -I/home/cbc/VTK7-build/Domains/ChemistryOpenGL2 -I/home/cbc/VTK-7.1.1/Domains/ChemistryOpenGL2 -I/home/cbc/VTK7-build/Domains/Chemistry -I/home/cbc/VTK-7.1.1/Domains/Chemistry -I/home/cbc/VTK7-build/ThirdParty/exodusII -I/home/cbc/VTK-7.1.1/ThirdParty/exodusII -I/home/cbc/VTK-7.1.1/ThirdParty/netcdf/vtknetcdf/include -I/home/cbc/VTK7-build/ThirdParty/netcdf/vtknetcdf -I/home/cbc/VTK7-build/ThirdParty/netcdf -I/home/cbc/VTK-7.1.1/ThirdParty/netcdf -I/home/cbc/VTK7-build/ThirdParty/hdf5/vtkhdf5 -isystem /home/cbc/VTK-7.1.1/ThirdParty/hdf5/vtkhdf5/hl/src -isystem /home/cbc/VTK-7.1.1/ThirdParty/hdf5/vtkhdf5/src -I/home/cbc/VTK7-build/ThirdParty/hdf5 -I/home/cbc/VTK-7.1.1/ThirdParty/hdf5 -I/home/cbc/VTK7-build/Imaging/Math -I/home/cbc/VTK-7.1.1/Imaging/Math -I/home/cbc/VTK7-build/IO/Exodus -I/home/cbc/VTK-7.1.1/IO/Exodus -I/home/cbc/VTK7-build/Filters/Texture -I/home/cbc/VTK-7.1.1/Filters/Texture -I/home/cbc/VTK7-build/IO/Export -I/home/cbc/VTK-7.1.1/IO/Export -I/home/cbc/VTK7-build/Rendering/GL2PSOpenGL2 -I/home/cbc/VTK-7.1.1/Rendering/GL2PSOpenGL2 -I/home/cbc/VTK7-build/ThirdParty/gl2ps -I/home/cbc/VTK-7.1.1/ThirdParty/gl2ps -I/home/cbc/VTK7-build/Filters/ParallelImaging -I/home/cbc/VTK-7.1.1/Filters/ParallelImaging -I/home/cbc/VTK7-build/Filters/Parallel -I/home/cbc/VTK-7.1.1/Filters/Parallel -I/home/cbc/VTK7-build/Parallel/Core -I/home/cbc/VTK-7.1.1/Parallel/Core -I/home/cbc/VTK7-build/IO/MINC -I/home/cbc/VTK-7.1.1/IO/MINC -I/home/cbc/VTK7-build/ThirdParty/jsoncpp -I/home/cbc/VTK-7.1.1/ThirdParty/jsoncpp -I/home/cbc/VTK7-build/Imaging/Stencil -I/home/cbc/VTK-7.1.1/Imaging/Stencil -I/home/cbc/VTK7-build/IO/Geometry -I/home/cbc/VTK-7.1.1/IO/Geometry -I/home/cbc/VTK7-build/Filters/AMR -I/home/cbc/VTK-7.1.1/Filters/AMR -I/home/cbc/VTK7-build/IO/NetCDF -I/home/cbc/VTK-7.1.1/IO/NetCDF -I/home/cbc/VTK7-build/IO/ParallelXML -I/home/cbc/VTK-7.1.1/IO/ParallelXML -I/home/cbc/VTK7-build/Filters/SMP -I/home/cbc/VTK-7.1.1/Filters/SMP -I/home/cbc/VTK7-build/IO/AMR -I/home/cbc/VTK-7.1.1/IO/AMR -I/home/cbc/VTK7-build/Filters/Programmable -I/home/cbc/VTK-7.1.1/Filters/Programmable -I/home/cbc/VTK7-build/Geovis/Core -I/home/cbc/VTK-7.1.1/Geovis/Core -I/home/cbc/VTK-7.1.1/ThirdParty/libproj4/vtklibproj4 -I/home/cbc/VTK7-build/ThirdParty/libproj4/vtklibproj4 -I/home/cbc/VTK7-build/ThirdParty/libproj4 -I/home/cbc/VTK-7.1.1/ThirdParty/libproj4 -I/home/cbc/VTK7-build/Rendering/Image -I/home/cbc/VTK-7.1.1/Rendering/Image -I/home/cbc/VTK7-build/GUISupport/QtSQL -I/home/cbc/VTK-7.1.1/GUISupport/QtSQL -I/home/cbc/VTK7-build/IO/SQL -I/home/cbc/VTK-7.1.1/IO/SQL -I/home/cbc/VTK7-build/Filters/FlowPaths -I/home/cbc/VTK-7.1.1/Filters/FlowPaths -I/home/cbc/VTK7-build/IO/Parallel -I/home/cbc/VTK-7.1.1/IO/Parallel -I/home/cbc/VTK7-build/Rendering/VolumeOpenGL2 -I/home/cbc/VTK-7.1.1/Rendering/VolumeOpenGL2 -I/home/cbc/VTK7-build/Rendering/ContextOpenGL2 -I/home/cbc/VTK-7.1.1/Rendering/ContextOpenGL2 -I/home/cbc/VTK7-build/IO/PLY -I/home/cbc/VTK-7.1.1/IO/PLY -I/home/cbc/VTK7-build/Rendering/Qt -I/home/cbc/VTK-7.1.1/Rendering/Qt -I/home/cbc/VTK7-build/Interaction/Image -I/home/cbc/VTK-7.1.1/Interaction/Image -I/home/cbc/VTK7-build/IO/Video -I/home/cbc/VTK-7.1.1/IO/Video -I/home/cbc/VTK7-build/Filters/HyperTree -I/home/cbc/VTK-7.1.1/Filters/HyperTree -I/home/cbc/VTK7-build/IO/Import -I/home/cbc/VTK-7.1.1/IO/Import -I/home/cbc/VTK7-build/IO/TecplotTable -I/home/cbc/VTK-7.1.1/IO/TecplotTable -I/home/cbc/VTK7-build/Filters/Points -I/home/cbc/VTK-7.1.1/Filters/Points -I/home/cbc/VTK7-build/IO/EnSight -I/home/cbc/VTK-7.1.1/IO/EnSight -I/usr/local/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/ni -I/usr/include/openni2 -isystem /home/cbc/Qt5.9.2/5.9.2/gcc_64/include -isystem /home/cbc/Qt5.9.2/5.9.2/gcc_64/include/QtWidgets -isystem /home/cbc/Qt5.9.2/5.9.2/gcc_64/include/QtGui -isystem /home/cbc/Qt5.9.2/5.9.2/gcc_64/include/QtCore -isystem /home/cbc/Qt5.9.2/5.9.2/gcc_64/./mkspecs/linux-g++ -isystem /home/cbc/VTK7-build/ThirdParty/hdf5/vtkhdf5/hl/src -isystem /home/cbc/VTK7-build/ThirdParty/hdf5/vtkhdf5/src 


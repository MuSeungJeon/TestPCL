QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    mainwindow.h

FORMS += \
    mainwindow.ui

#OPENCV

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../opencv/build/x64/vc15/lib/ -lopencv_world400
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../opencv/build/x64/vc15/lib/ -lopencv_world400d
else:unix: LIBS += -L$$PWD/../../../../opencv/build/x64/vc15/lib/ -lopencv_world400

INCLUDEPATH += $$PWD/../../../../opencv/build/include \
DEPENDPATH += $$PWD/../../../../opencv/build/include

LIBS += -LC:\opencv\build\lib\Debug \
    -lopencv_core400d               \
    -lopencv_calib3d400d            \
    -lopencv_dnn400d                \
    -lopencv_features2d400d         \
    -lopencv_flann400d              \
    -lopencv_gapi400d               \
    -lopencv_highgui400d            \
    -lopencv_imgcodecs400d          \
    -lopencv_imgproc400d            \
    -lopencv_ml400d                 \
    -lopencv_objdetect400d          \
    -lopencv_photo400d              \
    -lopencv_stitching400d          \
    -lopencv_ts400d                 \
    -lopencv_video400d              \
    -lopencv_videoio400d

# Kinect AZUR

AZUREINCLUDE    = "C:/Program Files/Azure Kinect SDK v1.4.1/sdk/include"
AZURELIB        = "C:/Program Files/Azure Kinect SDK v1.4.1/sdk/windows-desktop/amd64/release/lib"

INCLUDEPATH += $$PWD/'../../../../Program Files/Azure Kinect SDK v1.4.1/sdk/windows-desktop/amd64/release' \
            += $${AZUREINCLUDE}/     \
            += $${AZUREINCLUDE}/k4a
DEPENDPATH += $$PWD/'../../../../Program Files/Azure Kinect SDK v1.4.1/sdk/windows-desktop/amd64/release' \
            += $${AZUREINCLUDE}/    \
            += $${AZUREINCLUDE}/k4a \

LIBS += -L$${AZURELIB} \
    k4a.lib


#PCL+VTK

PCLINCLUDE      = "C:/Program Files/PCL 1.8.1/include/pcl-1.8"
BOOSTINCLUDE    = "C:/Program Files/PCL 1.8.1/3rdParty/Boost/include/boost-1_64"
EIGENINCLUDE    = "C:/Program Files/PCL 1.8.1/3rdParty/Eigen/eigen3"
FLANNINCLUDE    = "C:/Program Files/PCL 1.8.1/3rdParty/FLANN/include"
VTKINCLUDE      = "C:/Program Files/PCL 1.8.1/3rdParty/VTK/include/vtk-8.0"
PCLLIB          = "C:/Program Files/PCL 1.8.1/lib"
BOOSTLIB        = "C:/Program Files/PCL 1.8.1/3rdParty/Boost/lib"
VTKLIB          = "C:/Program Files/PCL 1.8.1/3rdParty/VTK/lib"

INCLUDEPATH +=  $${PCLINCLUDE}\
                $${BOOSTINCLUDE}\
                $${EIGENINCLUDE}\
                $${FLANNINCLUDE}\
                $${VTKINCLUDE}

LIBS += -L$${BOOSTLIB} \
-llibboost_thread-vc141-mt-gd-1_64

LIBS += -L$${PCLLIB}  \
-lpcl_common_debug              \
-lpcl_features_debug            \
-lpcl_filters_debug             \
-lpcl_io_debug                  \
-lpcl_io_ply_debug              \
-lpcl_kdtree_debug              \
-lpcl_keypoints_debug           \
-lpcl_ml_debug                  \
-lpcl_octree_debug              \
-lpcl_outofcore_debug           \
-lpcl_people_debug              \
-lpcl_recognition_debug         \
-lpcl_registration_debug        \
-lpcl_sample_consensus_debug    \
-lpcl_search_debug              \
-lpcl_segmentation_debug        \
-lpcl_stereo_debug              \
-lpcl_surface_debug             \
-lpcl_tracking_debug            \
-lpcl_visualization_debug


LIBS += -L$${VTKLIB} \
-lvtkalglib-8.0-gd\
-lvtkChartsCore-8.0-gd\
-lvtkCommonColor-8.0-gd\
-lvtkCommonComputationalGeometry-8.0-gd\
-lvtkCommonCore-8.0-gd\
-lvtkCommonDataModel-8.0-gd\
-lvtkCommonExecutionModel-8.0-gd\
-lvtkCommonMath-8.0-gd\
-lvtkCommonMisc-8.0-gd\
-lvtkCommonSystem-8.0-gd\
-lvtkCommonTransforms-8.0-gd\
-lvtkDICOMParser-8.0-gd\
-lvtkDomainsChemistry-8.0-gd\
-lvtkexoIIc-8.0-gd\
-lvtkexpat-8.0-gd\
-lvtkFiltersAMR-8.0-gd\
-lvtkFiltersCore-8.0-gd\
-lvtkFiltersExtraction-8.0-gd\
-lvtkFiltersFlowPaths-8.0-gd\
-lvtkFiltersGeneral-8.0-gd\
-lvtkFiltersGeneric-8.0-gd\
-lvtkFiltersGeometry-8.0-gd\
-lvtkFiltersHybrid-8.0-gd\
-lvtkFiltersHyperTree-8.0-gd\
-lvtkFiltersImaging-8.0-gd\
-lvtkFiltersModeling-8.0-gd\
-lvtkFiltersParallel-8.0-gd\
-lvtkFiltersParallelImaging-8.0-gd\
-lvtkFiltersPoints-8.0-gd\
-lvtkFiltersProgrammable-8.0-gd\
-lvtkFiltersSelection-8.0-gd\
-lvtkFiltersSMP-8.0-gd\
-lvtkFiltersSources-8.0-gd\
-lvtkFiltersStatistics-8.0-gd\
-lvtkFiltersTexture-8.0-gd\
-lvtkFiltersVerdict-8.0-gd\
-lvtkfreetype-8.0-gd\
-lvtkGeovisCore-8.0-gd\
-lvtkgl2ps-8.0-gd\
-lvtkGUISupportQt-8.0-gd\
-lvtkGUISupportQtSQL-8.0-gd\
-lvtkhdf5-8.0-gd\
-lvtkhdf5_hl-8.0-gd\
-lvtkImagingColor-8.0-gd\
-lvtkImagingCore-8.0-gd\
-lvtkImagingFourier-8.0-gd\
-lvtkImagingGeneral-8.0-gd\
-lvtkImagingHybrid-8.0-gd\
-lvtkImagingMath-8.0-gd\
-lvtkImagingMorphological-8.0-gd\
-lvtkImagingSources-8.0-gd\
-lvtkImagingStatistics-8.0-gd\
-lvtkImagingStencil-8.0-gd\
-lvtkInfovisCore-8.0-gd\
-lvtkInfovisLayout-8.0-gd\
-lvtkInteractionImage-8.0-gd\
-lvtkInteractionStyle-8.0-gd\
-lvtkInteractionWidgets-8.0-gd\
-lvtkIOAMR-8.0-gd\
-lvtkIOCore-8.0-gd\
-lvtkIOEnSight-8.0-gd\
-lvtkIOExodus-8.0-gd\
-lvtkIOExport-8.0-gd\
-lvtkIOGeometry-8.0-gd\
-lvtkIOImage-8.0-gd\
-lvtkIOImport-8.0-gd\
-lvtkIOInfovis-8.0-gd\
-lvtkIOLegacy-8.0-gd\
-lvtkIOLSDyna-8.0-gd\
-lvtkIOMINC-8.0-gd\
-lvtkIOMovie-8.0-gd\
-lvtkIONetCDF-8.0-gd\
-lvtkIOParallel-8.0-gd\
-lvtkIOParallelXML-8.0-gd\
-lvtkIOPLY-8.0-gd\
-lvtkIOSQL-8.0-gd\
-lvtkIOTecplotTable-8.0-gd\
-lvtkIOVideo-8.0-gd\
-lvtkIOXML-8.0-gd\
-lvtkIOXMLParser-8.0-gd\
-lvtkjpeg-8.0-gd\
-lvtkjsoncpp-8.0-gd\
-lvtklibxml2-8.0-gd\
-lvtkmetaio-8.0-gd\
-lvtkNetCDF-8.0-gd\
-lvtkNetCDF_c++-gd\
-lvtkoggtheora-8.0-gd\
-lvtkParallelCore-8.0-gd\
-lvtkpng-8.0-gd\
-lvtkRenderingAnnotation-8.0-gd\
-lvtkRenderingContext2D-8.0-gd\
-lvtkRenderingContextOpenGL-8.0-gd\
-lvtkRenderingCore-8.0-gd\
-lvtkRenderingFreeType-8.0-gd\
-lvtkRenderingGL2PS-8.0-gd\
-lvtkRenderingImage-8.0-gd\
-lvtkRenderingLabel-8.0-gd\
-lvtkRenderingLIC-8.0-gd\
-lvtkRenderingLOD-8.0-gd\
-lvtkRenderingOpenGL-8.0-gd\
-lvtkRenderingQt-8.0-gd\
-lvtkRenderingVolume-8.0-gd\
-lvtkRenderingVolumeOpenGL-8.0-gd\
-lvtksqlite-8.0-gd\
-lvtksys-8.0-gd\
-lvtktiff-8.0-gd\
-lvtkverdict-8.0-gd\
-lvtkViewsContext2D-8.0-gd\
-lvtkViewsCore-8.0-gd\
-lvtkViewsInfovis-8.0-gd\
-lvtkViewsQt-8.0-gd\
-lvtkzlib-8.0-gd
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

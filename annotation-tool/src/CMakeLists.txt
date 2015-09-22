
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${QT_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${QT_DEFINITIONS})

SET(app_tool_SOURCES main.cpp mainwindow.cpp pointcloudmodifier.cpp viewerinteractor.cpp newobjectdialog.cpp objectsinformation.cpp selectobjectdialog.cpp initialmessagedialog.cpp filtervaluesdialog.cpp qsr.cpp addobjectdialog.cpp)
SET(app_tool_HEADERS mainwindow.h pointcloudmodifier.h viewerinteractor.h newobjectdialog.h objectsinformation.h selectobjectdialog.h initialmessagedialog.h filtervaluesdialog.h qsr.h addobjectdialog.h)
SET(app_tool_UIS newobjectdialog.ui mainwindow.ui selectobjectdialog.ui initialmessagedialog.ui filtervaluesdialog.ui addobjectdialog.ui)
SET(QtApp_RCCS ./../icons/icons.qrc)

# this will run uic on .ui files:
QT4_WRAP_UI(app_tool_HDRS ${app_tool_UIS} )

# generate rules for building source files from the resources
QT4_ADD_RESOURCES(QtApp_RCC_SRCS ${QtApp_RCCS})

# generate rules for building source files that moc generates
QT4_WRAP_CPP(app_tool_HEADERS_MOC ${app_tool_HEADERS})

add_executable (annotation_tool ${app_tool_SOURCES} ${app_tool_HEADERS_MOC} ${QtApp_RCC_SRCS} ${app_tool_HDRS})
target_link_libraries (annotation_tool QVTK vtkHybrid ${PCL_LIBRARIES} ${QT_LIBRARIES} pcl_visualization ${catkin_LIBRARIES})

#add_custom_command(TARGET Annotation_tool POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy_directory
#        "/home/akshaya/technical/tinker-box/catkin_ws/src/3d_annotation_tool/annotation-tool/"
#        $<TARGET_FILE_DIR:Annotation_tool>)

#add_custom_command(TARGET Annotation_tool POST_BUILD
#   COMMAND ${CMAKE_COMMAND} -E remove $<TARGET_FILE_DIR:Annotation_tool/info_app.xml>)       

# Files and Folders that need to be copied to executable location

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

SET(EXECUTABLE_DEPENDENCIES /icons /documentation /data /info_app.xml /list_objects.xml /README.md)

    message(" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ")
    
execute_process(COMMAND mkdir ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

    message(" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ")

foreach (_itemname ${EXECUTABLE_DEPENDENCIES})
    message(STATUS "Copying ............ ${_itemname}")
    execute_process(COMMAND cp -r ${annotation_tool_SOURCE_DIR}${_itemname} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endforeach()
 
#########################################################################
# CMAKE TRIALS
#########################################################################
#    message("Executable going to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} from ${Annotation_tool_SOURCE_DIR}${LISTOBJECTS_PATH}")
    
#    get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

#SET(ICONS_PATH /icons)
#SET(DOCUMENTATION_PATH /documentation)
#SET(DOCUMENTATION_PATH_WITH_SLASH /documentation/*)
#SET(DATA_PATH /data)
#SET(INFOXML_PATH /info_app.xml)
#SET(LISTOBJECTS_PATH /list_objects.xml)
#SET(README_PATH /README.md)




#execute_process(COMMAND cp -r ${Annotation_tool_SOURCE_DIR}${DOCUMENTATION_PATH} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#execute_process(COMMAND cp -r ${Annotation_tool_SOURCE_DIR}${ICONS_PATH} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#execute_process(COMMAND cp -r ${Annotation_tool_SOURCE_DIR}${DATA_PATH} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#execute_process(COMMAND cp -r ${Annotation_tool_SOURCE_DIR}${INFOXML_PATH} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#execute_process(COMMAND cp -r ${Annotation_tool_SOURCE_DIR}${LISTOBJECTS_PATH} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#execute_process(COMMAND cp -r ${Annotation_tool_SOURCE_DIR}${README_PATH} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

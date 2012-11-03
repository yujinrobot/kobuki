#include $(shell rospack find mk)/cmake.mk

# Transfer this to the CMakeLists.txt later.

# Note: xxx_rc is the expected name for resources consuemd by ui files.

DIR = src/kobuki_qtestsuite/detail
UI_DIR = src/kobuki_qtestsuite/ui
RESOURCES_DIR = src/kobuki_qtestsuite/resources

all:
	mkdir -p ${DIR}
	touch ${DIR}/__init__.py
	pyrcc4 -o ${DIR}/common_rc.py ${RESOURCES_DIR}/common.qrc
	pyrcc4 -o ${DIR}/climbing_rc.py ${RESOURCES_DIR}/climbing.qrc
	pyuic4 -o ${DIR}/kobuki_testsuite_ui.py ${UI_DIR}/kobuki_testsuite.ui 
	pyuic4 -o ${DIR}/configuration_dock_ui.py ${UI_DIR}/configuration_dock.ui
	
clean:
	rm -rf  ${DIR}

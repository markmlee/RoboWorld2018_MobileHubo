#!/bin/bash
set -eu
mkdir -p build/ALBuild build/__DeviceManagerBuild build/__LaucherBuild build/__ModelBuild build/PODOGUIBuild build/__DaemonBuild build/ALBuild/share/Gain
qmakeProject() {
	qmake=qmake
	if which qmake-qt5 2>/dev/null 1>&2; then
		qmake=qmake-qt5
	fi
	cd ${1}/build/${3}
	${qmake} ${1}/src/${2}
}
topLevelDirectory=$(pwd)
qmakeProject ${topLevelDirectory} ALPrograms ALBuild
qmakeProject ${topLevelDirectory} __DeviceManager __DeviceManagerBuild
qmakeProject ${topLevelDirectory} __Daemon __DaemonBuild
qmakeProject ${topLevelDirectory} PODOGUI PODOGUIBuild
qmakeProject ${topLevelDirectory} __PODOLauncher __LaucherBuild
qmakeProject ${topLevelDirectory} __Lib_RBModel /__ModelBuild
cp -r ${topLevelDirectory}/share/Gain ${topLevelDirectory}/build/ALBuild/share/Gain

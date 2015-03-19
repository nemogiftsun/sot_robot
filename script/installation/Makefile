#
# Copyright (c) 2014 CNRS-LAAS
# Author: Florent Lamiraux
#

JRL_REPO=https://github.com/jrl-umi3218
LAAS_REPO=git@github.com:laas
SOT_REPO=git@github.com:stack-of-tasks
NG_REPO=git@github.com:nemogiftsun

SRC_DIR=${DEVEL_DIR}/src
BUILD_TYPE=Debug

# t3d
# urdfdom
# urdfdom_headers
# resource_retriever
# bullet
# realtime_tools

abstract-robot-dynamics_branch=masterot-
abstract-robot-dynamics_repository=${LAAS_REPO}

dynamic-graph_branch=master
dynamic-graph_repository=${JRL_REPO}

dynamic-graph-python_branch=master
dynamic-graph-python_repository=${JRL_REPO}

jrl-mathtools_branch=master
jrl-mathtools_repository=${JRL_REPO}

jrl-mal_branch=master
jrl-mal_repository=${JRL_REPO}

jrl-dynamics_branch=master
jrl-dynamics_repository=${JRL_REPO}

jrl_dynamics_urdf_branch=master
jrl_dynamics_urdf_repository=${LAAS_REPO}

sot-core_branch=master
sot-core_repository=${JRL_REPO}

sot-tools_branch=master
sot-tools_repository=${LAAS_REPO}

sot-dynamic_branch=master
sot-dynamic_repository=${SOT_REPO}

sot-dyninv_branch=master
sot-dyninv_repository=${SOT_REPO}

soth_branch=master
soth_repository=${SOT_REPO}

dynamic_graph_bridge_branch=master
dynamic_graph_bridge_repository=${NG_REPO}

dynamic_graph_bridge_msgs_branch=master
dynamic_graph_bridge_msgs_repository=${SOT_REPO}

sot_robot_branch=hydro
sot_robot_repository=${NG_REPO}


sot_robot.configure.dep: eigen3.install sot-dyninv.install dynamic_graph_bridge.install 

all: sot_robot.install

sot_robot.configure.dep: eigen3.install sot-dyninv.install dynamic_graph_bridge.install \ sot_robot.checkout

jrl_dynamics_urdf.configure.dep: sot-dynamic.install \ jrl_dynamics_urdf.checkout

dynamic_graph_bridge_msgs.configure.dep: \ dynamic_graph_bridge_msgs.checkout

dynamic_graph_bridge.configure.dep:dynamic_graph_bridge_msgs.install jrl_dynamics_urdf.install \ dynamic_graph_bridge.checkout

eigen3.configure.dep: eigen3.checkout

soth.configure.dep: soth.checkout

jrl-mathtools.configure.dep:  jrl-mathtools.checkout

jrl-mal.configure.dep: jrl-mathtools.install \ jrl-mal.checkout

abstract-robot-dynamics.configure.dep: abstract-robot-dynamics.checkout

jrl-dynamics.configure.dep: jrl-mal.install abstract-robot-dynamics.install \ jrl-dynamics.checkout

dynamic-graph.configure.dep: jrl-mal.install \ dynamic-graph.checkout

dynamic-graph-python.configure.dep: dynamic-graph.install \ dynamic-graph-python.checkout

sot-core.configure.dep: dynamic-graph-python.install \ sot-core.checkout

sot-tools.configure.dep: sot-core.install \ sot-tools.checkout

sot-dynamic.configure.dep: jrl-dynamics.install sot-tools.install \ sot-dynamic.checkout

sot-dyninv.configure.dep: sot-core.install soth.install/ sot-dyninv.checkout


status:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		cd "$$child_dir";\
		echo \
		"\033[1;36m------- Folder $$child_dir ---------------\033[0m"; \
		git --no-pager -c status.showUntrackedFiles=no status --short --branch; \
		cd ..; \
	done

log:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".log; \
	done

update:
	for d in dynamic-graph ; do \
	echo "Updating $$d";\
	make $$d.update; done

%.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		git clone --recursive -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/$(@:.checkout=); \
	fi \

%.update:
	cd ${SRC_DIR}/$(@:.update=);\
	git remote rm origin;\
	git remote add origin ${$(@:.update=)_repository}/$(@:.update=);\
	git fetch origin;\
	git checkout -b bce46g origin/${$(@:.update=)_branch};\
	git branch -D ${$(@:.update=)_branch};\
	git checkout -b ${$(@:.update=)_branch} bce46g;\
	git branch -D bce46g;\
	git submodule update

%.configure:%.configure.dep
	cd ${SRC_DIR}/$(@:.configure=);\
	mkdir -p build; \
	cd ${SRC_DIR}/$(@:.configure=)/build; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" ${$(@:.configure=)_extra_flags} ..

%.install:%.configure
	cd ${SRC_DIR}/$(@:.install=)/build;\
	make install

%.uninstall:
	cd ${SRC_DIR}/$(@:.uninstall=)/build;\
	make uninstall

%.clean:
	cd ${SRC_DIR}/$(@:.clean=)/build;\
	make clean

%.very-clean:
	rm -rf ${SRC_DIR}/$(@:.very-clean=)/build/*

%.status:
	cd ${SRC_DIR}/$(@:.status=); git status

%.log:
	@cd ${SRC_DIR}/$(@:.log=); \
	if [ -f .git/refs/heads/${$(@:.log=)_branch} ]; then \
		echo -n "$(@:.log=): "; \
		cat .git/refs/heads/${$(@:.log=)_branch}; \
	fi

eigen3.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		wget -O eigen3.tar.bz2 \
		http://bitbucket.org/eigen/eigen/get/3.0.5.tar.bz2;\
		cd ${SRC_DIR}; bunzip2 eigen3.tar.bz2; tar xvf eigen3.tar;\
		rm -f eigen3.tar; mv eigen-eigen-6e7488e20373 eigen3;\
#		rm -f eigen3.tar; mv eigen-eigen-6b38706d90a9 eigen3;\
	fi

eigen3.configure: eigen3.configure.dep
	cd ${SRC_DIR}/eigen3;\
	mkdir -p build; \
	cd ${SRC_DIR}/eigen3/build; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -Dpkg_config_libdir=${DEVEL_DIR}/install/lib ..




.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

.PHONY: clean
clean:
	@rm -rf build/ install/ log/ logs/ tests/

.PHONY: clean-docs
clean-docs:
	@rm -rf cross_reference/ docs_build/ docs_output/

.PHONY: purge
purge:
	@rm -rf build/ install/ log/ logs/ tests/ src/external

.PHONY: clean-test
clean-test:
	source ./tools/scripts/source_all.sh
	colcon test-result --delete-yes

.PHONY: vcs-import
vcs-import:
	@VCS_FILE="${VCS_FILE}"
	vcs import < ${VCS_FILE}

.PHONY: build-debug
build-debug:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to basestation_launch
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to ${PACKAGES}
	fi

.PHONY: build
build:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to basestation_launch
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to ${PACKAGES}
	fi

.PHONY: build-select
build-select:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ${PACKAGES}

.PHONY: build-select-debug
build-select-debug:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select ${PACKAGES}

.PHONY: build-ci
build-ci:
	source ./tools/scripts/source_all.sh
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld -DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=lld -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=lld --packages-up-to autonomy_launch tools_launch

.PHONY: test-ci
test-ci:
	source ./tools/scripts/source_all.sh
	colcon test --return-code-on-test-failure --packages-select $(shell cat .github/packages_for_test.txt); colcon test-result --verbose || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1)

.PHONY: test-select
test-select:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon test --packages-select $(shell cat .github/packages_for_test.txt); colcon test-result --verbose
	else
		colcon test --packages-select ${PACKAGES}; colcon test-result --verbose
	fi

.PHONY: test-cpp
test-cpp:
	source ./tools/scripts/source_all.sh
	ament_uncrustify ${PATHS}
	ament_cpplint ${PATHS}

.PHONY: reformat
reformat:
	source ./tools/scripts/source_all.sh
	autoflake --in-place --remove-unused-variables --remove-all-unused-imports --ignore-init-module-imports -r ${PATHS}
	black -l 99 ${PATHS}
	ament_uncrustify --reformat ${PATHS}

.PHONY: rosdep-install
rosdep-install:
	source ./tools/scripts/source_all.sh
	sudo apt update
	rosdep update
	rosdep install -y -r --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src

.PHONY: rosdep-install-eol
rosdep-install-eol:
	source ./tools/scripts/source_all.sh
	sudo apt update
	rosdep update --include-eol-distros
	rosdep install -y -r --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src

.PHONY: rosdep-install-list
rosdep-install-list:
	source ./tools/scripts/source_all.sh
	rosdep update --include-eol-distros
	rosdep install --as-root "apt:false pip:false" --simulate --reinstall --ignore-src -r -y --rosdistro ${ROS_DISTRO} --from-paths . | sort >> tools/image/ros-deps

.PHONY: build-docker-cpu-foxy
build-docker-cpu-foxy:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target robocar_image \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=ubuntu:20.04 \
		--build-arg ROS_DISTRO=foxy \
		--build-arg ROS_SOURCE=foxy \
		--build-arg ROS_INSTALL=ros-install.sh\
		--build-arg SKIP_KEYS=skip-keys \
		--build-arg APT_FILE=apt-packages \
		--build-arg APT_GPU_FILE=empty-deps \
		--build-arg PIP_FILE=pip3-packages \
		--build-arg PIP_GPU_FILE=empty-deps \
		--build-arg PYTORCH_FILE=pytorch-cpu \
		--build-arg EXPORTS_SCRIPT=exports.sh \
		--build-arg EXPORTS_GPU_SCRIPT=empty-script.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-foxy.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs.sh \
		--build-arg CYCLONEDDS_SCRIPT=cyclonedds-foxy.xml \
		-t ${IMG_NAME} .


.PHONY: build-docker-gpu-foxy-jetpack5
build-docker-gpu-foxy-jetpack5:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target robocar_image \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.4.1 \
		--build-arg ROS_DISTRO=foxy \
		--build-arg ROS_SOURCE="foxy" \
		--build-arg ROS_INSTALL=ros-install.sh \
		--build-arg SKIP_KEYS=skip-keys \
		--build-arg APT_FILE=apt-packages-jetson \
		--build-arg APT_GPU_FILE=empty-deps \
		--build-arg PIP_FILE=pip3-packages-jetson \
		--build-arg PIP_GPU_FILE=pip3-packages-jetson-gpu \
		--build-arg PYTORCH_FILE=pytorch-gpu-jetson \
		--build-arg EXPORTS_SCRIPT=exports-jetson.sh \
		--build-arg EXPORTS_GPU_SCRIPT=empty-script.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-foxy-jetson.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs-jetson.sh \
		--build-arg CYCLONEDDS_SCRIPT=cyclonedds-foxy.xml \
		-t ${IMG_NAME} .


.PHONY: session
session:
	@CONT_NAME="${CONT_NAME}"
	@IMG_NAME="${IMG_NAME}"
	@RUNTIME="${RUNTIME}"
	@INTERACTIVE="${INTERACTIVE}"
	if [ "${INTERACTIVE}" == "true" ]; then
		INTERACTIVE_FLAGS="-it"
		# ENTRYPOINT="/bin/bash"
	else
		INTERACTIVE_FLAGS=""
		ENTRYPOINT="tail -f /dev/null"
	fi
	if [ "${RUNTIME}" = "nvidia" ]; then
		echo "RUNTIME is set to nvidia"
		xhost +
		docker run \
			--name ${CONT_NAME} \
			--runtime nvidia \
			$${INTERACTIVE_FLAGS} \
			--rm \
			--privileged \
			--net=host \
			--gpus all \
			-e NVIDIA_DRIVER_CAPABILITIES=all \
			-e DISPLAY=${DISPLAY} \
			-v /dev/bus/usb:/dev/bus/usb \
			--device-cgroup-rule='c 189:* rmw' \
			--device /dev/video0 \
			--volume='/dev/input:/dev/input' \
			--volume='${HOME}/.Xauthority:/root/.Xauthority:rw' \
			--volume='/tmp/.X11-unix/:/tmp/.X11-unix' \
			--volume='${PWD}:/opt/robocar' \
			${IMG_NAME} $${ENTRYPOINT}
	else
		xhost +
		docker run \
			--name ${CONT_NAME} \
			$${INTERACTIVE_FLAGS} \
			--rm \
			--privileged \
			--net=host \
			-e DISPLAY=${DISPLAY} \
			-v /dev/bus/usb:/dev/bus/usb \
			--device-cgroup-rule='c 189:* rmw' \
			--device /dev/video0 \
			--volume='/dev/input:/dev/input' \
			--volume='${HOME}/.Xauthority:/root/.Xauthority:rw' \
			--volume='/tmp/.X11-unix/:/tmp/.X11-unix' \
			--volume='${PWD}:/opt/robocar' \
			${IMG_NAME} $${ENTRYPOINT}
	fi

.PHONY: join-session
join-session:
	@CONT_NAME="${CONT_NAME}"
	docker exec -it ${CONT_NAME} /bin/bash

.PHONY: image-update
image-update:
	docker pull ghcr.io/ucsd-ecemae-148/donkeycontainer:ros

.PHONY: docker-cache-clean
docker-cache-clean:
	docker builder prune --all --force

.PHONY: sick-driver
sick-driver:
	vcs import < dsc.repos
	./tools/scripts/sick_driver.sh

P1_HOST_DIR = ${PWD}/src/external/drivers/p1-host-tools
.PHONY: p1-driver
p1-driver:
	@GNSS_DIR="${GNSS_DIR}" # /dev/ttyUSB0 or /dev/ttyUSB1
	apt install python3.8-venv
	python3 -m venv ${P1_HOST_DIR}/p1_tools_venv
	source ${P1_HOST_DIR}/p1_tools_venv/bin/activate
	pip3 install -r ${P1_HOST_DIR}/requirements.txt	
	${P1_HOST_DIR}/bin/config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms --device ${GNSS_DIR}
	${P1_HOST_DIR}/bin/config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms --device ${GNSS_DIR}
	${P1_HOST_DIR}/bin/config_tool.py apply uart2_message_rate fe ROSIMUMessage 100ms --device ${GNSS_DIR}
	${P1_HOST_DIR}/bin/config_tool.py save --device ${GNSS_DIR}
	deactivate

.PHONY: p1-runner
p1-runner:
	@GNSS_DIR="${GNSS_DIR}" # /dev/ttyUSB0 or /dev/ttyUSB1
	@USERNAME="${USERNAME}" # gbUv1nO2
	@PASSWORD="${PASSWORD}" # zZLxzLpJ
	source ${P1_HOST_DIR}/p1_tools_venv/bin/activate
	${P1_HOST_DIR}/bin/runner.py --device-id ${USERNAME} --polaris ${PASSWORD} --device-port ${GNSS_DIR}
	deactivate

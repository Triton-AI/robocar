.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

.PHONY: build
build:
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: rosdep-install
rosdep-install:
	sudo apt update
	rosdep update
	rosdep install -y -r --rosdistro foxy --ignore-src --from-paths src

.PHONY: rosdep-install-eol
rosdep-install-eol:
	sudo apt update
	rosdep update --include-eol-distros
	rosdep install -y -r --rosdistro foxy --ignore-src --from-paths src

.PHONY: image-update
image-update:
	docker pull ghcr.io/ucsd-ecemae-148/donkeycontainer:ros

.PHONY: rocker-nvidia
rocker-nvidia:
	@CONT_NAME="${CONT_NAME}"
	rocker --network host --nvidia runtime -e NVIDIA_DRIVER_CAPABILITIES=all --git --ssh --x11 --privileged --name ${CONT_NAME} --user --volume ${shell pwd} -- ghcr.io/ucsd-ecemae-148/donkeycontainer:ros

.PHONY: sick-driver
sick-driver:
	vcs import < repos/dsc.repos
	./scripts/sick_driver.sh
	
.PHONY: livox-driver
livox-driver:
	vcs import < repos/livox.repos
	./scripts/livox_driver.sh

.PHONY: racer
racer:
	vcs import < repos/racer.repos

.PHONY: sim
sim:
	vcs import < repos/sim.repos
	pip install -e src/external/f1tenth_sim/f1tenth_gym

.PHONY: gb_opt
gb_opt:
	chmod +x scripts/gb_opt_setup.sh
	./scripts/gb_opt_setup.sh

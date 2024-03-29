SHELL := /bin/bash

# Author: Dr. Konstantin Selyunin

include .env

# HELP
# This will output the help for each task
# thanks to https://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help

help: ## Display this help
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

.DEFAULT_GOAL := help

.EXPORT_ALL_VARIABLES:
DISPLAY ?=

.PHONY: build
build: ## Build docker image
	docker compose build cyton-gamma

.PHONY: shell
shell: ## Start the bash shell inside the `cyton-gamma` docker container
	xhost +
	docker compose run --rm cyton-gamma /bin/bash

.PHONY: urdf-in-rviz
urdf-in-rviz: ## Visualize URDF mesh in RVIZ
	xhost +
	docker compose run --rm cyton-gamma-urdf-in-rviz

.PHONY: urdf-in-gazebo
urdf-in-gazebo: ## Visualize URDF mesh in gazebo simulator
	xhost +
	docker compose run --rm cyton-gamma-urdf-in-gazebo

.PHONY: joint-effort-controller
joint-effort-controller: ## Use joint_effort_controller from ros_control to steer the arm in Gazebo
	xhost +
	docker compose run --rm cyton-gamma-joint-effort-controller

.PHONY: joint-position-controller
joint-position-controller: ## Use joint_position_controller from ros_control to steer the arm in Gazebo
	xhost +
	docker compose run --rm cyton-gamma-joint-position-controller

.PHONY: joint-trajectory-controller
joint-trajectory-controller: ## Use joint_state_controller from ros_control to steer the arm in Gazebo
	xhost +
	docker compose run --rm cyton-gamma-joint-trajectory-controller

.PHONY: gazebo-moveit
gazebo-moveit: ## Launch MoveIt with Gazebo and joint_state_controller
	xhost +
	docker compose run --rm cyton-gamma-gazebo-moveit

.PHONY: robot-manipulator-manager
robot-manipulator-manager: ## Launch manipulator manager
	xhost +
	docker compose run --rm cyton-gamma-robot-manipulator-manager

.PHONY: robot-manipulator-controller-spawner
robot-manipulator-controller-spawner: ## Launch controller spawner for manipulator
	xhost +
	docker compose run --rm cyton-gamma-robot-manipulator-controller-spawner

.PHONY: robot-gripper-manager
robot-gripper-manager: ## Launch gripper manager
	xhost +
	docker compose run --rm cyton-gamma-robot-gripper-manager

.PHONY: robot-gripper-controller-spawner
robot-gripper-controller-spawner: ## Launch controller spawner for gripper
	xhost +
	docker compose run --rm cyton-gamma-robot-manipulator-controller-spawner


.PHONY: robot-moveit-movegroup
robot-moveit-movegroup: ## Launch MoveIt move_group
	xhost +
	docker compose run --rm cyton-gamma-robot-moveit-movegroup


.PHONY: robot-moveit-setup-assistant
robot-moveit-setup-assistant: ## Launch MoveIt setup assistant
	xhost +
	docker compose run --rm cyton-gamma-moveit-setup-assistant


.PHONY: robot-moveit
robot-moveit: ## Launch MoveIt with physical robot and dynamixel controllers
	xhost +
	docker compose run --rm cyton-gamma-robot-moveit
SHELL := /bin/bash
.DEFAULT_GOAL := help

PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
MUJOCO_PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,$(if $(wildcard IPE-quest-hand-teleop/.venv/bin/python),IPE-quest-hand-teleop/.venv/bin/python,$(PYTHON)))
SIDE ?= right
PORT ?= 8000
LISTEN ?= 0
CONTROL_FILE ?=

.PHONY: help setup check arm-sim arm-teleop hand-teleop ui hand-control _sim-viewer

help:
	@echo "FR5 + IH01 + Quest 3"
	@echo "  make setup         install local simulation dependencies (once)"
	@echo "  make check         run hardware-free project checks"
	@echo "  make arm-sim       open manual FR5 + IH01 simulation"
	@echo "  make arm-teleop    open clutch panel and linked simulation (Quest app stays in headset)"
	@echo "  make hand-teleop  Quest 3 -> IH01 sim + optional hardware (prompts mapping)"
	@echo "  make ui            open the simple simulation dashboard"
	@echo "  make hand-control  open the preserved IH01 manual-control console"

setup:
	python3 -m venv .venv
	.venv/bin/pip install -e '.[dev,sim]'

check:
	$(PYTHON) scripts/check_portability.py
	$(PYTHON) scripts/generate_combined_mujoco.py
	$(PYTHON) -m compileall -q src tests scripts
	PYTHONPATH=src $(PYTHON) -m pytest
	PYTHONPATH=src $(MUJOCO_PYTHON) -m fairino_fr5_vr.combined_sim --headless

arm-sim: _sim-viewer

_sim-viewer:
	$(PYTHON) scripts/generate_combined_mujoco.py
	PYTHONPATH=src $(MUJOCO_PYTHON) -m fairino_fr5_vr.combined_sim $(if $(filter 1,$(LISTEN)),--listen --host 127.0.0.1 --port "$(PORT)" --side "$(SIDE)",--no-grasp-props) $(if $(CONTROL_FILE),--control-file "$(CONTROL_FILE)")

arm-teleop:
	bash scripts/start_sim_teleop.sh

hand-teleop:
	$(MAKE) -C IPE-quest-hand-teleop wired-hardware $(if $(filter command line override,$(origin SIDE)),SIDE="$(SIDE)")

ui:
	@echo "正在验证实体灵巧手所需的 sudo 权限（本次 UI 会话只验证一次）..."
	@sudo -k
	@sudo -v
	$(PYTHON) scripts/teleop_ui.py

hand-control:
	$(MAKE) -C IPE-quest-hand-teleop hand-control $(if $(filter command line override,$(origin SIDE)),SIDE="$(SIDE)")

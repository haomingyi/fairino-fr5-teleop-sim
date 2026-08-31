SHELL := /bin/bash
.DEFAULT_GOAL := help

PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
MUJOCO_PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,$(if $(wildcard IPE-quest-hand-teleop/.venv/bin/python),IPE-quest-hand-teleop/.venv/bin/python,$(PYTHON)))
SIDE ?= right
PORT ?= 8000
LISTEN ?= 0

.PHONY: help setup check arm-sim arm-teleop sim sim-teleop hand-teleop hand-hardware ui hand-control _sim-viewer

help:
	@echo "FR5 + IH01 + Quest 3"
	@echo "  make setup         install local simulation dependencies (once)"
	@echo "  make check         run hardware-free project checks"
	@echo "  make arm-sim       open manual FR5 + IH01 simulation"
	@echo "  make arm-teleop    start Quest app and the linked simulation"
	@echo "  make hand-teleop  Quest 3 -> physical IH01 only (prompts for hand)"
	@echo "  make ui            open the simple simulation dashboard"
	@echo "  make hand-control  open the preserved IH01 manual-control console"

setup:
	python3 -m venv .venv
	.venv/bin/pip install -e '.[dev,sim]'

check:
	$(PYTHON) scripts/generate_combined_mujoco.py
	$(PYTHON) -m compileall -q src tests scripts
	PYTHONPATH=src $(PYTHON) -m pytest
	PYTHONPATH=src $(MUJOCO_PYTHON) -m fairino_fr5_vr.combined_sim --headless

arm-sim sim: _sim-viewer

_sim-viewer:
	$(PYTHON) scripts/generate_combined_mujoco.py
	PYTHONPATH=src $(MUJOCO_PYTHON) -m fairino_fr5_vr.combined_sim $(if $(filter 1,$(LISTEN)),--listen --host 127.0.0.1 --port "$(PORT)" --side "$(SIDE)")

arm-teleop sim-teleop:
	bash scripts/start_sim_teleop.sh

hand-teleop hand-hardware:
	$(MAKE) -C IPE-quest-hand-teleop wired-hardware $(if $(filter command line override,$(origin SIDE)),SIDE="$(SIDE)")

ui:
	$(PYTHON) scripts/teleop_ui.py

hand-control:
	$(MAKE) -C IPE-quest-hand-teleop hand-control $(if $(filter command line override,$(origin SIDE)),SIDE="$(SIDE)")

SHELL := /bin/bash
.DEFAULT_GOAL := help

# Optional machine-local EtherCAT settings. Never commit this file.
-include .env

PROJECT := $(abspath .)
VENV ?= $(PROJECT)/.venv
PYTHON ?= $(VENV)/bin/python
RUNTIME_SRC := $(PROJECT)/runtime
BRIDGE_SRC := $(PROJECT)/ih01_bridge/src
PYTHONPATH_LOCAL := $(RUNTIME_SRC):$(BRIDGE_SRC)
SIDE ?=
PORT ?= 8000
HOST ?= 0.0.0.0
APP_ID ?= com.haoming.ipe.handteleop
FORCE_INSTALL ?= 0
# Keep the distributable APK beside this Makefile so the portable folder can
# be copied to another workstation without relying on Unity's generated tree.
HTS_APK ?= $(CURDIR)/hand_tracking_streamer.apk
IH01_INTERFACE ?= enp130s0
IH01_SLAVE ?= 1
IH01_TELEOP_SPEED_STEPS_S ?= 1000
# Discover a local Unity 6 Hub installation without pinning this checkout to
# one workstation or editor patch version. Override with UNITY=/path/to/Unity.
UNITY ?= $(firstword $(sort $(wildcard $(HOME)/Unity/Hub/Editor/6000*/Editor/Unity)))
UNITY_PROJECT := $(CURDIR)/hand_tracking_streamer

.PHONY: help provenance bootstrap status install reverse wireless-connect launch receive sim hardware wired-sim wireless-sim wired-hardware wireless-hardware hand-control check unity-build build-backend clean-local

help:
	@echo "IPE Quest Hand Teleop"
	@echo "  make bootstrap                     create the local Python environment"
	@echo "  make check                         protocol/bridge tests"
	@echo "  make status                        Quest ADB authorization"
	@echo "  make unity-build                   build the IH01-owned APK"
	@echo "  make install reverse               install-if-missing APK and prepare transport"
	@echo "  FORCE_INSTALL=1 make install       replace APK intentionally (may restart app)"
	@echo "  make launch                        optionally start the Quest app via ADB"
	@echo "  make wireless-connect QUEST_IP=x   switch ADB to Wi-Fi"
	@echo "  make sim SIDE=left|right|both      HTS -> MuJoCo, read-only (default left)"
	@echo "  make wired-sim                    wired Quest + MuJoCo; prompts for hand"
	@echo "  make wireless-sim                 Wi-Fi Quest + MuJoCo; prompts for hand"
	@echo "  make build-backend                 build bundled SOEM/IH01 backend locally"
	@echo "  make hand-control                  manual sliders + feedback console"
	@echo "  make wired-hardware               wired Quest + IH01; prompts for mapping"
	@echo "  make wireless-hardware            Wi-Fi Quest + IH01; prompts for mapping"
	@echo "  make clean-local                   preview generated-only local cleanup"

provenance:
	@sed -n '1,120p' UPSTREAM.md

bootstrap:
	bash scripts/bootstrap.sh

status:
	bash scripts/check_quest.sh

install: status
	APP_ID="$(APP_ID)" FORCE_INSTALL="$(FORCE_INSTALL)" HTS_APK="$(HTS_APK)" bash scripts/install_ih01_apk.sh

reverse: status
	bash scripts/setup_adb_reverse.sh "$(PORT)"

wireless-connect:
	@test -n "$(QUEST_IP)" || (echo "Usage: make wireless-connect QUEST_IP=192.168.x.x"; exit 64)
	bash scripts/connect_quest_wifi.sh "$(QUEST_IP)"

launch: status
	@if adb shell pidof "$(APP_ID)" | grep -q '[0-9]'; then \
		echo "PASS: Quest app already running; keeping existing session"; \
	else \
		component="$$(adb shell cmd package resolve-activity --brief \
			-a android.intent.action.MAIN -c android.intent.category.LAUNCHER "$(APP_ID)" \
			| tr -d '\r' | tail -n 1)"; \
		case "$$component" in */*) ;; \
			*) echo "FAIL: no launcher activity found for $(APP_ID)" >&2; exit 3;; \
		esac; \
		adb shell am start -n "$$component"; \
	fi

receive:
	@$(PYTHON) --version >/dev/null || (echo "Run 'make bootstrap' first."; exit 127)
	PYTHONPATH="$(PYTHONPATH_LOCAL)" $(PYTHON) -m hts_ih01.live --host "$(HOST)" --port "$(PORT)" --side "$(if $(SIDE),$(SIDE),left)"

sim:
	@$(PYTHON) --version >/dev/null || (echo "Run 'make bootstrap' first."; exit 127)
	PYTHONPATH="$(PYTHONPATH_LOCAL)" $(PYTHON) -m hts_ih01.live --host "$(HOST)" --port "$(PORT)" --side "$(if $(SIDE),$(SIDE),left)" --simulate

wired-sim: status reverse
	@side="$$(bash scripts/select_side.sh "$(SIDE)")" || exit $$?; \
	  echo "Selected hand: $$side"; \
	  $(MAKE) sim SIDE="$$side" HOST="$(HOST)" PORT="$(PORT)"

wireless-sim: status
	@side="$$(bash scripts/select_side.sh "$(SIDE)")" || exit $$?; \
	  echo "Selected hand: $$side"; \
	  $(MAKE) sim SIDE="$$side" HOST="$(HOST)" PORT="$(PORT)"

build-backend:
	bash scripts/build_ethercat_backend.sh

hardware:
	@$(PYTHON) --version >/dev/null || (echo "Run 'make bootstrap' first."; exit 127)
	$(MAKE) build-backend
	@sudo -v
	PYTHONPATH="$(PYTHONPATH_LOCAL)" $(PYTHON) -m hts_ih01.live --host "$(HOST)" --port "$(PORT)" --side "$(SIDE)" --hardware --interface "$(IH01_INTERFACE)" --slave "$(IH01_SLAVE)" --speed "$(IH01_TELEOP_SPEED_STEPS_S)" --backend "$(PROJECT)/build/ih01_hand_control_backend"

wired-hardware:
	@side="$$(bash scripts/select_side.sh "$(SIDE)")" || exit $$?; \
	$(MAKE) status reverse SIDE="$$side"; \
	$(MAKE) hardware SIDE="$$side" HOST="$(HOST)" PORT="$(PORT)"

wireless-hardware:
	@side="$$(bash scripts/select_side.sh "$(SIDE)")" || exit $$?; \
	$(MAKE) status; \
	$(MAKE) hardware SIDE="$$side" HOST="$(HOST)" PORT="$(PORT)"

hand-control: build-backend
	@bash scripts/run_hand_control.sh

check:
	@$(PYTHON) --version >/dev/null || (echo "Run 'make bootstrap' first."; exit 127)
	PYTHONPATH="$(PYTHONPATH_LOCAL)" $(PYTHON) -m pytest -q ih01_bridge/tests

unity-build:
	test -x "$(UNITY)" || (echo "Unity not found: $(UNITY)"; exit 127)
	@if test -e "$(UNITY_PROJECT)/Temp/UnityLockfile"; then \
		if fuser "$(UNITY_PROJECT)/Temp/UnityLockfile" >/dev/null 2>&1; then \
			echo "Unity is currently using this project; close it before batch building."; exit 66; \
		else \
			echo "Removing stale Unity project lock."; rm -f "$(UNITY_PROJECT)/Temp/UnityLockfile"; \
		fi; \
	fi
	"$(UNITY)" -batchmode -quit -projectPath "$(UNITY_PROJECT)" -buildTarget Android -executeMethod IH01Build.Build -logFile -
	@if test -s "$(UNITY_PROJECT)/Builds/ih01-quest-hand-teleop.apk"; then \
		cp -f "$(UNITY_PROJECT)/Builds/ih01-quest-hand-teleop.apk" "$(HTS_APK)"; \
		echo "PASS: copied portable APK to $(HTS_APK)"; \
	else \
		echo "FAIL: Unity completed without $(UNITY_PROJECT)/Builds/ih01-quest-hand-teleop.apk" >&2; exit 5; \
	fi

clean-local:
	bash scripts/clean_local.sh

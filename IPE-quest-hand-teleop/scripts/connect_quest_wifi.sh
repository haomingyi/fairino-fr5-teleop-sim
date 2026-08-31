#!/usr/bin/env bash
set -euo pipefail

# Switch an already-authorized Quest from USB ADB to TCP/IP ADB.
# Author: haoming
quest_ip="${1:?Usage: connect_quest_wifi.sh QUEST_IP [ADB_PORT]}"
adb_port="${2:-5555}"
bash "$(dirname "$0")/check_quest.sh"
adb tcpip "$adb_port"
sleep 1
adb connect "${quest_ip}:${adb_port}"
echo "PASS: Quest ADB Wi-Fi target ${quest_ip}:${adb_port}"
echo "Configure the Quest app TCP host to this PC's LAN address and port 8000."

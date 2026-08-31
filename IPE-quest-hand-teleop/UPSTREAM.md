# Upstream provenance

This directory is an IH01-maintained fork of:

- Repository: https://github.com/wengmister/hand-tracking-streamer
- Upstream commit: `5ff7c1c` (`v1.1.0`)
- License: Apache-2.0 (see `LICENSE`)

The Unity project is kept as a separate application so the original
`experiments/quest3-hand-tracking` route remains independent.  IH01 additions
are under `ih01_bridge/` and are deliberately outside the upstream Unity
project until the protocol and retargeting changes are validated.

When pulling a newer upstream release, export a clean commit and compare the
protocol files before replacing this snapshot.  Do not copy Unity `Library/`,
`Temp/`, `Logs/`, or `UserSettings/` directories into Git.

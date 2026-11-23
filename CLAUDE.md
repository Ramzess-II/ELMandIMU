# ELMandIMU: agent instructions

ESP32 sensor box firmware (ESP-IDF 5.5, C) for the NonGPS Maps app, repo `Ramzess-II/NoNGPS-Maps`. The box sends
the gyro and OBD speed over BLE. The fork rules live in the app repo (`.claude/rules/nogps-*.md`), and a session
started here does not see them, so the ones that matter for firmware are repeated below.

## Build and flash

- `. ~/esp/esp-idf/export.sh` (ESP-IDF 5.5), then `idf.py build`; ESP32-C3:
  `idf.py -B build_c3 -D SDKCONFIG=sdkconfig.c3 -D IDF_TARGET=esp32c3 build`.
  `idf.py -p <port> flash monitor` writes the whole image (not `app-flash`).
- The image is signed. Without `keys/nogps_signing_key.pem` build unsigned: turn off "Require signed app images" in
  `menuconfig`. Never commit `keys/`.
- CI (`.github/workflows/build.yml`) builds S3 and C3 on every PR and push to `main`, or by hand
  (`workflow_dispatch`), with a throwaway signing key generated in the job: its images are not for a box.
  GitHub Actions has started no run in this repo so far, so a PR can still arrive without a check (#29).
- `README.md` (Russian, written by the owner) has the details: wiring, signing, release (`tools/make_release.py`).

## The protocol is shared with the app

`docs/nogps/esp32-firmware-spec.md` in the app repo is the box protocol that the firmware, the app and the box
simulator (`tools/python/nogps_box_sim.py` there) all follow. A change to a line, field, command, flag, cap or
error is made in both repos:

- the firmware here, the spec, the app's parser and the simulator in the app repo, with a test in
  `tools/python/nogps_box_sim_test.py`;
- a cap in `NGI` is reported only once the simulator does what it promises;
- a firmware change that leaves the protocol alone (driver, OBD, power rules) does not touch the simulator.

## How we work

- Write English only: issues, PRs, comments, commits, docs. Where a text needs a local language, use Ukrainian.
  Existing Russian text in `README.md` stays as it is.
- Work you don't finish now goes into a GitHub issue before the session ends or the PR opens, not into docs, PR
  text or code comments. Issues about the app and the protocol go to `Ramzess-II/NoNGPS-Maps`.
- Changes go through a fork and a PR; the owner merges. Never push to `main`.
- Commit messages: `[subsystem] Summary in imperative mood` (max 80 chars), `git commit -s`.
- In the PR say what was built and what was run on a box, and what was not checked on one.

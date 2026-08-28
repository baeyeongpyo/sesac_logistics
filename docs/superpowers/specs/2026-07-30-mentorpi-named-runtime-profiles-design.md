# MentorPi Named Runtime Profiles Design

## Goal

Require an explicitly named, local runtime profile for every `run.sh` command
so development PCs and Linux servers run from checked configuration rather than
repeated shell exports. Adding a new development environment must require only
one new local file, not a `run.sh` change.

## Command Interface

Every command begins with an environment selector:

```bash
./run.sh --env dev build
./run.sh --env dev sim-up
./run.sh --env dev1 test
./run.sh --env server sim-up
```

`--env <profile>` is required, appears before the command, and maps directly
to `vehicle_simulator_model/ubuntu/.env.<profile>`. Profiles are not a fixed
enumeration: a profile name matching `^[A-Za-z0-9][A-Za-z0-9_-]*$` is valid.
Thus `dev1`, `dev2`, and future names require only a corresponding local file.
Invalid names, a missing selector, a missing profile file, or repeated
selectors fail with exit code 2 before Docker is invoked.

## Profile Files and Precedence

Local files are ignored by Git:

```text
.env.dev
.env.dev1
.env.server
```

Tracked templates provide the starting point:

```text
.env.dev.example
.env.server.example
```

The selected profile is authoritative. Every assignment in it overwrites an
already exported shell variable; defaults in `run.sh` apply only to variables
not provided by the profile. This prevents a stale terminal export from
silently changing a selected profile. Docker Compose is also invoked with the
same selected file as its explicit `--env-file`, so Compose does not discover
an unrelated implicit `.env` file.

The parser remains data-only: blank lines, comments, and `NAME=value` are the
only accepted input. It never sources or evaluates a profile. The existing
`.env` automatic loader and `.env.example` are removed; use of a bare
`./run.sh <command>` is intentionally rejected.

## Environment Semantics

The development template uses Docker-internal networking:

```dotenv
SIM_NETWORK_MODE=internal
MENTORPI_IMAGE=mentorpi-sim:harmonic
TARGET_PLATFORM=linux/amd64
```

The server template uses LAN or VPN networking and requires the operator to
replace the example server address:

```dotenv
SIM_NETWORK_MODE=lan
GZ_SERVER_IP=192.168.50.10
MENTORPI_IMAGE=mentorpi-sim:harmonic
TARGET_PLATFORM=linux/amd64
```

Profile names do not imply networking behavior. Any profile may select
`internal` or `lan`; the values inside the selected file remain the source of
truth. A Mac development profile normally stays `internal`, because Docker
Desktop cannot provide the supported native-Gazebo-GUI transport path.

## Documentation and Verification

README commands will consistently show `--env <profile>`. It will include
copy-and-edit flows for `.env.dev` and `.env.server`, and explain that adding
`.env.dev2` needs no launcher change.

Regression coverage will prove all of the following with a fake Docker binary:

1. `--env dev1` loads `.env.dev1` and selects its Compose profile.
2. A value in the selected file overrides an inherited shell export.
3. Missing, malformed, invalid-name, and duplicate profile selections fail
   before Docker is called.
4. Compose receives the same profile through `--env-file`.
5. The previous bare `.env` path and bare launcher invocation are rejected.

The full existing `./run.sh --env dev test` suite must remain green.

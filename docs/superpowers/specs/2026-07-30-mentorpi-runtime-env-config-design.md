# MentorPi Runtime `.env` Configuration Design

## Goal

Allow a Linux simulation server to persist its runtime network configuration in
`vehicle_simulator_model/ubuntu/.env` so operators can run `./run.sh sim-up`
without prefixing every command with `SIM_NETWORK_MODE` and `GZ_SERVER_IP`.

## Scope

- Add a tracked `.env.example` with safe example values.
- Let `run.sh` load an optional sibling `.env` before it selects the Compose
  network profile.
- Keep the caller's exported environment as the highest-priority override.
- Document the one-time server setup and the LAN / VPN client connection flow.
- Add automated coverage for loading, override precedence, invalid syntax, and
  the existing `internal` default.

## Non-goals

- Do not create or commit a real `.env` file containing a server address.
- Do not add remote SSH resource verification or asset manifests.
- Do not change Gazebo Transport, port exposure, VPN, or browser viewer
  behavior.

## Configuration Contract

`vehicle_simulator_model/ubuntu/.env` is an optional local file. It accepts
plain dotenv assignments and comments:

```dotenv
# Remote Linux server configuration
SIM_NETWORK_MODE=lan
GZ_SERVER_IP=192.168.50.10
MENTORPI_IMAGE=mentorpi-sim:harmonic
```

Only blank lines, lines beginning with `#`, and `NAME=value` assignments are
accepted. `NAME` uses shell environment-variable naming rules. This is
deliberately a data file, not a shell script: command substitution, `export`
statements, and arbitrary shell syntax are rejected.

The precedence order is:

```text
exported command environment > .env > existing code default
```

For example, a server normally configured for LAN mode can perform a one-off
internal run with `SIM_NETWORK_MODE=internal ./run.sh sim-up`.

Without `.env`, the existing default of `SIM_NETWORK_MODE=internal` remains
unchanged. When LAN mode is selected, `GZ_SERVER_IP` remains required.

## Operator Flow

On the Linux server, copy the template once and edit only the local file:

```bash
cd vehicle_simulator_model/ubuntu
cp .env.example .env
# edit GZ_SERVER_IP to the server LAN or VPN address
./run.sh sim-up
```

The developer GUI client continues to use its actual LAN or VPN IP with
`scripts/gz-gui-connect.sh`. No public IP, NAT rule, or Gazebo Transport port
forwarding is introduced by this configuration.

## Implementation Boundaries

- `run.sh` owns loading because it must read `SIM_NETWORK_MODE` before Compose
  is invoked.
- Docker Compose will continue to consume the same `.env` values for its own
  substitutions.
- `.env.example` is the only tracked environment file; `.env` is already
  ignored at repository level.
- Tests will be placed in a new dedicated test module so existing user changes
  in `test_bundle.py` are not overwritten.

## Acceptance Criteria

1. A server with `.env` containing the LAN values can run `./run.sh sim-up`.
2. An exported environment variable overrides the value in `.env`.
3. Missing `.env` retains current internal-mode behavior.
4. Invalid `.env` input fails before Docker is called and explains the line
   that is invalid.
5. The documented setup never asks an operator to commit `.env`.

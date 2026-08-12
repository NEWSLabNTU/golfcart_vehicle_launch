#!/usr/bin/env bash
# Run a command inside its own tmux session, and block until it exits.
#
# Written for use as a ros2 launch `launch-prefix`, so that a node needing a
# terminal of its own (raw-tty keyboard input) does not have to share one with
# the launch log — interleaved log lines would scramble its interface.
#
#   run_in_tmux.sh SESSION -- <cmd> [args...]
#
# launch-prefix is split on whitespace and prepended to the node's argv, so the
# node command (including --ros-args) arrives as the trailing arguments.
#
# The wrapper must outlive the command: launch treats the prefix process exiting
# as the node having died. It therefore blocks on the pane and forwards the exit
# status, and kills the session on the way out so a launch shutdown never leaves
# an orphan behind.
set -uo pipefail

usage() {
    echo "Usage: $(basename "$0") SESSION -- <cmd> [args...]" >&2
}

if [ "$#" -lt 3 ]; then
    usage
    exit 2
fi

SESSION="$1"
shift
if [ "$1" != "--" ]; then
    echo "Expected '--' after the session name, got '$1'" >&2
    usage
    exit 2
fi
shift

if ! command -v tmux >/dev/null 2>&1; then
    echo "run_in_tmux.sh: tmux is not installed. Install it with:" >&2
    echo "    sudo apt install tmux" >&2
    exit 127
fi

# Refuse to touch a session someone else is using rather than stealing or
# silently reusing its name.
if tmux has-session -t "=${SESSION}" 2>/dev/null; then
    echo "run_in_tmux.sh: tmux session '${SESSION}' already exists." >&2
    echo "    attach: tmux attach -t ${SESSION}" >&2
    echo "    kill:   tmux kill-session -t ${SESSION}" >&2
    exit 1
fi

# tmux copies the client environment into a new session, but a tmux server that
# was already running keeps the environment it started with, so the ROS bits are
# passed explicitly. Unset variables are skipped.
ENV_ARGS=()
for var in \
    AMENT_PREFIX_PATH \
    CMAKE_PREFIX_PATH \
    COLCON_PREFIX_PATH \
    CYCLONEDDS_URI \
    FASTRTPS_DEFAULT_PROFILES_FILE \
    LD_LIBRARY_PATH \
    PATH \
    PYTHONPATH \
    RMW_IMPLEMENTATION \
    ROS_DISTRO \
    ROS_DOMAIN_ID \
    ROS_LOCALHOST_ONLY \
    ROS_PACKAGE_PATH \
    ROS_PYTHON_VERSION \
    ROS_VERSION \
    VEHICLE_ID
do
    if [ -n "${!var:-}" ]; then
        ENV_ARGS+=(-e "${var}=${!var}")
    fi
done

# tmux takes the command as a single string run through sh, so quote our argv
# back into one.
CMD=$(printf '%q ' "$@")

# The pane writes the command's exit status here and then parks, rather than
# exiting: a pane that exits takes the session with it, and a command that fails
# instantly would be gone before we could read anything back. Parking also keeps
# the scrollback attachable after a crash.
STATUS_FILE=$(mktemp -t "run_in_tmux.${SESSION}.XXXXXX")
PANE_CMD=$(printf '%s; echo $? > %q; exec sleep 86400' "${CMD}" "${STATUS_FILE}")

cleanup() {
    trap - EXIT INT TERM
    # Session first, status file second: the pane writes its status on the way
    # out, and would recreate a file removed before it died.
    if tmux has-session -t "=${SESSION}" 2>/dev/null; then
        if [ ! -s "${STATUS_FILE}" ]; then
            # Still running: SIGINT first, since rclcpp shuts down cleanly on it,
            # unlike the SIGHUP that kill-session sends.
            tmux send-keys -t "${SESSION}:" C-c 2>/dev/null
            for _ in 1 2 3 4 5 6; do
                [ -s "${STATUS_FILE}" ] && break
                sleep 0.25
            done
        fi
        tmux kill-session -t "=${SESSION}" 2>/dev/null
    fi
    rm -f "${STATUS_FILE}"
}
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap cleanup EXIT

if ! tmux new-session -d -s "${SESSION}" -c "${PWD}" "${ENV_ARGS[@]}" "${PANE_CMD}"; then
    echo "run_in_tmux.sh: failed to create tmux session '${SESSION}'" >&2
    exit 1
fi

echo "run_in_tmux.sh: '${SESSION}' started. Attach with: tmux attach -t ${SESSION}"
echo "run_in_tmux.sh:   detach again with Ctrl-b d (the session keeps running)"

# Block until the command exits, so launch keeps treating the node as alive.
STATUS=0
while true; do
    if [ -s "${STATUS_FILE}" ]; then
        STATUS=$(tr -dc '0-9' < "${STATUS_FILE}")
        [ -n "${STATUS}" ] || STATUS=0
        echo "run_in_tmux.sh: '${SESSION}' exited with status ${STATUS}. Last output:"
        tmux capture-pane -p -t "${SESSION}:" -S -30 2>/dev/null \
            | sed 's/[[:space:]]*$//' | grep -v '^$' | tail -20 | sed 's/^/    /'
        break
    fi
    if ! tmux has-session -t "=${SESSION}" 2>/dev/null; then
        # Session went away underneath us - someone killed it by hand.
        echo "run_in_tmux.sh: session '${SESSION}' is gone."
        break
    fi
    sleep 0.5
done

exit "${STATUS}"

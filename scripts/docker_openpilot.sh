#!/bin/bash

set -e

XAUTHORITY_FILE_ROW=($(xauth info | head -n 1))
XAUTHORITY_FILE="${XAUTHORITY_FILE_ROW[@]:2}"

DOCKER_IMAGE="openpilot_dev:1.0.1"
COMMA_OPTIONS="-v $HOME/.comma:/root/.comma -v /tmp/comma_download_cache:/tmp/comma_download_cache"
GUI_OPTIONS="-v /tmp/.X11-unix:/tmp/.X11-unix  -v $XAUTHORITY_FILE:/root/.Xauthority -h $HOSTNAME -e DISPLAY=$DISPLAY --shm-size 1G -e QT_X11_NO_MITSHM=1"
DOCKER_RUN="docker run --rm -it $COMMA_OPTIONS $GUI_OPTIONS" 

COMMAND="$1"

if [[ $COMMAND == "replay" ]]; then
    $DOCKER_RUN $DOCKER_RUN /bin/bash -c "/home/batman/openpilot/selfdrive/ui/ui & /home/batman/openpilot/tools/replay/replay ${@:2}"
elif [[ $COMMAND == "cabana" ]]; then
    $DOCKER_RUN $DOCKER_RUN /bin/bash -c "/home/batman/openpilot/tools/cabana/cabana ${@:2}"
elif [[ $COMMAND == "pj" ]]; then
    $DOCKER_RUN $DOCKER_RUN python3 /home/batman/openpilot/tools/plotjuggler/juggle.py ${@:2}
elif [[ $COMMAND == "shell" ]]; then
    $DOCKER_RUN $DOCKER_RUN /bin/bash
elif [[ $COMMAND == "work" ]]; then
    OPENPILOT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
    while [[ $(basename $OPENPILOT_ROOT) != *"openpilot"* ]]; do
        OPENPILOT_ROOT="$(dirname $OPENPILOT_ROOT)"
        if [[ $OPENPILOT_ROOT == "/" ]]; then
            echo "Error: Could not find openpilot root. Are you in an openpilot (or descendant) directory?"
            exit 1
        fi
    done

    WORK_DIR_OPTIONS="-v $OPENPILOT_ROOT:$OPENPILOT_ROOT -w $OPENPILOT_ROOT"
    USER_OPTIONS="-u $(id -u):$(id -g)"
    PYTHONPATH_OPTIONS="-e PYTHONPATH=$OPENPILOT_ROOT:"
    DEV_OPTIONS="$WORK_DIR_OPTIONS $USER_OPTIONS $PYTHONPATH_OPTIONS"

    SUBCOMMAND="$2"

    if [[ $SUBCOMMAND == "build" ]]; then
        $DOCKER_RUN $DEV_OPTIONS $DOCKER_IMAGE /bin/bash -c 'scons --cache-readonly -j$(nproc)'
    elif [[ $SUBCOMMAND == "shell" ]]; then
        $DOCKER_RUN $DEV_OPTIONS $DOCKER_IMAGE /bin/bash $BASH_ARG
    elif [[ $SUBCOMMAND == "run" ]]; then
        if [[ $# -le 2 ]]; then
            echo "Usage: $0 work run <command> [args...]"
            exit 1
        fi
        BASH_ARG="-c '${@:3}'"
        $DOCKER_RUN $DEV_OPTIONS $DOCKER_IMAGE /bin/bash -c "${@:3}"
    else
        echo "Error: Unknown subcommand: $SUBCOMMAND"
        echo "Usage: $0 work <command> [args...]"
        echo ""
        echo "Commands:"
        echo "  build       build openpilot"
        echo "  shell       open a shell in the docker container"
        echo "  run         run a command in the docker container"
        exit 1
    fi
else
    echo "Error: Unknown command: $COMMAND"
    exit 1
fi


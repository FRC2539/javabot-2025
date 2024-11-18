#!/bin/sh
# to setup, run "sh switch.sh setup"

if [ $# -ne 2 ]; then
    if [ "$1" = "setup" ]; then
        git config alias.name '!sh switch.sh'
        echo "Command 'git name' has been setup."
        exit 0
    fi
    echo "Usage: git name <email> <name>"
    echo "Current email: $(git config --global user.email)"
    echo "Current name: $(git config --global user.name)"
    exit 0
fi

git config --global user.email "$1"

git config --global user.name "$2"
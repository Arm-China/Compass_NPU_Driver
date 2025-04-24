#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "installing git hooks..."
cp -r ${SCRIPT_DIR}/git_hooks/* `git rev-parse --git-dir`/hooks/
echo "git hooks setup done!"

#!/usr/bin/env bash
# Source this file once to set up the VTOL simulation environment.
# Usage:  source /path/to/vtol_env.sh
#
# After sourcing, run in two terminals:
#   Terminal 1: gz sim -v4 -r "$VTOL_WORLDS_DIR/vtol_empty.sdf"
#   Terminal 2: python3 "$VTOL_SCRIPTS_DIR/mavlink_controller.py"

# Locate the installed share directory (works whether sourced from source tree
# or from the colcon install tree).
_this_script="${BASH_SOURCE[0]:-$0}"
_scripts_dir="$(cd "$(dirname "$_this_script")" && pwd)"

# If running from the install tree:  …/share/vtol_description/scripts/
# If running from the source tree:   …/src/vtol_description/scripts/
_share_dir="$(dirname "$_scripts_dir")"

export VTOL_SCRIPTS_DIR="$_scripts_dir"
export VTOL_WORLDS_DIR="$_share_dir/worlds"
export VTOL_MODELS_PARENT="$_share_dir"   # models/ lives under this

# Prepend our models directory so Gazebo resolves model://vtol
if [ -z "$GZ_SIM_RESOURCE_PATH" ]; then
    export GZ_SIM_RESOURCE_PATH="$VTOL_MODELS_PARENT/models"
else
    export GZ_SIM_RESOURCE_PATH="$VTOL_MODELS_PARENT/models:$GZ_SIM_RESOURCE_PATH"
fi

echo "VTOL env ready."
echo "  Models  : $VTOL_MODELS_PARENT/models"
echo "  Worlds  : $VTOL_WORLDS_DIR"
echo "  Scripts : $VTOL_SCRIPTS_DIR"
echo ""
echo "Run:"
echo "  gz sim -v4 -r \"\$VTOL_WORLDS_DIR/vtol_standalone.sdf\""
echo "  python3 \"\$VTOL_SCRIPTS_DIR/mavlink_controller.py\""

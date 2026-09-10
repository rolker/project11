#!/bin/bash
# ci_local upstream pruning hook (see .agent/scripts/ci_local.sh, #577).
# Run inside the CI container with cwd /ci/upstream_ws, after the
# upstream.repos clones and before the upstream build.
#
# vcs clones whole repos, but this repo needs only a few packages out of each
# one declared in upstream.repos. Without pruning colcon also builds the rest,
# and several of those depend on repos we deliberately do NOT clone --
# marine_nav_ca_safety needs marine_control, which nothing here uses. Those
# fail and take the whole run down with them (observed on #375 before this
# hook existed).
#
# COLCON_IGNORE everything except the transitive in-repo closure of what this
# repo actually consumes. This is industrial_ci's
# AFTER_SETUP_UPSTREAM_WORKSPACE pruning, done locally. Mirrors
# mru_transform/.agents/ci_local_upstream_extra.sh.
#
# geographic_info is NOT pruned: geodesy needs geographic_msgs from the same
# repo, and the third package is a metapackage that costs nothing.
#
# What each kept package is for:
#   marine_nav_interfaces  <- mission_manager, mission_manager_interfaces,
#                             marine_autonomy_integration_tests
#   marine_nav_tasks       <- mission_manager
#   marine_nav_utilities   <- pulled in by marine_nav_tasks itself
#   marine_ais_msgs        <- marine_web_view
set -euo pipefail

prune_repo() {
    local repo_dir="$1"; shift
    local keep=(" $@ ")

    if [[ ! -d "$repo_dir" ]]; then
        echo "ci_local_upstream_extra: $repo_dir not present; nothing to prune" >&2
        return 0
    fi

    local pruned=0 kept=0 manifest pkg_dir name
    while IFS= read -r manifest; do
        pkg_dir=$(dirname "$manifest")
        name=$(basename "$pkg_dir")
        if [[ "${keep[*]}" == *" $name "* ]]; then
            kept=$((kept + 1))
            continue
        fi
        touch "$pkg_dir/COLCON_IGNORE"
        pruned=$((pruned + 1))
    done < <(find "$repo_dir" -name package.xml)

    echo "ci_local_upstream_extra: $repo_dir - kept $kept, pruned $pruned"

    # A rename upstream must fail the run loudly, not silently drop a
    # dependency and surface later as a confusing find_package error.
    local k
    for k in "$@"; do
        if ! find "$repo_dir" -path "*/$k/package.xml" -print -quit | grep -q .; then
            echo "ci_local_upstream_extra: ERROR: $k not found in $repo_dir" >&2
            return 1
        fi
    done
}

prune_repo "src/unh_marine_navigation" \
    marine_nav_interfaces marine_nav_tasks marine_nav_utilities
prune_repo "src/marine_ais" marine_ais_msgs

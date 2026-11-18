#!/usr/bin/env bash
# Run the offline test suite. ROS Humble installs a pytest plugin (launch_testing) that crashes
# with newer pytest versions, so plugin auto-loading is switched off for our tests.
cd "$(dirname "$0")" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider "$@"

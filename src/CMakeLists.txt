INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

PYTHON_INSTALL("dynamic_graph/sot/dynamics" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/dynamics" "mobile_robot_v3.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "robot_v3.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "prologue.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "sot_interface_v3.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "trajectories.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "derivative_filters.py" "${PYTHON_SITELIB}")





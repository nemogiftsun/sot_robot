INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

PYTHON_INSTALL("dynamic_graph/sot/dynamics" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/dynamics" "mobile_robot.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "robot.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "prologue.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/ur" "sot_interface.py" "${PYTHON_SITELIB}")





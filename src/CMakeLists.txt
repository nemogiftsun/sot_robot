INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

PYTHON_INSTALL("dynamic_graph/sot/dynamics" "mobile_robot.py" "${PYTHON_SITELIB}")

PYTHON_INSTALL("dynamic_graph/sot/youbot" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/youbot" "robot.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/youbot" "prologue.py" "${PYTHON_SITELIB}")


PYTHON_INSTALL("dynamic_graph/sot/pr2" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2" "robot.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2" "prologue.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2" "sot_interface.py" "${PYTHON_SITELIB}")


PYTHON_INSTALL("rqt_rpc" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("rqt_rpc" "rpc_module.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("rqt_rpc" "rpc_module_real.py" "${PYTHON_SITELIB}")





# hrc_utils

This package contains utilitarian tools.

## ServicesCommonServer

This node aims to provide a common server for services that are shared among the nodes.

### Services provided

* `get_robot_pose`

## ExternalParamInterface (Python only)

This node aims to provide an interface to get or set parameters of other nodes.

It must be instantiated in another node to be used.

## Utilitarian functions

This package contains a collection of utilitarian functions that can be used in any node.

The functions are available for C++ and/or Python respectively in `include/hrc_utils/utils.hpp` and `hrc_utils/utils.py`

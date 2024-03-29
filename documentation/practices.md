# Practices

## GitHub

When working on a task, make sure you are working on a separate branch. Push any changes you make to the branch you are working on. When you are ready to merge to the main branch, submit a pull request. After your pull request is reviewed by the Software Lead (and any other individuals related to the merge), the merge will be approved.

## Nodes

### Structure
1. File named `<node_name>_node.py` which handles the creation of the node, and subscribing and publishing of messages through topics.
2. File named `<node_name>.py` which handles the logic of the node (e.g. finding patterns in an image).
3. For autonomy nodes, a file named `<node_name>_states.py` which contains the possible states. 

### Conventions

Nodes should be named `<packages_suffix>_<node_name>` (e.g. perception_scan_code).

Topics should be named `<package_suffix>/<topic>` (e.g. /perception/code)

## Python Conventions

Use PascalCase for class names.

Use snake_case for variable, method, and function names.

Prefix any private methods and variables with an underscore.

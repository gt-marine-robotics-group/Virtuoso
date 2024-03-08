# Launch Arguments

## usv

Depending on the competition (VRX, RobotX, Roboboat) and the environment (simulation or real-world), we may want to launch different sensors or run the same algorithms with different parameters. Rather than create five similar versions of Virtuoso for each competition + environment combination, we use a launch argument to specify the appropriate parameter files of our nodes.

The five possible values for this argument are `vrx`, `vrx_robotx`, `vrx_roboboat`, `robotx`, and `roboboat`.

Likewise, in most package's `config` directory, there are five sub-directories with the same names. Each of these sub-diretories contain parameter files for nodes in the package. The parameter files used are those found in the sub-directory whose name matches the value of the usv argument. For example, if `usv:=roboboat` is provided, only parameters found in the `roboboat` sub-directory will be used.

The `vrx` value corresponds to running Virtuoso for the VRX competition while in the VRX simulation.

The `vrx_robotx` value corresponds to running Virtuoso for the RobotX competition while in the VRX simulation.

The `vrx_roboboat` value corresponds to running Virtuoso for the RoboBoat competition wile in the VRX simulation.

The `robotx` value corresponds to running Virtuoso for the RobotX competition while in the real-world.

The `roboboat` value corresponds to running Virtuoso for the RoboBoat competition while in the real-world.

Additionally, if `roboboat` or `robotx` are passed in, RVIZ will not be launched as it is assumed the launch is being ran over ssh.

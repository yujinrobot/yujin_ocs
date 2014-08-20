== Overview ==

All your launchers should go in here - ros will automatically find them when you
use the roslaunch command.

== Debugging ==

To run gdb on any node in the launch file, simply insert

    launch-prefix="gdb --args"

before any of the node calls or nodelet manager/standalone calls. For example:

    <node launch-prefix="gdb --args" pkg="nodelet" type="nodelet" name="robot_core" ns="robot_core" args="manager"/>


== Launchers ==

To use with a launcher file, call it like follows:

<code>
<rosparam file="$(find goo)/resources/yaml/korus.yaml" command="load" />
</code>

== HowTos ==

Some of the less obvious concepts to loading and reading these from the server.

- joint_trajectory_tension_spline_controller : loading arrays from the parameter server

== Dumping/Loading ==

Use the following two commands to load/dump parameters from the rosparam server. This can save
the effort in launching ros programs all the time.

<code>
rosparam dump "filename.yaml"
rosparam load "filename.yaml"
</code>

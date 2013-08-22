# make file for robbuild system,
all: models/HiroNX.urdf

models/meshes:
	mkdir -p models/meshes

models/HiroNX.urdf: models/meshes
	echo "== generate urdf file from kawada-hironx.dae"
	(cd models/meshes; rosrun collada_tools collada_to_urdf `rospack find hironx_ros_bridge`/models/kawada-hironx.dae -A --mesh_output_dir ${CURDIR}/meshes --mesh_prefix 'package://hironx_moveit_config/models/meshes')


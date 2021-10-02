1. Install and build the ARIAC environment as per their tutorial
2. Place all files in ~/ariac_ws/src/ARIAC/ariac_example/script (othe folders under ARIAC may also work)
3. In a terminal execute the following commands:
	source ~/ariac_ws/devel/setup.bash
	roslaunch nist_gear sample_environment.launch load_moveit:=true
4. Wait for the simulation to load properly, the in a NEW terminal execute:
	source ~/ariac_ws/devel/setup.bash
	rosrun ariac_example my_node.py

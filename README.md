# vrx_wrench2thrust

## Simple procedure manual
1. `cd ouxt_ws`
2. `git clone git@github.com:OUXT-Polaris/vrx_wrench2trust.git`
2. `colcon build --packages-select vrx_wrench2thrust`
3. `source install/setup.bash`
4.  in another terminal, `source ~vrx_ws/install/setup.bash`,`ros2 launch vrx_gz competition.launch.py world:=sydney_regatta`
5.  back to original terminal ,`ros2 run vrx_wrench2thrust vrx_wrench2thrust_node`
5. in another terminal,`ros2 run rqt_gui rqt_gui`, and add a section `Plugins > Topics > Message Publisher`
6. set value like this.
![img.png](rqt_topic.png)

for now, Only respond to topic:`wamv/wrench`,data_field`wrench/force/x,y`.
 

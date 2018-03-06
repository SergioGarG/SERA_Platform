source ~/phd_ws/devel/setup.bash
roslaunch sera dummy.launch &
python ~/phd_ws/src/specificationmanager/rest_python.py &
java -jar ~/phd_ws/src/specificationmanager/specificationmanager.jar &
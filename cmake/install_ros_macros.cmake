

macro(install_ros_ethz)

    execute_process (
        COMMAND bash -c "mkdir ~/.flow/ -p && cd ~/.flow/ && mkdir thirdparty -p && cd thirdparty && mkdir catkin_ethz -p && cd catkin_ethz && mkdir src -p && cd src"
    )

    execute_process (
        COMMAND bash -c "git clone https://github.com/uzh-rpg/rpg_dvs_ros ~/.flow/thirdparty/catkin_ethz/src/rpg_dvs_ros"
    )

    execute_process (
        COMMAND bash -c "git clone https://github.com/catkin/catkin_simple ~/.flow/thirdparty/catkin_ethz/src/catkin_simple"
    )

    execute_process (
        COMMAND bash -c "cd ~/.flow/thirdparty/catkin_ethz && catkin_make dvs_msgs_gencpp"
    )

    execute_process (
        COMMAND bash -c "cd ~/.flow/thirdparty/catkin_ethz && catkin_make"
    )


    execute_process (
        COMMAND bash -c "cd ~/.flow/thirdparty/catkin_ethz && source devel/setup.bash"
    )

    execute_process (
        COMMAND bash -c "echo \"source ~/.flow/thirdparty/catkin_ethz/devel/setup.bash\" >> ~/.bashrc"
    )

endmacro()
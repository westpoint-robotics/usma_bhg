# Aliases used for BHG Project

alias cls='printf "\033c"'
alias bhg_launch='roslaunch usma_bhg master.launch'
alias bhg_start='rostopic pub -1 /record std_msgs/Bool True'
alias bhg_stop='rostopic pub -1 /record std_msgs/Bool False'
alias bhg_mavproxy='mavproxy.py --master=/dev/ttyACM0'
alias bhg_imgview='rqt_image_view'


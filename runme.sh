echo "     -----   running viso2 ros node   -----     "

cd ~/Desktop/stuff/TESE/ROS/code/viso_ws
source devel/setup.bash
catkin_make

ft=1
while [ $ft -lt 8 ]
do

  for bw in 40 45 50 55 60
  do

    for bh in 40 45 50 55 60
    do
      roslaunch viso2_ros DS_RI_46.launch max_features:=$ft bucket_width:=$bw bucket_height:=$bh
    done

  done

  (( ft++ ))
done

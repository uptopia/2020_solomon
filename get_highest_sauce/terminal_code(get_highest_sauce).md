[================================]
[  windows系統的電腦(控上銀手臂)  ]
[================================]
$打開: 本機/windows/opt
$執行ROS(會打開terminal)
$在terminal輸入：$roscore

$再打開另一個terminal
$cd hiwin_ws
$roslaunch huddle_mission pick_and_place_huddle.launch 
$選擇4

[================================]
[  linux系統電腦(影像處理計算用)  ]
[================================]
$cd ~/Documents/solomon_ws
$source devel/setup.bash
$roscore
$roslaunch get_highest_sauce get_highest_sauce.launch	//開realsense, yolov4
$rosrun get_highest_sauce get_highest_sauce		//運算物件點雲高度

[開始執行流程]
轉開紅色按鈕

//====================================================================//
#1.開啟realsense
	$cd rs_d435i_ws
	$source devel/setup.bash
	$roslaunch realsense2_camera rs_rgbd.launch filters:=pointcloud (align_depth:=true)

	*roslaunch realsense2_camera rs_camera.launch

#2.開啟Yolo detector
	$cd solomon_ws
	$source devel/setup.bash
	$roslaunch darknet_ros yolov4_sauce.launch
	
	*roslaunch yolo_detection yolo_get_test.launch

=>1 + 2 = get_highest_sauce.launch

#3.執行get_hightest_sauce程式
	$cd solomon_ws
	$source devel/setup.bash
	$rosrun get_highest_sauce get_highest_sauce


=>用client測試：
	$cd solomon_ws
	$. devel/setup.bash
	$rosrun client client

	=>client丟request給get_highest_sauce,
	get_highest_sauce 回傳 response

[Yolov4 mAP]
$cd ~/Documents/solomon_ws/src/darknet_ros/darknet_ros/config/yolov4_sauce.yaml

To check accuracy mAP@IoU=50: ./darknet detector map data/obj.data yolov4_sauce.cfg yolov4_sauce.weights

#yolov4_sauce.cfg
#yolov4_sauce.weights
#yolov4_sauce.yaml
#train.txt
#obj.names
#obj.data



            // cout << "===============\n" 
            //     << "Top 3 Class\n"
            //     << "===============\n";
            // int top3_cnt = 0;
            // for(int idx = 0; idx < sauces_all.size(); ++idx)
            // {   
            //     std::string obj_class = sauces_all[idx].sauce_class;
            //     if(!obj_class.compare("egg") |\
            //         !obj_class.compare("ketchup") |\
            //         !obj_class.compare("pepper"))
            //     {
            //         if(top3_cnt < 3)
            //         {                    
            //             PointTRGB pt_target;
            //             pt_target.x = sauces_all[idx].center_point.x;
            //             pt_target.y = sauces_all[idx].center_point.y;
            //             pt_target.z = sauces_all[idx].center_point.z;

            //             *top3_clouds = *top3_clouds + *(sauces_all[idx].rgb_cloud);
                
            //             //Plot Center Point on Top1 sauce
            //             if(idx == 0)
            //             {
            //                 target_sauce_center->clear();
            //                 target_sauce_center->push_back(pt_target);
            //             }

            //             top3_cnt++;
            //         }
            //     }
            //     else
            //     {
            //         cout << "Detect" << sauces_all[idx].sauce_class << endl;
            //     }
                
            // }



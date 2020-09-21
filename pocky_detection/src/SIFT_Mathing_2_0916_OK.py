#!/usr/bin/python
# -*- coding: UTF-8 -*-
import cv2 as cv
import numpy as np
import math
import pyrealsense2 as rs
import logging
 
# Configure depth and color streams...
# ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
# config_1.enable_device('908212070822')  #上 自己
config_1.enable_device('912112073118')  #上 手臂
config_1.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# ...from Camera 2
pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device('939622072919')  #下
config_2.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config_2.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming from both cameras
pipeline_1.start(config_1)
pipeline_2.start(config_2)

# dep
align_to_1 = rs.stream.color
align_1 = rs.align(align_to_1)

def dep_img(map1):
    try:
         while True:
            frame_1 = pipeline_1.wait_for_frames()
            aligned_frame_1 = align_1.process(frame_1)   
            depth_frame_1 = aligned_frame_1.get_depth_frame() 
            color_frame_1 = aligned_frame_1.get_color_frame()
            
            if not depth_frame_1 or not color_frame_1:
                continue
        
            depth_image_1 = np.asanyarray(depth_frame_1.get_data())
            color_image_1 = np.asanyarray(color_frame_1.get_data())
            map1 = cv.applyColorMap(cv.convertScaleAbs(depth_image_1, alpha=0.35), cv.COLORMAP_HSV)

            print("1.shape of color image:{0}".format(color_image_1.shape))
            print("1.shape of depth image:{0}".format(map1.shape))

            text_depth_1 = "1.depth value of point (640,360) is "+str(np.round(depth_frame_1.get_distance(640, 360),4))+"meter(s)"
            color_image_1 = cv.circle(color_image_1,(640,360),10,(0,255,255),-1)
            color_image_1 = cv.putText(color_image_1, text_depth_1, (10,20),  cv.FONT_HERSHEY_PLAIN, 1, (0,0,255), 1, cv.LINE_AA)
            print("depth value in m :".format(depth_frame.get_distance(640, 360)))
            
            images_1 = np.hstack((color_image_1, map1))
            cv.namedWindow('RealSense', cv.WINDOW_NORMAL)
            cv.imshow('RealSense', images_1)

            key = cv.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv.destroyAllWindows()
                break
    finally:
        return map1
        pipeline_1.stop()
        
def Matches(image, Match_image, angle, box, point, CenterX, CenterY):
    x = 0
    y = 0
    while y < len(Match_image):
        # cv.imshow("box", Match_image[y])
        # cv.imshow("image", image)

        # 创建sift特征检测器
        sift = cv.xfeatures2d.SIFT_create()
        kp1, des1 = sift.detectAndCompute(Match_image[y],None)
        kp2, des2 = sift.detectAndCompute(image,None)

        # 匹配
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1,des2,k=2)
        goodMatches = []

        # 筛选出好的描述子
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                goodMatches.append(m)
        # print('good',len(goodMatches)

        # 单独保存 obj 和 scene 好的点位置
        obj_pts = []
        scene_pts = []
        if len(goodMatches)>MIN_MATCH_COUNT:
            # 获取关键点的坐标
            obj_pts = np.float32([ kp1[m.queryIdx].pt for m in goodMatches ]).reshape(-1,1,2)
            scene_pts = np.float32([ kp2[m.trainIdx].pt for m in goodMatches ]).reshape(-1,1,2)
            #计算变换矩阵和MASK
            M, mask = cv.findHomography(obj_pts, scene_pts, cv.RANSAC, 5.0)
            matchesMask = mask.ravel().tolist()
            h, w = Match_image[y].shape

            # pts 為box照片的四個角落座標
            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            # cv.circle(box, (0,0), 10, (1, 227, 254), -1) #左上
            # cv.circle(box, (0,440), 10, (1, 227, 254), -1) #左下
            # cv.circle(box, (236,440), 10, (1, 227, 254), -1) #右下
            # cv.circle(box, (236,0), 10, (1, 227, 254), -1) #右上
            # print("PTS : ")
            # print(pts)
            # print(pts[0][0][1]) #左上
            # print(pts[1][0][1]) #左下
            # print(pts[2][0][1]) #右下
            # print(pts[3][0][1]) #右上
            # 使用得对到的变换矩阵原图像的四个角进行变换，获得在目标图像上对应的坐标
            dst = cv.perspectiveTransform(pts, M).reshape(-1, 2)
            # print("DST : ")
            # print(dst)
            # dst[0] 左上 
            # dst[1] 左下 
            # dst[2] 右下 
            # dst[3] 右上

            # 求左上->右下 中心點
            X_abs = int((dst[0][0]+dst[2][0])/2)
            Y_abs = int((dst[0][1]+dst[2][1])/2)
            print("X_abs : ",X_abs)
            print("Y_abs : ",Y_abs)
            cv.circle(image, (X_abs, Y_abs), 10, (1, 227, 254), -1)

            # 求右上->左下 中心點
            X_abs_2 = int((dst[1][0]+dst[3][0])/2)
            Y_abs_2 = int((dst[1][1]+dst[3][1])/2)
            print("X_abs_2 : ",X_abs_2)
            print("Y_abs_2 : ",Y_abs_2)

            # 求左上->右下 斜邊
            X_side = int((dst[0][0]+dst[2][0]))
            Y_side = int((dst[0][1]+dst[2][1]))
            print("X_side : ",X_side)
            print("Y_side : ",Y_side)

            # 求右上->左下 斜邊
            X_side_2 = int((dst[1][0]+dst[3][0]))
            Y_side_2 = int((dst[1][1]+dst[3][1]))
            print("X_side_2 : ",X_side_2)
            print("Y_side_2 : ",Y_side_2)

            X_side_Less = abs(X_side-X_side_2)
            Y_side_Less = abs(Y_side-Y_side_2)
            X_center_point_Less = abs(X_abs-X_abs_2)
            Y_center_point_Less = abs(Y_abs-Y_abs_2)            
            print("X_sidet_Less : ",X_side_Less)
            print("Y_side_Less : ",Y_side_Less)
            print("X_center_point_Less : ",X_center_point_Less)
            print("Y_center_point_Less : ",Y_center_point_Less)

            # 求角度
            X1 = pts[2][0][0] - pts[0][0][0]
            Y1 = pts[2][0][1] - pts[0][0][1]
            # print(X1,Y1)
            X2 = dst[2][0] - dst[0][0]
            Y2 = dst[2][1] - dst[0][1]
            # print(X2,Y2)
            angle1 = np.rad2deg(np.arctan2(Y1,X1))
            angle2 = np.rad2deg(np.arctan2(Y2,X2))
            # print(angle1,angle2)
            angle_diff = angle2 - angle1
            print('angle_1',angle1)
            print('angle_2',angle2)
            print('angle',angle_diff)
            print("scene_pts : ",len(scene_pts))
            # 加上偏移量
            for i in range(4):
                dst[i][0] += w

            draw_params = dict(singlePointColor=None,
                                matchesMask=matchesMask, 
                                flags=2)
            # result = cv.drawMatches(box, kp1, image, kp2, goodMatches, None)
            result = cv.drawMatches(Match_image[y], kp1, image, kp2, goodMatches, None,**draw_params)
            cv.polylines(result, [np.int32(dst)], True, (0, 0, 255), 3, cv.LINE_AA)
            # cv.namedWindow('flann-match', cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
            # cv.imshow('flann-match',result)

            if(X_side>=50 and X_side<=2000) and (Y_side>=50 and Y_side<=2000):
                if ((X_side_Less == 0 and Y_side_Less == 0) or (X_side_Less <= 6 and Y_side_Less <= 6)):
                    if ((X_center_point_Less == 0 and Y_center_point_Less == 0) or (X_center_point_Less <= 5 and Y_center_point_Less <= 5)):
                        angle.append(angle_diff)
                        point.append(len(scene_pts))
                        CenterX.append(X_abs)
                        CenterY.append(Y_abs)
                        if(y==0):
                            box.append('W')                        
                        if(y==1):
                            box.append('R')
                        if(y==2):
                            box.append('Y')
                        if(y==3):
                            box.append('G')
                        if(y==4):
                            box.append('WP')
                        if(y==5):
                            box.append('GP')
        
                y = y + 1
                # cv.waitKey(0)
                # cv.destroyAllWindows()
            
        else:
            print( "Not enough matches are found - %d/%d" % (len(goodMatches),MIN_MATCH_COUNT))
            # matchesMask = None
            y = y + 1

def Matches2(image, Match_image, angle, box, point, CenterX, CenterY):
    x = 0
    y = 0
    while y < len(Match_image):
        # cv.imshow("box", Match_image[y])
        # cv.imshow("image", image)

        # 创建sift特征检测器
        sift = cv.xfeatures2d.SIFT_create()
        kp1, des1 = sift.detectAndCompute(Match_image[y],None)
        kp2, des2 = sift.detectAndCompute(image,None)

        # 匹配
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1,des2,k=2)
        goodMatches = []

        # 筛选出好的描述子
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                goodMatches.append(m)
        # print('good',len(goodMatches)

        # 单独保存 obj 和 scene 好的点位置
        obj_pts = []
        scene_pts = []
        if len(goodMatches)>MIN_MATCH_COUNT:
            # 获取关键点的坐标
            obj_pts = np.float32([ kp1[m.queryIdx].pt for m in goodMatches ]).reshape(-1,1,2)
            scene_pts = np.float32([ kp2[m.trainIdx].pt for m in goodMatches ]).reshape(-1,1,2)
            #计算变换矩阵和MASK
            M, mask = cv.findHomography(obj_pts, scene_pts, cv.RANSAC, 5.0)
            matchesMask = mask.ravel().tolist()
            h, w = Match_image[y].shape

            # pts 為box照片的四個角落座標
            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            # cv.circle(box, (0,0), 10, (1, 227, 254), -1) #左上
            # cv.circle(box, (0,440), 10, (1, 227, 254), -1) #左下
            # cv.circle(box, (236,440), 10, (1, 227, 254), -1) #右下
            # cv.circle(box, (236,0), 10, (1, 227, 254), -1) #右上
            # print("PTS : ")
            # print(pts)
            # print(pts[0][0][1]) #左上
            # print(pts[1][0][1]) #左下
            # print(pts[2][0][1]) #右下
            # print(pts[3][0][1]) #右上
            # 使用得对到的变换矩阵原图像的四个角进行变换，获得在目标图像上对应的坐标
            dst = cv.perspectiveTransform(pts, M).reshape(-1, 2)
            # print("DST : ")
            # print(dst)
            # dst[0] 左上 
            # dst[1] 左下 
            # dst[2] 右下 
            # dst[3] 右上

            # 求左上->右下 中心點
            X_abs = int((dst[0][0]+dst[2][0])/2)
            Y_abs = int((dst[0][1]+dst[2][1])/2)
            print("X_abs : ",X_abs)
            print("Y_abs : ",Y_abs)
            cv.circle(image, (X_abs, Y_abs), 10, (1, 227, 254), -1)

            # 求右上->左下 中心點
            X_abs_2 = int((dst[1][0]+dst[3][0])/2)
            Y_abs_2 = int((dst[1][1]+dst[3][1])/2)
            print("X_abs_2 : ",X_abs_2)
            print("Y_abs_2 : ",Y_abs_2)

            # 求左上->右下 斜邊
            X_side = int((dst[0][0]+dst[2][0]))
            Y_side = int((dst[0][1]+dst[2][1]))
            print("X_side : ",X_side)
            print("Y_side : ",Y_side)

            # 求右上->左下 斜邊
            X_side_2 = int((dst[1][0]+dst[3][0]))
            Y_side_2 = int((dst[1][1]+dst[3][1]))
            print("X_side_2 : ",X_side_2)
            print("Y_side_2 : ",Y_side_2)

            X_side_Less = abs(X_side-X_side_2)
            Y_side_Less = abs(Y_side-Y_side_2)
            X_center_point_Less = abs(X_abs-X_abs_2)
            Y_center_point_Less = abs(Y_abs-Y_abs_2)            
            print("X_sidet_Less : ",X_side_Less)
            print("Y_side_Less : ",Y_side_Less)
            print("X_center_point_Less : ",X_center_point_Less)
            print("Y_center_point_Less : ",Y_center_point_Less)

            # 求角度
            X1 = pts[2][0][0] - pts[0][0][0]
            Y1 = pts[2][0][1] - pts[0][0][1]
            # print(X1,Y1)
            X2 = dst[2][0] - dst[0][0]
            Y2 = dst[2][1] - dst[0][1]
            # print(X2,Y2)
            angle1 = np.rad2deg(np.arctan2(Y1,X1))
            angle2 = np.rad2deg(np.arctan2(Y2,X2))
            # print(angle1,angle2)
            angle_diff = angle2 - angle1
            print('angle_1',angle1)
            print('angle_2',angle2)
            print('angle',angle_diff)
            print("scene_pts : ",len(scene_pts))
            # 加上偏移量
            for i in range(4):
                dst[i][0] += w

            draw_params = dict(singlePointColor=None,
                                matchesMask=matchesMask, 
                                flags=2)
            # result = cv.drawMatches(box, kp1, image, kp2, goodMatches, None)
            result = cv.drawMatches(Match_image[y], kp1, image, kp2, goodMatches, None,**draw_params)
            cv.polylines(result, [np.int32(dst)], True, (0, 0, 255), 3, cv.LINE_AA)
            cv.namedWindow('flann-match', cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
            cv.imshow('flann-match',result)

            if(X_side>=50 and X_side<=2000) and (Y_side>=50 and Y_side<=2000):
                if ((X_side_Less == 0 and Y_side_Less == 0) or (X_side_Less <= 6 and Y_side_Less <= 6)):
                    if ((X_center_point_Less == 0 and Y_center_point_Less == 0) or (X_center_point_Less <= 5 and Y_center_point_Less <= 5)):
                        angle.append(angle_diff)
                        point.append(len(scene_pts))
                        CenterX.append(X_abs)
                        CenterY.append(Y_abs)

                y = y + 1
                # cv.waitKey(0)
                # cv.destroyAllWindows()
            
        else:
            print( "Not enough matches are found - %d/%d" % (len(goodMatches),MIN_MATCH_COUNT))
            # matchesMask = None
            y = y + 1

def Bubble_Sort2(tag,point,obj_pts_all,Center_point1,Center_point2):
    for i in range(len(obj_pts_all)):
        # 二次取出數列的下標
        for j in range(len(obj_pts_all)):
            # 判斷第一次取出下標的值和第二次取出下標的值得大小
            if obj_pts_all[j] > obj_pts_all[i]:
                # 如果第二個數大於第一個數，把第二個數放在第一個數後面
                obj_pts_all[j], obj_pts_all[i] = obj_pts_all[i], obj_pts_all[j]
                tag[j], tag[i] = tag[i], tag[j]
                point[j], point[i] = point[i], point[j]
                Center_point1[j], Center_point1[i] = Center_point1[i], Center_point1[j]
                Center_point2[j], Center_point2[i] = Center_point2[i], Center_point2[j]
    return tag,point

def take_pic(): # 手動拍照                         
    try:
        while(True):
            # Camera 1
            frames_1 = pipeline_1.wait_for_frames()
            color_frame_1 = frames_1.get_color_frame()
            color_image_1 = np.asanyarray(color_frame_1.get_data())

            # Camera 21
            frames_2 = pipeline_2.wait_for_frames()
            color_frame_2 = frames_2.get_color_frame()
            color_image_2 = np.asanyarray(color_frame_2.get_data())

            # Stack all images horizontally
            images = np.hstack((color_image_1,color_image_2))

            # Show images
            cv.namedWindow('RealSense', cv.WINDOW_NORMAL)
            cv.imshow('RealSense', images)
            key = cv.waitKey(1)
            if key & 0xFF == ord('s'):
                cv.imwrite('/home//junan/Desktop/00.png',color_image_1)
                cv.imwrite('/home//junan/Desktop/01.png',color_image_2)

            elif key & 0xFF == ord('q')or key ==27:
                break
    finally:
            pipeline_1.stop()
            pipeline_2.stop() 

def RealSense_dep(map1):
    # try:
    while True:
        frame_1 = pipeline_1.wait_for_frames()
        aligned_frame_1 = align_1.process(frame_1)   
        depth_frame_1 = aligned_frame_1.get_depth_frame() 
        color_frame_1 = aligned_frame_1.get_color_frame()
        
        if not depth_frame_1 or not color_frame_1:
            continue
    
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        map1 = cv.applyColorMap(cv.convertScaleAbs(depth_image_1, alpha=0.35), cv.COLORMAP_HSV)

        # print("1.shape of color image:{0}".format(color_image_1.shape))
        # print("1.shape of depth image:{0}".format(map1.shape))

        text_depth_1 = "1.depth value of point (640,360) is "+str(np.round(depth_frame_1.get_distance(640, 360),4))+"meter(s)"
        color_image_1 = cv.circle(color_image_1,(640,360),10,(0,255,255),-1)
        color_image_1 = cv.putText(color_image_1, text_depth_1, (10,20),  cv.FONT_HERSHEY_PLAIN, 1, (0,0,255), 1, cv.LINE_AA)

        images_1 = np.hstack((color_image_1, map1))
        # cv.namedWindow('1.RealSense', cv.WINDOW_NORMAL)
        # cv.imshow('1.RealSense', images_1)
        
        # pipeline_1.stop()
        return map1
            # key = cv.waitKey(1)
            # if key & 0xFF == ord('q') or key == 27:
            #     cv.destroyAllWindows()
            #     break
    # finally:
    #     return map1
    #     pipeline_1.stop()

def RealSense2(topA,topB):
    # try:
    while True:
        frames_1 = pipeline_1.wait_for_frames()
        aligned_frames_1 = align_1.process(frames_1)   
        depth_frame_1 = aligned_frames_1.get_depth_frame() 
        color_frame_1 = aligned_frames_1.get_color_frame()
        
        if not depth_frame_1 or not color_frame_1:
            continue
    
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())

        depth_colormap_1 = cv.applyColorMap(cv.convertScaleAbs(depth_image_1, alpha=0.35), cv.COLORMAP_HSV)

        z = 0
        x1 = len(topA)
        while(z < x1-20):
            cm = depth_frame_1.get_distance(topA[z],topB[z])
            if(cm < 0.535 and cm > 0.52):
                # print("depth value in m:{0}",cm)
                color_image_1 = cv.circle(color_image_1,(topA[z],topB[z]),10,(0,255,255),-1)
            z = z+1

        images_1 = np.hstack((color_image_1, depth_colormap_1))
        # cv.namedWindow('RealSense_1', cv.WINDOW_AUTOSIZE)
        # cv.imshow('RealSense_1', images_1)
        # cv.waitKey(1)
        # pipeline_1.stop()

        return cm
            # key = cv.waitKey(1)
            # if key & 0xFF == ord('q') or key == 27:
            #     cv.destroyAllWindows()
            #     break

    # finally:
    #     pipeline_1.stop()
    #     return cm

def RealSense3(topA,topB):
    # try:
    while True:
        # frames_1 = pipeline_1.wait_for_frames()
        aligned_frames_1 = align_1.process(frames_1)   
        depth_frame_1 = aligned_frames_1.get_depth_frame() 
        color_frame_1 = aligned_frames_1.get_color_frame()
        
        if not depth_frame_1 or not color_frame_1:
            continue

        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())

        depth_colormap_1 = cv.applyColorMap(cv.convertScaleAbs(depth_image_1, alpha=0.35), cv.COLORMAP_HSV)

        z = 0
        x1 = len(topA)
        while(z < x1-20):
            cm = depth_frame_1.get_distance(topA[z],topB[z])
            if(cm < 0.56 and cm > 0.54):
                # print("depth value in m:{0}",cm)
                color_image_1 = cv.circle(color_image_1,(topA[z],topB[z]),10,(0,255,255),-1)
            z = z+1

        images_1 = np.hstack((color_image_1, depth_colormap_1))

        return cm
            
            # cv.namedWindow('RealSense_1', cv.WINDOW_AUTOSIZE)
            # cv.imshow('RealSense_1', images_1)
                
    #         key = cv.waitKey(1)
    #         if key & 0xFF == ord('q') or key == 27:
    #             cv.destroyAllWindows()
    #             break
    # finally:
    #     return cm
    #     pipeline_1.stop()
    
def contours_demo(image,x,y):
    # gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    ret,thresh1 = cv.threshold(image,0,255,cv.THRESH_BINARY)
    d = cv.absdiff(image, thresh1)
    cv.namedWindow("d", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("d",d)
    hsv = cv.cvtColor(d, cv.COLOR_BGR2HSV) 
    cv.namedWindow("hsv1", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("hsv1",hsv)
    lower_red = np.array([0,48,100]) 
    upper_red = np.array([111, 255, 255]) 
    mask = cv.inRange(hsv, lower_red, upper_red) 
    cv.namedWindow("hsv", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow('hsv',mask)

    # 腐蝕影象
    kernel1 = cv.getStructuringElement(cv.MORPH_RECT, (16, 16))
    kernel2 = cv.getStructuringElement(cv.MORPH_RECT, (12, 12))
    kernel3 = cv.getStructuringElement(cv.MORPH_RECT, (20, 20))
    eroded = cv.erode(mask, kernel1)
    cv.namedWindow("eroded", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("eroded",eroded)

    # 膨脹影象
    dilated = cv.dilate(eroded, kernel2)
    cv.namedWindow("dilated", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("dilated",dilated)

    # 閉運算
    result = cv.morphologyEx(dilated, cv.MORPH_CLOSE, kernel3)
    cv.namedWindow("result", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("result",result)

    #影象邊緣提取
    edge_output = cv.Canny(mask,50,150)
    cv.namedWindow("CannyEdge", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("CannyEdge",edge_output)

    cloneImage,contours,hierarchy = cv.findContours(edge_output,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    for i ,contour in enumerate(contours):
        cv.drawContours(image,contours,i,(0,255,0),-2)

    for c in contours:
        # find minimum area
        # 計算包圍目標的最小矩形區域
        rect = cv.minAreaRect(c)
        # calculate coordinate of the minimum area rectangle
        box = cv.boxPoints(rect)
        # normalize coordinates to integers
        box =np.int0(box)
        # 註：OpenCV沒有函數能直接從輪廓信息中計算出最小矩形頂點的坐標。所以需要計算出最小矩形區域，
        # 然後計算這個矩形的頂點。由於計算出來的頂點坐標是浮點型，但是所得像素的坐標值是整數（不能獲取像素的一部分），
        # 所以需要做一個轉換
        # draw contours
        cv.drawContours(edge_output, [box], 0, (0, 0, 255), 3)  # 畫出該矩形
        cv.circle(image,(box[2][0],box[2][1]),5,(0,255,255),-1)
        x.append(box[2][0])
        y.append(box[2][1])
        # cv.namedWindow("img contours", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
        # cv.imshow("img contours",image)
    return mask

def contours_demo2(image,x,y):
    # gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    ret,thresh1 = cv.threshold(image,0,255,cv.THRESH_BINARY)
    d = cv.absdiff(image, thresh1)
    cv.namedWindow("d", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("d",d)
    hsv = cv.cvtColor(d, cv.COLOR_BGR2HSV) 
    cv.namedWindow("hsv1", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("hsv1",hsv)
    lower_red = np.array([0,43,85]) 
    upper_red = np.array([111, 255, 255]) 
    mask = cv.inRange(hsv, lower_red, upper_red) 
    cv.namedWindow("hsv", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow('hsv',mask)

    # 腐蝕影象
    kernel1 = cv.getStructuringElement(cv.MORPH_RECT, (16, 16))
    kernel2 = cv.getStructuringElement(cv.MORPH_RECT, (12, 12))
    kernel3 = cv.getStructuringElement(cv.MORPH_RECT, (20, 20))
    eroded = cv.erode(mask, kernel1)
    cv.namedWindow("eroded", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("eroded",eroded)

    # 膨脹影象
    dilated = cv.dilate(eroded, kernel2)
    cv.namedWindow("dilated", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("dilated",dilated)

    # 閉運算
    result = cv.morphologyEx(dilated, cv.MORPH_CLOSE, kernel3)
    cv.namedWindow("result", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    cv.imshow("result",result)

    #影象邊緣提取
    edge_output = cv.Canny(mask,50,150)
    # cv.namedWindow("CannyEdge", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
    # cv.imshow("CannyEdge",edge_output)

    cloneImage,contours,hierarchy = cv.findContours(edge_output,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    for i ,contour in enumerate(contours):
        cv.drawContours(image,contours,i,(0,255,0),-2)

    for c in contours:
        # find minimum area
        # 計算包圍目標的最小矩形區域
        rect = cv.minAreaRect(c)
        # calculate coordinate of the minimum area rectangle
        box = cv.boxPoints(rect)
        # normalize coordinates to integers
        box =np.int0(box)
        # 註：OpenCV沒有函數能直接從輪廓信息中計算出最小矩形頂點的坐標。所以需要計算出最小矩形區域，
        # 然後計算這個矩形的頂點。由於計算出來的頂點坐標是浮點型，但是所得像素的坐標值是整數（不能獲取像素的一部分），
        # 所以需要做一個轉換
        # draw contours
        cv.drawContours(edge_output, [box], 0, (0, 0, 255), 3)  # 畫出該矩形
        cv.circle(image,(box[2][0],box[2][1]),5,(0,255,255),-1)
        x.append(box[2][0])
        y.append(box[2][1])
        # cv.namedWindow("img contours", cv.WINDOW_NORMAL) #設置為WINDOW_NORMAL可以任意縮放
        # cv.imshow("img contours",image)
    return mask

def access_pixels(image,image2):       #获取图片像素的函数
    i = 0
    print(image.shape)      #显示图片形状
    hight = image.shape[0]
    width = image.shape[1]
    channels = image.shape[2]
    print("width: %s,hight: %s,channels: %s"%(width,hight,channels))
    for row in range(hight):    #循环改变每个像素点的RGB值
        for col in range(width):
            for c in range(channels):
                pv = image[row,col,c]
                pv2 = image2[row,col,c]
                if(pv2==0):
                    image[row,col,c] = pv-pv
    # cv.imshow("pixels_demo",image)  #创建新的窗口用显示改变后的图片

    return image

if __name__ == '__main__':
    MIN_MATCH_COUNT = 15 #设置最低特征点匹配数量为5
    
    # take_pic()
    # Auto_Take_Pic() 

    # 跳过前300帧以设置自动曝光时间   
    for x in range(50):
        pipeline_1.wait_for_frames()

while True:
    start_input = int(input('開始兩層策略請按1 開始單層策略請按2 手動拍照請按3 深度資料請按4 離開請按5 : ')) #輸入開始指令

    if start_input == 1:
        # Camera 1
        frames_1 = pipeline_1.wait_for_frames()
        color_frame_1 = frames_1.get_color_frame()
        color_image_1 = np.asanyarray(color_frame_1.get_data())

        # Camera 2
        frames_2 = pipeline_2.wait_for_frames()
        color_frame_2 = frames_2.get_color_frame()
        color_image_2 = np.asanyarray(color_frame_2.get_data())

        # Stack all images horizontally
        images = np.hstack((color_image_1,color_image_2))

        # Show images
        # cv.namedWindow('RealSense', cv.WINDOW_NORMAL)
        # cv.imshow('RealSense', images)
        # cv.waitKey(1)

        cv.imwrite('/home//junan/Desktop/00.png',color_image_1)
        cv.imwrite('/home//junan/Desktop/01.png',color_image_2)

        box_in_sence_1 = cv.imread('/home/junan/Desktop/00.png') # A處照片top 輸入圖檔
        box_in_sence_2 = cv.imread('/home//junan/Desktop/01.png',0) # A處照片bottom 輸入圖檔

        # 正面
        box_1 = cv.imread('/home/junan/Desktop/照片/W/w_top.png',0)
        box_2 = cv.imread('/home/junan/Desktop/照片/R/r_top.png',0)
        box_3 = cv.imread('/home/junan/Desktop/照片/Y/y_top_JP.png',0)
        box_4 = cv.imread('/home/junan/Desktop/照片/G/g_top.png',0)
        box_5 = cv.imread('/home/junan/Desktop/照片/W_P/wp_top.png',0)
        box_6 = cv.imread('/home/junan/Desktop/照片/G_P/gp_top.png',0)
        # 反面
        box_7 = cv.imread('/home/junan/Desktop/照片/W/w_bottom.png',0)
        box_8 = cv.imread('/home/junan/Desktop/照片/R/r_bottom.png',0)
        box_9 = cv.imread('/home/junan/Desktop/照片/Y/y_bottom_JP.png',0)
        box_10 = cv.imread('/home/junan/Desktop/照片/G/g_bottom.png',0)
        box_11 = cv.imread('/home/junan/Desktop/照片/W_P/wp_bottom.png',0)
        box_12 = cv.imread('/home/junan/Desktop/照片/G_P/gp_bottom.png',0)

        box_list_1 = [box_1,box_2,box_3,box_4,box_5,box_6]
        # box_list_1 = []
        # box_list_2 = [box_7,box_8,box_9,box_10,box_11,box_12]
        box_list_2 = []
        top_box = []
        top_Angle = []
        top_point = []
        top_CenterX = []
        top_CenterY = []
        bottom_box = []
        bottom_Angle = []
        bottom_point = []
        bottom_CenterX = []
        bottom_CenterY = []
        abc = []

        #   dep
        depth_colormap_1 = []
        box_x = []
        box_y = []
        dep = RealSense_dep(depth_colormap_1)
        # print(top)
        src = dep

        AA = contours_demo(src,box_x,box_y)
        dep_cm = RealSense2(box_x,box_y)
        # print(AA)
        cv.imwrite('AA.png', AA)
        #   dep

        src2 = cv.imread("/home/junan/Desktop/AA.png")
        top_img = access_pixels(box_in_sence_1,src2)

        top = Matches(top_img,box_list_1,top_Angle,top_box,top_point,top_CenterX,top_CenterY)
        if(len(top_box)==6):
            out = Bubble_Sort2(top_box,top_Angle,top_point,top_CenterX,top_CenterY) #tag and 角度
            print('top_box : ',top_box)
            print('top_CenterX : ',top_CenterX)
            print('top_CenterY : ',top_CenterY)
            print('top_Angle : ',top_Angle) 
            print('top_point : ',top_point)
            print('bottom_box : ',bottom_box)
            print('bottom_CenterX : ',bottom_CenterX)
            print('bottom_CenterY : ',bottom_CenterY)
            print('bottom_Angle : ',bottom_Angle) 
            print('bottom_point : ',bottom_point) 
            print('********** 結束 **********') 
        else :
            n = len(top_box)
            out = Bubble_Sort2(top_box,top_Angle,top_point,top_CenterX,top_CenterY) #tag and 角度
            a = (6-n)
            print(a)
            print('top_box : ',top_box)
            print('top_CenterX : ',top_CenterX)
            print('top_CenterY : ',top_CenterY)
            print('top_Angle : ',top_Angle) 
            print('top_point : ',top_point) 
            loop = a
            if((loop)>0):
                print(loop)
                while 1:
                    if 'W' not in top_box:
                        box_7 = cv.imread('/home/junan/Desktop/照片/W/w_bottom.png',0)
                        # bottom_box.append('W')
                        box_list_2.append(box_7)
                        loop = loop-1
                    if 'R' not in top_box:
                        box_8 = cv.imread('/home/junan/Desktop/照片/R/r_bottom.png',0)
                        # bottom_box.append('R')
                        box_list_2.append(box_8)
                        loop = loop-1
                    if 'Y' not in top_box:
                        box_9 = cv.imread('/home/junan/Desktop/照片/Y/y_bottom_JP.png',0)
                        # bottom_box.append('Y')
                        box_list_2.append(box_9)
                        loop = loop-1
                    if 'G' not in top_box:
                        box_10 = cv.imread('/home/junan/Desktop/照片/G/g_bottom.png',0)
                        # bottom_box.append('G')
                        box_list_2.append(box_10)
                        loop = loop-1
                    if 'WP' not in top_box:
                        box_11 = cv.imread('/home/junan/Desktop/照片/W_P/wp_bottom.png',0)
                        # bottom_box.append('WP')
                        box_list_2.append(box_11)
                        loop = loop-1
                    if 'GP' not in top_box:
                        box_12 = cv.imread('/home/junan/Desktop/照片/G_P/gp_bottom.png',0)
                        # bottom_box.append('GP')
                        box_list_2.append(box_12)
                        loop = loop-1
                    if(loop<=0):
                        break
            
            bottom = Matches2(box_in_sence_2,box_list_2,bottom_Angle,bottom_box,bottom_point,bottom_CenterX,bottom_CenterY)
            
            loop2 = a
            if((loop2)>0):
                print(loop2)
                while 1:
                    if box_7 in box_list_2:
                        bottom_box.append('W')
                        loop2 = loop2-1
                    if box_8 in box_list_2:
                        bottom_box.append('R')
                        loop2 = loop2-1
                    if box_9 in box_list_2:
                        bottom_box.append('Y')
                        loop2 = loop2-1
                    if box_10 in box_list_2:
                        bottom_box.append('G')
                        loop2 = loop2-1
                    if box_11 in box_list_2:
                        bottom_box.append('WP')
                        loop2 = loop2-1
                    if box_12 in box_list_2:
                        bottom_box.append('GP')
                        loop2 = loop2-1
                    if(loop2<=0):
                        break
            # while True:
            #     if(len(bottom_Angle) != a and len(bottom_point) != a):
            #         bottom = Matches2(box_in_sence_2,box_list_2,bottom_Angle,bottom_box,bottom_point,bottom_Center)
            #     else:
            #         break

            print(bottom_box)
            out = Bubble_Sort2(bottom_box,bottom_Angle,bottom_point,bottom_CenterX,bottom_CenterY) #tag and 角度
            print('top_box : ',top_box)
            print('top_CenterX : ',top_CenterX)
            print('top_CenterY : ',top_CenterY) 
            print('top_Angle : ',top_Angle) 
            print('top_point : ',top_point)
            print('bottom_box : ',bottom_box)
            print('bottom_CenterX : ',bottom_CenterX)
            print('bottom_CenterY : ',bottom_CenterY) 
            print('bottom_Angle : ',bottom_Angle) 
            print('bottom_point : ',bottom_point)
            print('********** 結束 **********')

    if start_input == 2:
            # Camera 1
            frames_1 = pipeline_1.wait_for_frames()
            color_frame_1 = frames_1.get_color_frame()
            color_image_1 = np.asanyarray(color_frame_1.get_data())

            # Camera 2
            frames_2 = pipeline_2.wait_for_frames()
            color_frame_2 = frames_2.get_color_frame()
            color_image_2 = np.asanyarray(color_frame_2.get_data())

            # Stack all images horizontally
            images = np.hstack((color_image_1,color_image_2))

            # Show images
            # cv.namedWindow('RealSense', cv.WINDOW_NORMAL)
            # cv.imshow('RealSense', images)
            # cv.waitKey(1)

            cv.imwrite('/home//junan/Desktop/00.png',color_image_1)
            cv.imwrite('/home//junan/Desktop/01.png',color_image_2)

            box_in_sence_1 = cv.imread('/home/junan/Desktop/00.png') # A處照片top 輸入圖檔
            box_in_sence_2 = cv.imread('/home//junan/Desktop/01.png',0) # A處照片bottom 輸入圖檔

            # 正面
            box_1 = cv.imread('/home/junan/Desktop/照片/W/w_top.png',0)
            box_2 = cv.imread('/home/junan/Desktop/照片/R/r_top.png',0)
            box_3 = cv.imread('/home/junan/Desktop/照片/Y/y_top_JP.png',0)
            box_4 = cv.imread('/home/junan/Desktop/照片/G/g_top.png',0)
            box_5 = cv.imread('/home/junan/Desktop/照片/W_P/wp_top.png',0)
            box_6 = cv.imread('/home/junan/Desktop/照片/G_P/gp_top.png',0)
            # 反面
            box_7 = cv.imread('/home/junan/Desktop/照片/W/w_bottom.png',0)
            box_8 = cv.imread('/home/junan/Desktop/照片/R/r_bottom.png',0)
            box_9 = cv.imread('/home/junan/Desktop/照片/Y/y_bottom_JP.png',0)
            box_10 = cv.imread('/home/junan/Desktop/照片/G/g_bottom.png',0)
            box_11 = cv.imread('/home/junan/Desktop/照片/W_P/wp_bottom.png',0)
            box_12 = cv.imread('/home/junan/Desktop/照片/G_P/gp_bottom.png',0)

            box_list_1 = [box_1,box_2,box_3,box_4,box_5,box_6]
            # box_list_1 = []
            # box_list_2 = [box_7,box_8,box_9,box_10,box_11,box_12]
            box_list_2 = []
            top_box = []
            top_Angle = []
            top_point = []
            top_CenterX = []
            top_CenterY = []
            bottom_box = []
            bottom_Angle = []
            bottom_point = []
            bottom_CenterX = []
            bottom_CenterY = []
            abc = []

            #   dep
            depth_colormap_1 = []
            box_x = []
            box_y = []
            dep = RealSense_dep(depth_colormap_1)
            # print(top)
            src = dep

            AA = contours_demo2(src,box_x,box_y)
            dep_cm = RealSense3(box_x,box_y)
            # print(AA)
            cv.imwrite('AA.png', AA)
            #   dep

            src2 = cv.imread("/home/junan/Desktop/AA.png")
            top_img = access_pixels(box_in_sence_1,src2)

            top = Matches(top_img,box_list_1,top_Angle,top_box,top_point,top_CenterX,top_CenterY)
            if(len(top_box)==6):
                out = Bubble_Sort2(top_box,top_Angle,top_point,top_CenterX,top_CenterY) #tag and 角度
                print('top_box : ',top_box)
                print('top_CenterX : ',top_CenterX)
                print('top_CenterY : ',top_CenterY)
                print('top_Angle : ',top_Angle) 
                print('top_point : ',top_point)
                print('bottom_box : ',bottom_box)
                print('bottom_CenterX : ',bottom_CenterX)
                print('bottom_CenterY : ',bottom_CenterY)
                print('bottom_Angle : ',bottom_Angle) 
                print('bottom_point : ',bottom_point) 
                print('********** 結束 **********') 
            else :
                n = len(top_box)
                out = Bubble_Sort2(top_box,top_Angle,top_point,top_CenterX,top_CenterY) #tag and 角度
                a = (6-n)
                print(a)
                print('top_box : ',top_box)
                print('top_CenterX : ',top_CenterX)
                print('top_CenterY : ',top_CenterY)
                print('top_Angle : ',top_Angle) 
                print('top_point : ',top_point) 
                loop = a
                if((loop)>0):
                    print(loop)
                    while 1:
                        if 'W' not in top_box:
                            box_7 = cv.imread('/home/junan/Desktop/照片/W/w_bottom.png',0)
                            box_list_2.append(box_7)
                            loop = loop-1
                        if 'R' not in top_box:
                            box_8 = cv.imread('/home/junan/Desktop/照片/R/r_bottom.png',0)
                            box_list_2.append(box_8)
                            loop = loop-1
                        if 'Y' not in top_box:
                            box_9 = cv.imread('/home/junan/Desktop/照片/Y/y_bottom_JP.png',0)
                            box_list_2.append(box_9)
                            loop = loop-1
                        if 'G' not in top_box:
                            box_10 = cv.imread('/home/junan/Desktop/照片/G/g_bottom.png',0)
                            box_list_2.append(box_10)
                            loop = loop-1
                        if 'WP' not in top_box:
                            box_11 = cv.imread('/home/junan/Desktop/照片/W_P/wp_bottom.png',0)
                            box_list_2.append(box_11)
                            loop = loop-1
                        if 'GP' not in top_box:
                            box_12 = cv.imread('/home/junan/Desktop/照片/G_P/gp_bottom.png',0)
                            box_list_2.append(box_12)
                            loop = loop-1
                        if(loop<=0):
                            break
                
                bottom = Matches2(box_in_sence_2,box_list_2,bottom_Angle,bottom_box,bottom_point,bottom_CenterX,bottom_CenterY)
                while True:
                    if(len(bottom_box) != a and len(bottom_point) != a):
                        bottom = Matches2(box_in_sence_2,box_list_2,bottom_Angle,bottom_box,bottom_point,bottom_CenterX,bottom_CenterY)
                    else:
                        break
                out = Bubble_Sort2(bottom_box,bottom_Angle,bottom_point,bottom_CenterX,bottom_CenterY) #tag and 角度
                print('top_box : ',top_box)
                print('top_CenterX : ',top_CenterX)
                print('top_CenterY : ',top_CenterY)
                print('top_Angle : ',top_Angle) 
                print('top_point : ',top_point)
                print('bottom_box : ',bottom_box)
                print('bottom_CenterX : ',bottom_CenterX)
                print('bottom_CenterY : ',bottom_CenterY)
                print('bottom_Angle : ',bottom_Angle) 
                print('bottom_point : ',bottom_point)
                print('********** 結束 **********')

    if start_input == 3:
        take_pic()
        break

    if start_input == 4:
        depth_colormap = []
        box_x = []
        box_y = []
        dep1 = dep_img(depth_colormap)
        # print(top)
        src = dep1
        # print(src)
        AA = contours_demo(src,box_x,box_y)
        dep_cm = RealSense2(box_x,box_y)
        # print(AA)
        cv.imwrite('AA.png', AA)
        break

    if start_input == 5:
        break
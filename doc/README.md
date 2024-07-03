# PlyCal
> LIAO Qinghai  2019.2.25

[for English Version click here](#english-version)


本程序只实现了利用四边形板校准，暂未实现利用任意polygon的校准。  

## 准备  
如example文件夹所示，使用本软件需要一个config.json文件和图像、点云对应的文件。  
config.json：参考example/config.json，正常只需要修改其中相机的K、D。此文件路径无要求  
dataset：dataset指example下的`image_orig`和`pointcloud`，这两个文件夹名称不可以更改且必须在同一文件夹下。`image_orig`内需要有N(N>=4)张未反畸变的图片，`pointcloud`中有N份对应的pcd点云文件，二者应该时间上已经对应同步。  

## 操作  
1. 打卡config文件。在终端启动./PlyCal-qt后会直接进入config.json文件的选择。 
2. 打开dataset。如下图为主界面，点击`Open Dataset`选择保护`image_orig`和`pointcloud`的文件夹。主界面会显示总的数据帧数。  
![main](./main.png)

3. 手动调整初值。打开dataset后主界面会隐藏，出现如下的调整初值的界面和一个显示点云的窗口、一个显示图像的窗口（此时点云会投影到图像、图像颜色也会投影到点云）。再下图的界面中手工条件rotation、translation且观察图像界面中的点云深度投影，OK是关掉下界面即可。
![init](./init.png)
4. 手动条件polygon。返回主界面后，目前还不能第一次在点云和图像中都直接检测出四边形。对于图像，可以先点击`Start Selection`，然后鼠标在图片窗口中点击目标四边形的四个角点，完成后点击`Finish Selection`，正常检测结果如下图。对于点云，使用`Pointcloud Control`下的四个sliderbar来切割点云，缩小检测的范围，正常结果如下。

![img](./img.jpg)
![pc](./pc.png)

5. 检测。第一帧手动调整结束后，可以使用`Next Pose`来一帧一帧的处理下一帧，或者使用`Quick`快速处理，当数据检测结果不好时可以回到步骤4手动调整或者`Delete`
6. 校准。点击`Calibrate`调用校准优化，结果在终端有打印，也可以点击`Save Result`。config文件此时也可以保存（会覆写之前的config）。

# English Version
(Translated with AI): 
This program only implements calibration with quadrilateral plates, not with any polygon for the moment.

## Intend
As shown in the example folder, to use this software you need a config.json file and the corresponding files for the image and point cloud.
config.json: refer to example/config.json, normally you only need to change the K and D of the camera in it, there is no requirement for the path of this file.
dataset: dataset refers to image_orig and pointcloud under example, the name of these two folders can not be changed and must be in the same folder. image_orig needs to have N (N>=4) undistorted images in it, and there are N corresponding pcd pointcloud files in pointcloud, the two should be synchronised in time. be synchronised in time.

## Manipulate
1. Punch the config file. In the terminal launching . /PlyCal-qt will take you directly to the config.json file selection.

2. Open dataset. as shown below for the main interface, click Open Dataset to select the folder protecting image_orig and pointcloud. The main interface will show the total number of data frames.
![main](./main.png)

3. Adjust the initial value manually. After opening dataset, the main interface will be hidden, and the following interface for adjusting the initial value will appear, together with a window for displaying the point cloud and a window for displaying the image (at this time, the point cloud will be projected to the image, and the image colour will also be projected to the point cloud). In the following interface, manually set the conditions of rotation and translation and observe the depth projection of the point cloud in the image interface, and then close the following interface.
![init](./init.png)

4. After returning to the main interface, it is not yet possible to detect the quadrilateral directly in both point cloud and image for the first time (for now?). For the image, you can click Start Selection first, then click the four corner points of the target quadrilateral in the image window, and then click Finish Selection when you are done, the normal detection result is as below. For point cloud, use the four sliderbar under Pointcloud Control to cut the point cloud and narrow the detection range, the normal results are as follows.

![img](./img.jpg)
![pc](./pc.png)

5. Detection. After the first frame is manually adjusted, you can use Next Pose to process the next frame one by one, or use Quick Quick Processing, when the data detection result is not good you can go back to step 4 to manually adjust or Delete.
6. Calibrate. Click Calibrate to call calibration optimisation, the result will be printed in the terminal, you can also click Save Result. config file can also be saved at this time (will overwrite the previous config).

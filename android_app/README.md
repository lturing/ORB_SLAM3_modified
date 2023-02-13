> 调用安卓手机上的摄像头和imu，获取图像、加速度、角加速度等信息

分辨率 (DEFAULT_VIDEO_SIZE )

帧率（mFrameRate）

固定焦距(DEFAULT_FOCUS_MODE)
动态曝光时间(CameraSettingExposureMode)

imu rate(mSensorRate)

app/build/outputs/apk/debug

## 编译成apk
```
./gradlew clean && ./gradlew assembleDebug
```


## app

<div align=center><img src="./../images/app.jpg" width="80%"/></div>

## setting

<div align=center><img src="./../images/app_settings.jpg" width="80%"/></div>


参考
[VideoIMUCapture-Android](https://github.com/DavidGillsjo/VideoIMUCapture-Android)
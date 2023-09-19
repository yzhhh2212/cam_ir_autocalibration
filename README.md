## 1. 功能描述
自动标定ir和rgb相机的外参

## 2. 目录结构
```shell
.
├── CMakeLists.txt
├── include                      //头文件
├── README.md
├── src
│   ├── camera.cpp               //rgb每一帧的类，实现每一帧的pnp
│   ├── ir_camera.cpp            //ir每一帧的类，实现每一帧的pnp
│   ├── main.cpp//主函数
│   ├── optimizer.cpp            //优化+ransac的实现
│   └── optimizer_types.cpp      //自定义g2o边
└── thirdparty
    └── g2o

```

## 3. 依赖
* Eigen
* OpenCv



## 4. 使用方法
1. 修改main函数中的ir图片和rgb图片存放地址
2. 编译运行

## 5. todo
1. 增加3D点为优化变量
# 安装pcl库
    sudo apt-get install python3-pcl
# 安装pybind11
    pip install pybind11

# 编译并安装pybind11
    cd point_processing_cpp/pybind11/build
    cmake ..
    make -j8
    sudo make install

# 编译cpp文件生成.so文件
    cd point_processing_cpp
    sh build.sh
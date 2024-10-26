cmake_minimum_required(VERSION 3.5)
project(rotary_encoder)

# Python用のamentビルドシステムを使用
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# パッケージディレクトリを設定
ament_python_install_package(${PROJECT_NAME})

# パッケージのビルドターゲットを設定
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# パッケージをamentにエクスポート
ament_package()

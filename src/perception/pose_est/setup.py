from setuptools import find_packages, setup
from pybind11.setup_helpers import Pybind11Extension, build_ext
import os
from glob import glob

package_name = 'pose_est'

# Build robust paths
here = os.path.abspath(os.path.dirname(__file__))
mycpp_src = os.path.join(here, 'mycpp', 'src')
mycpp_include = os.path.join(here, 'mycpp', 'include')

# 配置 mycpp pybind11 扩展
ext_modules = [
    Pybind11Extension(
        "mycpp",
        [
            os.path.join(mycpp_src, "Utils.cpp"),
            os.path.join(mycpp_src, "app", "pybind_api.cpp")
        ],
        include_dirs=[mycpp_include, "/usr/include/eigen3"],
        cxx_std=14,
    ),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装权重文件
        *[(os.path.join('share', package_name, 'weights', os.path.relpath(root, 'weights')), 
           [os.path.join(root, f) for f in files])
          for root, dirs, files in os.walk('weights') if files],
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py') if os.path.exists('launch') else []),
        # 修复：安装 config 文件（与 setup.py 同级的 config 目录）
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'pybind11'],
    zip_safe=True,
    maintainer='ls',
    maintainer_email='user@todo.todo',
    description='FoundationPose ROS 2 wrapper for 6D pose estimation',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'auto_tracker_node = pose_est.yolo_predict:main',
            'auto_predict_node = pose_est.yolo_predict_notrack:main',
        ],
    },
)
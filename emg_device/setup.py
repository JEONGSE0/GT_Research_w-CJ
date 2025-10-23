from setuptools import setup
import os

package_name = 'emg_device'

# 모델 파일이 소스에 있으면 설치, 없으면 스킵
model_src_path = os.path.join('emg_device', 'models', 'emg_cnn_model.pt')
model_data_files = []
if os.path.isfile(model_src_path):
    model_data_files.append(('share/' + package_name + '/models', [model_src_path]))
else:
    print(f"[setup.py] WARNING: model not found at {model_src_path}; skipping model install")

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', [
        'launch/emg_signal_launch.py',
        'launch/emg_full_pipeline_launch.py',
    ]),
    ('share/' + package_name + '/LICENSE', ['LICENSE']) if os.path.isfile('LICENSE') else (),
]
# 빈 튜플 제거
data_files = [df for df in data_files if df]
data_files += model_data_files

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meow',
    maintainer_email='tj6774@gmail.com',
    description='ROS 2 EMG driver + real-time gesture classification.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emg_node = emg_device.emg_node:main',
            'emg_classifier_node = emg_device.emg_classifier_node:main',
        ],
    },
)

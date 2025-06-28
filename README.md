# AutolifeRobotKiosk



## Build 

```
pip install Cython
# For Source Code Encryption
python cythonizer.py -s src
# For Building Wheel Package
python setup.py sdist bdist_wheel
```

After installing the package, run the following command for program settings:

```
cd ~/miniconda3/envs/robot_env/lib/python3.10/site-packages/autolife_robot_kiosk
cp settings.toml.example settings.toml

python -m autolife_robot_kiosk.main
```

## To setup auto startup systemd-service service

```
bash ~/miniconda3/envs/robot_env/lib/python3.10/site-packages/autolife_robot_kiosk/scripts/setup_logo_display.sh
```


Clone this repository inside ros2_ws and call it src
```bash
git clone {url} src
```

# Set environment 

## create virtual environment (move to ros2_ws directory)
```bash
virtualenv .venv --system-site-packages
```

## start environment
```bash
source .venv/bin/activate

source /opt/ros/humble/setup.bash

source install/setup.bash
```

## import ROS library in PyCharm
Linux
```bash
vim /home/{name}/.local/JetBrains/Toolbox/apps/{pycharm_version}/bin/pycharm.sh
```

WSL
```bash
vim /home/{name}/.cache/JetBrains/RemoteDev/dist/{cached_pycharm_version}/bin/pycharm.sh
```

insert the next line:
```bash
. /opt/ros/humble/setup.sh' 
```


# Flask 

Install Flask

```bash
pip install Flask
```
## start flask-backend (on localhost:5000)

```bash
python3 {name_file.py}
```

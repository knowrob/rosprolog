rosprolog_rest
===

The ros node rosprolog_rest provides an REST API for KnowRob.

### Installation instructions

1. Install the following python packages:
```
sudo -H pip3 install flask==1.1.4 flask_restplus gevent werkzeug==0.16.1
```

2. Update rosdep
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Tutorial

To run this API, make sure that rosprolog is running and include this in launch file
```
<node pkg="rosprolog" type="rosprolog_rest.py" name="rosprolog_rest"/>
```

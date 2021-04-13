rosprolog_rest
===

The ros node rosprolog_rest provides an REST API for KnowRob.

### Installation instructions

1. Install the following python packages:
```
sudo -H pip3 install flask_restplus gevent
```

2. Run this code to fix some known bugs:
```
sudo sed -i 's/werkzeug import cached_property/werkzeug.utils import cached_property/' /usr/local/lib/python*/dist-packages/flask_restplus/fields.py
sudo sed -i 's/werkzeug import cached_property/werkzeug.utils import cached_property/' /usr/local/lib/python*/dist-packages/flask_restplus/api.py
```

3. Update rosdep
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Tutorial

To run this API, make sure that rosprolog is running and include this in launch file
```
<node pkg="rosprolog" type="rosprolog_rest.py" name="rosprolog_rest"/>
```


# What did I do?

I have created a node inside the package name **my_py_pkg**. 
The name of the node is **py_test**, which can be found in the **my_first_node.py**
The name of the executable is **py_node**, which can be found in the **package.xml** file

# How to run this node?

Firstly, we have to build the package using

	$ colcon build

Then you can run the node inside this package using

	$ ros2 run my_py_pkg py_node
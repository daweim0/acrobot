A machine learning based algorithm for controlling an 'acrobot', a two body robot with very interesting(challenging) dynamics. This repository also houses an acrobot simulator and code to interface with acrobot hardware. Most of the code is housed in pyode/src/

A summary of my work from a research  perspective here can be found [here](http://www.davidmichelman.com/acrobot.pdf): http://www.davidmichelman.com/acrobot.pdf

The meat of the control algorithm is [here](https://github.com/daweim0/acrobot/blob/master/pyode/src/lookup_table_hopper.py)

Some Cython 5 dimensional interpolation code is [here](https://github.com/daweim0/acrobot/blob/master/pyode/src/lookup_table_hopper_helper.pyx)

A bunch of simulation and UI code can be found [here](https://github.com/daweim0/acrobot/blob/master/pyode/src/simulation.py)

[This paper](http://www.iau.dtu.dk/Equipment/Acrobot/iser00.pap.pdf) has a fairly good description of acrobots and the difficulties in controlling them

Windows ODE bindings can be downloaded [here](http://www.lfd.uci.edu/~gohlke/pythonlibs/#ode)

Here are some pretty (and completely out of context) pictures. They were generated when testing early controllers:
![](https://github.com/daweim0/acrobot/blob/master/pyode/src/child_child%20simulations%203%20friction%20%3D%208.pngauto1.csv.png)

Q1 is the base joint angle (attached to the ground, no motor) and Q2 is the middle joint angle (the middle joint with a motor in it). There's a dot at every point the acrobot visited in a short simulation run (keep in mind that neither axis is time).

![](https://github.com/daweim0/acrobot/blob/master/pyode/src/child_child%20simulations%203%20friction%20%3D%201.pngauto1.csv.png)

Contact David Michelman (daweim0@gmail.com) with any questions.

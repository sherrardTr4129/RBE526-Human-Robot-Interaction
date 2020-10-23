# relaxed_ik


<b> Development update 5/13/20 </b>


<strong>Robot autonomy for camera control assistance.</strong>

Demo Video (Click to play)

[![Kinova dual arm robot manipulator](http://img.youtube.com/vi/lwVXYiFdlOc/0.jpg)](https://youtu.be/lwVXYiFdlOc "Kinova dual arm robot manipulator")



<b> Development update 1/3/20 </b>

RelaxedIK has been substantially rewritten in the Rust programming language.  Everything is still completely ROS compatible and should serve as a drop-in replacement for older versions of the solver.  

The Rust relaxedIK solver is MUCH faster than its python and julia alternatives.  Testing on my laptop has indicated that the solver can run at over 3000Hz for single arm robots (tested on ur3, ur5, jaco, sawyer, panda, kuka iiwa, etc) and about 2500Hz for bimanual robots (tested on ABB Yumi and Rainbow Robotics DRC-Hubo+). All of the new code has been pushed to the Development branch, and will be pushed to the main branch after a brief testing phase.  It is highly recommended that the development branch be used at this point, as it has many more features and options than the main branch.

<pre> git clone -b dev https://github.com/uwgraphics/relaxed_ik.git </pre>

If you are working with an older version of relaxedIK, note that you will have to start from a fresh repo and go through the start_here.py procedures again to work with the Rust version of the solver.  

If you have any comments or questions on any of this, or if you encounter any bugs in the new rust version of the solver, feel free to post an issue or email me directly at rakita@cs.wisc.edu


<b> RelaxedIK Solver </b>

Welcome to RelaxedIK! This solver implements the methods discussed in our paper <i> RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion </i> (http://www.roboticsproceedings.org/rss14/p43.html)

Video of presentation at RSS 2018 (RelaxedIK part starts around 12:00) :
https://youtu.be/bih5e9MHc88?t=737

Video explaining relaxedIK
https://youtu.be/AhsQFJzB8WQ

RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion
between Cartesian end-effector pose goals (such as "move the robot's right arm end-effector to position X, while maintaining an end-effector
orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is
done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the
desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions,
kinematic-singularities, or joint-space discontinuities.

To start using the solver, please follow the step-by-step instructions in the file start_here.py (in the root directory)

If anything with the solver is not working as expected, or if you have any feedback, feel free to let us know! (email: rakita@cs.wisc.edu, website: http://pages.cs.wisc.edu/~rakita)
We are actively supporting and extending this code, so we are interested to hear about how the solver is being used and any positive or negative experiences in using it.

<b> Citation </b>

If you use our solver, please cite our RSS paper RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion
http://www.roboticsproceedings.org/rss14/p43.html

<pre>
@INPROCEEDINGS{Rakita-RSS-18, 
    AUTHOR    = {Daniel Rakita AND Bilge Mutlu AND Michael Gleicher}, 
    TITLE     = {{RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion}}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2018}, 
    ADDRESS   = {Pittsburgh, Pennsylvania}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2018.XIV.043} 
} 
</pre>

If you use our solver for a robot teleoperation interface, also consider citing our prior work that shows the effectiveness of RelaxedIK in this setting:


A Motion Retargeting Method for Effective Mimicry-based Teleoperation of Robot Arms
https://dl.acm.org/citation.cfm?id=3020254
<pre>
@inproceedings{rakita2017motion,
  title={A motion retargeting method for effective mimicry-based teleoperation of robot arms},
  author={Rakita, Daniel and Mutlu, Bilge and Gleicher, Michael},
  booktitle={Proceedings of the 2017 ACM/IEEE International Conference on Human-Robot Interaction},
  pages={361--370},
  year={2017},
  organization={ACM}
}
</pre>


An Autonomous Dynamic Camera Method for Effective Remote Teleoperation
https://dl.acm.org/citation.cfm?id=3171221.3171279
<pre>
@inproceedings{rakita2018autonomous,
  title={An autonomous dynamic camera method for effective remote teleoperation},
  author={Rakita, Daniel and Mutlu, Bilge and Gleicher, Michael},
  booktitle={Proceedings of the 2018 ACM/IEEE International Conference on Human-Robot Interaction},
  pages={325--333},
  year={2018},
  organization={ACM}
}

</pre>

<b> Dependencies </b>

kdl urdf parser:
<div> >> sudo apt-get install ros-[your ros distro]-urdfdom-py </div>
<div> >> sudo apt-get install ros-[your ros distro]-kdl-parser-py </div>
<div> >> sudo apt-get install ros-[your ros distro]-kdl-conversions </div> 

<br>

fcl collision library:
https://github.com/BerkeleyAutomation/python-fcl

<!--
boost: https://www.boost.org/doc/libs/1_67_0/more/getting_started/unix-variants.html
The boost c++ libraries are used to interface between c++ and python code in the solver.  The solver will look for boost library files in the directory /usr/local/lib/ (the default install directory); if the library files are not found, the solver will try to move on anyway using the default python implementation, though performance will be slower. (UPDATE: Boost implementations are not turned on in the current version, but these will be included in the next RelaxedIK update after some testing).
-->


scikit learn:
http://scikit-learn.org/stable/index.html


<b> Tutorial </b>

For full setup and usage details, please refer to start_here.py in the src directory.




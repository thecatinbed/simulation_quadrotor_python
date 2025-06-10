# Introductions:
This is a project for quadrotor simulation.
# Documentations:
[Modeling and control of quadrotor](.\Modeling and control of quadrotor.pdf)  
[Introduction](.\Introduction.pdf)
# Packages:
Beside the packages that come with Python, this project need some extra packages:  
numpy: `pip install numpy -i https://pypi.tuna.tsinghua.edu.cn/simple`  
matplotlib: `pip install matplotlib -i https://pypi.tuna.tsinghua.edu.cn/simple`  
# How to use:
* Set up the model's parameters in `.\model\quadrotors_parameters.py`   
* Set up the controller's parameters in `.\controllers\geometric_controller.py`.  
* Run the `geometric_control_demo.py`, the simulation is starting. The `geometric_control_demo.py` is the main process of project. The instantiation of classes of the controller parameters, controller 
and model will be done in the main function. By the way, the `trajectory_generator.py` is a class used to generate the desire trajectory, and the trajectory type is specified in initiation. And cyclic call class method to complete the simulation. Finally, draw the 
curve with draw.py and draw3d.py.  
* The figures will be saved in the folder named images.
  






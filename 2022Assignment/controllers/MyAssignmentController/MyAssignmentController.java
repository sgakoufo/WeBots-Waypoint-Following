// File:          MyAssignmentController.java
// Date:          15/12/2022
// Description:
// Author:        Achilleas Koufonikos, Student ID: 201510075
// Modifications:

// ==============================================================
// COMP329 2022 Programming Assignment
// ==============================================================
// 
// The aim of the assignment is to move the robot around the arena in such a way
// as to generate an occupancy grid map of the arena itself.  Full details can be
// found on CANVAS for COMP329
//
// Only add code to the controller file - do not modify the other java files in this project.
// You can add code (such as constants, instance variables, initialisation etc) anywhere in 
// the file, but the navigation itself that occurs in the main loop shoudl be done after checking
// the current pose, and having updated the two displays.
//
// Note that the size of the occupancy grid can be changed (see below) as well as the update
// frequency of the map, adn whether or not a map is generated.  Changing these values may be
// useful during the debugging phase, but ensure that the solution you submit generates an
// occupancy grid map of size 100x100 cells (with a recommended update frequency of 2).
//
// ==============================================================


import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Camera;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class MyAssignmentController {

  public static void main(String[] args) {

    Supervisor robot = new Supervisor();

    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    Camera camera = robot.getCamera("camera");
    if (camera != null)
      camera.enable(timeStep);

    PioneerNavX nav = new PioneerNavX(robot);
    Pose my_pose = nav.get_real_pose();
    PioneerProxSensors prox_sensors = new PioneerProxSensors(robot, "sensor_display", my_pose);

    // 2nd argument determines how many cells per meter of the arena.
    // Use 20 for testing navigation, but 50 for high-quality map (slow)
    OccupancyGrid ogrid = new OccupancyGrid(robot, 20, "occupancy_grid_display", my_pose, prox_sensors);

    // define schedule
    PioneerNavX.MoveState[] schedule = { PioneerNavX.MoveState.WAYPOINT };
    PioneerNavX.MoveState state = schedule[0]; // current state

    while (robot.step(timeStep) != -1) {
      my_pose = nav.get_real_pose();
      prox_sensors.set_pose(my_pose);
      prox_sensors.paint();  // Render sensor Display
      
      ogrid.set_pose(my_pose);
      ogrid.map();
      ogrid.paint();
      
      
      // ADD CODE BELOW 
      
      if (state == PioneerNavX.MoveState.WANDER){
        nav.wander(prox_sensors, 0.4);
      } else if(state == PioneerNavX.MoveState.WAYPOINT) {
        nav.waypoint(prox_sensors, 0.4);
      } else if(state == PioneerNavX.MoveState.STOP){
        nav.stop();
      }
      
    };

    // Enter here exit cleanup code.
  }
}


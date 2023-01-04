// PioneerNavX.java
/*
 * PioneerNavigation Class Definition
 * Date: 18th Oct 2022
 * Description: Simple Navigation Class support for 2022 Assignment
 * Author: Terry Payne (trp@liv.ac.uk)
 */
 
 import com.cyberbotics.webots.controller.Motor;
 import com.cyberbotics.webots.controller.Supervisor;
 import com.cyberbotics.webots.controller.Node;
 
 public class PioneerNavX {
 
   public static enum MoveState {
     STOP,
     FORWARD,
     WANDER,
     ARC,
     WAYPOINT
    };
 
   private Supervisor robot;       // reference to the robot
   private Node robot_node;        // reference to the robot node
   private Pose robot_pose;        // the robots believed pose, based on real location
   private Motor left_motor;
   private Motor right_motor;
   private MoveState state;
   
   private double velocity;
   private double angularVelocity = 2.0;// This constant will be used as the angular velocity. It is a speed slow enough
   // for the sensors to capture everything, but fast enough for simulation to be complete in under 5 minutes.

   // waypoints:
   private Pose[] wpArray = {
                            new Pose(-2.2, 2.0,0.0),// first waypoint, starting position
                            
                            new Pose( 2.2, 2.0,0.0),
                            new Pose( 2.1,-1.5,0),
                            new Pose( 0.0,-2.3,0),//
                            new Pose( -2.1,-1.5,0),
                            new Pose( -2.1,-0.7,0),
                            new Pose( -1.0 ,-0.6,0),
                            new Pose( -1.1, 0.4,0),//
                            new Pose( -2.0, 0.5,0),
                            new Pose( -2.4, 1.0,0),//
                            new Pose( -0.2, 1.3,0),
                            new Pose( -0.2,-0.3,0),
                            new Pose(  0.6,-0.3,0),
                            new Pose(  0.6, 1.1,0),
                            new Pose(  2.0, 1.1,0),
                            new Pose(  2.0,-2.2,0),
                            new Pose( -2.1,-2.2,0),
                            new Pose( -2.2,-0.8,0),
                            new Pose(  2.1,-0.8,0),
                            
                            new Pose( 2.2, 2.0,0)// final waypoint, ending position
                            };
                            // This is the waypoint array that is hard-coded in. The way points are hard-coded in a way such
                            // that the robot will cover the entire map, and go over it twice in slightly different poses
                            // to get better sensor readings. It also has the last waypoint as it desired ending position

   Pose targetWaypoint; // this variable will be used to track the next waypoint the robot wants to reach

   int wpIndex = 1;// the variable with which we will ieterate thorugh wpArray
 
   private final double WHEEL_RADIUS = 0.0957; // in meters - found using CONFIGURE 
   private final double AXEL_LENGTH = 0.323;   // in meters - found using CONFIGURE
 
 
   // ==================================================================================
   // Constructor
   // ==================================================================================
   public PioneerNavX(Supervisor robot) {
     this.robot = robot;                       // reference to the robot
     this.robot_node = this.robot.getSelf();   // reference to the robot node
     this.robot_pose = this.get_real_pose();   // the robots believed pose, based on real location
     this.state = MoveState.STOP;
 
     // enable motors
     this.left_motor = robot.getMotor("left wheel");
     this.right_motor = robot.getMotor("right wheel");
     this.left_motor.setPosition(Double.POSITIVE_INFINITY);
     this.right_motor.setPosition(Double.POSITIVE_INFINITY);
 
     // Initialise motor velocity
     this.left_motor.setVelocity(0.0);
     this.right_motor.setVelocity(0.0);   
   } 
   
  public void waypoint(PioneerProxSensors prox_sensors, double robot_linearvelocity){
    robot_pose = get_real_pose();// update robot pose
    if(wpIndex == wpArray.length){// the robot has reached the last waypoint. Cleanup
      // note that this is needed as closeToWaypoint function does not get the robot close enough to the exact location of the waypoint.
      reachFinalWaypoint(wpArray[wpArray.length - 1]);
    
    }else{// else go to the next waypoint
      robot_pose = get_real_pose();// update the robot pose

      targetWaypoint = wpArray[wpIndex];
      System.out.println("Current Robot Pose: " + robot_pose);                            // print current pose coordinates to the console
      System.out.println("Current Target Waypoint: WP" + wpIndex + ", " +targetWaypoint); // print target waypoint to the console

      if (closeToWaypoint(targetWaypoint)){
        // robot has reached the targetWaypoint, increase wpIndex by 1 to target the next waypoint
        System.out.println("Reached waypoint " + wpIndex +", "+ targetWaypoint + ". Targetting next waypoint.");
        wpIndex++;
      }else if (closeToWaypoint(wpArray[wpIndex - 1])&&(needsRotating(targetWaypoint))){ 
        // robot is close to the previous waypoint, and it it needs to arc around it to get to the next waypoint
        arcAroundWaypoint(wpArray[wpIndex - 1]);
      }else{
        // move towards the next waypoint as there is no need to arc. The function below will however keep checking if there is need for 
        // slight adjustemnets
        moveTowardsWaypoint(targetWaypoint);
      }
    }
    System.out.println(" ");// print new line
  }



   // ==================================================================================
   // helper fucntions:
   // ==================================================================================
   public Pose get_real_pose() {
     if (this.robot_node == null)
       return new Pose(0,0,0);
       
     double[] realPos = robot_node.getPosition();
     double[] rot = this.robot_node.getOrientation(); // 3x3 Rotation matrix as vector of length 9
     double theta1 = Math.atan2(-rot[0], rot[3]);
     double halfPi = Math.PI/2;
     double theta2 = theta1 + halfPi;
     if (theta1 > halfPi)
         theta2 = -(3*halfPi)+theta1;
     
     return new Pose(realPos[0], realPos[1], theta2);
   }
   public boolean closeToWaypoint(Pose wp){
    // robot is close to the waypoint wp
    return(Math.abs(wp.getX() - robot_pose.getX()) < 0.5)&&(Math.abs(wp.getY() - robot_pose.getY()) < 0.5);
    // 0.5 is used as it is close enough to the waypoint and the robot can start arcing
   }
   public boolean needsRotating(Pose wp){
    // robot needs to rotate to head towards wp
    double diff = getAngleDiff(wp);
    return((Math.abs(diff)) >= 0.01); // if diff is higher than 0.01, robot needs some rotating/adjusting
   }
   public void reachFinalWaypoint(Pose wp){
    // function that helps robot reaach the final waypoint, and stop exactly at the pose of the final waypoint
    System.out.println("Reaching final waypoint... "+ " " + wp);
    if ((Math.abs(wp.getX() - robot_pose.getX()) > 0.01)||(Math.abs(wp.getY() - robot_pose.getY()) > 0.01)){
      System.out.println("Moving towards final waypoint");
      moveTowardsWaypoint((wp));
    } else{
      System.out.println("Reached Final Waypoint. Stopping robot.");
      stop();

      System.out.println("========================");
      System.out.println("End.");
      System.out.println("========================");
    }
   }
   public void arcAroundWaypoint(Pose wp){
    // arcs the robot around wp
    System.out.println("Arching around waypoint " + (wpIndex - 1) + " " + wp);
    double diff = getAngleDiff(targetWaypoint);

    if(diff > 0){// robot needs to rotate left
      System.out.println("Robot rotating left");
      left_motor.setVelocity( angularVelocity*2/5);
      right_motor.setVelocity(  angularVelocity);
    } else{// robot needs to rotate right 
      System.out.println("Robot rotating right");
      left_motor.setVelocity(   angularVelocity);
      right_motor.setVelocity(angularVelocity/2);
    }
   }
   public void moveTowardsWaypoint(Pose wp){
    // moves towards wp while checking if there is need for angle adjustments
    if (needsRotating(wp)){// check if robot needs slight adjustments
      System.out.println("Adjusting rotation...");
      double diff = getAngleDiff((wp));
      if(diff > 0){// robot needs to adjust to left
        System.out.print(" Adjusting to the left");
        left_motor.setVelocity( -angularVelocity);
        right_motor.setVelocity( angularVelocity);
      }else{
        System.out.print(" Adjusting to the right");
        left_motor.setVelocity(  angularVelocity);
        right_motor.setVelocity(-angularVelocity);
      }

    } else{
      System.out.println("Moving towards waypoint " + wpIndex + " " + wp);
      left_motor.setVelocity(angularVelocity);
      right_motor.setVelocity(angularVelocity);
    }
   }

   public double getAngleDiff(Pose wp){
    // get the angle difference between the robot's current oreintaiton, and the one required
    // to reach waypoint wp
    double angle = Math.atan2(wp.getY() - robot_pose.getY(), wp.getX() - robot_pose.getX());
    double diff  = Math.atan2(Math.sin(angle - robot_pose.getTheta()), Math.cos(angle - robot_pose.getTheta()));

    return(diff);
   }

   public void stop() {
     this.left_motor.setVelocity(0.0);
     this.right_motor.setVelocity(0.0);
     this.state = MoveState.STOP;
   }
   
   public MoveState getState() {
     return this.state;
   }

   // ==================================================================================
   // Unused functions, kept for the sake of testing by the marker: (if neccessary)
   // ==================================================================================


   // The following code is based on the avoid obstacle code supplied by the Webots
  // platform for the ePuck and allows the robot to wander randomly around the arena
  public void wander(PioneerProxSensors prox_sensors, double robot_linearvelocity) {

    double leftVel, rightVel;
    double wheel_av = (robot_linearvelocity/this.WHEEL_RADIUS);
    double left_vel = wheel_av;
    double right_vel = wheel_av;

    // detect obstacles
    boolean right_obstacle =
        prox_sensors.get_value(4) < 0.30 ||
        prox_sensors.get_value(5) < 0.25 ||
        prox_sensors.get_value(6) < 0.20 ||
        prox_sensors.get_value(7) < 0.15;
    boolean left_obstacle =
        prox_sensors.get_value(0) < 0.15 ||
        prox_sensors.get_value(1) < 0.20 ||
        prox_sensors.get_value(2) < 0.25 ||
        prox_sensors.get_value(3) < 0.30;

    if (left_obstacle)
      right_vel = -left_vel;
    else if (right_obstacle)
      left_vel = -right_vel;
      
    this.left_motor.setVelocity(left_vel);
    this.right_motor.setVelocity(right_vel);
    this.state = MoveState.WANDER;
  }

  public int forward(double target_dist, double robot_linearvelocity) {
    double wheel_av = (robot_linearvelocity/this.WHEEL_RADIUS);
    double target_time = target_dist/robot_linearvelocity;
    
    this.left_motor.setVelocity(wheel_av);
    this.right_motor.setVelocity(wheel_av);
    this.state = MoveState.FORWARD;
        
    // return target_time as millisecs          
    return (int) (1000.0*target_time);
  }

  public int arc(double icr_angle, double icr_r, double icr_omega) {
    double target_time = icr_angle / icr_omega;

    // Calculate each wheel velocity around ICR
    double vl = icr_omega * (icr_r - (this.AXEL_LENGTH / 2));
    double vr = icr_omega * (icr_r + (this.AXEL_LENGTH / 2));
        
    double leftwheel_av = (vl/this.WHEEL_RADIUS);
    double rightwheel_av = (vr/this.WHEEL_RADIUS);
        
    this.left_motor.setVelocity(leftwheel_av);
    this.right_motor.setVelocity(rightwheel_av);
    this.state = MoveState.ARC;

    // return target_time as millisecs          
    return (int) (1000.0*target_time);
  } 
 }    
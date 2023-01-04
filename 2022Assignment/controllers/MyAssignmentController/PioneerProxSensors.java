// File: PioneerProxSensors.java
// Date: 28th Oct 2021
// Description: Represent the local area of the adept robot given its sonar sensors
// Author: Terry Payne
// Modifications:
//    * Based on SensorView.java which was developed on 28th Oct 2021 and then 
//      Updated for the Programming Assignment 2021 (29th Nov 20201)
//    * Builds upon the Python version (pioneer_proxsensors.py - 24th Jan 2022)
//      that integrates the Pioneer sensors into the view for cleaner code
//

import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.Robot;

public class PioneerProxSensors {
  // --------------------------------
  // Robot state variables  
  private Robot robot;          // Reference to the Robot itself
  private Pose robot_pose;       // we only track the orientation, as the robot is always centered
  private double radius;
  
  // --------------------------------
  // Display state variables  
  private Display display;      // reference to the display device on the robot
  private int device_width;     // width of the display device
  private int device_height;    // height of the display device
  private double scaleFactor;   // Scale factor to scale rendered map to the maximal dimension on the display

  // --------------------------------
  // Distance Sensor state variables  
  private DistanceSensor[] ps;  // array of distance sensors attached to the robot
  private double maxRange;      // we'll get this from the lookup table of so0
  private double maxValue;      // we'll get this from the parameters of so0

  private Pose[] psPose;        // the pose of each sensor (assuming the robot is a round cylinder)
	
  // --------------------------------
  // Colours used by the display
  private final static int DARKGREY = 0x3C3C3C;
  private final static int GREY = 0xABABAB;
  private final static int BLACK = 0x000000;
  private final static int WHITE = 0xFFFFFF;
  
  private final static int MAX_NUM_SENSORS = 16;            // Number of sensors on the robot

  // ==================================================================================
  // Constructors
  // ==================================================================================
  public PioneerProxSensors(Robot r, String display_name, Pose p) {

    this.robot = r;      
    this.robot_pose = p;
    
    // get the time step of the current world.
    int timeStep = (int) Math.round(this.robot.getBasicTimeStep());    

    // Note that the dimensions of the robot are not strictly circular, as 
    // according to the data sheet the length is 485mm, and width is 381mm
    // so we assume for now the aprox average of the two (i.e. 430mm), in meters
    this.radius = 0.215;         // in meters

    // ---------------------------------------------------------------------------
    // set up proximity detectors
    // ---------------------------------------------------------------------------
    ps = new DistanceSensor[MAX_NUM_SENSORS];

    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
      String sensorName = "so" + i;
      this.ps[i] = robot.getDistanceSensor(sensorName);
      this.ps[i].enable(timeStep);
    }
    
    // The following array determines the orientation of each sensor, based on the
    // details of the Pioneer Robot Stat sheet.  Note that the positions may be slightly
    // inaccurate as the pioneer is not perfectly round.  Also these values are in degrees
    // and so may require converting to radians.  Finally, we assume that the front of the
    // robot is between so3 and so4.  As the angle between these is 20 deg, we assume that 
    // they are 10 deg each from the robot heading
    double[] psAngleDeg = { 90, 50, 30, 10, -10, -30, -50, -90,
                            -90, -130, -150, -170, 170, 150, 130, 90};

    double[] psAngles = new double[MAX_NUM_SENSORS];
    
    //------------------------------------------
    // determine max range from lookup table  
    double[] lt = ps[0].getLookupTable();
    this.maxRange=0.0;                      // start with a minimum value
    for (int i=0; i< lt.length; i++) {
      if ((i%3)==0) this.maxRange=lt[i];
      if ((i%3)==2) System.out.println("\n");
    }
    this.maxValue=ps[0].getMaxValue();
    
    //------------------------------------------
    // Determine the pose (relative to the robot) of each of the sensors
    this.psPose = new Pose[psAngleDeg.length];    // Allocate the pose array
    for (int i=0; i< psAngleDeg.length; i++) {
      double theta = Math.toRadians(psAngleDeg[i]);
      this.psPose[i] = new Pose(Math.cos(theta)*this.radius,
                                Math.sin(theta)*this.radius,
                                theta);      
    }

    this.display = this.robot.getDisplay(display_name);
    if (this.display != null) {
      this.device_width = this.display.getWidth();
      this.device_height = this.display.getHeight();
      this.scaleFactor = Math.min(this.device_width, this.device_height)/(2*(this.maxRange+this.radius));   
    } else {
      this.device_width = 0;
      this.device_height = 0;
      this.scaleFactor = 0.0;
    }    

  }
  
  // ==================================================================================
  // Internal (Private) methods
  // ==================================================================================
  // Map the real coordinates to screen coordinates assuming
  // the origin is in the center and y axis is inverted
  private int scale(double l) {
    return (int) (this.scaleFactor * l);
  }
  private int mapX(double x) {
    return (int) ((device_width/2.0) + scale(x));
  }
  private int mapY(double y) {
    return (int) ((device_height/2.0) - scale(y));
  }
  private double rotX(double x, double y, double theta) {
    return Math.cos(theta)*x - Math.sin(theta)*y;
  }
  private double rotY(double x, double y, double theta) {
    return Math.sin(theta)*x + Math.cos(theta)*y;
  }



  // ==================================================================================
  // External (Public) methods
  // ==================================================================================  

 
  public double get_maxRange() {
    return maxRange;
  }
  
  public int get_number_of_sensors() {
    return MAX_NUM_SENSORS;
  }
  
  public Pose get_sensor_pose(int sensorID) {
    if (sensorID < MAX_NUM_SENSORS) {
      Pose p = new Pose();
      p.set_pose_position(this.psPose[sensorID]);
      return p; // send a copy to avoid risk of side effecting
    } else
      System.err.println("Out of range error in getSensorPose");
    return null;
  }

  public double get_value(int sensorID) {
    if (sensorID < MAX_NUM_SENSORS)
      return this.maxRange - (this.maxRange/this.maxValue * this.ps[sensorID].getValue());
    else
      System.err.println("Out of range error in getSensorValue");
    return this.maxRange;
  }
  
  public double get_radius() {
    return this.radius;
  }
  
  public void set_pose(Pose p) {
    this.robot_pose.set_pose_position(p);
  } 

  public void paint() {
    if (this.display == null)
      return;

    // ===================================================================================
    // draw a background
    this.display.setColor(0xF0F0F0);     // Off White
    this.display.fillRectangle(0, 0, this.device_width, this.device_height);
 
    // Draw Robot Body      
    this.display.setColor(WHITE);     // White
    this.display.fillOval(mapX(0.0),
                          mapY(0.0),
                          scale(this.radius),
                          scale(this.radius));
    
    this.display.setColor(DARKGREY);     // Dark Grey
    this.display.drawOval(mapX(0.0),
                          mapY(0.0),
                          scale(this.radius),
                          scale(this.radius));
                          
    // Need to indicate heading          
    this.display.drawLine(mapX(0.0),
                          mapY(0.0),
                          mapX(Math.cos(this.robot_pose.getTheta()) * this.radius),
                          mapY(Math.sin(this.robot_pose.getTheta()) * this.radius));

    this.display.setColor(BLACK);         // Black
    this.display.setFont("Arial", 8, true);  // font size = 8, with antialiasing
    
    // For each sensor, get the value of the sensor and display the distance
    // Note that we assume a array of four values.  Each polygon only requires 
    // three, but the fourthi is used to determine a posion for the text
    int[] xArc = {0, 0, 0, 0};
    int[] yArc = {0, 0, 0, 0};
    double[] xd = {0.0, 0.0, 0.0, 0.0};
    double[] yd = {0.0, 0.0, 0.0, 0.0};
    double d;      // distance measured

    
    // Draw triangles of 2*Math.PI/36.0 either side either side of the sensor orientation
    for (int i = 0; i < ps.length ; i++) {

      d = this.get_value(i);

      xd[0] = psPose[i].getX();
      xd[1] = psPose[i].getX() + (d * Math.cos(psPose[i].getTheta()-(Math.PI/18.0)));
      xd[2] = psPose[i].getX() + (d * Math.cos(psPose[i].getTheta()+(Math.PI/18.0)));
      xd[3] = psPose[i].getX() + ((d+0.3) * Math.cos(psPose[i].getTheta()));
      
      yd[0] = psPose[i].getY();
      yd[1] = psPose[i].getY() + (d * Math.sin(psPose[i].getTheta()-(Math.PI/18.0)));
      yd[2] = psPose[i].getY() + (d * Math.sin(psPose[i].getTheta()+(Math.PI/18.0)));
      yd[3] = psPose[i].getY() + ((d+0.3) * Math.sin(psPose[i].getTheta()));
      
      // Need to rotate each point using the rotation matrix and the robot orientation
      for (int j=0; j<4; j++) {
        xArc[j] = mapX(rotX(xd[j], yd[j], this.robot_pose.getTheta()));
        yArc[j] = mapY(rotY(xd[j], yd[j], this.robot_pose.getTheta()));
      } 
      
      // Only use the first three points for the polygon
      this.display.fillPolygon(xArc, yArc, 3);
      
      // Use the fourth point for the distance string, with a slight offset as the
      // coordinates are for the location of the bottom left of the string
      this.display.drawText(String.format("%.02f", d), xArc[3]-10, yArc[3]);
    } 

    // Provide information on the actual pose of the robot
    this.display.setFont("Arial", 10, true);  // font size = 10, with antialiasing
    this.display.drawText("Pose", 1,470);
    this.display.drawText(robot_pose.toString(), 1,486);
  }   
}
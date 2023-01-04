// File: Pose.java
// Date: 30th Dec 2020
// Description: Pose Class support for COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:
//    * Addition of getDeltaTheta() and update to the toString() method - 26th Nov 2021
/**
 * The Pose class for a robot 
 * Based on Worksheet 3 for COMP329, Nov 2020
 * 
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */
 
public class Pose {
  private double x;       // position on x axis - assume units are meters
  private double y;       // position on y axis - assume units are meters
  private double theta;   // This determines the angle (radians) anticlockwise from the x-axis line
  
  // ==================================================================================
  // Constructors
  // ==================================================================================
  public Pose() {
    this(0.0,0.0,0.0);
  }

  public Pose(double xpos, double ypos, double theta, double weight) {
    this.x = xpos;
    this.y = ypos;
    this.setTheta(theta);
  }

  public Pose(double xpos, double ypos, double theta) {
    this(xpos, ypos, theta, 0.0);
  }

  /*
   * Removed for simplicity for the python class
  public Pose(Pose p) {
    this(p.getX(), p.getY(), p.getTheta(), 0.0);
  }
  */

  // ==================================================================================
  // Getters / Setters  
  // ==================================================================================
  public void setTheta(double theta) {
    // Ensure that theta is in the range -\pi..\pi
    if (theta>Math.PI) {
      // The following line is the correct version for handling cases where the angle
      // is greater than pi.  There are still possibly better ways of doing this.
      this.theta=theta-(2 * Math.PI);
      // this.theta=(2 * Math.PI)-theta; Old Incorrent version
    } else if (theta < -Math.PI) {
      this.theta = (2 * Math.PI)+theta;
    } else {
      this.theta = theta;
    }
  }

  public void set_position(double xpos, double ypos, double theta) {
    this.x = xpos;
    this.y = ypos;
    this.setTheta(theta);
  }

  public void set_pose_position(Pose p) {
    this.set_position(p.getX(), p.getY(), p.getTheta());
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getTheta() {
    return theta;
  }

  // ==================================================================================
  // External (Public) methods
  // ==================================================================================  

  public String toString() {
    return String.format("<%.03f", this.x) + ", " +
        String.format("%.03f", this.y) + ", " +
        String.format("%.03f", this.theta) +">";
  }
  
  // Find the difference in radians between some heading and the current pose
  public double get_dtheta(double theta) {
    double d = theta - this.theta;
    if (d > Math.PI)
      d = -(2*Math.PI) + d;
    else if (d < -Math.PI)
      d = (2*Math.PI) + d;
    return d;
  }

}
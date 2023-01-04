// File: OccupancyGrid.java
// Date: 20 Nov 2021
// Description: OccupancyGrid Class support for COMP329 Programming Assignment (2021)
// Author: Terry Payne
// Modifications:
/**
 *
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */
 
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

 
public class OccupancyGrid {
  // --------------------------------
  // Robot state variables  
  private Supervisor robot;     // Reference to the Robot itself
  private Pose robot_pose;       // track the pose of the robot in the global coordinate system
  private PioneerProxSensors prox_sensors;  // reference to the sensor object
  private double radius;        // radius of the robot (assume it is round)
  
  // --------------------------------
  // Display state variables  
  private Display display;      // reference to the display device on the robot
  private int device_width;     // width of the display device
  private int device_height;    // height of the display device
  private double scale_factor;  // Scale factor to scale rendered map to the maximal dimension on the display


  // --------------------------------
  // Occupancy Grid variables
  private double[] grid;         // Array of log odds elements
  private int num_row_cells;	    // number of cells across
  private int num_col_cells;	    // number of cells up
  private int cell_width;     // Width of a cell (wrt arena)
  private int cell_height;    // Height of a cell (wrt arena)
  private double arena_width;    // Width of the arena
  private double arena_height;   // Height of the arena
  private double coverage;       // normalised level of map coverage

  // Fixed log odds values (found empirically)  
  private final double lprior = Math.log(0.5/(1-0.5));
  private final double locc = Math.log(0.95/(1-0.95));
  private final double lfree = Math.log(0.45/(1-0.45));		

  // Constants for the inverse sensor model (values are halved to optimise performance)
  private final double HALFALPHA = 0.02;           // Thickness of any wall found
  private final double HALFBETA = Math.PI/36.0;    // sensor cone opening angle 


  // ==================================================================================
  // Constructor
  // ==================================================================================
  public OccupancyGrid(Supervisor r,
                       int grid_scale,
                       String display_name,
                       Pose p,
                       PioneerProxSensors prox_sensors) {
    this.robot = r;
    this.robot_pose = p;
    this.prox_sensors = prox_sensors;
    this.radius = prox_sensors.get_radius();

    // ---------------------------------------------------------------------------
    // Store Arena state instance variables
    Node arena = robot.getFromDef("ARENA");
    Field floorSizeField = arena.getField("floorSize");
    double floorSize[] = floorSizeField.getSFVec2f();
    this.arena_width = floorSize[0];
    this.arena_height = floorSize[1];
    
    // ---------------------------------------------------------------------------
    // Initialise grid - grid_scale cells per m
    this.num_row_cells = (int) (grid_scale*arena_width);
    this.num_col_cells = (int) (grid_scale*arena_height);
    System.out.println(String.format("Buidling an Occupancy Grid Map of size %d x %d",
                       this.num_row_cells, this.num_col_cells));

    this.grid = new double[this.num_row_cells*this.num_col_cells];
    for (int i=0; i < this.grid.length; i++)
      this.grid[i] = lprior;    

     // ------------------------------------------
     // If provided, set up the display
    this.display = this.robot.getDisplay(display_name);
    if (this.display != null) {
      this.device_width = this.display.getWidth();
      this.device_height = this.display.getHeight();
      // Determine the rendering scale factor
      double wsf = ((double) this.device_width) / this.arena_width;
      double hsf = ((double) this.device_height) / this.arena_height;
      this.scale_factor = Math.min(wsf, hsf);   

      this.cell_width = this.device_width / this.num_row_cells;
      this.cell_height = this.device_height / this.num_col_cells;
    

    } else {
      this.device_width = 0;
      this.device_height = 0;
      this.scale_factor = 0.0;
    }
  }

  // ==================================================================================
  // Getters / Setters  
  // ================================================================================== 

  // The following can be used externally to check the status of the grid map,
  // for example, to develop an exploration strategy.  
  public int get_num_row_cells() {
    return this.num_row_cells;
  }
  public int get_num_col_cells() {
    return this.num_col_cells;
  }
  public int get_grid_size() {
    return this.grid.length;
  }
  public double get_cell_probability(int index) {
    return cell_probability(grid[index]);
  }
 
  // ================================================================================== 
  public void set_pose(Pose p) {
    // Sets the pose of the robot.  If an instance of a pose has yet to be created, then create one
    // Always copy the pose value rather than retain the instance, to ensure it is not side effected
    this.robot_pose.set_pose_position(p);
  }
  
  // Map the real coordinates to screen coordinates assuming
  // the origin is in the center and y axis is inverted
  private int scale(double l) {
    return (int) (this.scale_factor * l);
  }
  private int mapx(double x) {
    return (int) ((this.device_width/2.0) + this.scale(x));
  }
  private int mapy(double y) {
    return (int) ((this.device_height/2.0) - this.scale(y));
  }

  private double cell_probability(double lodds) {
    return 1-(1/(1+Math.exp(lodds)));
  }
    
  private double invSensorModel(Pose p, double x, double y) {

    // Determine the range and bearing of the cell
    double deltaX = x-p.getX();
    double deltaY = y-p.getY();
    double r = Math.sqrt(Math.pow(deltaX,2)+Math.pow(deltaY,2));     // range
    double phi = Math.atan2(deltaY, deltaX) - p.getTheta();          // bearing
    int k = 0;                                                       // sensor index
    double kDelta;
    double logodds = lprior;                                         // default return value

    // Note that the above code assumes range based
    // on the robot and not the sensor position
    if (r>this.radius)
      r=r-this.radius;    // Remove the distance from the robot center to sensor
    else  
      r=0.0;              // If negative, then cell center is behind the sensor and
                          // within the robot radiuis.  So just reset to zero.

    // Find the nearest sensor to the cell
    // Initialise the angle to be PI as this all other angles
    // will be less (clockwise or anticlockwise)
    double kMinDelta=Math.PI; // Smallest distance away from a sensor (we try to minimise this)
        
    for (int j=0; j<this.prox_sensors.get_number_of_sensors(); j++) {
      PioneerProxSensors prox = this.prox_sensors;
      kDelta = Math.abs(this.prox_sensors.get_sensor_pose(j).get_dtheta(phi));
            
      if (kDelta < kMinDelta) {
        k=j;
        kMinDelta = kDelta;
      }
    }
    // we now known that k=closest sensor, and kMinDelta is the difference in angle
    
    // Determine which region the cell is in
    double z = this.prox_sensors.get_value(k);
    if (z == this.prox_sensors.get_maxRange()) {
      logodds = lprior;
    } else if ((r > Math.min(this.prox_sensors.get_maxRange(), z+HALFALPHA)) || (kMinDelta > HALFBETA)) {
      // Region 3 - (unknown)
      logodds = lprior;
    } else if ((z < this.prox_sensors.get_maxRange()) && (Math.abs(r-z) < HALFALPHA)) {
      // Region 1 - (occupied)
      logodds = locc;
    } else if (r <= z) {
      // Region 1 - (free)
      logodds = lfree;
    }
    return logodds;
  }

  // ==================================================================================
  // External Methods  
  // ==================================================================================
  public void map() {
    
    double x;                              // x coord
    double y;                              // y coord
    double x_orig_offset = this.arena_width/2.0;   // Offset for origin along x axis
    double y_orig_offset = this.arena_height/2.0;  // Offset for origin along y axis
    
    double x_inc = this.arena_width/this.num_row_cells;
    double y_inc = this.arena_height/this.num_col_cells;

    double x_cell_offset = x_inc/2.0;          // offset to center of cell along x axis
    double y_cell_offset = y_inc/2.0;          // offset to center of cell along y axis
    
    
    for (int i=0; i < this.grid.length; i++) {
      // Convert cell into a coordinate.  Recall that the arena is dimensions -n..+n
      x = x_inc * (i%this.num_row_cells)-x_orig_offset+x_cell_offset;
      y = -(y_inc * (i/this.num_row_cells)-y_orig_offset+y_cell_offset);
      
      // Log Odds Update Function
      this.grid[i] = this.grid[i] + invSensorModel(this.robot_pose, x, y) - lprior;
    }
  } 
  
  public void paint() { 
    if (this.display==null)
      return;
    
    // draw a background
    this.display.setColor(0xF0F0F0);     // Off White
    this.display.fillRectangle(0, 0, this.device_width, this.device_height);
    
    double cellProb;          // probabilty of occuupancy for each cell
    this.coverage = 0.0;
    for (int i=0; i< this.grid.length; i++) {
      cellProb = this.cell_probability(this.grid[i]);
      int x = this.cell_width * (i%this.num_row_cells);
      int y = this.cell_height * (i/this.num_row_cells);
                
      // Determine colour - Uses a very simple approach for now with graduated grey shades
      if (cellProb < 0.1) this.display.setColor(0xFFFFFF);
      else if (cellProb < 0.2) this.display.setColor(0xDDDDDD);
      else if (cellProb < 0.3) this.display.setColor(0xBBBBBB);
      else if (cellProb < 0.4) this.display.setColor(0x999999);
      else if (cellProb > 0.9) this.display.setColor(0x000000);
      else if (cellProb > 0.8) this.display.setColor(0x222222);
      else if (cellProb > 0.7) this.display.setColor(0x444444);
      else if (cellProb > 0.6) this.display.setColor(0x666666);
      else this.display.setColor(0x787878);
      
      this.display.fillRectangle(x, y, this.cell_width, this.cell_height);
      
      if ((cellProb < 0.1) || (cellProb > 0.9))
        this.coverage += 1.0;
    }
    // normalise coverage
    this.coverage = this.coverage / this.grid.length;

    this.display.setColor(0x3C3C3C);     // Dark Grey
    // Draw Vertical Lines
    for (int i=0, x=0; i <= this.num_row_cells; i++, x+= this.cell_width) {    // Draw extra border after last cell
      this.display.drawLine(x, 0, x, this.device_height);
    }           
      
    // Draw Horizontal Lines
     for (int i=0, y=0; i <= this.num_row_cells; i++, y+= this.cell_height) {    // Draw extra border after last cell
      this.display.drawLine(0, y, this.device_width, y);
    }           

    // Draw Robot Body          
    this.display.setColor(0xFFFFFF);     // White
    this.display.fillOval(this.mapx(this.robot_pose.getX()),
                          this.mapy(this.robot_pose.getY()),
                          this.scale(this.radius),
                          this.scale(this.radius));    
    
    this.display.setColor(0x3C3C3C);     // Dark Grey
    this.display.drawOval(this.mapx(this.robot_pose.getX()),
                          this.mapy(this.robot_pose.getY()),
                          this.scale(this.radius),
                          this.scale(this.radius));    
    // Need to indicate heading          
    this.display.drawLine(this.mapx(this.robot_pose.getX()),
                          this.mapy(this.robot_pose.getY()),
                          this.mapx(this.robot_pose.getX() + Math.cos(this.robot_pose.getTheta()) * this.radius),
                          this.mapy(this.robot_pose.getY() + Math.sin(this.robot_pose.getTheta()) * this.radius));
                           
    // Provide coverage percentage
    this.display.setColor(0xF0F0F0);     // Off White
    this.display.fillRectangle(this.device_width-80, this.device_height-18, this.device_width-20, this.device_height);
    this.display.setColor(0x000000);     // Black
    this.display.drawRectangle(this.device_width-80, this.device_height-18, this.device_width-20, this.device_height);


    this.display.setFont("Arial", 10, true);  // font size = 10, with antialiasing
    this.display.drawText(String.format("%.2f%%", this.coverage * 100), this.device_width-60,this.device_height-14);
  
  } 
}
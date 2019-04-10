package ca.mcgill.ecse211.wifi;

import java.util.Map;
import ca.mcgill.ecse211.demo.FinalDemo;

/**
 * Represents a rectangle with an UR and LL point.
 * @author jacob
 *
 */
public class Rect {
  public final int URx;
  public final int URy;
  public final int LLx;
  public final int LLy;

  /**
   * Creates a rectangle given a prefix, and a set of data from the 
   * server. Because rectangles are specified by a prefix (say, TNG)
   * and a series of points (_UR_x, _LL_y, etc), specifying the prefix
   * is enough to find the 4 points from the server to construct the rect.
   * @param prefix The prefix to use
   * @param data The information from the server
   */
  public Rect(String prefix, Map data) {
    this.URx = ((Long) data.get(prefix + "_UR_x")).intValue();
    this.LLx = ((Long) data.get(prefix + "_LL_x")).intValue();
    this.URy = ((Long) data.get(prefix + "_UR_y")).intValue();
    this.LLy = ((Long) data.get(prefix + "_LL_y")).intValue(); 
  }
  
  /**
   * Whether or not a point (x,y) (in cm) is contained within
   * the area on the game map represented by the rectangle
   * @param x The x coordinate of the point in cm
   * @param y The y coordinate of the point in cm
   * @return True if the point is contained, else false
   */
  public boolean contains(double x, double y) {
    return x <= FinalDemo.GRID_WIDTH*URx && x >= FinalDemo.GRID_WIDTH*LLx
        && y <= FinalDemo.GRID_WIDTH*URy && y >= FinalDemo.GRID_WIDTH*LLy;
  }
  
  /**
   * Whether or not a point {x,y,t} (in cm) is contained within
   * the area on the game map represented by the rectangle
   * @param xyt The location of the robot as given by it's odometer's data.
   * @return True if the point is contained, else false
   */
  public boolean contains(double[] xyt) {
    return contains(xyt[0], xyt[1]);
  }
}

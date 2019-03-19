package ca.mcgill.ecse211.wifi;

import java.util.Map;
import ca.mcgill.ecse211.demo.BetaDemo;

public class Rect {
  public final int URx;
  public final int URy;
  public final int LLx;
  public final int LLy;

  public Rect(String prefix, Map data) {
    this.URx = ((Long) data.get(prefix + "_UR_x")).intValue();
    this.LLx = ((Long) data.get(prefix + "_LL_x")).intValue();
    this.URy = ((Long) data.get(prefix + "_UR_y")).intValue();
    this.LLy = ((Long) data.get(prefix + "_LL_y")).intValue(); 
  }

  public boolean contains(double x, double y) {
    return x <= BetaDemo.GRID_WIDTH*URx && x >= BetaDemo.GRID_WIDTH*LLx
        && y <= BetaDemo.GRID_WIDTH*URy && y >= BetaDemo.GRID_WIDTH*LLy;
  }
  
  public boolean contains(double[] xyt) {
    return contains(xyt[0], xyt[1]);
  }
}

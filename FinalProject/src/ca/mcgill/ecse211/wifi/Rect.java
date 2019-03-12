package ca.mcgill.ecse211.wifi;

import java.util.Map;

public class Rect {
  int URx;
  int URy;
  int LLx;
  int LLy;

  public Rect(String prefix, Map data) {
    this.URx = ((Long) data.get(prefix + "_UR_x")).intValue();
    this.LLx = ((Long) data.get(prefix + "_LL_x")).intValue();
    this.URy = ((Long) data.get(prefix + "_UR_y")).intValue();
    this.LLy = ((Long) data.get(prefix + "_LL_y")).intValue(); 
  }

  public boolean contains(double x, double y) {
    return x <= URx && x >= LLx && y <= URy && y >= LLy;
  }
}

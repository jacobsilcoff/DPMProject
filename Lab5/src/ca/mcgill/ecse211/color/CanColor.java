package ca.mcgill.ecse211.color;
//TODO: add stddev
public enum CanColor {
  RED("Red", new int[] {47, 18, 16}), GREEN("Green", new int[] {18, 46, 20}), BLUE("Blue",
      new int[] {24, 43, 57}), YELLOW("Yellow", new int[] {61, 43, 31}), UNKOWN;

  private String name;
  private int[] avgRGB;

  CanColor(String name, int[] c) {
    this.name = name;
    avgRGB = c;
  }

  CanColor() {
    this.name = "?";
  }

  public static CanColor getWithName(String s) {
    switch (s) {
      case "Red":
        return RED;
      case "Green":
        return GREEN;
      case "Blue":
        return BLUE;
      case "Yellow":
        return YELLOW;
      default:
        return UNKOWN;
    }
  }

  /**
   * Returns the closest can color to the given value
   * 
   * @param c A byte array of r g and b values
   * @return the color w/ the closest euclidean distance to c
   */
  public static CanColor getClosestColor(int[] color) {
    CanColor closest = UNKOWN;
    double minDist = 28;
    for (CanColor c : new CanColor[] {RED, GREEN, YELLOW, BLUE}) {
      if (c.distanceTo(color) < minDist) {
        minDist = c.distanceTo(color);
        closest = c;
      }
    }
    return closest;
  }

  public double distanceTo(int[] c) {
    int sum = 0;
    for (int i = 0; i < c.length; i++) {
      sum += Math.pow((int) c[i] - avgRGB[i], 2);
    }
    return Math.sqrt(sum);
  }


  public String toString() {
    return name;
  }
  
  public static CanColor fromNumber(int n) {
    switch(n) {
      case 1:
        return BLUE;
      case 2: 
        return GREEN;
      case 3:
        return YELLOW;
      case 4:
        return RED;
      default:
        return UNKOWN;
    }
  }
}

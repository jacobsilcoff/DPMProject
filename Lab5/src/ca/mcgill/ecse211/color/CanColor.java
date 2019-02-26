package ca.mcgill.ecse211.color;

/**
 * Represents the 4 different possible can colors. 
 * @author Group 6
 */
public enum CanColor {
  RED("Red", new int[] {117, 37, 36}), GREEN("Green", new int[] {18, 46, 20}), BLUE("Blue",
      new int[] {24, 43, 57}), YELLOW("Yellow", new int[] {61, 43, 31}), UNKNOWN;

  private String name;
  private int[] avgRGB;

  /**
   * Creates a CanColor
   * @param name The name of the color
   * @param c An array of 3 integers representing the R, G, and B intensity. 
   * this array will be normalized so magnitudes don't matter.
   */
  CanColor(String name, int[] c) {
    this.name = name;
    avgRGB = c;
  }
  
  /**
   * Creates an UNKOWN can color, with no parameters.
   */
  CanColor() {
    this.name = "?";
  }
  
  /**
   * Returns the can color with the given name
   * @param s The name to search for
   * @return The can color with that name. UKNOWN if no such can exists.
   */
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
        return UNKNOWN;
    }
  }

  /**
   * Returns the closest can color to the given value,
   * using the euclidean distance to the mean of each color.
   * @param c An int array of r g and b values
   * @return the color w/ the closest euclidean distance to c
   */
  public static CanColor getClosestColor(int[] color) {
    CanColor closest = UNKNOWN;
    
    double minDist = 2;
    for (CanColor c : new CanColor[] {RED, GREEN, YELLOW, BLUE}) {
      if (c.normalizedDistTo(color) < minDist) {
        minDist = c.normalizedDistTo(color);
        closest = c;
      }
    }
    return closest;
  }
  
  /**
   * Finds the euclidean distance between the color's RGB values
   * and a given colors RGB values, after normalizing both to unit
   * vectors.
   * @param c An array of three integers representing an RGB color
   * @return The distance from this color to the given color c, once normalized.
   */
  public double normalizedDistTo(int[] c) {
    double sum = 0;
    double[] normalizedAvg = normalize(avgRGB);
    double[] normalizedIn = normalize(c);
    for (int i = 0; i < c.length; i++) {
      sum += Math.pow(normalizedIn[i] - normalizedAvg[i], 2);
    }
    return Math.sqrt(sum);
  }
  
  /**
   * Turns an integer array into a unit vector
   * @param c An array of integers representing a vector.
   * @return An array representing a unit vector in the
   * direction of c.
   */
  public static double[] normalize(int[] c) {
    int sum = 0;
    for (int i : c) {
      sum += i;
    }
    double[] retVal = new double[3];
    for (int i = 0; i < 3; i++) {
      retVal[i] = ((double) c[i]) / sum;
    }
    return retVal;
  }


  public String toString() {
    return name;
  }
  
  /**
   * Gets the can color associated with a certain target number,
   * as specified by the lab guidlines.
   * @param n The target can number
   * @return The color associated with n
   */
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
        return UNKNOWN;
    }
  }
}

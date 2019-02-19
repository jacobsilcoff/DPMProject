package ca.mcgill.ecse211;

public enum CanColor {
  RED("Red"), 
  GREEN("Green"),
  BLUE("Blue"), 
  YELLOW("Yellow"), 
  UNKOWN("?");
  
  private String name;
  
  CanColor(String name) {
    this.name = name;
  }
  
  public static CanColor getWithName(String s) {
    switch(s) {
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
  
  public String toString() {
    return name;
  }
}

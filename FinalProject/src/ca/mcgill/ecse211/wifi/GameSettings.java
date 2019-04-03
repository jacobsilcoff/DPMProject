package ca.mcgill.ecse211.wifi;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.canhandling.CanColor;
import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

/**
 * Gets and holds values from the server to set up a game
 * @author jacob
 */
public abstract class GameSettings {
  
  /** Set these as appropriate for your team and current situation **/
  private static final String SERVER_IP = "192.168.2.38";
  private static final int TEAM_NUMBER = 6;

  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;
  
  public static boolean initialized = false;
  public static boolean redTeam = false;
  public static int corner = -1;
  public static Rect startZone = null;
  public static Rect island = null;
  public static Rect searchZone = null;
  public static Rect tunnel = null;
  public static CanColor targetColor = CanColor.BLUE;
  
  /**
   * Communicates with the server to get access to game information
   * Initializes public static fields
   */
  public static void init() {

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
      Map data = conn.getData();
      LCD.clear();

      //Get team assignment

      long rTeam = ((Long) data.get("RedTeam")).intValue();
      long greenTeam = ((Long) data.get("GreenTeam")).intValue();
      //targetColor = CanColor.fromNumber((int) greenTeam);
      if (rTeam == TEAM_NUMBER) {
        redTeam = true;
      } else if (greenTeam != TEAM_NUMBER) {
        return;
      }
      
      String color = (redTeam? "Red" : "Green");
      char colorAbrv = (redTeam? 'R' : 'G');

      corner = ((Long) data.get(color + "Corner")).intValue();
      System.out.println("Corner: " + corner);

      startZone = new Rect(color, data);
      island = new Rect("Island", data);
      tunnel = new Rect("TN" + colorAbrv , data);
      searchZone = new Rect("SZ" + colorAbrv, data);
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
      initialized = false;
    }
    initialized = true;
  }
  
  public static Point2D getStartingCornerPoint() {
    if (GameSettings.initialized) {
      switch (GameSettings.corner) {
        case 1:
          return new Point2D.Double(14*FinalDemo.GRID_WIDTH,
              FinalDemo.GRID_WIDTH);
        case 2:
          return new Point2D.Double(14*FinalDemo.GRID_WIDTH,
              8*FinalDemo.GRID_WIDTH);
        case 3:
          return new Point2D.Double(FinalDemo.GRID_WIDTH,
              8*FinalDemo.GRID_WIDTH);
        default:
          return new Point2D.Double(FinalDemo.GRID_WIDTH, FinalDemo.GRID_WIDTH);
      }
    } 
    return new Point2D.Double(FinalDemo.GRID_WIDTH, FinalDemo.GRID_WIDTH);
  }
}

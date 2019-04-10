package ca.mcgill.ecse211.wifi;

import java.awt.geom.Point2D;
import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.canhandling.CanColor;
import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import lejos.hardware.lcd.LCD;

/**
 * Gets and holds values from the server to set up a game
 * @author jacob
 */
public abstract class GameSettings {
  
  /** Set these as appropriate for your team and current situation **/
  private static final String SERVER_IP = "192.168.2.14";
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
  public static double[] tunnelEntrance;
  public static double[] tunnelExit;
  public static double[] safeLocStart;
  public static double[] safeLocIsland;
  public static double[] startSearch;
  public static double[] searchAngles;
  
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

      double[][] entranceAndExit = tunnelEntranceAndExit();
      tunnelEntrance = entranceAndExit[0];
      tunnelExit = entranceAndExit[1];
      safeLocStart = safeLightLocalizationPointStart();
      safeLocIsland = safeLightLocalizationPointIsland();
      setSearchParams();
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
  
  /**
   * Finds a safe point for light localization
   * outside the tunnel.
   * @return A point of the form {x1,y1}
   */
  private static double[] safeLightLocalizationPointStart() {
    double g = FinalDemo.GRID_WIDTH;
    double[] bestPoint = {g,g};
    double bestDist = Navigation.dist(bestPoint, tunnelEntrance);
    for (int x = startZone.LLx; x < startZone.URx; x++) {
      for (int y = startZone.LLy; y < startZone.URy; y++) {
        if (startZone.contains((x-.5)*g, (y-.5)*g) &&
            startZone.contains((x+.5)*g, (y-.5)*g) &&
            startZone.contains((x-.5)*g, (y+.5)*g) &&
            startZone.contains((x+.5)*g, (y+.5)*g) &&
            !tunnel.contains((x-.5)*g, (y-.5)*g) &&
            !tunnel.contains((x+.5)*g, (y-.5)*g) &&
            !tunnel.contains((x-.5)*g, (y+.5)*g) &&
            !tunnel.contains((x+.5)*g, (y+.5)*g)) {
          double dist = Navigation.dist(tunnelEntrance, new double[] {x*g,y*g});
          if (dist < bestDist) {
            bestDist = dist;
            bestPoint = new double[] {x*g,y*g};
          }
        }
      }
    }
    return bestPoint;
  }
  
  /**
   * Finds a safe point for light localization
   * before going back through the tunnel.
   * @return A point of the form {x1,y1}
   */
  private static double[] safeLightLocalizationPointIsland() {
    double g = FinalDemo.GRID_WIDTH;
    double[] bestPoint = {g,g};
    double bestDist = Navigation.dist(bestPoint, tunnelExit);
    for (int x = island.LLx; x < island.URx; x++) {
      for (int y = island.LLy; y < island.URy; y++) {
        if (island.contains((x-.5)*g, (y-.5)*g) &&
            island.contains((x+.5)*g, (y-.5)*g) &&
            island.contains((x-.5)*g, (y+.5)*g) &&
            island.contains((x+.5)*g, (y+.5)*g) &&
            !tunnel.contains((x-.5)*g, (y-.5)*g) &&
            !tunnel.contains((x+.5)*g, (y-.5)*g) &&
            !tunnel.contains((x-.5)*g, (y+.5)*g) &&
            !tunnel.contains((x+.5)*g, (y+.5)*g)) {
          double dist = Navigation.dist(tunnelExit, new double[] {x*g,y*g});
          if (dist < bestDist) {
            bestDist = dist;
            bestPoint = new double[] {x*g,y*g};
          }
        }
      }
    }
    return bestPoint;
  }
  
  /**
   * Returns an array of the form 
   * {{x1,y1},{x2,y2}}
   * where (x1,y1) is the start side of the tunnel
   * and x2,y2 is the island side of the tunnel
   * @return the entrance & exit to the tunnel
   */
  private static double[][] tunnelEntranceAndExit() {
    Rect tunnel = GameSettings.tunnel;
    Rect startZone = GameSettings.startZone;
    Rect island = GameSettings.island;
    double[] llBlock = {(tunnel.LLx + .5) * FinalDemo.GRID_WIDTH, 
                        (tunnel.LLy + .5) * FinalDemo.GRID_WIDTH};
    double[] urBlock = {(tunnel.URx - .5) * FinalDemo.GRID_WIDTH, 
                        (tunnel.URy - .5) * FinalDemo.GRID_WIDTH};
    double[] N = translate(urBlock, 0, FinalDemo.GRID_WIDTH);
    double[] S = translate(llBlock, 0, -FinalDemo.GRID_WIDTH);
    double[] E = translate(urBlock,FinalDemo.GRID_WIDTH, 0);
    double[] W = translate(llBlock, -FinalDemo.GRID_WIDTH, 0);
    //strictly one of N, S, E, W is contained in start
    double[] entrance = N, exit = S;
    if (startZone.contains(N) && island.contains(S)) {
      entrance = N; exit = S;
    } else if (startZone.contains(S) && island.contains(N)) {
      entrance = S; exit = N;
    } else if (startZone.contains(E) && island.contains(W)) {
      entrance = E; 
      exit = W;
    } else if (startZone.contains(W) && island.contains(E)){
      entrance = W;
      exit = E;
    }
    return new double[][] {entrance,exit};
  }
  
  /**
   * Modifies a point by adding x and y
   * @param pt the original point
   * @param x the translation x amt
   * @param y the translation yamt
   * @return the translated point
   */
  private static double[] translate(double[] pt, double x, double y) {
    return new double[] {pt[0] + x, pt[1] + y};
  }
  
  /**
   * Sets up the data needed to safely search w/o hitting cans
   */
  private static void setSearchParams() {
    double[][] opts = {{searchZone.LLx, searchZone.LLy},
                       {searchZone.URx, searchZone.LLy},
                       {searchZone.URx, searchZone.URy},
                       {searchZone.LLx, searchZone.URy}};
    double[][] angles = {{0, 90},
                         {270, 360},
                         {180, 270},
                         {90, 180}};
    startSearch = opts[0];
    int bestInd = 0;
    double bestDist = Double.MAX_VALUE;
    double g = FinalDemo.GRID_WIDTH;
    for (int i = 0; i < 4; i++) {
      double d = Point2D.distance(opts[i][0]*g, opts[i][1]*g, tunnelExit[0], tunnelExit[1]);
      if ((d < bestDist) && opts[i][0] > 0 && opts[i][0] < 15
          && opts[i][1] > 0 && opts[i][1] < 9) {
        bestInd = i;
        bestDist = d;
      }
    }
    startSearch = opts[bestInd];
    searchAngles = angles[bestInd];
  }
}

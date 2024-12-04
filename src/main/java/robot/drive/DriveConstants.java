package robot.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

public class DriveConstants {
  public static final double WHEEL_RADIUS = 0.08;
  public static final double CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS;
  public static final double GEARING = 8.0;
  public static final double POSITION_FACTOR = CIRCUMFERENCE * GEARING;
  public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;
  public static double MOI = 7.5;
  public static double DRIVE_MASS = 60.0;
  public static double TRACK_WIDTH = 0.7112;
  public static double MAX_VOLTAGE;
  public static final Matrix<N7, N1> STD_DEVS = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);

  public static final double MAX_SPEED = 2.0;

  public final class FF {
    public static final double kS = 1;
    public static final double kV = 3;
  }

  public final class PID {
    public static final double kP = 8.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}

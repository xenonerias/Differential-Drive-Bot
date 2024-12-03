package robot.drive;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import robot.Constants;
import robot.Ports;
import robot.Robot;
import robot.drive.DriveConstants.FF;
import robot.drive.DriveConstants.PID;

public class Drive extends SubsystemBase {

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);

  private final CANSparkMax leftLeader =
      new CANSparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
  private final CANSparkMax leftFollower =
      new CANSparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
  private final CANSparkMax rightLeader =
      new CANSparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
  private final CANSparkMax rightFollower =
      new CANSparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  @Log.NT private final Field2d field2d = new Field2d();

  private final DifferentialDriveOdometry odometry;
  private Rotation2d simRotation = new Rotation2d();
  private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);
  private double rightVoltage;
  private double leftVoltage;
  private final DifferentialDrivetrainSim driveSim;

  public Drive() {

    leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
    {
      rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);

      leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
      rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);

      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);

      for (CANSparkMax spark : List.of(leftLeader, leftFollower, rightLeader, rightFollower)) {
        spark.restoreFactoryDefaults();
        spark.setIdleMode(IdleMode.kBrake);
      }

      rightFollower.follow(rightLeader);
      leftFollower.follow(leftLeader);

      leftLeader.setInverted(true);
    }

    gyro.reset();

    odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

    driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getMiniCIM(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.DRIVE_MASS,
            DriveConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            DriveConstants.STD_DEVS);
  }
    private final PIDController leftPIDController =
      new PIDController(PID.kP, PID.kI, PID.kD);
  private final PIDController rightPIDController =
      new PIDController(PID.kP, PID.kI, PID.kD);

  public void updateOdometry(Rotation2d rotation) {
    odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  @Log.NT
  public Pose2d pose() {
    return odometry.getPoseMeters();
  }

  @Log.NT
  public double getDistance() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }

  @Override
  public void periodic() {
    updateOdometry(Robot.isReal() ? gyro.getRotation2d() : simRotation);

    if (Robot.isSimulation()) {
      driveSim.setInputs(leftVoltage, rightVoltage);
      field2d.setRobotPose(pose());
    }
  }

  @Override
  public void simulationPeriodic() {
    driveSim.update(Constants.PERIOD.in(Seconds));
    leftEncoder.setPosition(driveSim.getLeftPositionMeters());
    rightEncoder.setPosition(driveSim.getRightPositionMeters());
    simRotation = driveSim.getHeading();
  }

  public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
    return new RunCommand(
        () -> {
          double leftSpeed = vLeft.getAsDouble();
          double rightSpeed = vRight.getAsDouble();
          drive(leftSpeed, rightSpeed);
        },
        this);
  }

  private void driver(double vLeft, double vRight) {
    leftLeader.set(vLeft);
    rightLeader.set(vRight);
  }

  public void drive(double leftSpeed, double rightSpeed) {
	final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
	final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
	
    final double leftFeedforward = feedforward.calculate(realLeftSpeed);
    final double rightFeedforward = feedforward.calculate(realRightSpeed);

    final double leftPID = 
      leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
    final double rightPID = 
      rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);

      double leftVoltage = leftPID + leftFeedforward;
      double rightVoltage = rightPID + rightFeedforward;

      leftLeader.setVoltage(leftVoltage);
      rightLeader.setVoltage(rightVoltage);
  }

}
    
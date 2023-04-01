package frc.robot;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.management.RobotHandler;
// // import frc.robot.subsystems.management.Subsystem;
// import frc.robot.subsystems.sensors.Gyro;
import frc.robot.subsystems.sensors.Sensors;
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final Sensors sensors = new Sensors();
  private final RobotHandler robotHandler = new RobotHandler();

  private final Drivetrain drivetrain = new Drivetrain();
  private final Turret turret = new Turret();
  private final pneumatics pneumatics = new pneumatics();
  // Creates a PIDController with gains kP, kI, and kD
  // TODO: Tune PID Vars
  public PIDController leftside_pid = new PIDController(0.001, 0, 0);
  public PIDController rightside_pid = new PIDController(0.001, 0, 0);
  public PIDController gyro_pid = new PIDController(0.001, 0, 0);

  // private final Gyro gyro = new Gyro();

  public GenericHID driver = new GenericHID(0);
  public GenericHID aid = new GenericHID(1);

  private final Autonomous autonomous = new Autonomous();


  @Override
  public void robotInit() {
    /**
     * Rev 2m distance sensor can be initialized with the Onboard I2C port
     * or the MXP port. Both can run simultaneously.
     */
    this.m_chooser.setDefaultOption("Default Auto", "Default");
    this.m_chooser.addOption("levelOutRobot", "levelOutRobot");
    this.m_chooser.addOption("bumpSideLocation", "bumpSideLocation");
    this.m_chooser.addOption("nonBumpSideLocation", "nonBumpSideLocation");
    this.m_chooser.addOption("chargerSideLocation", "chargerSideLocation");
    this.m_chooser.addOption("theClawwa", "theClawwa");
    SmartDashboard.putData("Auto choices", (Sendable)this.m_chooser);
    robotHandler
    .add(drivetrain)
    .add(turret)
    .add(pneumatics)
    // .add(gyro)
    //.addTouchSensor(sensors, 9)
    //.addTouchSensor(sensors, 8)
    //.addLimelight(sensors)
    .add(sensors)
    //.addUltrasonicSensor(sensors, 0, 5000, 0)
    //.addUltrasonicSensor(sensors, 1, 5000, 0)
    .allocateSensors(sensors, turret)
    .allocateGamepads(driver, aid, drivetrain)
    .allocateGamepads(driver, aid, turret)
    .allocateGamepads(driver, aid, pneumatics);
    this.robotHandler.onRobotInit();
  }

  @Override
  public void robotPeriodic() {
    this.robotHandler.onRobotPeriodic();
  }

  @Override
  public void autonomousInit() {
    this.m_autoSelected = (String)this.m_chooser.getSelected();
    this.autonomous.onAutonomousInit();
      if (this.m_autoSelected == "Default") {
        this.autonomous.test(this.drivetrain, this.turret, this.pneumatics, this.sensors, 0.02D, 0, 0, isAutonomous());
      } else if (this.m_autoSelected == "levelOutRobot") {
        this.autonomous.levelOutRobot(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
      } else if (this.m_autoSelected == "bumpSideLocation") {
        this.autonomous.bumpSideLocation(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
      } else if (this.m_autoSelected == "nonBumpSideLocation") {
        this.autonomous.nonBumpSideLocation(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
      } else if (this.m_autoSelected == "chargerSideLocation") {
        this.autonomous.chargerSideLocation(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
      } else if (this.m_autoSelected == "theClawwa") {
        this.autonomous.theClawwa(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
      }
    System.out.println("Auto selected: " + this.m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        break;
      case kDefaultAuto:
      default:
        break;
    }
    // if (this.m_autoSelected == "levelOutRobot") {
    //   this.autonomous.levelOutRobot(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
    // } else if (this.m_autoSelected == "bumpSideLocation") {
    //   this.autonomous.bumpSideLocation(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
    // } else if (this.m_autoSelected == "nonBumpSideLocation") { // chargerSideLocation
    //   this.autonomous.nonBumpSideLocation(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
    // } else if (this.m_autoSelected == "chargerSideLocation") {
    //   this.autonomous.chargerSideLocation(this.drivetrain, this.turret, this.pneumatics, this.sensors, this.gyro_pid, isAutonomous());
    // }
  }                 

  @Override
  public void teleopInit() {
    this.robotHandler.onTeleopInit();
  }

  @Override
  public void teleopPeriodic() {
    /*NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    double x = tx.getDouble(0);
    double y = ty.getDouble(0);
    double area = ta.getDouble(0);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);*/
    this.robotHandler.onTeleopPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.management.RobotHandler;
import frc.robot.subsystems.management.Subsystem;
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

  public GenericHID driver = new GenericHID(0);
  public GenericHID aid = new GenericHID(1);
  
  private final Autonomous autonomous = new Autonomous();

  //creates network table instance
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  //defines network table "datatable"
  NetworkTable datatable = inst.getTable("datatable");
  NetworkTable limelightVar = inst.getTable("limelight");
  final DoublePublisher usb0Pub = datatable.getDoubleTopic("USB0").publish();
  final DoublePublisher usb1Pub = datatable.getDoubleTopic("USB1").publish();
  // final DoublePublisher limelightLightPub = limelightVar.getDoubleTopic("");
  // final DoubleSubscriber limelightLightSub
  
  @Override
  public void robotInit() {
    this.m_chooser.setDefaultOption("Default Auto", "Default");
    this.m_chooser.addOption("My Auto", "My Auto");
    SmartDashboard.putData("Auto choices", (Sendable)this.m_chooser);

    // Helps with ASD (automatic stick detection)
    usb0Pub.set(DriverStation.getStickAxisCount(0));
    usb1Pub.set(DriverStation.getStickAxisCount(1));



    robotHandler
    .add(drivetrain)
    .add(turret)
    .add(pneumatics)
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
    usb0Pub.set(DriverStation.getStickAxisCount(0));
    usb1Pub.set(DriverStation.getStickAxisCount(1));

    if(DriverStation.getStickAxisCount(0) == 4){
      driver = new GenericHID(0);
      aid = new GenericHID(1);
    }

    if(DriverStation.getStickAxisCount(0) == 6){
      driver = new GenericHID(1);
      aid = new GenericHID(0);
    }
  }
  
  @Override
  public void autonomousInit() {
    this.m_autoSelected = (String)this.m_chooser.getSelected();

    this.autonomous.onAutonomousInit();
    
      this.autonomous.test(this.drivetrain, this.turret, this.pneumatics, this.sensors, 0.02D, 0, 0, isAutonomous());
    
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

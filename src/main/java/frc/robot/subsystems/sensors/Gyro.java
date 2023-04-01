package frc.robot.subsystems.sensors;
import java.util.List;
import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.IMUProtocol;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.management.PID;
import frc.robot.subsystems.management.Subsystem;
import frc.robot.subsystems.sensors.Gyro;
// import frc.robot.subsystems.sensors.Sensors;
public class Gyro extends Subsystem {
  public GenericHID driver;
  public GenericHID aid;
  public AHRS gyro = new AHRS(Port.kMXP);

  public CANSparkMax leftFrontMotor;
  public CANSparkMax leftBackMotor;
  public CANSparkMax rightFrontMotor;
  public CANSparkMax rightBackMotor;
  public RelativeEncoder LeftFrontEncoder;
  public RelativeEncoder RightFrontEncoder;
  public RelativeEncoder LeftBackEncoder;
  public RelativeEncoder RightBackEncoder;
  public RelativeEncoder m_leftEncoder;
  public RelativeEncoder m_rightEncoder;
  private Sensors sensors;
  public Trajectory m_trajectory;
  public Pose2d m_pose;
  public Shuffleboard shuffleboard;

  // Creating my kinematics object: track width of 27 inches
DifferentialDriveKinematics kinematics =
new DifferentialDriveKinematics(Units.inchesToMeters(27.0));
// Example chassis speeds: 2 meters per second linear velocity,
// 1 radian per second angular velocity.
public ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
// Convert to wheel speeds
DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
// Left velocity
double leftVelocity = wheelSpeeds.leftMetersPerSecond;
// Right velocity
double rightVelocity = wheelSpeeds.rightMetersPerSecond;
  public MotorController m_leftBackMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  public MotorController m_leftFrontMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  public MotorController m_rightFrontMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  public MotorController m_rightBackMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  public MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftBackMotor, m_leftFrontMotor);
  public MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightBackMotor, m_rightFrontMotor);
  // Creating my odometry object. Here,
  // our starting pose is 5 meters along the long end of the field and in the
  // center of the field along the short end, facing forward.
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(),
    LeftBackEncoder.getPosition(), m_rightEncoder.getPosition(),
    new Pose2d(5.0, 13.5, new Rotation2d()));

  // The robot is moving at 3 meters per second forward, 2 meters
  // per second to the right, and rotating at half a rotation per
  // second counterclockwise.
  public ChassisSpeeds speeds1 = new ChassisSpeeds(3.0, -2.0, Math.PI);

  // The desired field relative speed here is 2 meters per second
  // toward the opponent's alliance station wall, and 2 meters per
  // second toward the left field boundary. The desired rotation
  // is a quarter of a rotation per second counterclockwise. The current
  // robot angle is 45 degrees.
  ChassisSpeeds speeds2 = ChassisSpeeds.fromFieldRelativeSpeeds(
    2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
  private Field2d m_field = new Field2d();

  public boolean getGyroangle() {
    gyro.getAngle();
    return this != null;
  }
  public boolean zeroYaw() {
    gyro.zeroYaw();
    float h = gyro.getCompassHeading();
    SmartDashboard.putNumber("number of subsystems", gyro.getCompassHeading());
    return this != null;
  }
   public final void IdealPitch() {
    if (zeroYaw()) {
        leftFrontMotor.set(0);
        leftBackMotor.set(0);
        rightFrontMotor.set(0);
        rightBackMotor.set(0);
    } else {
        leftFrontMotor.set(0.5);
        leftBackMotor.set(0.5);
        rightFrontMotor.set(0.5);
        rightBackMotor.set(0.5);
    }
   }
public void Drivetrain() {
    SmartDashboard.putData("Field", m_field);
  }
  public void tankDriveEncoders(double Distance) {
    RelativeEncoder LeftBackEncoder = leftBackMotor.getEncoder();
    RelativeEncoder LeftFrontEncoder = leftFrontMotor.getEncoder();
    RelativeEncoder RightBackEncoder = rightBackMotor.getEncoder();
    RelativeEncoder RightFrontEncoder = rightFrontMotor.getEncoder();
 
    LeftBackEncoder.setPosition(Distance);
    LeftFrontEncoder.setPosition(Distance);
    RightBackEncoder.setPosition(Distance);
    RightFrontEncoder.setPosition(Distance);
  }

public double lbEncoderGet() {
    RelativeEncoder LeftBackEncoder = leftBackMotor.getEncoder();
    return LeftBackEncoder.getPosition();
  }

  public double lfEncoderGet() {
    RelativeEncoder LeftFrontEncoder = leftFrontMotor.getEncoder();
    return LeftFrontEncoder.getPosition();
  }

  public double rbEncoderget() {
    RelativeEncoder RightBackEncoder = rightBackMotor.getEncoder();
    return RightBackEncoder.getPosition();
  }

  public double rfEncoderGet() {
    RelativeEncoder RightFrontEncoder = rightFrontMotor.getEncoder();
    return RightFrontEncoder.getPosition();
  }

  @Override
  public void onRobotInit() {
    leftBackMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFrontMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    // Get the rotation of the robot from the gyro.
    var gyroAngle = gyro.getRotation2d();
 
    // Update the pose
    m_pose = m_odometry.update(gyroAngle,
      LeftBackEncoder.getPosition(),
      m_rightEncoder.getPosition());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);
    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(m_trajectory);
  }

  @Override
  public void onTeleopInit() {
    //SmartDashboard.putString("Drivetrain", "RAN");
  }

  @Override
  public void onTeleopPeriodic() {
      //Field centric drive (to flightstick)
      /*double Forward = driver.getRawAxis(1);
      double strafe = driver.getRawAxis(0);
      double Turn = driver.getRawAxis(2);

      double pi = 3.1415926;

      double gyroDegrees = Axes.getYaw();
      double gyroRadians = gyroDegrees * pi/180;

      double temp = Forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
      double Strafe = -Forward * Math.sin(gyroRadians) + strafe * Math.cos(gyroRadians);
      double forward = temp;
      leftFrontMotor.set(forward - Strafe - Turn);
      leftBackMotor.set(forward + Strafe - Turn);
      rightFrontMotor.set(forward + Strafe + Turn);
      rightBackMotor.set(forward - Strafe + Turn);
*/
  
  }

  @Override
  public void onRobotPeriodic() {}

  @Override
  public void getSensors(Sensors sensors) {
    this.sensors = sensors;
  }

  @Override
  public void getDrivetrain(Drivetrain drivetrain) {}

  @Override
  public void getGamepad(GenericHID driver, GenericHID aid) {
    this.driver = driver;
    this.aid = aid;
  }

  @Override
  public void getTurret(Turret turret) {}

  @Override
  public void getPneumatics(pneumatics pneumatics) {}
  @Override
  public void getGyro(Gyro paramGyro) {
    // TODO Auto-generated method stub
 
  }
  @Override
  public void getPID(PID parampid) {
    // TODO Auto-generated method stub
    
  }
}
package frc.robot.subsystems.sensors;
// import java.util.zip.GZIPInputStream;
// import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.frc.AHRS.SerialDataType;
// import com.kauailabs.navx.frc.AHRS.BoardYawAxis;
// import com.kauailabs.navx.frc.AHRS.BoardAxis;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.management.PID;
import frc.robot.subsystems.management.Subsystem;
public class Sensors extends Subsystem {
  // public AHRS gyro = new AHRS(Port.kUSB);
  private DigitalInput touchSensorLeft = new DigitalInput(5);
  private DigitalInput touchSensorRight = new DigitalInput(4);
  public DigitalInput touchSensorArm = new DigitalInput(3);
  public DigitalInput DistanceSensor = new DigitalInput(0);

  private NetworkTable table;

  // public float getYaw() {
  //   return this.gyro.getYaw();
  // }
  // public double getGyroangle() {
  //   return this.gyro.getAngle();
  // }
  // public float getRoll() {
  //   return this.gyro.getRoll();
  // }
  // public float getPitch() {
  //   return this.gyro.getPitch();
  // }
  public Sensors setLimelight() {
    this.table = NetworkTableInstance.getDefault().getTable("limelight-test");
    return this;
  }

  public Sensors setDistanceSensor() {
    this.table = NetworkTableInstance.getDefault().getTable("distance-test");
    return this;
  }

  public NetworkTable getLimelightArea() {
    return this.table;
  }

  public Sensors setUltrasonic(int port, int max, int zeroV) {
    return this;
  }

  public Sensors setTouch(int port) {
    return this;
  }

  public boolean getDistanceSensor(int index) {
    return this.DistanceSensor.get();
  }
  public boolean getTouchSensorLeft(int index) {
    return this.touchSensorLeft.get();
  }

  public boolean getTouchSensorRight(int index) {
    return this.touchSensorRight.get();
  }

  public boolean getTouchSensorArmData(int index) {
    return this.touchSensorArm.get();
  }

  @Override
  public void onRobotInit() {}

  @Override
  public void onRobotPeriodic() {}

  @Override
  public void onTeleopInit() {}

  @Override
  public void onTeleopPeriodic() {}

  @Override
  public void getSensors(Sensors sensors) {
 
  }

  @Override
  public void getDrivetrain(Drivetrain drivetrain) {}

  @Override
  public void getGamepad(GenericHID driver, GenericHID aid) {}

  @Override
  public void getTurret(Turret turret) {}

  @Override
  public void getPneumatics(pneumatics pneumatics) {}
  // @Override
  // public void getGyro(Gyro paramGyro) {
  //   // TODO Auto-generated method stub

  @Override
  public void getGyro(Gyro paramGyro) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void getPID(PID parampid) {
    // TODO Auto-generated method stub
    
  }
 
  // }
}

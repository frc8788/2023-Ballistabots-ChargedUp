package frc.robot.subsystems.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.management.Subsystem;

public class Sensors extends Subsystem {

  private DigitalInput touchSensorLeft = new DigitalInput(5);
  private DigitalInput touchSensorRight = new DigitalInput(4);
  public DigitalInput touchSensorArm = new DigitalInput(3);
  
  private NetworkTable table;
  
  public Sensors setLimelight() {
    this.table = NetworkTableInstance.getDefault().getTable("limelight-test");
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


}

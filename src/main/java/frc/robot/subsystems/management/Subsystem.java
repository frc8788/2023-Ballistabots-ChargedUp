package frc.robot.subsystems.management;
import frc.robot.subsystems.sensors.Sensors;
public abstract class Subsystem implements ResourceManagement {
  public abstract void onRobotInit();

  public abstract void onRobotPeriodic();

  public abstract void onTeleopInit();

  public abstract void onTeleopPeriodic();
public void getSensors(Sensors sensors) {
}
}

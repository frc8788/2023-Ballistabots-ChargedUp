package frc.robot.subsystems.management;

public abstract class Subsystem implements ResourceManagement {
  public abstract void onRobotInit();
  
  public abstract void onRobotPeriodic();
  
  public abstract void onTeleopInit();
  
  public abstract void onTeleopPeriodic();
}

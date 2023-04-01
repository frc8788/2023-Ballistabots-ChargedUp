package frc.robot.subsystems;

public interface Subsystem {
  void onRobotInit();
  
  void onRobotPeriodic();
  
  void onTeleopInit();
  
  void onTeleopPeriodic();
}

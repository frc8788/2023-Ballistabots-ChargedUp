package frc.robot.subsystems;

import java.util.ArrayList;

public class SubsystemHandler implements Subsystem {
  private ArrayList<Subsystem> subsystems = new ArrayList<>();
  
  public void add(Subsystem subsystem) {
    this.subsystems.add(subsystem);
  }
  
  public void onRobotInit() {
    for (Subsystem subsystem : this.subsystems)
      subsystem.onRobotInit(); 
  }
  
  public void onRobotPeriodic() {
    for (Subsystem subsystem : this.subsystems)
      subsystem.onRobotPeriodic(); 
  }
  
  public void onTeleopInit() {
    for (Subsystem subsystem : this.subsystems)
      subsystem.onTeleopInit(); 
  }
  
  public void onTeleopPeriodic() {
    for (Subsystem subsystem : this.subsystems)
      subsystem.onTeleopPeriodic(); 
  }
}

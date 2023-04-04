package frc.robot.subsystems.management;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.sensors.Sensors;

public interface ResourceManagement {
  void getSensors(Sensors paramSensors);
  
  void getDrivetrain(Drivetrain paramDrivetrain);
  
  void getGamepad(GenericHID paramGenericHID1, GenericHID paramGenericHID2);
  
  void getTurret(Turret paramTurret);
  
  void getPneumatics(pneumatics paramPneumatics);
  
}

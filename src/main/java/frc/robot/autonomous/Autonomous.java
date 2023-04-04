package frc.robot.autonomous;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.management.ResourceManagement;
import frc.robot.subsystems.sensors.Sensors;
import edu.wpi.first.wpilibj.Timer;

public class Autonomous implements ResourceManagement {

  public void onAutonomousInit() {}
  


  public Autonomous test(Drivetrain drivetrain, Turret turret, pneumatics pneumatics, Sensors sensors, double range, int index1, int index2, boolean found) {
    
    //resets extendo encoder
    turret.ExtendoEncoder(0);

    //moves extendo out towards high
    while (turret.eEncoderGet() < 325) {
      turret.ExtendoMove(.75); 
    }
    turret.ExtendoMove(0);

    //resets turret encoder
    turret.TurretEncoder(0);

    //moves turret towards high
    while (turret.tEncoderGet() < 20){
      turret.TurretMove(.5); 
    }

    turret.TurretMove(0);

    //moves arm out more
    while (turret.eEncoderGet() < 360) {
        turret.ExtendoMove(.1);
    }
    turret.ExtendoMove(0);
    Timer.delay(.5);

    //opens claw; drops game element
    pneumatics.OpenClaw();
    Timer.delay(.5);

    //moves extendo inwards
    while (turret.eEncoderGet() > 60) {
      turret.ExtendoMove(-.75); 
    }
    turret.ExtendoMove(0);

    //resets drivetrain encoder
    drivetrain.tankDriveEncoders(0);

    //moves drivetrain out of community
    while (drivetrain.rfEncoderGet() < 20) {
      drivetrain.tankDriveCustom(.25, .25); 
    }
    drivetrain.tankDriveCustom(0, 0);
    SmartDashboard.putString("END", "END");

    //END OF AUTO

    return this;
  }

  




  @Override
  public void getSensors(Sensors sensors) {}
  
  @Override
  public void getDrivetrain(Drivetrain drivetrain) {}
  
  @Override
  public void getGamepad(GenericHID driver, GenericHID aid) {}
  
  @Override
  public void getTurret(Turret turret) {}
  

@Override
public void getPneumatics(pneumatics pneumatics) {
    // TODO Auto-generated method stub
    
}


  
}

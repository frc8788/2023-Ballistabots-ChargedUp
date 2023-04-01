package frc.robot.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.SerialPort.Port;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import frc.robot.subsystems.management.PID;
// import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.management.ResourceManagement;
import frc.robot.subsystems.sensors.Gyro;
// import frc.robot.subsystems.sensors.Gyro;
import frc.robot.subsystems.sensors.Sensors;
public class Autonomous implements ResourceManagement {
  public Timer timer;
  public AHRS gyro;

  public static CANSparkMax leftFrontMotor;
  public static CANSparkMax leftBackMotor;
  public static CANSparkMax rightFrontMotor;
  public static CANSparkMax rightBackMotor;
  public void onAutonomousInit() {
    timer.reset();
    timer.start();
 
  }
  public boolean getGyroangle() {
    gyro.getAngle();
    return this != null;
  }
  public boolean zeroYaw() {
    gyro.zeroYaw();
    return this != null;
  }
  public void tankDriveCustom(double leftSidePower, double rightSidePower) {
    leftBackMotor.setInverted(true);
    leftFrontMotor.setInverted(true);
    leftFrontMotor.set(leftSidePower);
    leftBackMotor.set(leftSidePower);
    rightFrontMotor.set(rightSidePower);
    rightBackMotor.set(rightSidePower);
  }
   public final void IdealPitch() {
    if (zeroYaw()) {
      tankDriveCustom(0, 0);
    } else {
      tankDriveCustom(.25, .25); 
    }
}
public Autonomous bumpSideLocation(Drivetrain drivetrain, Turret turret, pneumatics pneumatics, Sensors sensors, PIDController gyro_pid, boolean found) {
  // Deploy Gamepiece
  turret.TurretEncoder(-350);
  turret.TurretMove(0.75);
  Timer.delay(1);
  turret.ExtendoEncoder(-360);
  turret.ExtendoMove(.5);
  Timer.delay(3);
  pneumatics.OpenClaw();
  Timer.delay(0.25);

  // Exit Community
  drivetrain.tankDriveEncoders(-300);
  drivetrain.tankDriveCustom(0.25, 0.25);
  return this;
}
public Autonomous nonBumpSideLocation(Drivetrain drivetrain, Turret turret, pneumatics pneumatics, Sensors sensors, PIDController gyro_pid, boolean found) {
  // Deploy Gamepiece
  turret.TurretEncoder(-350);
  turret.TurretMove(0.75);
  Timer.delay(1);
  turret.ExtendoEncoder(-360);
  turret.ExtendoMove(.5);
  Timer.delay(3);
  pneumatics.OpenClaw();
  Timer.delay(0.25);

  // Exit Community
  drivetrain.tankDriveEncoders(-300);
  drivetrain.tankDriveCustom(0.25, 0.25);
  return this;
}
public Autonomous chargerSideLocation(Drivetrain drivetrain, Turret turret, pneumatics pneumatics, Sensors sensors, PIDController gyro_pid, boolean found) {
  // Deploy Gamepiece
  turret.TurretEncoder(-350);
  turret.TurretMove(0.75);
  Timer.delay(1);
  turret.ExtendoEncoder(-360);
  turret.ExtendoMove(.5);
  Timer.delay(3);
  pneumatics.OpenClaw();
  Timer.delay(0.25);

  // Drive Over Charge Station
  drivetrain.tankDriveEncoders(-100);
  drivetrain.tankDriveCustom(0.5, 0.5);
  drivetrain.tankDriveEncoders(-200);
  drivetrain.tankDriveCustom(0.25, 0.25);

  // Dock With Charging Station
  drivetrain.tankDriveEncoders(100);
  drivetrain.tankDriveCustom(0.6, 0.6);

  // Attempt Balance
  gyro_pid.setTolerance(2, 10);
  while (Timer.getMatchTime() > 1) {
    drivetrain.setDrivePower(MathUtil.clamp(gyro_pid.calculate(gyro.getPitch(), 0), -0.6, 0.6));
  }
  drivetrain.setDrivePower(0);
  return this;
}
public Autonomous theClawwa(Drivetrain drivetrain, Turret turret, pneumatics pneumatics, Sensors sensors, PIDController gyro_pid, boolean found) {
  turret.ExtendoEncoder(-360);
  turret.ExtendoMove(0.1);
  Timer.delay(2);
  turret.ExtendoEncoder(360);
  turret.ExtendoMove(0.1);
  return this;
}
public Autonomous levelOutRobot(Drivetrain drivetrain, Turret turret, pneumatics pneumatics, Sensors sensors, PIDController gyro_pid, boolean found) {
  // Attempt Balance
  gyro_pid.setTolerance(2, 10);
  while (Timer.getMatchTime() > 1) {
    drivetrain.setDrivePower(MathUtil.clamp(gyro_pid.calculate(gyro.getPitch(), 0), -0.6, 0.6));
  }
  drivetrain.setDrivePower(0);
  return this;
}
  public Autonomous test(Drivetrain drivetrain, Turret turret, pneumatics pneumatics, Sensors sensors, double range, int index1, int index2, boolean found) {
 
    //resets extendo encoder
  /*
    turret.ExtendoEncoder(0);
    //moves extendo out towards high
    while (turret.eEncoderGet() > -300) {
      turret.ExtendoMove(-.75); 
    }
    turret.ExtendoMove(0);
    //resets turret encoder
    turret.TurretEncoder(0);
    //moves turret towards high
    while (turret.tEncoderGet() < 20){
      turret.TurretMove(.5); 
    }
*/
//resets extendo encoder
turret.ExtendoEncoder.setPosition(0.0);
gyro.reset();
timer.reset();
//moves extendo out towards high
drivetrain.tankDriveCustom(.5, .5);
Timer.delay(2.5);
IdealPitch();
if (timer.hasElapsed(0.0)) {
  turret.TurretEncoder(-350);
  Timer.delay(1);

  // turret.ExtendoMove(-.75);

  Timer.delay(1);
  //opens claw; drops game element
  pneumatics.OpenClaw();
  Timer.delay(1);
  // turret.ExtendoMove(0);
  //resets turret encoder
  turret.TurretEncoder.setPosition(0.0);
}
// if (timer.hasElapsed(10)){
//     //moves extendo inwards
//     turret.ExtendoEncoder(-60);
//     turret.ExtendoMove(.75);
//     Timer.delay(0.4);
//     turret.ExtendoMove(0);
// }
if (timer.hasElapsed(12)) {
//resets drivetrain encoder
drivetrain.tankDriveEncoders(0);
//moves drivetrain out of community
while (drivetrain.rfEncoderGet() < 25) {
  drivetrain.tankDriveCustom(.25, .25); 
}
}
  drivetrain.tankDriveCustom(0, 0);
  SmartDashboard.putString("END", "END");
  //END OF AUTO
  return this;
}
  
// //resets extendo encoder
// turret.ExtendoEncoder.setPosition(0.0);
// timer.reset();
// // //moves extendo out towards high
// // if (timer.hasElapsed(0.5)) {
// //     turret.TurretEncoder(-350);
//     Timer.delay(1);
 
//     // turret.ExtendoMove(-.75);
 
//     Timer.delay(1);
//     //opens claw; drops game element
//     pneumatics.OpenClaw();
//     Timer.delay(1);
//     // turret.ExtendoMove(0);
//     //resets turret encoder
//     turret.TurretEncoder.setPosition(0.0);
// }
// // if (timer.hasElapsed(10)){
// //     //moves extendo inwards
// //     turret.ExtendoEncoder(-60);
// //     turret.ExtendoMove(.75);
// //     Timer.delay(0.4);
// //     turret.ExtendoMove(0);
// // }
// if (timer.hasElapsed(12)) {
//   //resets drivetrain encoder
//   drivetrain.tankDriveEncoders(0);
//   //moves drivetrain out of community
//   while (drivetrain.rfEncoderGet() < 25) {
//     drivetrain.tankDriveCustom(.25, .25); 
//   }
// }
//     drivetrain.tankDriveCustom(0, 0);
//     SmartDashboard.putString("END", "END");
//     //END OF AUTO
//     return this;
// }
// if (timer.hasElapsed())
// //moves turret towards high
// while (turret.tEncoderGet() < 20){
//   turret.TurretMove(.5); 
// }
// turret.TurretMove(0);
//     //moves arm out more
//     while (turret.eEncoderGet() > -350) {
//         turret.ExtendoMove(-.1);
//     }
//     turret.ExtendoMove(0);
//     //opens claw; drops game element
//     pneumatics.OpenClaw();
//     //moves extendo inwards
//     while (turret.eEncoderGet() < -60) {
//       turret.ExtendoMove(.75); 
//     }
//     turret.ExtendoMove(0);*/
//     //resets drivetrain encoder
//     drivetrain.tankDriveEncoders(0);
//     //moves drivetrain out of community
//     while (drivetrain.rfEncoderGet() < 25) {
//       drivetrain.tankDriveCustom(.25, .25); 
//     }
//     drivetrain.tankDriveCustom(0, 0);
//     SmartDashboard.putString("END", "END");
//     //END OF AUTO
//     return this;
  //}
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
// public void getGyro(Gyro gyro2) {
// }
@Override
public void getGyro(Gyro paramGyro) {
  // TODO Auto-generated method stub
  
}
@Override
public void getPID(PID parampid) {
  // TODO Auto-generated method stub
  
}
}

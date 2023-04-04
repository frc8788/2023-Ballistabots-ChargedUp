package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.management.Subsystem;
import frc.robot.subsystems.sensors.Sensors;

public class Turret extends Subsystem {

  public GenericHID driver;
  public GenericHID aid;
  
  private Sensors sensors;
  
  public TalonFX lift;
  public CANSparkMax Turret1;
  public CANSparkMax Extendo;
  

  public void ExtendoEncoder(double Position) {
    RelativeEncoder ExEncoder = Extendo.getEncoder();
    ExEncoder.setPosition(Position);
  }
  
  public void TurretEncoder(double Position) {
    RelativeEncoder Turret1Encoder = Turret1.getEncoder();
    Turret1Encoder.setPosition(Position);
  }
  
  public double eEncoderGet() {
    RelativeEncoder ExEncoder = Extendo.getEncoder();
    return ExEncoder.getPosition();
  }
  
  public double tEncoderGet() {
    RelativeEncoder Turret1Encoder = Turret1.getEncoder();
    return Turret1Encoder.getPosition();
  }
  
  public void liftMove(double power) {
    lift.set(ControlMode.PercentOutput, power);
  }
  
  public void ExtendoMove(double power) {
    Extendo.set(power);
  }
  
  public void TurretMove(double power) {
    Turret1.set(power);
  }

  public void TurretWait(long time) throws InterruptedException{
    Turret1.wait(time);
  }
  
  @Override
  public void onRobotInit() {
    lift = new TalonFX(5);
    Turret1 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    Extendo = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    Extendo.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }
  
  @Override
  public void onRobotPeriodic() {}
  
  @Override
  public void onTeleopInit() {}
  
  @Override
  public void onTeleopPeriodic() {
    //Extendo.setInverted(true);
    SmartDashboard.putBoolean("Left Turret Touch:", sensors.getTouchSensorLeft(5));
    SmartDashboard.putBoolean("Right Turret Touch:", sensors.getTouchSensorRight(4));
    SmartDashboard.putBoolean("Arm Touch:", sensors.getTouchSensorArmData(3));

    SmartDashboard.putNumber("Ye", eEncoderGet());
    SmartDashboard.putNumber("YE2", tEncoderGet());

    //lift code
    lift.set(ControlMode.PercentOutput, aid.getRawAxis(1));

    //turret code
    Turret1.setOpenLoopRampRate(0.5);
    Turret1.set(0.4 * (aid.getRawAxis(3) - aid.getRawAxis(2)));

    if (sensors.getTouchSensorLeft(5))
      Turret1.set(.25 * aid.getRawAxis(3)); 

    if (sensors.getTouchSensorRight(4))
      Turret1.set(.25 * -aid.getRawAxis(2)); 


      //Extendo code
    Extendo.setOpenLoopRampRate(1);
      Extendo.set(-.7 * aid.getRawAxis(5)); 

    if (sensors.getTouchSensorArmData(3)) {
      Extendo.set(0);
      if (aid.getRawButton(10))
        Extendo.set(.5 * aid.getRawAxis(5));
    }
  }
  
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
}

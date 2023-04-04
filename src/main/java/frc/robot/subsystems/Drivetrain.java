package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.management.Subsystem;
import frc.robot.subsystems.sensors.Sensors;

public class Drivetrain extends Subsystem {

  public GenericHID driver;
  public GenericHID aid;
  
  public CANSparkMax leftFrontMotor;
  public CANSparkMax leftBackMotor;
  public CANSparkMax rightFrontMotor;
  public CANSparkMax rightBackMotor;
  private Sensors sensors;
  
  //math behnd arcade drive
  private void mecanumDriveCustom(double y, /*double x, */double rx, double k) {

    //y = forward/backwards movement, left stick y axis
    //x = turns, left stick x axis
    //rx = strafe, right stick x axis
    //k = speed variable

    double denominator = Math.max(Math.abs(y) /*+ Math.abs(x)*/ + Math.abs(rx), 1.0D);
    double frontLeftPower = (y/*  + (1.5 * x)*/ + rx) / denominator;
    double backLeftPower = (y/*  - (1.5 * x)*/ + rx) / denominator;
    double frontRightPower = (y/*  - (1.5 * x)*/ - rx) / denominator;
    double backRightPower = (y/*  + (1.5 * x)*/ - rx) / denominator;

    leftFrontMotor.set(frontLeftPower * k);
    leftBackMotor.set(backLeftPower * k);
    rightFrontMotor.set(frontRightPower * k);
    rightBackMotor.set(backRightPower * k);
    
    k = 0.5;
  }
  
  //math for tank drive
  private void TankDriveTeleOp(double ly, double ry, double lt, double rt, double k) {

    //ly = left motors forwards/backward, left stick y axis
    //ry = right motors forwards/backwards, right stick y axis
    //lt = left strafe, left trigger
    //rt = right strafe, right trigger
    //k = speed variable

    double denominator = Math.max(Math.abs(ly) + Math.abs(ry) + Math.abs(lt) + Math.abs(rt), 1.0D);
    double frontLeftPower = (ly + lt - rt) / denominator;
    double backLeftPower = (ly - lt + rt) / denominator;
    double frontRightPower = (ry - lt + rt) / denominator;
    double backRightPower = (ry + lt - rt) / denominator;

    leftFrontMotor.set(frontLeftPower * k);
    leftBackMotor.set(backLeftPower * k);
    rightFrontMotor.set(frontRightPower * k);
    rightBackMotor.set(backRightPower * k);

    k = 0.5;
  }
  
  //math behind flight stick drivetrain
  private void FlightStickDriveCustom(double y, double z, /*double x,*/ double k) {

    //y = forward/backwards movement, y axis
    //z = turns, z rotate axis
    //x = strafe, x axis
    //k = speed variable

    double denominator = -Math.max(Math.abs(y) + Math.abs(z)/* + Math.abs(x)*/, 1.0D);
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    double powerPercentage = 1;

    if (driver.getRawButton(1)) {
      frontLeftPower = (y - (powerPercentage * Math.pow(z, 3))/* + (1.25 * x)*/) / denominator;
      backLeftPower = (y - (powerPercentage * Math.pow(z, 3))/*  - (1.25 * x)*/) / denominator;
      frontRightPower = (y + (powerPercentage * Math.pow(z, 3)) /* - (1.25 * x)*/) / denominator;
      backRightPower = (y + (powerPercentage * Math.pow(z, 3))/* + (1.25 * x)*/) / denominator;
    } else {
      frontLeftPower = (y + (powerPercentage * Math.pow(z, 3))/* + (1
      .25 * x)*/) / -denominator;
      backLeftPower = (y + (powerPercentage * Math.pow(z, 3))/*  - (1.25 * x)*/) / -denominator;
      frontRightPower = (y - (powerPercentage * Math.pow(z, 3)) /* - (1.25 * x)*/) / -denominator;
      backRightPower = (y - (powerPercentage * Math.pow(z, 3))/* + (1.25 * x)*/) / -denominator;
    }
    leftFrontMotor.set(frontLeftPower * k);
    leftBackMotor.set(backLeftPower * k);
    rightFrontMotor.set(frontRightPower * k);
    rightBackMotor.set(backRightPower * k);
  }
  
  public void tankDriveCustom(double leftSidePower, double rightSidePower) {
    leftBackMotor.setInverted(true);
    leftFrontMotor.setInverted(true);

    leftFrontMotor.set(leftSidePower);
    leftBackMotor.set(leftSidePower);
    rightFrontMotor.set(rightSidePower);
    rightBackMotor.set(rightSidePower);
  }
  
  public void strafeLeft(double power) {
    leftFrontMotor.set(-power);
    leftBackMotor.set(power);
    rightFrontMotor.set(-power);
    rightBackMotor.set(power);
  }
  
  public void strafeRight(double power) {
    leftFrontMotor.set(power);
    leftBackMotor.set(-power);
    rightFrontMotor.set(power);
    rightBackMotor.set(-power);
  }
  
  public void tankDriveEncoders(double Distance) {
    RelativeEncoder LeftBackEncoder = leftBackMotor.getEncoder();
    RelativeEncoder LeftFrontEncoder = leftFrontMotor.getEncoder();
    RelativeEncoder RightBackEncoder = rightBackMotor.getEncoder();
    RelativeEncoder RightFrontEncoder = rightFrontMotor.getEncoder();

    LeftBackEncoder.setPosition(Distance);
    LeftFrontEncoder.setPosition(Distance);
    RightBackEncoder.setPosition(Distance);
    RightFrontEncoder.setPosition(Distance);
  }
  
  public double lbEncoderGet() {
    RelativeEncoder LeftBackEncoder = leftBackMotor.getEncoder();
    return LeftBackEncoder.getPosition();
  }
  
  public double lfEncoderGet() {
    RelativeEncoder LeftFrontEncoder = leftFrontMotor.getEncoder();
    return LeftFrontEncoder.getPosition();
  }
  
  public double rbEncoderget() {
    RelativeEncoder RightBackEncoder = rightBackMotor.getEncoder();
    return RightBackEncoder.getPosition();
  }
  
  public double rfEncoderGet() {
    RelativeEncoder RightFrontEncoder = rightFrontMotor.getEncoder();
    return RightFrontEncoder.getPosition();
  }
  
  @Override
  public void onRobotInit() {
    leftBackMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFrontMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  }
  
  @Override
  public void onTeleopInit() {
    //SmartDashboard.putString("Drivetrain", "RAN");
  }
  
  @Override
  public void onTeleopPeriodic() {

    leftBackMotor.setOpenLoopRampRate(1);
    leftFrontMotor.setOpenLoopRampRate(1);
    rightBackMotor.setOpenLoopRampRate(1);
    rightFrontMotor.setOpenLoopRampRate(1);

    leftBackMotor.setInverted(true);
    leftFrontMotor.setInverted(true);
    rightBackMotor.setInverted(false);
    rightFrontMotor.setInverted(false);

    //mecanum arcade drive   
    //each line corresponds to a variable, e.g. y
     /*mecanumDriveCustom(
        (Math.abs(driver.getRawAxis(1)) < .1) ? 0 : driver.getRawAxis(1),
        //(Math.abs(driver.getRawAxis(0)) < .1) ? 0 : -driver.getRawAxis(0),
        (Math.abs(driver.getRawAxis(4)) < .1) ? 0 : -driver.getRawAxis(4),
        (driver.getRawAxis(3) > 0 ? .3 : 1));*/
    
    //mecanum tank drive
    //each line corresponds to a variable, e.g. rt
     /*TankDriveTeleOp(
        (Math.abs(driver.getRawAxis(1)) < .1) ? 0 : driver.getRawAxis(1),
        (Math.abs(driver.getRawAxis(5)) < .1) ? 0 : driver.getRawAxis(5),
        (Math.abs(driver.getRawAxis(2)) < .1) ? 0 : driver.getRawAxis(2),
        (Math.abs(driver.getRawAxis(3)) < .1) ? 0 : driver.getRawAxis(3),
        (driver.getRawAxis(4) > 0 ? .3 : 1));*/

      //flight stick drive code
      //each line corresponds to a variable, e.g. z

    FlightStickDriveCustom(
        (Math.abs(driver.getRawAxis(1)) < .15) ? 0 : driver.getRawAxis(1), 
        (Math.abs(driver.getRawAxis(2)) < .15) ? 0 : -driver.getRawAxis(2), 
        //(Math.abs(driver.getRawAxis(0)) < .1) ? 0 : -driver.getRawAxis(0), 
        (driver.getRawAxis(3) > 0) ? .3 : 1);

        // if (driver.getRawButton(1)) {
        //   leftBackMotor.setInverted(false);
        //  leftFrontMotor.setInverted(false);
        //  rightBackMotor.setInverted(true);
        //  rightFrontMotor.setInverted(true);
        // }
  
    /*while (driver.getRawButton(1)){
      leftBackMotor.setInverted(false);
      leftFrontMotor.setInverted(false);
      rightBackMotor.setInverted(true);
      rightFrontMotor.setInverted(true);
    }*/
      //Field centric drive (to flightstick)
      /*double Forward = driver.getRawAxis(1);
      double strafe = driver.getRawAxis(0);
      double Turn = driver.getRawAxis(2);
  
      double pi = 3.1415926;
  
      double gyroDegrees = Axes.getYaw();
      double gyroRadians = gyroDegrees * pi/180;
  
      double temp = Forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
      double Strafe = -Forward * Math.sin(gyroRadians) + strafe * Math.cos(gyroRadians);
      double forward = temp;


      leftFrontMotor.set(forward - Strafe - Turn);
      leftBackMotor.set(forward + Strafe - Turn);
      rightFrontMotor.set(forward + Strafe + Turn);
      rightBackMotor.set(forward - Strafe + Turn);
*/
     
  }
  
  @Override
  public void onRobotPeriodic() {}
  
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

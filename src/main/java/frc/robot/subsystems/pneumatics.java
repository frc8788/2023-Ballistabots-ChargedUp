package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.management.PID;
import frc.robot.subsystems.management.Subsystem;
import frc.robot.subsystems.sensors.Gyro;
// import frc.robot.subsystems.sensors.Gyro;
import frc.robot.subsystems.sensors.Sensors;
public class pneumatics extends Subsystem {
    public GenericHID driver;
    public GenericHID aid;
    private Sensors sensors; 
 
    public DoubleSolenoid clawSolenoid = new DoubleSolenoid(12, PneumaticsModuleType.CTREPCM, 0, 1);
    public Compressor compressor = new Compressor(12, PneumaticsModuleType.CTREPCM);
    public void OpenClaw(){
        clawSolenoid.set(Value.kReverse);
    }
    public void CloseClaw(){
        clawSolenoid.set(Value.kForward);
    }
@Override
 public void onRobotInit() {
 }   
@Override 
public void onRobotPeriodic() {
 }
@Override 
public void onTeleopInit(){
}
@Override 
public void onTeleopPeriodic(){
if (aid.getRawButton(5)){
    clawSolenoid.set(Value.kForward);
}
if (aid.getRawButton(6)){
    clawSolenoid.set(Value.kReverse);
}
}
@Override
public void getSensors(Sensors sensors) {
    this.sensors = sensors;
}
@Override 
public void getDrivetrain(Drivetrain drivetrain){
} 
@Override 
public void getGamepad(GenericHID driver, GenericHID aid){
    this.driver = driver;
    this.aid = aid;
}
@Override
public void getTurret(Turret turret) {
    // TODO Auto-generated method stub
 
}
@Override
public void getPneumatics(pneumatics pneumatics) {
    // TODO Auto-generated method stub
 
}
// @Override
// public void getGyro(Gyro paramGyro) {
//     // TODO Auto-generated method stub
@Override
public void getGyro(Gyro paramGyro) {
    // TODO Auto-generated method stub
    
}
@Override
public void getPID(PID parampid) {
    // TODO Auto-generated method stub
    
}
 
// }
}
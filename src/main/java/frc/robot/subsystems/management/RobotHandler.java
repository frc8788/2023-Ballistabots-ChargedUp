package frc.robot.subsystems.management;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.sensors.Gyro;
import frc.robot.subsystems.sensors.Sensors;
public class RobotHandler extends Subsystem {
    private ArrayList<Subsystem> subsystems = new ArrayList<Subsystem>();
    public RobotHandler add(Subsystem subsystem) {
        subsystems.add(subsystem);
        return this;
    }
    public RobotHandler addLimelight(Sensors sensors) {
        sensors.setLimelight();
        return this;
    }
    public RobotHandler addUltrasonicSensor(Sensors sensors, int port, int max, int zeroV) {
        sensors.setUltrasonic(port, max, zeroV);
        return this;
    }
    public RobotHandler addTouchSensor(Sensors sensors, int port) {
        sensors.setTouch(port);
        return this;
    }
    // public RobotHandler add(Gyro gyro) {
    //     subsystems.add(gyro);
    //     return this;
    // }
    public RobotHandler allocateSensors(Sensors sensors, Autonomous autonomous) {
        autonomous.getSensors(sensors);
        return this;
    }
    public RobotHandler allocateSensors(Sensors sensors, Subsystem subsystem) {
        subsystem.getSensors(sensors);
        return this;
    }
    public RobotHandler allocateSensors(Sensors sensors, Turret turret) {
        turret.getSensors(sensors);
        return this;
    }
    public RobotHandler allocateSensors(Gyro gyro, Autonomous autonomous) {
        autonomous.getGyro(gyro);
        return this;
    }
    public RobotHandler allocateSensors(Gyro gyro, Subsystem subsystem) {
        subsystem.getGyro(gyro);
        return this;
    }
    public RobotHandler allocateSensors(Gyro gyro, Turret turret) {
    turret.getGyro(gyro);
        return this;
    }
    public RobotHandler allocateSensors(Gyro gyro, Drivetrain drivetrain) {
        drivetrain.getGyro(gyro);
        return this;
    }
    public RobotHandler allocatePID(PID pid, Drivetrain drivetrain) {
        drivetrain.getPID(pid);
        return this;
    }
    public RobotHandler allocatePID(PID pid, Subsystem subsystem) {
        subsystem.getPID(pid);
        return this;
    }
    public RobotHandler allocatePID(PID pid, Turret turret) {
        turret.getPID(pid);
        return this;
    }
    public RobotHandler allocatePID(PID pid, Sensors sensors) {
        sensors.getPID(pid);
        return this;
    }
    public RobotHandler allocatePID(PID pid, Autonomous autonomous) {
        autonomous.getPID(pid);
        return this;
    }
    public RobotHandler allocateDrivetrain(Drivetrain drivetrain, Autonomous autonomous) {
        autonomous.getDrivetrain(drivetrain);
        return this;
    }
    public RobotHandler allocateDrivetrain(Drivetrain drivetrain, PID pid) {
        pid.getDrivetrain(drivetrain);
        return this;
    }
    public RobotHandler allocateDrivetrain(Drivetrain drivetrain, Gyro gyro) {
        gyro.getDrivetrain(drivetrain);
        return this;
    }
    public RobotHandler allocateDrivetrain(Drivetrain drivetrain, Subsystem subsystem) {
        subsystem.getDrivetrain(drivetrain);
        return this;
    }
    public RobotHandler allocateDrivetrain(Drivetrain drivetrain, Turret turret) {
        turret.getDrivetrain(drivetrain);
        return this;
    }
    public RobotHandler allocateDrivetrain(Drivetrain drivetrain, pneumatics pneumatics) {
        pneumatics.getDrivetrain(drivetrain);
        return this;
    }
    public RobotHandler allocateGamepads(GenericHID driver, GenericHID aid, Subsystem subsystem) {
        subsystem.getGamepad(driver, aid);
        return this;
    }
    public RobotHandler allocateTurret(Turret turret, Sensors sensors){
        sensors.getTurret(turret);
        return this;
    }
    public RobotHandler allocateTurret(Turret turret, PID pid){
        pid.getTurret(turret);
        return this;
    }
    public RobotHandler allocateTurret(Turret turret, Gyro gyro){
        gyro.getTurret(turret);
        return this;
    }
    public RobotHandler allocateTurret(Turret turret, Subsystem subsystem){
        subsystem.getTurret(turret);
        return this;
    }
    public RobotHandler allocateTurret(Turret turret, Autonomous autonomous){
        autonomous.getTurret(turret);
        return this;
    }
    public RobotHandler allocateTurret(Turret turret, Drivetrain drivetrain){
        drivetrain.getTurret(turret);
        return this;
    }
    public RobotHandler allocateTurret(Turret turret, pneumatics pneumatics){
        pneumatics.getTurret(turret);
        return this;
    }
    public RobotHandler allocatepneumatics(pneumatics pneumatics, Sensors sensors){
        sensors.getPneumatics(pneumatics);
        return this;
    }
    public RobotHandler allocatepneumatics(pneumatics pneumatics, PID pid){
        pid.getPneumatics(pneumatics);
        return this;
    }
    public RobotHandler allocatepneumatics(pneumatics pneumatics, Gyro gyro){
        gyro.getPneumatics(pneumatics);
        return this;
    }
    public RobotHandler allocatepneumatics(pneumatics pneumatics, Subsystem subsystem){
        subsystem.getPneumatics(pneumatics);
        return this;
    }
    public RobotHandler allocatepneumatics(pneumatics pneumatics, Autonomous autonomous){
        autonomous.getPneumatics(pneumatics);
        return this;
    }
    public RobotHandler allocatepneumatics(pneumatics pneumatics, Drivetrain drivetrain){
        drivetrain.getPneumatics(pneumatics);
        return this;
    }
 
    @Override
    public void onRobotInit() {
        SmartDashboard.putNumber("number of subsystems", subsystems.size());
        for (Subsystem subsystem : subsystems) {
            subsystem.onRobotInit();
        }
    }
    @Override
    public void onRobotPeriodic() {
        for (Subsystem subsystem : subsystems) {
            subsystem.onRobotPeriodic();
        }
    }
    @Override
    public void onTeleopInit() {
        for (Subsystem subsystem : subsystems) {
            subsystem.onTeleopInit();
        }
    }
    @Override
    public void onTeleopPeriodic() {
        for (Subsystem subsystem : subsystems) {
            subsystem.onTeleopPeriodic();
        }
    }
    @Override
    public void getSensors(Sensors sensors) {
        // TODO Auto-generated method stub
    }
    @Override
    public void getDrivetrain(Drivetrain drivetrain) {
        // TODO Auto-generated method stub
    }
    @Override
    public void getGamepad(GenericHID driver, GenericHID aid) {
        // TODO Auto-generated method stub
    }
    @Override
    public void getTurret(Turret turret) {
        // TODO Auto-generated method stub
     
    }
    @Override
    public void getPneumatics(pneumatics pneumatics) {
        // TODO Auto-generated method stub
     
    }
    @Override
    public void getGyro(Gyro paramGyro) {
        // TODO Auto-generated method stub
     
    }
    // @Override
    // public void getGyro(Gyro paramGyro) {
    //     // TODO Auto-generated method stub
    @Override
    public void getPID(PID parampid) {
        // TODO Auto-generated method stub
        
    }
     
    // }
}
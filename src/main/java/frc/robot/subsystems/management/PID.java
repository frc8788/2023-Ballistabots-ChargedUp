package frc.robot.subsystems.management;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.gamepad.Gamepad;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.sensors.Sensors;

public class PID {
    // Creates a PIDController with gains kP, kI, and kD
    // TODO: Tune PID Vars
    public PIDController leftside_pid = new PIDController(0.001, 0, 0);
    public PIDController rightside_pid = new PIDController(0.001, 0, 0);
    public PIDController gyro_pid = new PIDController(0.001, 0, 0);
    public PIDController lift_pid = new PIDController(0.001, 0, 0);
    private GenericHID driver;
    private GenericHID aid;

    public void getDrivetrain(Drivetrain drivetrain) {

    }
    public void getTurret(Turret turret) {

    }
    public void getPneumatics(pneumatics pneumatics) {

    }
    public void getGamepad(Gamepad gamepad) {
        this.driver = driver;
        this.aid = aid;
    }
    public void getSensors(Sensors sensors) {

    }
}

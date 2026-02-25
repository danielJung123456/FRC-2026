package frc.robot.test;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterTest extends SubsystemBase {
    private static ShooterTest shooterTest = null;
    private shooter ballShooter = shooter.getInstance();
    double TestFlyWheelRPM = SmartDashboard.getNumber("RPM", 0);
    AngularVelocity testFlyWheelVelocity = RotationsPerSecond.of(TestFlyWheelRPM/60);

    public void SetTestVelocity(boolean SetTestVel){
        
        if (SetTestVel) ballShooter.setFlyWheelVel(testFlyWheelVelocity);
    }

    public static ShooterTest getInstance(){
        if (shooterTest == null){
            shooterTest = new ShooterTest();
        }
        return shooterTest;
    }
}

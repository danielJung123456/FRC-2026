package frc.robot.subsystems.automations;

import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.autoConstants;
import frc.robot.subsystems.drive;

public class autoAlign extends SubsystemBase{
    private static autoAlign align = null;
    private drive drivetrain = drive.getInstance();
    Map<Pose2d, Pose2d> pathMap = new HashMap<>();
    Map<Integer, Pose2d> tagMap = new HashMap<>();

    //Commented out because all of the auto align paths haven't been made yet

    private autoAlign(){
  
    }

    public Rotation2d getHubYaw(){
        Pose2d botPos = drivetrain.getRobotPose();
        Pose2d hubPos = getHubPos();

        double targetRadians = Math.atan2(
            hubPos.getY() - botPos.getY(),
            hubPos.getX() - botPos.getX()
        );

        return new Rotation2d(targetRadians);
    }

    public Distance getHubDist(){
        Pose2d botPos = drivetrain.getRobotPose();
        Pose2d hubPos = getHubPos();

        Distance hubDist = Meters.of(
            Math.hypot((hubPos.getX() - botPos.getX()), (hubPos.getY() - botPos.getY()))
        );
        return hubDist;
    }

    private Pose2d getHubPos(){
        Pose2d hubPos = (Constants.currentAlliance.get() == Alliance.Blue) ? autoConstants.HubB : autoConstants.HubR;
        return hubPos;
    }

    //Path finding
    public void followGeneratedPath(Pose2d intermediate, Pose2d target) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivetrain.getRobotPose(), intermediate, target);
        PathConstraints constraints = new PathConstraints(4.0, 3.0, 2 * Math.PI, 2 * Math.PI);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, target.getRotation())
        );
        path.preventFlipping = true;
        AutoBuilder.followPath(path).schedule();  //Follow path
    }

    public void travelToTower(){
        Pose2d towerPos = (Constants.currentAlliance.get() == Alliance.Blue) ? autoConstants.TowerB : autoConstants.TowerR;
        Pose2d towerIPos = (Constants.currentAlliance.get() == Alliance.Blue) ? autoConstants.TowerBI : autoConstants.TowerRI;
        followGeneratedPath(towerIPos, towerPos);
    }

    public static autoAlign getInstance(){
        if (align == null){
            align = new autoAlign();
        }
        return align;
    }
}
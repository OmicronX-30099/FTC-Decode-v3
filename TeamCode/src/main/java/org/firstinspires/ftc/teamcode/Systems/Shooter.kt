package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Turret
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.TurretState
import org.firstinspires.ftc.teamcode.Util.currAlliance
import kotlin.math.atan2

// Shooter Object
object Shooter: SubsystemGroup(Turret, Flywheel) {
    // States to track turret and flywheel
    internal var turretState: TurretState = TurretState.AUTO_AIM
    internal var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    // Function to calculate and set Turret/Flywheel targets if states are at AUTO_AIM
    internal fun update() {
        val currPose: Pose = follower.pose
        // Calculate and set target for Turret
        if (turretState == TurretState.AUTO_AIM) { Turret.targetTurretAngle = calculateTurretAngle(currPose) }
        Turret.update()
        // Calculate and set target for Flywheel, if idle-ing set to 1140 TPS
        if (flywheelState == FlywheelState.AUTO_AIM) { Flywheel.flywheelTarget = calculateFlywheelVel(currPose) }
        else { Flywheel.flywheelTarget = Flywheel.IDLE_VELOCITY }
        Flywheel.update()
    }

    // Function to calculate turret angle(Deg)
    private fun calculateTurretAngle(botPose: Pose): Double = Math.toDegrees(atan2(botPose.y - currAlliance.turretTargetPose().y, botPose.x - currAlliance.turretTargetPose().x) - botPose.heading)

    internal fun moveTurretBy(deg: Double) { Turret.targetTurretAngle += deg }

    // Function to calculate flywheel velocity(TPS)
    private fun calculateFlywheelVel(botPose: Pose): Double {
        val d: Double = botPose.distanceFrom(currAlliance.goalPose)
        return 0.0142645 * d * d + 1.26161 * d + 748.88095
    }
}
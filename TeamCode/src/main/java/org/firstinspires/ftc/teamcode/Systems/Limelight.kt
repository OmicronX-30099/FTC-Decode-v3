package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.Systems.Load.Stupid
import org.firstinspires.ftc.teamcode.Util.Motif
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.toPose
import kotlin.math.cos
import kotlin.math.tan

object Limelight: Subsystem {
    val limelight: Limelight3A by lazy { ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight") }

    const val LIMELIGHT_HEIGHT: Double = 8.5
    const val MOTIF_PIPELINE: Int = 0
    const val BLOBBY_PIPELINE: Int = 3

    fun startMotifDetection() {
        limelight.pipelineSwitch(MOTIF_PIPELINE)
        limelight.start()
    }
    fun startBlobDetection() {
        limelight.pipelineSwitch(BLOBBY_PIPELINE)
        limelight.start()
    }
    fun detectMotif() {
        val currTagID: Int = limelight.latestResult.fiducialResults[0].fiducialId
        Motif.entries.forEach {
            if (it.tagID == currTagID) {
                ROBOT.currStage.currMotif = it
            }
        }
    }
    fun getFollowPath(): Command {
        val result = limelight.latestResult
        if (!result.isValid || result == null) {
            return NullCommand()
        }
        val tx = result.tx
        val ty = result.ty
        val targetPose = follower.pose + Vector((1 / tan(Math.toRadians(-ty))) * LIMELIGHT_HEIGHT, follower.pose.heading - Math.toRadians(tx)).toPose()
        val path = follower.pathBuilder()
            .addPath(BezierLine(follower.pose, targetPose))
            .setConstantHeadingInterpolation(follower.pose.heading - Math.toRadians(tx))
            .build()
        ActiveOpMode.telemetry.addData("Pose", "${targetPose.x}, ${targetPose.y}, ${Math.toDegrees(targetPose.heading)}")
        return FollowPath(path, true, 0.5).setRequirements(Stupid)
    }
}
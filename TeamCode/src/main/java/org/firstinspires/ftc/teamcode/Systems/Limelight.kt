package org.firstinspires.ftc.teamcode.Systems

import com.bylazar.telemetry.PanelsTelemetry
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
    lateinit var limelight: Limelight3A

    const val LIMELIGHT_HEIGHT: Double = 8.5
    const val MOTIF_PIPELINE: Int = 0
    const val BLOBBY_PIPELINE: Int = 3

    fun startMotifDetection() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
        limelight.pipelineSwitch(MOTIF_PIPELINE)
        limelight.start()
    }
    fun startBlobDetection() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
        limelight.pipelineSwitch(BLOBBY_PIPELINE)
        limelight.start()
    }
    fun detectMotif() {
        if (!limelight.latestResult.isValid || limelight.latestResult == null) {
            return
        }
        val currTagID: Int = limelight.latestResult.fiducialResults[0].fiducialId
        Motif.entries.forEach {
            if (it.tagID == currTagID) {
                ROBOT.currStage.currMotif = it
            }
        }
    }
    fun getFollowPath(): Triple<Pose, Double, Pose> {
        val result = limelight.latestResult
        if (result == null || !result.isValid) {
            if (result == null) {
                PanelsTelemetry.telemetry.addData("Is detecting?", "Null")
            } else {
                PanelsTelemetry.telemetry.addData("Is detecting?", "Invalid")
            }
            return Triple(follower.pose, follower.pose.heading, follower.pose)
        }
        val tx = result.tx
        val ty = result.ty
        val d = -8.50 / tan(ty)
        val a = Vector(d, follower.pose.heading - Math.toRadians(tx)).toPose()
        val targetPose = follower.pose + a
        PanelsTelemetry.telemetry.addData("distance", a)
        return Triple(targetPose, (follower.pose.heading - Math.toRadians(tx)), follower.pose)
    }
}
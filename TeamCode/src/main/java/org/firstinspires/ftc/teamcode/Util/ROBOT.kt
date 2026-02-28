package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import kotlin.math.cos
import kotlin.math.sin

data object ROBOT {
    private const val TURRET_Y_OFFSET: Double = -1.774

    fun shooterPose(): Pose {
        val a: Double = follower.heading
        return follower.pose + Pose(
            TURRET_Y_OFFSET * cos(a),
            TURRET_Y_OFFSET * sin(a)
        )
    }
}
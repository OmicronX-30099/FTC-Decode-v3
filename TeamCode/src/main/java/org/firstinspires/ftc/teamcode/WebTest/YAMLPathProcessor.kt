package org.firstinspires.ftc.teamcode.WebTest

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Curve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import java.util.ArrayList

object YAMLPathProcessor {
    fun loadFile(fName: String) {

    }
    fun loadFile(fName: String, grouping: String) {

    }

    fun parsePath(l: Map<String, Any>, pBuilder: PathBuilder, lastPose: Pose): Pair<PathBuilder, Pose> {
        val ctrlPts: ArrayList<Map<String, Any>> = l["controlPoints"]!! as ArrayList<Map<String, Any>>
        val endPoint: Map<String, Any> = l["endPoint"]!! as Map<String, Any>
        val pts: ArrayList<Pose> = arrayListOf(lastPose)
        for (p in ctrlPts) {
            pts.add(parsePose(p))
        }

        val currPoint = parsePose(endPoint)
        pts.add(currPoint)

        val path: Curve = if (pts.size == 2) {
            BezierLine(pts[0], pts[1])
        } else {
            BezierCurve(pts)
        }

        pBuilder.addPath(path)

        val headingType: String = endPoint["heading"]!! as String
        when (headingType) {
            "tangential" -> {
                pBuilder.setTangentHeadingInterpolation()
                if (endPoint["reverse"]!! as Boolean) { pBuilder.setReversed() }
            }
            "constant" -> {
                pBuilder.setConstantHeadingInterpolation(
                    Math.toRadians((endPoint["degrees"]!! as Number).toDouble())
                )
            }
            "linear" -> {
                pBuilder.setLinearHeadingInterpolation(
                    Math.toRadians((endPoint["startDeg"]!! as Number).toDouble()),
                    Math.toRadians((endPoint["endDeg"]!! as Number).toDouble())
                )
            }
        }
        return Pair(pBuilder, currPoint)
    }

    fun parsePose(d: Map<String, Any>): Pose =
        Pose((d["x"]!! as Number).toDouble(), (d["y"]!! as Number).toDouble())
}
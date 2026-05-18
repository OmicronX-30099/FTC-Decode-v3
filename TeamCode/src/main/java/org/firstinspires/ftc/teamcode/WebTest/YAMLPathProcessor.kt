package org.firstinspires.ftc.teamcode.WebTest

import android.util.Log
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Curve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder as pb
import com.pedropathing.paths.PathChain
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower as f
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.yaml.snakeyaml.Yaml
import java.io.File
import java.io.FileInputStream
import java.util.ArrayList

object YAMLPathProcessor {
    private val dir = File(AppUtil.FIRST_FOLDER, "yaml_files")
    private val paths: MutableMap<String, PathChain> = mutableMapOf()

    fun loadFile(fName: String, grouping: String) {

    }

    fun loadFile(fName: String) {
        val yamlEngine = Yaml()
        val pathFile: File = File(dir, fName)
        paths.clear()

        try {
            val data: Map<String, Any> =
                FileInputStream(pathFile).use { inputStream -> yamlEngine.load(inputStream) }

            val lines: ArrayList<Map<String, Any>> = data["lines"] as ArrayList<Map<String, Any>>

            var prevPose = parsePose(data)

            for (l in lines) {
                val r = parsePath(l, f.pathBuilder(), prevPose)
                paths.put(
                    l["name"]!! as String,
                    r.first.build()
                )
                prevPose = r.second
            }
        } catch (e: Exception) {
            Log.e("YAML-PP", e.stackTrace.toString())
            error("Something went wrong with the yaml processing")
        }
    }

    fun getPath(pName: String) = paths[pName]

    fun parsePath(l: Map<String, Any>, pBuilder: pb, lastPose: Pose): Pair<pb, Pose> {
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
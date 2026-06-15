import org.yaml.snakeyaml.Yaml
import java.io.File
import java.io.FileInputStream

fun main() {
    AutoManager.loadFile("format.yaml")
}

object AutoManager {
    fun loadFile(fileName: String) {
        val yamlReader = Yaml()
        val pathFile = File("Teamcode/src/main/java/org/firstinspires/ftc/teamcode", fileName)

        val data: Map<String, Any> = FileInputStream(pathFile).use {
            inputStream -> yamlReader.load(inputStream)
        }

        println(data)
    }
}
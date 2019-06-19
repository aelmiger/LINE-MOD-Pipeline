
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ccalib.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/videoio.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>


#include <fstream>

#include "opengl_render.h"
#include "model_buffer.h"
#include "utility.h"
#include "defines.h"

#include "high_level_linemod.h"
#include "high_level_linemod_icp.h"
#include "Template_Generator.h"
#include "Pose_Detection.h"

#ifdef _DEBUG
#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "glew32s.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "opencv_core410d.lib")
#pragma comment(lib, "opencv_imgproc410d.lib")
#pragma comment(lib, "opencv_rgbd410d.lib")
#pragma comment(lib, "opencv_imgcodecs410d.lib")
#pragma comment(lib, "opencv_highgui410d.lib")
#pragma comment(lib, "opencv_videoio410d.lib")
#pragma comment(lib, "opencv_ccalib410d.lib")
#pragma comment(lib, "opencv_calib3d410d.lib")
#pragma comment(lib, "opencv_surface_matching410d.lib")
#pragma comment(lib, "opencv_videoio410d.lib")
#pragma comment(lib, "freenect2.lib")



#else
#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "glew32s.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "opencv_core410.lib")
#pragma comment(lib, "opencv_imgproc410.lib")
#pragma comment(lib, "opencv_rgbd410.lib")
#pragma comment(lib, "opencv_imgcodecs410.lib")
#pragma comment(lib, "opencv_highgui410.lib")
#pragma comment(lib, "opencv_videoio410.lib")
#pragma comment(lib, "opencv_ccalib410.lib")
#pragma comment(lib, "opencv_calib3d410.lib")
#pragma comment(lib, "opencv_surface_matching410.lib")
#pragma comment(lib, "opencv_videoio410.lib")
#pragma comment(lib, "freenect2.lib")


#endif

int main() {

	Template_Generator templateGen = Template_Generator(CameraParameters(), TemplateGenerationSettings());
	templateGen.run();
	templateGen.~Template_Generator();

	Pose_Detection poseDetect = Pose_Detection(CameraParameters(), TemplateGenerationSettings());
	poseDetect.run();


	std::getchar();
}

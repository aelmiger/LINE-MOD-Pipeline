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
#pragma comment(lib, "assimp-vc140-mt.lib")

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
#pragma comment(lib, "assimp-vc140-mt.lib")

#endif

int main() {
	Template_Generator templateGen = Template_Generator(CameraParameters(), TemplateGenerationSettings());
	//templateGen.run();
	templateGen.~Template_Generator();

	Pose_Detection poseDetect = Pose_Detection(CameraParameters(), TemplateGenerationSettings());
	poseDetect.run();

	std::getchar();
}
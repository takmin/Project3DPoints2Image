#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


void printHelp(int argc, char **argv)
{
	printf("Syntax is: %s -pcd=<point cloud> -img=<image> -int=<camera param> -ext=<external param>\n", argv[0]);
	printf("  where options are:\n");
	printf("                   -h                    = Print this help\n");
	printf("                   -pcd file_name        = Input PCD/PLY file name\n");
	printf("                   -img file_name        = Input image file name\n");
	printf("                   -int file_name        = Internal camera parameter file\n");
	printf("                   -ext file_name        = External camera parameter file\n");
	printf("\n");
}


bool parse_parameters(int argc, char** argv, std::string& input_pcd,
	std::string& input_img, std::string& int_file, std::string& ext_file)
{
	if (argc < 4 || pcl::console::find_argument(argc, argv, "-h") > 0) {
		printHelp(argc, argv);
		return false;
	}


	if (pcl::console::parse_argument(argc, argv, "-pcd", input_pcd) < 0) {
		std::cerr << "\"-pcd\" option must be indicated" << std::endl;
		printHelp(argc, argv);
		return false;
	}

	if (!pcl::console::parse_argument(argc, argv, "-img", input_img) < 0) {
		std::cerr << "\"-img\" option must be indicated" << std::endl;
		printHelp(argc, argv);
		return false;
	}

	if (!pcl::console::parse_argument(argc, argv, "-int", int_file) < 0) {
		std::cerr << "\"-int\" option must be indicated" << std::endl;
		printHelp(argc, argv);
		return false;
	}

	if (!pcl::console::parse_argument(argc, argv, "-ext", ext_file) < 0) {
		std::cerr << "\"-ext\" option must be indicated" << std::endl;
		printHelp(argc, argv);
		return false;
	}

	return true;
}


bool readIntCameraParameters(const cv::FileNode& cvfn,
	cv::Mat& camera_matrix, cv::Mat& distortion, cv::Size& image_size, bool& fisheye)
{
	try {
		cvfn["camera_matrix"] >> camera_matrix;
		cvfn["distortion"] >> distortion;
		cvfn["image_size"] >> image_size;
		int f;
		cvfn["fisheye"] >> f;
		fisheye = f > 0;
	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		return false;
	}
	return true;
}


bool loadIntCameraParameters(const std::string& filename,
	cv::Mat& camera_matrix, cv::Mat& distortion, cv::Size& image_size, bool& fisheye)
{
	cv::FileStorage cvfs(filename, cv::FileStorage::READ);
	if (!cvfs.isOpened())
		return false;
	return readIntCameraParameters(cvfs["IntParam"], camera_matrix, distortion, image_size, fisheye);
}


bool readExtCameraParameters(const cv::FileNode& cvfn, cv::Mat& rvec, cv::Mat& tvec)
{
	try {
		cvfn["rvec"] >> rvec;
		cvfn["tvec"] >> tvec;
	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		return false;
	}
	return true;
}


bool loadExtCameraParameters(const std::string& filename, cv::Mat& rvec, cv::Mat& tvec)
{
	cv::FileStorage cvfs(filename, cv::FileStorage::READ);
	if (!cvfs.isOpened())
		return false;
	return readExtCameraParameters(cvfs["ExtParam"], rvec, tvec);
}


cv::Mat PointCloud2CvMat(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	cv::Mat mat_pts(3, cloud.size(), CV_64FC1);
	for (int i = 0; i < cloud.size(); i++) {
		pcl::PointXYZ pt = cloud[i];
		mat_pts.at<double>(0, i) = pt.x;
		mat_pts.at<double>(1, i) = pt.y;
		mat_pts.at<double>(2, i) = pt.z;
	}
	return mat_pts;
}


cv::Mat filterWithZ(const cv::Mat& points3d, double z_threshold)
{
	std::vector<int> ids;
	for (int i = 0; i < points3d.cols; i++) {
		if (points3d.at<double>(2, i) > z_threshold) {
			ids.push_back(i);
		}
	}

	cv::Mat filtered(3, ids.size(), CV_64FC1);
	for (int i = 0; i < ids.size(); i++) {
		for (int j = 0; j < 3; j++) {
			filtered.at<double>(j, i) = points3d.at<double>(j, ids[i]);
		}
	}
	return filtered;
}


int main(int argc, char * argv[])
{
	std::string pcd_file, img_file, int_file, ext_file;
	if (!parse_parameters(argc, argv, pcd_file, img_file, int_file, ext_file))
		return -1;

	// load point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string ext = boost::filesystem::path(pcd_file).extension().string();
	int ret = -1;
	if (ext == ".pcd" || ext == ".PCD") {
		ret = pcl::io::loadPCDFile(pcd_file, *cloud);
	}
	else if (ext == ".ply" || ext == ".PLY") {
		ret = pcl::io::loadPLYFile(pcd_file, *cloud);
	}
	if (ret < 0) {
		std::cerr << "Fail to load " << pcd_file << std::endl;
		return false;
	}

	cv::Mat img = cv::imread(img_file);
	if (img.empty()) {
		std::cerr << "Fail to load image file " << img_file << std::endl;
		return -1;
	}

	cv::Mat camera_matrix;
	cv::Mat distortion;
	cv::Size image_size;
	bool fisheye;
	if (!loadIntCameraParameters(int_file, camera_matrix, distortion, image_size, fisheye)) {
		std::cerr << "Fail to load internal camera parameters " << int_file << std::endl;
	}
	if (fisheye) {
		std::cerr << "This program does not support fisheye camera." << std::endl;
		return -1;
	}

	cv::Mat rvec, tvec;
	if (!loadExtCameraParameters(ext_file, rvec, tvec)) {
		std::cerr << "Fail to load external camera parameters " << int_file << std::endl;
	}

	cv::Mat points3d = PointCloud2CvMat(*cloud);
	cloud->clear();

	cv::Mat R;
	cv::Rodrigues(rvec, R);
	//R = R.t();
	//cv::Mat T = -R * tvec;
	cv::Mat points3d_camera_coord = R * points3d;
	for (int c = 0; c < points3d_camera_coord.cols; c++)
		points3d_camera_coord.col(c) += tvec;

	cv::Mat points3d_filtered = filterWithZ(points3d_camera_coord, 0);

	cv::Mat z_val_f = points3d_filtered.row(2);
	cv::Mat z_val_8u;
	cv::normalize(z_val_f, z_val_8u, 255, 0, cv::NORM_MINMAX, CV_8UC1);

	cv::Mat color_map;
	cv::applyColorMap(z_val_8u, color_map, cv::COLORMAP_AUTUMN);
	//cv::imwrite("z_val_8u.bmp", z_val_8u);
	//cv::imwrite("color_map.bmp", color_map);

	cv::Mat rvec2 = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec2 = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat points_on_image;
	cv::projectPoints(points3d_filtered.t(), rvec2, tvec2, camera_matrix, distortion, points_on_image);


	for (int r = 0; r < points_on_image.rows; r++) {
		double x = points_on_image.at<double>(r, 0);
		if (x < 0 || x >= img.cols)
			continue;
		double y = points_on_image.at<double>(r, 1);
		if (y < 0 || y >= img.rows)
			continue;
		cv::circle(img, cv::Point(x, y), 1, color_map.at<cv::Vec3b>(0, r),-1);
	}

	cv::namedWindow("result");
	cv::imshow("result", img);
	cv::waitKey();
	cv::destroyWindow("result");

	return 0;
}
#include <iostream>
#include <fstream>
#include <vector>
#include "Panorama2333.h"
int main() {
	Panarama2333 panorama;
	std::string data_root = "./data/panorama-data2/";
	
	std::ifstream list_file(data_root + "list.txt");
	std::ifstream focus_file(data_root + "K.txt");
	// focus parameter of the camera
	float focus;
	focus_file >> focus;
	focus_file.close();
	std::vector<cv::Mat> img_vec;
	std::string img_name;
	while (list_file >> img_name)
	{
		std::cout << img_name << std::endl;
		img_vec.push_back(cv::imread(data_root + img_name));
	}
	cv::Mat img_out;

	panorama.makePanorama(img_vec, img_out, focus);
	cv::imshow("output", img_out);
	cv::waitKey();
	list_file.close();
	getchar();
}
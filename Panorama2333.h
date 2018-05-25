#pragma once
#include "hw6_pa.h"
// references: https://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html

using namespace cv::xfeatures2d;
using namespace cv;
using namespace std;
class Panarama2333 : public CylindricalPanorama {

public:
	void cylinderical_projection(cv::Mat img, double f) {
		
		
	}

    Mat stitch(Mat img1, Mat img2) {

		// detect keypoints
		cv::Ptr<SIFT> sift = SIFT::create();
		std::vector<cv::KeyPoint> kpts1, kpts2;
		
		sift->detect(img1, kpts1);
		sift->detect(img2, kpts2);
		// Add results to image and save.
		Mat out1, out2;
		drawKeypoints(img1, kpts1, out1);
		drawKeypoints(img2, kpts2, out2);

		// calculate descriptors (feature vectors)
		cv::Mat des1, des2;

		sift->compute(img1, kpts1, des1);
		sift->compute(img2, kpts2, des2);

		// matching descriptor vectors 

		cv::FlannBasedMatcher matcher;
		std::vector< cv::DMatch > matches;
		matcher.match(des1, des2, matches);
		std::vector<cv::DMatch> good_matches = filter_good_matches(matches);
		cv::Mat matches_img;
		drawMatches(img1, kpts1, img2, kpts2,
			good_matches, matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
			std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		// find homography to stitch the image

		//-- Localize the object
		std::vector<cv::Point2f> pts1;
		std::vector<cv::Point2f> pts2;

		for (int i = 0; i < good_matches.size(); i++)
		{
			//-- Get the keypoints from the good matches
			pts1.push_back(kpts1[good_matches[i].queryIdx].pt);
			pts2.push_back(kpts2[good_matches[i].trainIdx].pt);
		}
		// find a transform H, such that H(pts1) = pts2
		Mat H = cv::findHomography(pts2, pts1, CV_RANSAC);

		// img1: left, img2: right
		
		// now do transform H to img2 to fit with img1
		// put the 
		cv::Mat transformed_img2;
		cv::warpPerspective(img2, transformed_img2, H, Size(img1.cols + img2.cols, img1.rows));

		// copy to the image
		Mat tmp = transformed_img2(cv::Rect(0, 0, img1.cols, img1.rows));
		img1.copyTo(tmp);
		//imshow("transformed_img", transformed_img2);
		return transformed_img2;
	}
	std::vector<cv::DMatch> filter_good_matches(const std::vector<cv::DMatch> & matches) {
		// filter to get good matches

		double max_dist = 0; double min_dist = 100;
		//-- Quick calculation of max and min distances between keypoints
		for (int i = 0; i < matches.size(); i++)
		{
			double dist = matches[i].distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}

		//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
		std::vector<cv::DMatch> good_matches;
		for (int i = 0; i < matches.size(); i++)
		{
			if (matches[i].distance < 3 * min_dist)
			{
				good_matches.push_back(matches[i]);
			}
		}
		return good_matches;
	}
	Mat crop_image(Mat img) {
		Mat bw;
		imshow("img", img);
		waitKey();
		cvtColor(img, bw, COLOR_BGR2GRAY);
		threshold(bw, bw, 1, 255, THRESH_BINARY);

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(bw, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		vector<Point> contour = contours[0];
		Rect rect = boundingRect(contour);
		rect.width = rect.width - 20;
		Mat out = img(rect).clone();

		return out;
	}
	bool makePanorama(std::vector<cv::Mat>& img_vec, cv::Mat& img_out, double f) {
		// img_vec: list of images in left to right order
		// img_out: the final composite image
		// f: focal length
		img_out = img_vec[0];

		for (int i = 1; i < img_vec.size(); i++) {
			std::cout << i << std::endl;
			img_out = stitch(img_out, img_vec[i]);
			// crop image
			img_out = crop_image(img_out);
			imshow("img", img_out);
			waitKey();
			
		}
		return true;
	}
};
#include "CharacterRecognition.h"
#include <omp.h>

using namespace std;
using namespace cv;

void getBlack(cv::Mat src, cv::Mat &dst, cv::Scalar blackUpperValue)
{
	dst.create(src.size(), CV_8UC1);
	dst.setTo(Scalar(0));

	inRange(src, blackUpperValue, Scalar(255, 255, 255), dst);
}

/*
 *	×Ô¶¨ÒåÅÅÐòº¯Êý£¬¸ù¾Ýr.area()ÉýÐò
 */
bool SortByRectAreaUp(const Rect &r1, const Rect &r2)//×¢Òâ£º±¾º¯ÊýµÄ²ÎÊýµÄÀàÐÍÒ»¶¨ÒªÓëvectorÖÐÔªËØµÄÀàÐÍÒ»ÖÂ
{
	return r1.area() < r2.area();//ÉýÐòÅÅÁÐ
}

/*
*	×Ô¶¨ÒåÅÅÐòº¯Êý£¬¸ù¾Ýr.area()½µÐò
*/
bool SortByRectAreaDown(const Rect &r1, const Rect &r2)//×¢Òâ£º±¾º¯ÊýµÄ²ÎÊýµÄÀàÐÍÒ»¶¨ÒªÓëvectorÖÐÔªËØµÄÀàÐÍÒ»ÖÂ
{
	return r1.area() > r2.area();//½µÐòÅÅÁÐ
}

void getCharCandRegions(const cv::Mat black, cv::Mat &charImg, cv::Rect &charRect)
{
	CV_Assert(black.type() == CV_8UC1);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Mat threshold_output;
	black.copyTo(threshold_output);

	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());

	for (size_t i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}

	//std::sort(boundRect.begin(), boundRect.end(), SortByRectAreaUp);
	std::sort(boundRect.begin(), boundRect.end(), [&](Rect r1, Rect r2){return r1.area() < r2.area(); });

	boundRect.pop_back();

	int char_rect_width = (int)(boundRect[boundRect.size() - 1].width * 0.6);
	int char_rect_hight = (int)(boundRect[boundRect.size() - 1].height * 0.6);

	Point rect_center(boundRect[boundRect.size() - 1].tl() + Point(boundRect[boundRect.size() - 1].width / 2, boundRect[boundRect.size() - 1].height / 2));

	Rect char_rect(rect_center - Point(char_rect_width / 2, char_rect_hight / 2), rect_center + Point(char_rect_width / 2, char_rect_hight / 2));

	/*Mat drawing = Mat::zeros(black.size(), CV_8UC3);
	RNG rng(12345);
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));*/

	//rectangle(drawing, rect.tl(), rect.br(), color, 2, 8, 0);

	/*for (int i = 0; i < boundRect.size(); i++)
	{
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
	}
	drawContours(drawing, contours_poly, contours.size() - 1, color, 1, 8, vector<Vec4i>(), 0, Point());

	imshow("Contours", drawing);*/

	charImg.create(char_rect.height, char_rect.width, CV_8UC1);
	charImg = black(char_rect).clone();

	charRect = char_rect;
}

void detectRectangles(cv::Mat &thresImg, std::vector< std::vector< cv::Point > > &outPolyCanditates)
{
	vector< vector< Point > > candidates;
	vector< cv::Mat > thres_v;
	thres_v.push_back(thresImg);
	detectRectangles(thres_v, candidates);
	// create the output
	outPolyCanditates.resize(candidates.size());
	for (size_t i = 0; i < outPolyCanditates.size(); i++)
		outPolyCanditates[i] = candidates[i];

}

void detectRectangles(std::vector< cv::Mat > &thresImgv, std::vector< std::vector< cv::Point > > &outPolyCanditates)
{
	Params _params;

	vector< vector< vector< Point > > > PolyCanditatesV(omp_get_max_threads());

	int maxSize = _params._maxSize * (std::max)(thresImgv[0].cols, thresImgv[0].rows) * 4;
	int minSize = (std::min)(float(_params._minSize_pix), _params._minSize* (std::max)(thresImgv[0].cols, thresImgv[0].rows) * 4);

#pragma omp parallel for
	for (int img_idx = 0; img_idx < thresImgv.size(); img_idx++)
	{
		std::vector< cv::Vec4i > hierarchy2;
		std::vector< std::vector< cv::Point > > contours2;
		cv::Mat thres2;
		thresImgv[img_idx].copyTo(thres2);
		cv::findContours(thres2, contours2, hierarchy2, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		vector< Point > approxCurve;
		/// for each contour, analyze if it is a paralelepiped likely to be the marker
		for (unsigned int i = 0; i < contours2.size(); i++) {

			// check it is a possible element by first checking is has enough points
			if (minSize < contours2[i].size() && contours2[i].size() < maxSize) {
				// approximate to a poligon
				approxPolyDP(contours2[i], approxCurve, double(contours2[i].size()) * 0.05, true);
				// 				drawApproxCurve(copy,approxCurve,Scalar(0,0,255));
				// check that the poligon has 4 points
				if (approxCurve.size() == 4)
				{
					// and is convex
					if (isContourConvex(Mat(approxCurve)))
					{
						float minDist = 1e10;
						for (int j = 0; j < 4; j++)
						{
							float d = std::sqrt((float)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) * (approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
												(approxCurve[j].y - approxCurve[(j + 1) % 4].y) * (approxCurve[j].y - approxCurve[(j + 1) % 4].y));
							// 		norm(Mat(approxCurve[i]),Mat(approxCurve[(i+1)%4]));
							if (d < minDist) minDist = d;
						}
						// check that distance is not very small
						if (minDist > 10)
						{
							PolyCanditatesV[omp_get_thread_num()].push_back(approxCurve);
						}
					}
				}
			}
		}
	}

	// join all candidates
	vector< vector< Point > > PolyCanditates;

	for (size_t i = 0; i < PolyCanditatesV.size(); i++)
	{
		for (size_t j = 0; j < PolyCanditatesV[i].size(); j++)
		{
			PolyCanditates.push_back(PolyCanditatesV[i][j]);
		}
	}

	/// sort the points in anti-clockwise order
	//valarray< bool > swapped(false, PolyCanditates.size()); // used later
	for (unsigned int i = 0; i < PolyCanditates.size(); i++)
	{

		// trace a line between the first and second point.
		// if the thrid point is at the right side, then the points are anti-clockwise
		double dx1 = PolyCanditates[i][1].x - PolyCanditates[i][0].x;
		double dy1 = PolyCanditates[i][1].y - PolyCanditates[i][0].y;
		double dx2 = PolyCanditates[i][2].x - PolyCanditates[i][0].x;
		double dy2 = PolyCanditates[i][2].y - PolyCanditates[i][0].y;
		double o = (dx1 * dy2) - (dy1 * dx2);

		if (o > 0.0)
		{ // if the third point is in the left side, then sort in anti-clockwise order
			swap(PolyCanditates[i][1], PolyCanditates[i][2]);
			//swapped[i] = true;
			// sort the contour points
			//  	    reverse(PolyCanditates[i].contour.begin(),PolyCanditates[i].contour.end());//????
		}
	}

	for (size_t i = 0; i < PolyCanditates.size(); i++)
	{
		int mindis = thresImgv[0].cols*thresImgv[0].cols + thresImgv[0].rows*thresImgv[0].rows;
		int maxdis = 1;
		int maxidx = 0;
		int minidx = 0;
		for (size_t j = 0; j < 4; j++)
		{
			int a = PolyCanditates[i][j].x*PolyCanditates[i][j].x + PolyCanditates[i][j].y*PolyCanditates[i][j].y;

			if (a < mindis)
			{
				mindis = a;
				minidx = j;
			}
			if (a > maxdis)
			{
				maxdis = a;
				maxidx = j;
			}
		}

		Point t;
		t = PolyCanditates[i][0];
		PolyCanditates[i][0] = PolyCanditates[i][minidx];
		PolyCanditates[i][minidx] = t;

		t = PolyCanditates[i][2];
		PolyCanditates[i][2] = PolyCanditates[i][maxidx];
		PolyCanditates[i][maxidx] = t;

		if (abs(PolyCanditates[i][0].x - PolyCanditates[i][1].x) > abs(PolyCanditates[i][0].y - PolyCanditates[i][1].y))
		{
			t = PolyCanditates[i][1];
			PolyCanditates[i][1] = PolyCanditates[i][3];
			PolyCanditates[i][3] = t;
		}

	}

	/// remove these elements which corners are too close to each other
	// first detect candidates to be removed
	vector< vector< pair< int, int > > > TooNearCandidates_omp(omp_get_max_threads());
#pragma omp parallel for
	for (unsigned int i = 0; i < PolyCanditates.size(); i++)
	{
		// calculate the average distance of each corner to the nearest corner of the other marker candidate
		for (unsigned int j = i + 1; j < PolyCanditates.size(); j++)
		{
			valarray< float > vdist(4);
			for (int c = 0; c < 4; c++)
			{
				vdist[c] = sqrt((PolyCanditates[i][c].x - PolyCanditates[j][c].x) * (PolyCanditates[i][c].x - PolyCanditates[j][c].x) +
								(PolyCanditates[i][c].y - PolyCanditates[j][c].y) * (PolyCanditates[i][c].y - PolyCanditates[j][c].y));
			}

			// if distance is too small
			if (vdist[0] < 10 && vdist[1] < 10 && vdist[2] < 10 && vdist[3] < 10)
			{
				TooNearCandidates_omp[omp_get_thread_num()].push_back(pair< int, int >(i, j));
			}
		}
	}


	// join
	vector< pair< int, int > > TooNearCandidates;
	joinVectors(TooNearCandidates_omp, TooNearCandidates);
	// mark for removal the element of  the pair with smaller perimeter
	valarray< bool > toRemove(false, PolyCanditates.size());
	for (unsigned int i = 0; i < TooNearCandidates.size(); i++)
	{
		if (perimeter(PolyCanditates[TooNearCandidates[i].first]) > perimeter(PolyCanditates[TooNearCandidates[i].second]))
			toRemove[TooNearCandidates[i].second] = true;
		else
			toRemove[TooNearCandidates[i].first] = true;
	}

	// remove the invalid ones
	// finally, assign to the remaining candidates the contour
	outPolyCanditates.reserve(PolyCanditates.size());
	for (size_t i = 0; i < PolyCanditates.size(); i++)
	{
		if (!toRemove[i])
		{
			outPolyCanditates.push_back(PolyCanditates[i]);
		}
	}

	// remove markers with corners too near the image limits
	valarray< bool > toRemove1(false, outPolyCanditates.size());
	int borderDistThresX = _params._borderDistThres * float(thresImgv[0].cols);
	int borderDistThresY = _params._borderDistThres * float(thresImgv[0].rows);
	for (size_t i = 0; i < outPolyCanditates.size(); i++)
	{
		// delete if any of the corners is too near image border
		for (size_t c = 0; c < outPolyCanditates[i].size(); c++)
		{
			if (outPolyCanditates[i][c].x < borderDistThresX || outPolyCanditates[i][c].y < borderDistThresY ||
				outPolyCanditates[i][c].x > thresImgv[0].cols - borderDistThresX || outPolyCanditates[i][c].y > thresImgv[0].rows - borderDistThresY)
			{
				toRemove1[i] = true;
			}
		}
	}

	// remove the markers marker
	size_t indexValid = 0;
	for (size_t i = 0; i < toRemove1.size(); i++) {
		if (!toRemove1[i]) {
			if (indexValid != i)
				outPolyCanditates[indexValid] = outPolyCanditates[i];
			indexValid++;
		}
	}
	outPolyCanditates.resize(indexValid);
}


bool isAntiClockWise(cv::Point o, cv::Point a, cv::Point b)
{
	Point oa = Point(a.x - o.x, a.y - o.y);
	Point ob = Point(b.x - o.x, b.y - o.y);
	if (oa.x*ob.y - oa.y*ob.x < 0)
		return true;
	return false;
}

void makeAntiClockWise(std::vector< std::vector< cv::Point > > &polys)
{
	/// sort the points in anti-clockwise order
	valarray< bool > swapped(false, polys.size()); // used later
	for (unsigned int i = 0; i < polys.size(); i++) {

		// trace a line between the first and second point.
		// if the thrid point is at the right side, then the points are anti-clockwise
		double dx1 = polys[i][1].x - polys[i][0].x;
		double dy1 = polys[i][1].y - polys[i][0].y;
		double dx2 = polys[i][2].x - polys[i][0].x;
		double dy2 = polys[i][2].y - polys[i][0].y;
		double o = (dx1 * dy2) - (dy1 * dx2);

		if (o < 0.0) { // if the third point is in the left side, then sort in anti-clockwise order
			swap(polys[i][1], polys[i][3]);
			swapped[i] = true;
			// sort the contour points
			//  	    reverse(PolyCanditates[i].contour.begin(),PolyCanditates[i].contour.end());//????
		}
	}
}

void getCharRect(cv::Mat img, std::vector< std::vector< cv::Point > > polys, std::vector<CharacterImg> &charRectImg, cv::Size size_)
{
	if (polys.size() <= 0)
	{
		return;
	}
	Mat gray;

	if (img.channels() == 3)
	{
		cvtColor(img, gray, CV_BGR2GRAY);
	}
	else
	{
		img.copyTo(gray);
	}

	vector<Point2f> _rect;
	_rect.push_back(Point2f(0, 0));
	_rect.push_back(Point2f(0, size_.height));
	_rect.push_back(Point2f(size_.width, size_.height));
	_rect.push_back(Point2f(size_.width, 0));

	charRectImg.clear();

	vector<Point2f> _poly;
	CharacterImg charImgtemp;

	int boundwidth = size_.height / 10;
	Rect r(boundwidth, boundwidth, size_.width - boundwidth * 2, size_.height - boundwidth * 2);
	Mat thresimg;

	for (size_t i = 0; i < polys.size(); i++)
	{
		_poly.clear();

		for (size_t j = 0; j < polys[i].size(); j++)
		{
			_poly.push_back(Point2f(polys[i][j].x, polys[i][j].y));
		}

		Mat m = getPerspectiveTransform(_poly, _rect);
		Mat charRegion;
		warpPerspective(gray, charRegion, m, size_);

		//imshow("charRegion", charRegion);
		threshold(charRegion, thresimg, 20, 255, THRESH_OTSU);
		//imshow("otsu", thresimg);

		Scalar s1 = sum(thresimg);
		Scalar s2 = sum(thresimg(r));

		int diff_ave = (s1[0] - s2[0]) / (thresimg.cols*thresimg.rows - r.area());
		//cout << "diff = " << diff_ave << endl;

		if (diff_ave <= 100)
		{
			charImgtemp.poly_.clear();
			for (size_t j = 0; j < _poly.size(); j++)
			{
				charImgtemp.poly_.push_back(_poly[j]);
			}
			charImgtemp.img_ = thresimg(r).clone();
			charRectImg.push_back(charImgtemp);
		}
	}
}

int perimeter(std::vector< cv::Point > &a)
{
	int sum = 0;
	for (unsigned int i = 0; i < a.size(); i++) {
		int i2 = (i + 1) % a.size();
		sum += sqrt((a[i].x - a[i2].x) * (a[i].x - a[i2].x) + (a[i].y - a[i2].y) * (a[i].y - a[i2].y));
	}
	return sum;
}

cv::Point getCharRectCenter(CharacterImg charcterImg)
{
	Point center(0, 0);
	for (size_t j = 0; j < charcterImg.poly_.size(); j++)
	{
		center = center + charcterImg.poly_[j];
		//cout<<center;
	}
	center.x /= charcterImg.poly_.size();
	center.y /= charcterImg.poly_.size();

	return center;
}

void detectNumber(cv::Mat src, tesseract::TessBaseAPI &tess, std::vector<NumberPosition> &result)
{
	result.clear();

	Mat gray;
	vector<vector<Point>> PolyCanditates;
	vector<CharacterImg> characterImgV;

	Mat img_threshold;

	cvtColor(src, gray, CV_BGR2GRAY);
	adaptiveThreshold(gray, img_threshold, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 7, 7);

	detectRectangles(img_threshold, PolyCanditates);

	//Mat drawing = Mat::zeros(img_threshold.size(), CV_8UC3);
	//RNG rng(12345);

	for (size_t i = 0; i < PolyCanditates.size(); i++)
	{
		//Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		//drawContours(drawing, PolyCanditates, i, color, 2, 8, vector<Vec4i>(), 0, Point());
		drawContours(src, PolyCanditates, i, CV_RGB(255,0,0), 2, 8, vector<Vec4i>(), 0, Point());
		//rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
	}

	//imshow("Contours", drawing);

	cv::Size size_(200, 250);

	characterImgV.clear();
	getCharRect(src, PolyCanditates, characterImgV, size_);

	Size mask_size(size_.width / 4 * 3, size_.height / 5 * 4);

	Rect r(10, 10, mask_size.width - 20, mask_size.height - 20);

	Mat mask(mask_size, CV_8UC1, Scalar::all(255));
	mask(r).setTo(Scalar::all(0));
	int g_nStructElementSize = 1; //ç»æåç´ (åæ ¸ç©éµ)çå°ºå¯?

	//è·åèªå®ä¹æ ¸
	Mat element = getStructuringElement(MORPH_RECT,
										Size(2 * g_nStructElementSize + 1, 2 * g_nStructElementSize + 1),
										Point(g_nStructElementSize, g_nStructElementSize));
	for (size_t i = 0; i < characterImgV.size(); i++)
	{
		Mat img_character = characterImgV[i].img_.clone();
		Mat tempgray;

		morphologyEx(img_character, img_character, MORPH_OPEN, element, Point(-1, -1), 3);
		//dilate(img_character, img_character, element, Point(-1, -1), 3);
		//cv::erode(img_character, img_character, element, Point(-1, -1), 3);

		img_character.setTo(255, mask);
		imshow("img_character",img_character);

		tess.SetImage(img_character.data, img_character.cols, img_character.rows, 1, img_character.cols);

		char* UTF8Text = tess.GetUTF8Text();

		string tess_result_text(UTF8Text);

		delete[] UTF8Text;

		if (isNumberChar(tess_result_text[0]))
		{
			NumberPosition np;
			np.number_ = tess_result_text[0];
			np.position_ = getCharRectCenter(characterImgV[i]);
			np.boundRect = boundingRect(Mat(characterImgV[i].poly_));

			result.push_back(np);
		}
	}
}

CharacterImg::~CharacterImg()
{

}

CharacterImg::CharacterImg()
{

}

NumberPosition::~NumberPosition()
{

}

NumberPosition::NumberPosition()
{
	this->number_ = 'N';
	this->position_ = cv::Point(-100, -100);
}

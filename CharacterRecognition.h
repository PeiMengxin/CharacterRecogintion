#ifndef __CharacterRecognition_H__
#define __CharacterRecognition_H__

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <tesseract/strngs.h>

/**Operating params
*/
struct Params{
	//method emplpoyed for image threshold
	//ThresholdMethods _thresMethod;
	// Threshold parameters
	//double _thresParam1, _thresParam2, _thresParam1_range;

	// Current corner method
	//CornerRefinementMethod _cornerMethod;
	//when using subpix, this indicates the search range for optimization (in pixels)
	//int _subpix_wsize;
	//size of the image passed to the MarkerLabeler
	//int _markerWarpSize;
	// border around image limits in which corners are not allowed to be detected. (0,1)
	float _borderDistThres;

	// minimum and maximum size of a contour lenght. We use the following formula
	// minLenght=  min ( _minSize_pix , _minSize* Is)*4
	// maxLenght=    _maxSize* Is*4
	// being Is=max(imageWidth,imageHeight)
	//the values  _minSize and _maxSize are normalized, thus, depends on camera image size
	//However, _minSize_pix is expressed in pixels and  prevents a marker large enough, but relatively small to the image dimensions
	//to be discarded. For instance, imagine a image of 6000x6000 and a marker of 100x100 in it.
	// The marker is visible, but relatively small, so, we set a minimum size expressed in pixels to avoid discarding it
	float _minSize, _maxSize;
	int _minSize_pix;
	Params(){
		//_thresMethod = ADPT_THRES;
		//_thresParam1 = _thresParam2 = 7;
		//_cornerMethod = LINES;

		//_thresParam1_range = 0;
		//_markerWarpSize = 56;

		_minSize = 0.04; _maxSize = 0.9; _minSize_pix = 25;
		_borderDistThres = 0.005; // corners at a distance from image boundary nearer than 2.5% of image are ignored
		//_subpix_wsize = 5;//window size employed for subpixel search (in vase you use _cornerMethod=SUBPIX
	}

};

class CharacterImg
{
public:
	CharacterImg();
	~CharacterImg();

	cv::Mat img_;
	std::vector<cv::Point> poly_;

private:

};

class NumberPosition
{
public:
	NumberPosition();
	~NumberPosition();

	std::string number_;
	cv::Point position_;
	cv::Rect boundRect;

private:

};

void getBlack(cv::Mat src, cv::Mat &dst, cv::Scalar blackUpperValue);	// get black area

void getCharCandRegions(const cv::Mat black, cv::Mat &charImg, cv::Rect &charRect);	// get character candidate regions

void detectRectangles(cv::Mat &thresImg, std::vector< std::vector< cv::Point > > &outPolyCanditates);
void detectRectangles(std::vector< cv::Mat > &thresImgv, std::vector< std::vector< cv::Point > > &outPolyCanditates);

void makeAntiClockWise(std::vector< std::vector< cv::Point > > &polys);
bool isAntiClockWise(cv::Point o, cv::Point a, cv::Point b);

void getCharRect(cv::Mat img, std::vector< std::vector< cv::Point > > polys, std::vector<CharacterImg> &charRectImg, cv::Size size_);

cv::Point getCharRectCenter(CharacterImg charcterImg);

void detectNumber(cv::Mat src, tesseract::TessBaseAPI &tess, std::vector<NumberPosition> &result);

int perimeter(std::vector< cv::Point > &a);

inline bool isNumberChar(char c){ return ((c >= '0') && (c <= '9')); }


template < typename T > void joinVectors(std::vector< std::vector< T > > &vv, std::vector< T > &v, bool clearv = false) 
{
	if (clearv)
		v.clear();
	for (size_t i = 0; i < vv.size(); i++)
		for (size_t j = 0; j < vv[i].size(); j++)
			v.push_back(vv[i][j]);
}

template < typename T > void removeElements(std::vector< T > &vinout, const std::vector< bool > &toRemove)
{
	// remove the invalid ones by setting the valid in the positions left by the invalids
	size_t indexValid = 0;
	for (size_t i = 0; i < toRemove.size(); i++) {
		if (!toRemove[i]) {
			if (indexValid != i)
				vinout[indexValid] = vinout[i];
			indexValid++;
		}
	}
	vinout.resize(indexValid);
}

#endif // __CharacterRecognition_H__


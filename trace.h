/*
 * trace.h
 *
 *  Created on: May 5, 2016
 *      Author: odroid
 */

#ifndef TRACE_H_
#define TRACE_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include "trace.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "uart.h"
#include <iostream>
#include <ctype.h>
#include "CharacterRecognition.h"

extern int circle_check,circles_x,circles_y,circles_r,track_check,state_v;
extern unsigned char circle_control[4];

extern std::vector<NumberPosition> number_position_send;
extern bool is_print_character;

#endif /* TRACE_H_ */

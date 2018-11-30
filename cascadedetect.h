#pragma once

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef __OPENCV_OBJDETECT_HPP__
#define __OPENCV_OBJDETECT_HPP__

#include "opencv2/core/core.hpp"
//#include <map>
//#include <deque>

namespace cv
{
#define CC_CASCADE_PARAMS "cascadeParams"
#define CC_STAGE_TYPE     "stageType"
#define CC_FEATURE_TYPE   "featureType"
#define CC_HEIGHT         "height"
#define CC_WIDTH          "width"

#define CC_STAGE_NUM    "stageNum"
#define CC_STAGES       "stages"
#define CC_STAGE_PARAMS "stageParams"

#define CC_BOOST            "BOOST"
#define CC_MAX_DEPTH        "maxDepth"
#define CC_WEAK_COUNT       "maxWeakCount"
#define CC_STAGE_THRESHOLD  "stageThreshold"
#define CC_WEAK_CLASSIFIERS "weakClassifiers"
#define CC_INTERNAL_NODES   "internalNodes"
#define CC_LEAF_VALUES      "leafValues"

#define CC_FEATURES       "features"
#define CC_FEATURE_PARAMS "featureParams"
#define CC_MAX_CAT_COUNT  "maxCatCount"

#define CC_HAAR   "HAAR"
#define CC_RECTS  "rects"
#define CC_TILTED "tilted"

#define CC_LBP  "LBP"
#define CC_RECT "rect"

#define CC_HOG  "HOG"

#define CV_SUM_PTRS( p0, p1, p2, p3, sum, rect, step )                    \
	/* (x, y) */                                                          \
	(p0) = sum + (rect).x + (step) * (rect).y,                            \
	/* (x + w, y) */                                                      \
	(p1) = sum + (rect).x + (rect).width + (step) * (rect).y,             \
	/* (x + w, y) */                                                      \
	(p2) = sum + (rect).x + (step) * ((rect).y + (rect).height),          \
	/* (x + w, y + h) */                                                  \
	(p3) = sum + (rect).x + (rect).width + (step) * ((rect).y + (rect).height)

#define CV_TILTED_PTRS( p0, p1, p2, p3, tilted, rect, step )                        \
	/* (x, y) */                                                                    \
	(p0) = tilted + (rect).x + (step) * (rect).y,                                   \
	/* (x - h, y + h) */                                                            \
	(p1) = tilted + (rect).x - (rect).height + (step) * ((rect).y + (rect).height), \
	/* (x + w, y + w) */                                                            \
	(p2) = tilted + (rect).x + (rect).width + (step) * ((rect).y + (rect).width),   \
	/* (x + w - h, y + w + h) */                                                    \
	(p3) = tilted + (rect).x + (rect).width - (rect).height                         \
	+ (step) * ((rect).y + (rect).width + (rect).height)

#define CALC_SUM_(p0, p1, p2, p3, offset) \
	((p0)[offset] - (p1)[offset] - (p2)[offset] + (p3)[offset])

#define CALC_SUM(rect,offset) CALC_SUM_((rect)[0], (rect)[1], (rect)[2], (rect)[3], offset)
	///////////////////////////// Object Detection ////////////////////////////

	CV_EXPORTS void groupRectangles(CV_OUT CV_IN_OUT vector<Rect>& rectList, int groupThreshold, double eps=0.2);
	CV_EXPORTS_W void groupRectangles(CV_OUT CV_IN_OUT vector<Rect>& rectList, CV_OUT vector<int>& weights, int groupThreshold, double eps=0.2);
	CV_EXPORTS void groupRectangles( vector<Rect>& rectList, int groupThreshold, double eps, vector<int>* weights, vector<int>* levelWeights );
	CV_EXPORTS void groupRectangles(vector<Rect>& rectList, vector<int>& rejectLevels,
		vector<int>& levelWeights, int groupThreshold, double eps=0.2);


	class CV_EXPORTS FeatureEvaluator
	{
	public:
		struct Feature
		{
			Feature();
			Feature( int x, int y, int _block_w, int _block_h  ) :
			rect(x, y, _block_w, _block_h) {}

			int calc( int offset ) const;
			void updatePtrs( const Mat& sum );
			//bool read(const FileNode& node );

			Rect rect; // weight and height for block
			const int* p[16]; // fast
		};
		FeatureEvaluator();
		virtual ~FeatureEvaluator();

		//virtual bool read(const FileNode& node);
		bool read(int flag);
		virtual Ptr<FeatureEvaluator> clone() const;

		virtual bool setImage(const Mat& img, Size origWinSize);
		virtual bool setWindow(Point p);

		int operator()(int featureIdx) const
		{ return featuresPtr[featureIdx].calc(offset); }
		virtual int calcCat(int featureIdx) const
		{ return (*this)(featureIdx); }

		static Ptr<FeatureEvaluator> create(int type);
	public:
		Size origWinSize;
		Ptr<vector<Feature> > features;
		Feature* featuresPtr; // optimization
		Mat sum0, sum;
		Rect normrect;

		int offset;
	};
	inline FeatureEvaluator::Feature :: Feature()
	{
		rect = Rect();
		for( int i = 0; i < 16; i++ )
			p[i] = 0;
	}

	inline int FeatureEvaluator::Feature :: calc( int _offset ) const
	{
		int cval = CALC_SUM_( p[5], p[6], p[9], p[10], _offset );

		return (CALC_SUM_( p[0], p[1], p[4], p[5], _offset ) >= cval ? 128 : 0) |   // 0
			(CALC_SUM_( p[1], p[2], p[5], p[6], _offset ) >= cval ? 64 : 0) |    // 1
			(CALC_SUM_( p[2], p[3], p[6], p[7], _offset ) >= cval ? 32 : 0) |    // 2
			(CALC_SUM_( p[6], p[7], p[10], p[11], _offset ) >= cval ? 16 : 0) |  // 5
			(CALC_SUM_( p[10], p[11], p[14], p[15], _offset ) >= cval ? 8 : 0)|  // 8
			(CALC_SUM_( p[9], p[10], p[13], p[14], _offset ) >= cval ? 4 : 0)|   // 7
			(CALC_SUM_( p[8], p[9], p[12], p[13], _offset ) >= cval ? 2 : 0)|    // 6
			(CALC_SUM_( p[4], p[5], p[8], p[9], _offset ) >= cval ? 1 : 0);
	}

	inline void FeatureEvaluator::Feature :: updatePtrs( const Mat& _sum )
	{
		const int* ptr = (const int*)_sum.data;
		size_t step = _sum.step/sizeof(ptr[0]);
		Rect tr = rect;
		CV_SUM_PTRS( p[0], p[1], p[4], p[5], ptr, tr, step );
		tr.x += 2*rect.width;
		CV_SUM_PTRS( p[2], p[3], p[6], p[7], ptr, tr, step );
		tr.y += 2*rect.height;
		CV_SUM_PTRS( p[10], p[11], p[14], p[15], ptr, tr, step );
		tr.x -= 2*rect.width;
		CV_SUM_PTRS( p[8], p[9], p[12], p[13], ptr, tr, step );
	}

	class CV_EXPORTS_W CascadeClassifier
	{
	public:
		CV_WRAP CascadeClassifier();
		//CV_WRAP CascadeClassifier( const string& filename );
		virtual ~CascadeClassifier();

		CV_WRAP virtual bool empty() const;
		//CV_WRAP bool load( const string& filename );
		CV_WRAP bool load_file(int flag);
		//virtual bool read( const FileNode& node );
		bool read(int flag);
		CV_WRAP virtual void detectMultiScale( const Mat& image, 
			CV_OUT vector<Rect>& objects,
			double scaleFactor=1.1,
			int minNeighbors=3, int flags=0,
			Size minSize=Size(),
			Size maxSize=Size());

		CV_WRAP virtual void detectMultiScale( const Mat& image,
			CV_OUT vector<Rect>& objects,
			vector<int>& rejectLevels,
			vector<int>& levelWeights,
			double scaleFactor=1.1,
			int minNeighbors=3, int flags=0,
			Size minSize=Size(),
			Size maxSize=Size(),
			bool outputRejectLevels=false);


		virtual Size getOriginalWindowSize() const;
		bool setImage( const Mat& );

	protected:
		//virtual bool detectSingleScale( const Mat& image, int stripCount, Size processingRectSize,
		//                                int stripSize, int yStep, double factor, vector<Rect>& candidates );
		friend class CascadeClassifierInvoker;
		virtual bool detectSingleScale( const Mat& image, int stripCount, Size processingRectSize,
			int stripSize, int yStep, double factor, vector<Rect>& candidates,
			vector<int>& rejectLevels, vector<int>& levelWeights, bool outputRejectLevels=false);
	protected:
		enum { BOOST = 0 };
		enum { DO_CANNY_PRUNING = 1, SCALE_IMAGE = 2,
			FIND_BIGGEST_OBJECT = 4, DO_ROUGH_SEARCH = 8 };

		template<class FEval>
		friend int predictCategorical( CascadeClassifier& cascade, Ptr<FeatureEvaluator> &featureEvaluator, int& weight);
		template<class FEval>
		friend int predictCategoricalStump( CascadeClassifier& cascade, Ptr<FeatureEvaluator> &featureEvaluator, int& weight);
		virtual void cascade_detection(CascadeClassifier *classifier,Size processingRectSize, int stripSize, int yStep, double scalingFactor,
			vector<Rect>& rectangles, vector<int>& rejectLevels, vector<int>& levelWeights, bool outputLevels, Mat mask);
		bool setImage( Ptr<FeatureEvaluator>& feval, const Mat& image);
		virtual int runAt( Ptr<FeatureEvaluator>& feval, Point pt, int& weight);

	public:
		class Data
		{
		public:
			struct CV_EXPORTS DTreeNode
			{
				int featureIdx;
				int threshold; // for ordered features only
				int left;
				int right;
			};

			struct CV_EXPORTS DTree
			{
				int nodeCount;
			};

			struct CV_EXPORTS Stage
			{
				int first;
				int ntrees;
				int threshold;
			};

			bool read(const FileNode &node);
			bool read(int flag);
			bool isStumpBased;

			int stageType;
			int featureType;
			int ncategories;
			Size origWinSize;

			vector<Stage> stages;
			vector<DTree> classifiers;
			vector<DTreeNode> nodes;
			vector<int> leaves;
			vector<int> subsets;
		};

		Data data;
		Ptr<FeatureEvaluator> featureEvaluator;
	};
}

#endif


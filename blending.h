/* --------------------------------------------------------------------------
 * File:    blending.h
 * Created: 2015-10-30
 * --------------------------------------------------------------------------
 *
 *
 *
 * ------------------------------------------------------------------------*/


#ifndef __blending__h
#define __blending__h

#include "Image.h"
#include "matrix.h"
#include "filtering.h"
#include "homography.h"
#include "panorama.h"
#include "basicImageManipulation.h"
#include <iostream>
#include <cmath>

using namespace std;

// encodes which type of blending to use
enum class BlendType {
    BLEND_NONE,
    BLEND_LINEAR,
    BLEND_2LAYER
};

// linear blending
Image blendingweight(int imwidth, int imheight);
void applyhomographyBlend(const Image &source,
                          const Image &weight,
                          Image &out,
                          const Matrix &H,
                          bool bilinear = false);
Image stitchLinearBlending(const Image &im1,
                           const Image &im2,
                           const Image &we1,
                           const Image &we2,
                           const Matrix &H);

//all
vector<Image> scaledecomp(const Image &im, float sigma = 2.0);
Image stitchBlending(const Image &im1,
                     const Image &im2,
                     const Matrix &H,
                     BlendType blend);
Image autostitch(const Image &im1,
                 const Image &im2,
                 BlendType blend,
                 float blurDescriptor = 0.5,
                 float radiusDescriptor = 4);

// mini planets
Image pano2planet(const Image &pano, int newImSize, bool clamp=true);

// stitch N images
vector<Matrix> sequenceHs(const vector<Image> &ims,
                          float blurDescriptor = 0.5,
                          float radiusDescriptor = 4);
BoundingBox bboxN(const vector<Matrix> &Hs, const vector<Image> &ims);
Image autostitchN(const vector<Image> &ims,
                  int refIndex,
                  float blurDescriptor = 0.5,
                  float radiusDescriptor = 4);

// helpful functions
Image copychannels(const Image &im, int nChannels);
vector<Matrix> stackHomographies(const vector<Matrix> &Hs, int refIndex);

#endif

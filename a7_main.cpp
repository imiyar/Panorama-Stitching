/* --------------------------------------------------------------------------
 * File:    a7_main.cpp
 * Created: 2015-10-17
 * --------------------------------------------------------------------------
 *
 *
 *
 * ------------------------------------------------------------------------*/


#include "matrix.h"
#include "panorama.h"
#include "blending.h"
#include <iostream>
#include <cassert>
#include <ctime>

using namespace std;

void testComputeTensor() {
    // load images
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");

    // compute tensors
    Image tensor1 = computeTensor(stata1);
    tensor1.write("./Output/stataTensor-1.png");
    float maxi = tensor1.max();
    if(maxi != 0) {
        tensor1 = tensor1 / maxi ;
        tensor1.write("./Output/stataTensor-1normed.png");
    }

    Image tensor2 = computeTensor(stata2);
    tensor2.write("./Output/stataTensor-2.png");
    maxi = tensor2.max();
    if(maxi != 0) {
        tensor2 = tensor2 / maxi ;
        tensor2.write("./Output/stataTensor-2normed.png");
    }
}


void testCornerResponse() {
    // load images
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");

    // corner responses
    Image cr1 = cornerResponse(stata1);
    float maxi = cr1.max();
    if(maxi != 0) {
        (cr1/maxi).write("./Output/stata1-cornerResponse.png");
    }
    Image cr2 = cornerResponse(stata2);
    maxi = cr2.max();
    if(maxi != 0) {
        (cr2/maxi).write("./Output/stata2-cornerResponse.png");
    }
}


void testHarrisCorners() {
    // load images
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");

    // get harris corners
    vector<Point> h1 = HarrisCorners(stata1);
    vector<Point> h2 = HarrisCorners(stata2);
    visualizeCorners(stata1, h1).write("./Output/stata1-corners.png");
    visualizeCorners(stata2, h2).write("./Output/stata2-corners.png");
}


void testDescriptor() {
    Image stata1("./Input/stata-1.png");
    vector<Image> lc = lumiChromi(stata1);
    Image desc = descriptor(lc[0], Point(10, 10));
    BoundingBox bbox(10-4,10+4,10-4,10+4);
    drawBoundingBox(lc[0], bbox).write("./Output/stata1-descriptor-location.png");
    desc.write("./Output/stata1-descriptor-test.png");
}


void testComputeFeatures() {
    // load images
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");

    // get and output features
    vector<Point> h1   = HarrisCorners(stata1);
    vector<Feature> f1 = computeFeatures(stata1, h1);
    visualizeFeatures(stata1, f1).write("./Output/stata1-features.png");

    vector<Point> h2 = HarrisCorners(stata2);
    vector<Feature> f2 = computeFeatures(stata2, h2);
    visualizeFeatures(stata2, f2).write("./Output/stata2-features.png");
}

// Test your l2Features function !


void testFindCorrespondences() {
    // load images
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");

    // get and output features
    vector<Point> h1   = HarrisCorners(stata1);
    vector<Feature> f1 = computeFeatures(stata1, h1);
    vector<Point> h2   = HarrisCorners(stata2);
    vector<Feature> f2 = computeFeatures(stata2, h2);

    // correspondances
    vector<FeatureCorrespondence> corr = findCorrespondences(f1, f2);
    visualizePairs(stata1, stata2, corr).write("./Output/stata-featcorr.png");
}


void testRANSAC() {
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");

    // get and output features
    vector<Point> h1   = HarrisCorners(stata1);
    vector<Feature> f1 = computeFeatures(stata1, h1);
    vector<Point> h2   = HarrisCorners(stata2);
    vector<Feature> f2 = computeFeatures(stata2, h2);

    // correspondances
    vector<FeatureCorrespondence> corr = findCorrespondences(f1, f2);

    Matrix H = RANSAC(corr, 100);
    vector<bool> ins = inliers(H, corr);
    visualizePairsWithInliers(stata1, stata2, corr, ins).write("./Output/stata-RANSAC-featcorr-inliers.png");

    vector<Image> output = visualizeReprojection(stata1, stata2, H, corr, ins);
    output[0].write("./Output/stata-RANSAC-1-featreproject.png");
    output[1].write("./Output/stata-RANSAC-2-featreproject.png");
}


void testVisualizeCorrespondenceInliers() {
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");

    // get and output features
    vector<Point> h1   = HarrisCorners(stata1);
    vector<Feature> f1 = computeFeatures(stata1, h1);
    vector<Point> h2   = HarrisCorners(stata2);
    vector<Feature> f2 = computeFeatures(stata2, h2);

    // correspondances
    vector<FeatureCorrespondence> corr = findCorrespondences(f1, f2);

    Matrix H(3,3);
    vector<FeatureCorrespondence> corrSample = sampleFeatureCorrespondences(corr);
    vector<CorrespondencePair> listOfPairs = getListOfPairs(corr); // Call this line inside of your RANsac for loop

    H = computeHomography(listOfPairs.data());

    vector<bool> ins = inliers(H, corr);
    visualizePairsWithInliers(stata1, stata2, corr, ins).write("./Output/stata-featcorr-inliers.png");

    vector<Image> output = visualizeReprojection(stata1, stata2, H, corr, ins);
    output[0].write("./Output/stata-1-featreproject.png");
    output[1].write("./Output/stata-2-featreproject.png");
}


void testAutoStitch() {
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");
    autostitch(stata1, stata2).write("./Output/Stata-final-stitch.png");
}


void testAutoStitchBoston() {
    Image boston1("./Input/boston-skyline-1.png");
    Image boston2("./Input/boston-skyline-2.png");
    autostitch(boston1, boston2).write("./Output/boston-skyline-final-stitch.png");
}

// test blending weight
void testBlendingWeight() {
    Image poster("Input/poster.png");
    Image w = blendingweight(poster.width(), poster.height());
    w.write("Output/poster-weight.png");
}

// test apply homography function
void testApplyHomographyBlend() {
    Matrix H(3, 3);
    H <<
        1.23445,  -0.0258246,    96,
       0.210363,     1.00462,   171,
     0.00108756, -0.00010989,     1;

    // apply homography and output image
    Image poster("Input/poster.png");
    Image green("Input/green.png");

    Image w = blendingweight(poster.width(), poster.height());
    w.write("Output/posterWeight.png");

    // this doesn't look very meaningful for this image
    applyhomographyBlend(poster, w, green, H, false);
    green.write("Output/greenwithposterblendweight.png");

    // so we are going to weight it evenly
    green = Image("Input/green.png");
    applyhomographyBlend(poster, w*0+0.5, green, H, false);
    green.write("Output/greenwithposter05weight.png");
}

// test stitch with stata
void testStitchBlendStata() {
    Image im1("Input/stata-1.png");
    Image im2("Input/stata-2.png");
    Matrix H(3, 3);
    H <<
        1.30954, -0.174891, -244.261,
        0.390429, 1.1775, -60.5194,
        0.000985249, -0.000229697,    1;
    stitchBlending(im1, im2, H, BlendType::BLEND_NONE).write("./Output/stata-stitch-noblending.png");
    stitchBlending(im1, im2, H, BlendType::BLEND_LINEAR).write("./Output/stata-stitch-linearblending.png");
    stitchBlending(im1, im2, H, BlendType::BLEND_2LAYER).write("./Output/stata-stitch-2scaleblending.png");
}

// test autostitchers
void testAutoStitchBlendStata() { // stata
    Image stata1("./Input/stata-1.png");
    Image stata2("./Input/stata-2.png");
    autostitch(stata1, stata2, BlendType::BLEND_LINEAR).write("./Output/Stata-autostitch-linearblending.png");
    autostitch(stata1, stata2, BlendType::BLEND_2LAYER).write("./Output/Stata-autostitch-2scaleblending.png");
}

void testAutoStitchBlendBoston() { // boston
    Image boston1("./Input/boston-sunset-1.png");
    Image boston2("./Input/boston-sunset-2.png");
    autostitch(boston1, boston2, BlendType::BLEND_LINEAR).write("./Output/Boston-autostitch-linearblending.png");
    autostitch(boston1, boston2, BlendType::BLEND_2LAYER).write("./Output/Boston-autostitch-2scaleblending.png");
}

void testAutoStitchBlendCastle() { // castle
    Image castle1("./Input/guedelon-1.png");
    Image castle2("./Input/guedelon-2.png");
    autostitch(castle1, castle2, BlendType::BLEND_LINEAR).write("./Output/guedelon-autostitch-linearblending.png");
    autostitch(castle1, castle2, BlendType::BLEND_2LAYER).write("./Output/guedelon-autostitch-2scaleblending.png");
}

// test plane
void testPano2Planet() {
    Image im("./Input/boston_winter.png");
    Image world = pano2planet(im, 500);
    world.write("./Output/boston_winter_planet.png");

    Image mars("./Input/mars_pano.png");
    pano2planet(mars, 300).write("./Output/mars_planet.png");

}

// 6.865 - N stitch - Boston
void testAutoStitchNBoston() {
    vector<Image> ims;
    ims.push_back(Image("./Input/boston-sunset-1.png"));
    ims.push_back(Image("./Input/boston-sunset-2.png"));
    ims.push_back(Image("./Input/boston-sunset-3.png"));
    autostitchN(ims, 1).write("./Output/boston-autostitchN.png");
}

// 6.865 - N stitch - Castle
void testAutoStitchNCastle() {
    vector<Image> ims;
    ims.push_back(Image("./Input/guedelon-1.png"));
    ims.push_back(Image("./Input/guedelon-2.png"));
    ims.push_back(Image("./Input/guedelon-3.png"));
    autostitchN(ims, 1).write("./Output/guedelon-autostitchN.png");
}

void testMyPanorama() {
    vector<Image> ims;
    ims.push_back(Image("./Input/IMG_1278-min.png"));
    ims.push_back(Image("./Input/IMG_1279-min.png"));
    ims.push_back(Image("./Input/IMG_1280-min.png"));
    ims.push_back(Image("./Input/IMG_1281-min.png"));
    autostitchN(ims, 1).write("./Output/room.png");
}


// This is a way for you to test your functions.
int main() {
    cout << "nothing done in a7_main.cpp, but read me! There's some useful info here." << endl;
    //this sets the random number generator seed, without varying this value, all
    //runs should produce the same results (e.g. same sequence of "random" number)
    //when you are ready for multiple runs try srand(time(0));
    //this will set it to the current system time
    srand(time(0)); // Fixed seed for deterministic results

    testComputeTensor();
    testCornerResponse();
    testHarrisCorners();
    testDescriptor();
    testComputeFeatures();
    testFindCorrespondences();
    testRANSAC();
    testVisualizeCorrespondenceInliers();
    testAutoStitch();
    testAutoStitchBoston();

    testBlendingWeight();
    testApplyHomographyBlend();
    testStitchBlendStata();
    testAutoStitchBlendStata();
    testAutoStitchBlendBoston();
    testAutoStitchBlendCastle();
    testPano2Planet();
    testAutoStitchNBoston();
    testAutoStitchNCastle();
    testMyPanorama();
}


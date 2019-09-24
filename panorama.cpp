#include "panorama.h"
#include "matrix.h"
#include <unistd.h>
#include <ctime>
#include <cassert>

using namespace std;

Image computeTensor(const Image &im, float sigmaG, float factorSigma) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Compute xx/xy/yy Tensor of an image. (stored in that order)
    Image lumi = lumiChromi(im)[0];
    lumi = gaussianBlur_separable(lumi, sigmaG);
    Image gradX = gradientX(lumi);
    Image gradY = gradientY(lumi);

    int width = im.width();
    int height = im.height();
    Image output = Image(width, height, 3);

    for (int i=0; i<width; ++i) {
        for (int j=0; j<height; ++j){
            output(i, j, 0) = pow(gradX(i, j), 2);
            output(i, j, 1) = gradX(i, j) * gradY(i, j);
            output(i, j, 2) = pow(gradY(i, j), 2);
        }
    }
    output = gaussianBlur_separable(output, sigmaG*factorSigma);
    return output;
}

Image cornerResponse(const Image &im, float k, float sigmaG, float factorSigma) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Compute response = det(M) - k*[(trace(M)^2)] at every pixel location,
    // using the structure tensor of im.

    Image tensor = computeTensor(im, sigmaG, factorSigma);
    int width = im.width();
    int height = im.height();
    Image output = Image(width, height, 1);
    Matrix M(2, 2);

    for (int i=0; i<width; ++i) {
        for (int j=0; j<height; ++j) {
            M <<    tensor(i, j, 0), tensor(i, j, 1),
                    tensor(i, j, 1), tensor(i, j, 2);
            output(i, j) = M.determinant() - k * pow(M.trace(), 2);
        }
    }
    return output;
}


vector<Point> HarrisCorners(const Image &im,
                            float k,
                            float sigmaG,
                            float factorSigma,
                            float maxiDiam,
                            float boundarySize) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Compute Harris Corners by maximum filtering the cornerResponse map.
    // The corners are the local maxima.
    Image CR = cornerResponse(im, k, sigmaG, factorSigma);
    Image max_pixels = maximum_filter(CR, maxiDiam);
    int width = im.width();
    int height = im.height();
    vector<Point> points = {};

    for (int i=boundarySize; i<width-boundarySize; ++i) {
        for (int j=boundarySize; j<height-boundarySize; ++j) {
            if (max_pixels(i, j) == CR(i, j)) {
                points.push_back(Point(i, j));
            }
        }
    }
    return points;
}


Image descriptor(const Image &blurredIm, const Point &p, float radiusDescriptor) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Extract a descriptor from blurredIm around point p, with a radius 'radiusDescriptor'.
    Image output = Image(radiusDescriptor * 2 + 1, radiusDescriptor * 2 + 1, 1);
    for (int i=-radiusDescriptor; i<=radiusDescriptor; ++i) {
        for (int j=-radiusDescriptor; j<=radiusDescriptor; ++j) {
            output(i+radiusDescriptor, j+radiusDescriptor) = blurredIm(i+p.x, j+p.y);
        }
    }

    output = output - output.mean();
    output = output / pow(output.var(), 0.5);

    return output;
}


vector<Feature> computeFeatures(const Image &im,
                                const vector<Point> &cornersL,
                                float sigmaBlurDescriptor,
                                float radiusDescriptor) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Pset07. obtain corner features from a list of corner points
    vector<Feature> features = {};
    Image lumi = lumiChromi(im)[0];
    Image blurredIm = gaussianBlur_separable(lumi, sigmaBlurDescriptor);

    for (unsigned int i=0; i<cornersL.size(); ++i) {
        Image desc = descriptor(blurredIm, cornersL[i], radiusDescriptor);
        features.push_back(Feature(cornersL[i], desc));
    }
    return features;
}



float l2Features(const Feature &f1, const Feature &f2) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Compute the squared Euclidean distance between the descriptors of f1, f2.
    int width = f1.desc().width();
    int height = f1.desc().height();
    float dist = 0.0f;

    for (int i=0; i<width; ++i) {
        for (int j=0; j<height; ++j) {
            dist += pow((f1.desc()(i, j) - f2.desc()(i, j)), 2);
        }
    }
    return dist;
}


vector<FeatureCorrespondence> findCorrespondences(
        const vector<Feature> &listFeatures1,
        const vector<Feature> &listFeatures2,
        float threshold) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Find correspondences between listFeatures1 and listFeatures2 using the
    // second-best test.
    float sqr_threshold = threshold * threshold;
    vector<FeatureCorrespondence> correspondences = {};

    for (unsigned int i=0; i<listFeatures1.size(); ++i) {
        Feature best = listFeatures2[0];
        float best_dist = -1;
        float second_dist = -1;
        for (unsigned int j=0; j<listFeatures2.size(); ++j) {
            float dist = l2Features(listFeatures1[i], listFeatures2[j]);
            if ((dist < best_dist) || (best_dist == -1)) {
                second_dist = best_dist;
                best_dist = dist;
                best = listFeatures2[j];
            }
            else if ((dist < second_dist) || (second_dist == -1)) {
                second_dist = dist;
            }
        }
        if ((second_dist/best_dist) >= sqr_threshold) {
            correspondences.push_back(FeatureCorrespondence(listFeatures1[i], best));
        }
    }
    return correspondences;
}


vector<bool> inliers(const Matrix &H,
                     const vector<FeatureCorrespondence> &listOfCorrespondences,
                     float epsilon) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Pset07: Implement as part of RANSAC
    // return a vector of bools the same size as listOfCorrespondences indicating
    //  whether each correspondance is an inlier according to the homography H and threshold epsilon
    std::vector<bool> result = {};

    for (unsigned int i=0; i<listOfCorrespondences.size(); ++i) {
        Point p1 = listOfCorrespondences[i].feature(0).point();
        Point p2 = listOfCorrespondences[i].feature(1).point();

        Vec3f v1 = Vec3f(p1.x, p1.y, 1);
        Matrix M = H * v1;
        float x = (float)M(0, 0) / M(2, 0);
        float y = (float)M(1, 0) / M(2, 0);
        Vec2f vnew = Vec2f(x, y);
        Vec2f v2 = Vec2f(p2.x, p2.y);

        result.push_back((v2 - vnew).squaredNorm() < epsilon);
    }

    return result;
}

Matrix RANSAC(const vector<FeatureCorrespondence> &listOfCorrespondences,
              int Niter,
              float epsilon) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Put together the RANSAC algorithm.
    int best = 0;
    Matrix best_H(3, 3);

    for (int i=0; i<Niter; ++i) {
        vector<FeatureCorrespondence> list = sampleFeatureCorrespondences(listOfCorrespondences);
        vector<FeatureCorrespondence> randomFour(list.begin(), list.begin()+4);
        vector<CorrespondencePair> correspondences = getListOfPairs(randomFour);

        Matrix H = computeHomography(correspondences.data());
        if (H.determinant() == 0) H = Matrix::Identity(3, 3);

        vector<bool> fit = inliers(H, listOfCorrespondences, epsilon);
        int score = 0;
        for (unsigned int j=0; j<listOfCorrespondences.size(); ++j) {
            if (fit[j] == true) {
                score += 1;
            }
        }

        if (score > best) {
            best = score;
            best_H = H;
        }

    }
    return best_H;
}


Image autostitch(const Image &im1,
                 const Image &im2,
                 float blurDescriptor,
                 float radiusDescriptor) {
    // // --------- HANDOUT  PS07 ------------------------------
    // Now you have all the ingredients to make great panoramas without using a
    // primitive javascript UI !
    int imwidth = im1.width();
    int imheight = im1.height();

    vector<Point> corners1 = HarrisCorners(im1);
    vector<Point> corners2 = HarrisCorners(im2);
    vector<Feature> features1 = computeFeatures(im1, corners1,blurDescriptor, radiusDescriptor);
    vector<Feature> features2 = computeFeatures(im2, corners2,blurDescriptor, radiusDescriptor);

    vector<FeatureCorrespondence> correspondences = findCorrespondences(features1, features2);
    Matrix H = RANSAC(correspondences);

    BoundingBox B1 = computeTransformedBBox(imwidth, imheight, H);
    BoundingBox B2 = BoundingBox(0, im2.width()-1, 0, im2.height()-1);
    BoundingBox B = bboxUnion(B1, B2);
    Matrix T = makeTranslation(B);

    int width = B.x2-B.x1+1;
    int height = B.y2-B.y1+1;
    Image output = Image(width, height, 3);
    
    applyHomography(im1, T*H, output);
    applyHomography(im2, T, output);

    return output;
}




// *****************************************************************************
//  * Helpful optional functions to implement
// ****************************************************************************

Image getBlurredLumi(const Image &im, float sigmaG) {
    return Image(1,1,1);
}

int countBoolVec(const vector<bool> &ins) {
    return 0;
}

// *****************************************************************************
//  * Do Not Modify Below This Point
// *****************************************************************************

// Pset07 RANsac helper. re-shuffle a list of correspondances
vector<FeatureCorrespondence> sampleFeatureCorrespondences(
        vector<FeatureCorrespondence> listOfCorrespondences) {
    random_shuffle(listOfCorrespondences.begin(), listOfCorrespondences.end());
    return listOfCorrespondences;
}

// Pset07 RANsac helper: go from 4 correspondances to a list of points [4][2][3] as used in Pset06.
// Note: The function uses the first 4 correspondences passed
vector<CorrespondencePair> getListOfPairs(
        const vector<FeatureCorrespondence> &listOfCorrespondences) {
    assert(listOfCorrespondences.size() >= 4);
    vector<CorrespondencePair> out;
    for (int i = 0; i < 4; i++) {
        out.push_back(listOfCorrespondences[i].toCorrespondencePair());
    }
    return out;
}

// Corner visualization.
Image visualizeCorners(const Image &im,
                       const vector<Point> &pts,
                       int rad,
                       const vector<float> &color) {
    Image vim = im;
    for (int i = 0; i < (int) pts.size(); i++) {
        int px = pts[i].x;
        int py = pts[i].y;

        int minx = max(px - rad, 0);

        for (int delx = minx; delx < min(im.width(), px + rad + 1); delx++)
        for (int dely = max(py - rad, 0); dely < min(im.height(), py + rad + 1); dely++)
        {
            if ( sqrt(pow(delx-px, 2) + pow(dely - py, 2)) <= rad) {
                for (int c = 0; c < im.channels(); c++) {
                    vim(delx, dely, c) = color[c];
                }
            }
        }
    }
    return vim;
}

Image visualizeFeatures(const Image &im,
                        const vector<Feature> &LF,
                        float radiusDescriptor) {
    // assumes desc are within image range
    Image vim = im;
    int rad = radiusDescriptor;

    for (int i = 0; i < (int) LF.size(); i++) {
        int px = LF[i].point().x;
        int py = LF[i].point().y;
        Image desc = LF[i].desc();

        for (int delx = px - rad; delx < px + rad + 1; delx++) {
            for (int dely = py - rad; dely < py + rad + 1; dely++) {
                vim(delx, dely, 0) = 0;
                vim(delx, dely, 1) = 0;
                vim(delx, dely, 2) = 0;

                if (desc(delx - (px-rad), dely - (py - rad)) > 0) {
                    vim(delx, dely, 1) = 1;
                } else if (desc(delx - (px-rad), dely - (py - rad)) < 0) {
                    vim(delx, dely, 0) = 1;
                }
            }
        }
    }
    return vim;
}

void drawLine(const Point &p1,
              const Point &p2,
              Image &im,
              const vector<float> &color) {
    float minx = min(p1.x, p2.x);
    float miny = min(p1.y, p2.y);
    float maxx = max(p1.x, p2.x);
    float maxy = max(p1.y, p2.y);

    int spaces = 1000;
    for (int i = 0; i < spaces; i++) {
        float x = minx + (maxx - minx) / spaces * (i+1);
        float y = miny + (maxy - miny) / spaces * (i+1);
        for (int c = 0; c < im.channels(); c++) {
            im(x, y, c) = color[c];
        }
    }
}

Image visualizePairs(const Image &im1,
                     const Image &im2,
                     const vector<FeatureCorrespondence> &corr) {
    Image vim(im1.width() + im2.width(), im1.height(), im1.channels());

    // stack the images
    for (int j = 0; j < im1.height(); j++) {
        for (int c = 0; c < im1.channels(); c++) {
            for (int i = 0; i < im1.width(); i++) {
                vim(i,j,c) = im1(i,j,c);
            }
            for (int i = 0; i < im2.width(); i++) {
                vim(i+im1.width(),j,c) = im2(i,j,c);
            }
        }
    }

    // draw lines
    for (int i = 0; i < (int) corr.size(); i++) {
        Point p1 = corr[i].feature(0).point();
        Point p2 = corr[i].feature(1).point();
        p2.x = p2.x + im1.width();
        drawLine(p1, p2, vim);
    }
    return vim;
}

Image visualizePairsWithInliers(const Image &im1,
                                const Image &im2,
                                const vector<FeatureCorrespondence> &corr,
                                const vector<bool> & ins) {
    Image vim(im1.width() + im2.width(), im1.height(), im1.channels());

    // stack the images
    for (int j = 0; j < im1.height(); j++) {
        for (int c = 0; c < im1.channels(); c++) {
            for (int i = 0; i < im1.width(); i++) {
                vim(i,j,c) = im1(i,j,c);
            }
            for (int i = 0; i < im2.width(); i++) {
                vim(i+im1.width(),j,c) = im2(i,j,c);
            }
        }
    }

    // draw lines
    vector<float> red(3,0);
    vector<float> green(3,0);
    red[0] = 1.0f;
    green[1]= 1.0f;

    for (int i = 0; i < (int) corr.size(); i++) {
        Point p1 = corr[i].feature(0).point();
        Point p2 = corr[i].feature(1).point();
        p2.x = p2.x + im1.width();
        if (ins[i]) {
            drawLine(p1, p2, vim, green);
        } else {
            drawLine(p1, p2, vim, red);
        }
    }
    return vim;

}

// Inliers:  Detected corners are in green, reprojected ones are in red
// Outliers: Detected corners are in yellow, reprojected ones are in blue
vector<Image> visualizeReprojection(const Image &im1,
                                    const Image &im2,
                                    const Matrix &H,
                                    const vector<FeatureCorrespondence> &corr,
                                    const vector<bool> &ins) {
    // Initialize colors
    vector<float> red(3,0);
    vector<float> green(3,0);
    vector<float> blue(3,0);
    vector<float> yellow(3,0);
    red[0] = 1.0f;
    green[1]= 1.0f;
    blue[2] = 1.0f;
    yellow[0] = 1.0f;
    yellow[1] = 1.0f;

    vector<Point> detectedPts1In;
    vector<Point> projectedPts1In;
    vector<Point> detectedPts1Out;
    vector<Point> projectedPts1Out;

    vector<Point> detectedPts2In;
    vector<Point> projectedPts2In;
    vector<Point> detectedPts2Out;
    vector<Point> projectedPts2Out;

    for (int i = 0 ; i < (int) corr.size(); i++) {
        Point pt1 = corr[i].feature(0).point();
        Point pt2 = corr[i].feature(1).point();
        Matrix P1 = pt1.toHomogenousCoords();
        Matrix P2 = pt2.toHomogenousCoords();
        Matrix P2_proj = H*P1;
        Matrix P1_proj = H.inverse()*P2;
        Point reproj1 = Point(P1_proj(0)/P1_proj(2), P1_proj(1)/P1_proj(2));
        Point reproj2 = Point(P2_proj(0)/P2_proj(2), P2_proj(1)/P2_proj(2));
        if (ins[i]) { // Inlier
            detectedPts1In.push_back(pt1);
            projectedPts1In.push_back(reproj1);
            detectedPts2In.push_back(pt2);
            projectedPts2In.push_back(reproj2);
        } else { // Outlier
            detectedPts1Out.push_back(pt1);
            projectedPts1Out.push_back(reproj1);
            detectedPts2Out.push_back(pt2);
            projectedPts2Out.push_back(reproj2);
        }
    }

    vector<Image> output;
    Image vim1(im1);
    Image vim2(im2);
    vim1 = visualizeCorners(im1, detectedPts1In,2, green);
    vim1 = visualizeCorners(vim1, projectedPts1In,1, red);
    vim1 = visualizeCorners(vim1, detectedPts1Out,2, yellow);
    vim1 = visualizeCorners(vim1, projectedPts1Out,1, blue);

    vim2 = visualizeCorners(im2, detectedPts2In,2, green);
    vim2 = visualizeCorners(vim2, projectedPts2In,1, red);
    vim2 = visualizeCorners(vim2, detectedPts2Out,2, yellow);
    vim2 = visualizeCorners(vim2, projectedPts2Out,1, blue);

    output.push_back(vim1);
    output.push_back(vim2);
    return output;
}



/***********************************************************************
 * Point and Feature Definitions *
 **********************************************************************/
Point::Point(int xp, int yp)
  : x(xp), y(yp) {}

Point::Point()
  : x(0.0f), y(0.0f) {}

void Point::print() const {
  printf("(%d, %d)\n", x, y);
}

Vec3f Point::toHomogenousCoords() const {
  return Vec3f(x, y, 1.0f);
}

// Feature Constructors
Feature::Feature(const Point &ptp, const Image &descp)
    : pt(ptp), dsc(descp) {
    pt = ptp;
    dsc = descp;
}

// getter functions
const Point& Feature::point() const { return pt;}
const Image& Feature::desc() const { return dsc;}

// printer
void Feature::print() const {
    printf("Feature:");
    point().print();
    for (int j = 0; j < dsc.height(); j++) {
        for (int i = 0; i < dsc.width(); i++) {
            printf("%+07.2f ", dsc(i, j));
        }
        printf("\n");
    }
}

// FeatureCorrespondence Constructors
FeatureCorrespondence::FeatureCorrespondence(const Feature &f0p, const Feature &f1p)
    : f0(f0p), f1(f1p) {
}


vector<Feature> FeatureCorrespondence::features() const {
    return vector<Feature>{f0, f1};
}


const Feature& FeatureCorrespondence::feature(int i) const {
    assert(i >= 0 && i < 2);
    if (i == 0)
        return f0;
    else
        return f1;
}


// printer
void FeatureCorrespondence::print() const {
    printf("FeatureCorrespondence:");
    f0.print();
    f1.print();
}


CorrespondencePair FeatureCorrespondence::toCorrespondencePair() const {
    return CorrespondencePair(
        (float) f0.point().x,
        (float) f0.point().y,
        1,
        (float) f1.point().x,
        (float) f1.point().y,
        1
    );
}

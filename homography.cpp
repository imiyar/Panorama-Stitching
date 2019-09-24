#include "homography.h"
#include "matrix.h"

using namespace std;


void applyHomography(const Image &source, const Matrix &H, Image &out, bool bilinear) {
    // // --------- HANDOUT  PS06 ------------------------------
    // Transform image source using the homography H, and composite in onto out.
    // if bilinear == true, using bilinear interpolation. Use nearest neighbor
    // otherwise.
    int width = out.width();
    int height = out.height();
    int channel = source.channels();
    Matrix inv_H = H.inverse();

    for(int i=0; i<width; ++i){
        for(int j=0; j<height; ++j) {
            Vec3f v(i, j, 1);
            Matrix M = inv_H * v;
            float x = (float)M(0, 0) / M(2, 0);
            float y = (float)M(1, 0) / M(2, 0);
            
            for(int c=0; c<channel; ++c) {
                if (bilinear == true) {
                    if (!(x >= source.width() || x < 0 || y >= source.height() || y < 0)) {
                        out(i, j, c) = interpolateLin(source, x, y, c);
                    }
                }
                else {
                    if (!(round(x) >= source.width() || round(x) < 0 || round(y) >= source.height() || round(y) < 0)) {
                        out(i, j, c) = source(round(x), round(y), c);
                    }
                }
            }
        }
    }
}




Matrix computeHomography(const CorrespondencePair correspondences[4]) {
    // --------- HANDOUT  PS06 ------------------------------
    // Compute a homography from 4 point correspondences.
    Matrix M(9, 9);
    M.fill(0);
    M(8, 8) = 1;
    Matrix B(9, 1);
    B.fill(0);
    B(8, 0) = 1;
    for(int i=0; i<4; ++i) {
        M(i*2, 0) = correspondences[i].point1(0);
        M(i*2, 1) = correspondences[i].point1(1);
        M(i*2, 2) = 1;
        M(i*2, 6) = -correspondences[i].point1(0) * correspondences[i].point2(0);
        M(i*2, 7) = -correspondences[i].point1(1) * correspondences[i].point2(0);
        M(i*2, 8) = -correspondences[i].point2(0);
        M(i*2+1, 3) = correspondences[i].point1(0);
        M(i*2+1, 4) = correspondences[i].point1(1);
        M(i*2+1, 5) = 1;
        M(i*2+1, 6) = -correspondences[i].point1(0) * correspondences[i].point2(1);
        M(i*2+1, 7) = -correspondences[i].point1(1) * correspondences[i].point2(1);
        M(i*2+1, 8) = -correspondences[i].point2(1);
    }
    Matrix x = M.fullPivLu().solve(B);
    Matrix output(3, 3);
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            output(i, j) = x(i*3+j, 0);
        }
    }
    return output;
}


BoundingBox computeTransformedBBox(int imwidth, int imheight, Matrix H) {
    // --------- HANDOUT  PS06 ------------------------------
    // Predict the bounding boxes that encompasses all the transformed
    // coordinates for pixels frow and Image with size (imwidth, imheight)
    Vec3f p[4];
    p[0] = Vec3f(0,0,1);
    p[1] = Vec3f(imwidth-1, 0, 1);
    p[2] = Vec3f(0, imheight-1, 1);
    p[3] = Vec3f(imwidth-1, imheight-1, 1);
    int x[4], y[4];
    for(int i=0; i<4; ++i) {
        Matrix B(3, 1);
        B = H * p[i];
        x[i] = (float)B(0, 0) / B(2, 0);
        y[i] = (float)B(1, 0) / B(2, 0);
    }

    return BoundingBox(*min_element(x,x+4),*max_element(x,x+4),*min_element(y,y+4),*max_element(y,y+4));
}


BoundingBox bboxUnion(BoundingBox B1, BoundingBox B2) {
    // --------- HANDOUT  PS06 ------------------------------
    // Compute the bounding box that tightly bounds the union of B1 an B2.
    
    BoundingBox B = BoundingBox(min(B1.x1, B2.x1),max(B1.x2, B2.x2),min(B1.y1, B2.y1),max(B1.y2, B2.y2));
    return B;

}


Matrix makeTranslation(BoundingBox B) {
    // --------- HANDOUT  PS06 ------------------------------
    // Compute a translation matrix (as a homography matrix) that translates the
    // top-left corner of B to (0,0).
    Matrix output(3, 3);
    output <<
        1, 0, -B.x1,
        0, 1, -B.y1,
        0, 0, 1;

    return output;
}


Image stitch(const Image &im1, const Image &im2, const CorrespondencePair correspondences[4]) {
    // --------- HANDOUT  PS06 ------------------------------
    // Transform im1 to align with im2 according to the set of correspondences.
    // make sure the union of the bounding boxes for im2 and transformed_im1 is
    // translated properly (use makeTranslation)
    int imwidth = im1.width();
    int imheight = im1.height();

    Matrix H = computeHomography(correspondences);
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

// debug-useful
Image drawBoundingBox(const Image &im, BoundingBox bbox) {
    // // --------- HANDOUT  PS06 ------------------------------
    /*
      ________________________________________
     / Draw me a bounding box!                \
     |                                        |
     | "I jumped to my                        |
     | feet, completely thunderstruck. I      |
     | blinked my eyes hard. I looked         |
     | carefully all around me. And I saw a   |
     | most extraordinary small person, who   |
     | stood there examining me with great    |
     | seriousness."                          |
     \              Antoine de Saint-Exupery  /
      ----------------------------------------
             \   ^__^
              \  (oo)\_______
                 (__)\       )\/\
                     ||----w |
                     ||     ||
    */
    Image output = im;
    output.create_line(bbox.x1, bbox.y1, bbox.x2, bbox.y1);
    output.create_line(bbox.x1, bbox.y1, bbox.x1, bbox.y2);
    output.create_line(bbox.x2, bbox.y1, bbox.x2, bbox.y2);
    output.create_line(bbox.x1, bbox.y2, bbox.x2, bbox.y2);
    return output;
}

void applyHomographyFast(const Image &source, const Matrix &H, Image &out, bool bilinear) {
    // // --------- HANDOUT  PS06 ------------------------------
    // Same as apply but change only the pixels of out that are within the
    // predicted bounding box (when H maps source to its new position).
    int imwidth = source.width();
    int imheight = source.height();
    int channel = source.channels();
    BoundingBox B1 = computeTransformedBBox(imwidth, imheight, H);
    Matrix inv_H = H.inverse();
    
    for(int i=B1.x1; i<=B1.x2; ++i){
        for(int j=B1.y1; j<=B1.y2; ++j) {
            Vec3f v(i, j, 1);
            Matrix M = inv_H * v;
            float x = (float)M(0, 0) / M(2, 0);
            float y = (float)M(1, 0) / M(2, 0);
            
            for(int c=0; c<channel; ++c) {
                if (bilinear == true) {
                    if (!(x >= source.width() || x < 0 || y >= source.height() || y < 0)) {
                        out(i, j, c) = interpolateLin(source, x, y, c);
                    }
                }
                else {
                    if (!(round(x) >= source.width() || round(x) < 0 || round(y) >= source.height() || round(y) < 0)) {
                        out(i, j, c) = source(round(x), round(y), c);
                    }
                }
            }
        }
    }
}

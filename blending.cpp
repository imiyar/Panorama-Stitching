#include "blending.h"
#include "matrix.h"
#include <ctime>

using namespace std;

Image blendingweight(int imwidth, int imheight) {
    // // --------- HANDOUT  PS07 ------------------------------
	Image output = Image(imwidth, imheight, 1);
	float centerX = imwidth / 2.0f;
	float centerY = imheight / 2.0f;

	for (int i=0; i<imwidth; ++i) {
		for (int j=0; j<imheight; ++j){
			float x = 1 - abs(centerX - (i+0.5))/ centerX;
			float y = 1 - abs(centerY - (j+0.5)) / centerY;
			output(i, j) = x * y;
		}      
	}

	return output;
}

//  ****************************************************************************
//  * blending related functions re-written from previous assignments
//  ****************************************************************************

// instead of writing source in out, *add* the source to out based on the weight
// so out(x,y) = out(x, y) + weight * image
void applyhomographyBlend(const Image &source,
	const Image &weight,
	Image &out,
	const Matrix &H,
	bool bilinear) {
    // // --------- HANDOUT  PS07 ------------------------------
	int width = source.width();
	int height = source.height();
	int channels = source.channels();
	BoundingBox B1 = computeTransformedBBox(width, height, H);
	Matrix inv_H = H.inverse();

	Image weighted_source = Image(width, height, channels);
	for (int i=0; i<width; ++i) {
		for (int j=0; j<height; ++j) {
			for (int c=0; c<channels; ++c) {
				weighted_source(i, j, c) = source(i, j, c) * weight(i, j);
			}
		}
	}

	for(int i=B1.x1; i<=B1.x2; ++i)
		for(int j=B1.y1; j<=B1.y2; ++j) {
			Vec3f v(i, j, 1);
			Matrix M = inv_H * v;
			float x = (float)M(0, 0) / M(2, 0);
			float y = (float)M(1, 0) / M(2, 0);

			for(int c=0; c<channels; ++c) {
				if (bilinear == true) {
					if (!(x >= source.width() || x < 0 || y >= source.height() || y < 0)) {
						out(i, j, c) = out(i, j, c) + interpolateLin(weighted_source, x, y, c);
					}
				}
				else {
					if (!(round(x) >= source.width() || round(x) < 0 || round(y) >= source.height() || round(y) < 0)) {
						out(i, j, c) = out(i, j, c) + weighted_source(round(x), round(y), c);
					}
				}
			}
		}
	}

	Image stitchLinearBlending(const Image &im1,
		const Image &im2,
		const Image &we1,
		const Image &we2,
		const Matrix &H) {
    // // --------- HANDOUT  PS07 ------------------------------
    // stitch using image weights.
    // note there is no weight normalization.

		BoundingBox B1 = computeTransformedBBox(im1.width(), im1.height(), H);
		BoundingBox B2 = BoundingBox(0, im2.width()-1, 0, im2.height()-1);
		BoundingBox B = bboxUnion(B1, B2);
		Matrix T = makeTranslation(B);

		Image output = Image(B.x2-B.x1+1, B.y2-B.y1+1, im1.channels());

		applyhomographyBlend(im1, we1, output, T*H);
		applyhomographyBlend(im2, we2, output, T);

		return output;
	}


/*****************************************************************************
 * blending functions Pset 08
 *****************************************************************************/


// low freq and high freq (2-scale decomposition)
	vector<Image> scaledecomp(const Image &im, float sigma) {
		vector <Image> ims;
		ims.push_back(gaussianBlur_separable(im, sigma));
		ims.push_back(im - ims[0]);
		return ims;
	}

// stitch using different blending models
// blend can be 0 (none), 1 (linear) or 2 (2-layer)
	Image stitchBlending(const Image &im1,
		const Image &im2,
		const Matrix &H,
		BlendType blend) {
    // // --------- HANDOUT  PS07 ------------------------------
		Image one_map_1 = Image(im1.width(), im1.height());
		Image one_map_2 = Image(im2.width(), im2.height());
		one_map_1.set_color(1, 1, 1);
		one_map_2.set_color(1, 1, 1);

		if (blend == BlendType::BLEND_NONE) {
			BoundingBox B1 = computeTransformedBBox(im1.width(), im1.height(), H);
			BoundingBox B2 = BoundingBox(0, im2.width()-1, 0, im2.height()-1);
			BoundingBox B = bboxUnion(B1, B2);
			Matrix T = makeTranslation(B);

			Image output = Image(B.x2-B.x1+1, B.y2-B.y1+1, im2.channels());

			applyHomography(im1, T*H, output);
			applyHomography(im2, T, output);

			return output;
		}

		else if (blend == BlendType::BLEND_LINEAR) {
			Image we1 = blendingweight(im1.width(), im1.height());
			Image we2 = blendingweight(im2.width(), im2.height());
			Image output = stitchLinearBlending(im1, im2, we1, we2, H);

			Image weight_sum = stitchLinearBlending(we1, we2, one_map_1, one_map_2, H);

			for (int i=0; i<output.width(); ++i) 
				for (int j=0; j<output.height(); ++j) {
					if (weight_sum(i, j) != 0) {
						for (int c=0; c<output.channels(); ++c) {
							output(i, j, c) = output(i, j, c) / weight_sum(i, j);
						}
					}
				}
			return output;
		}

		else if (blend == BlendType::BLEND_2LAYER) {
			Image low_warp = stitchBlending(scaledecomp(im1, 2)[0], scaledecomp(im2, 2)[0], H, BlendType::BLEND_LINEAR);
			int width = low_warp.width();
			int height = low_warp.height();
			int channels = low_warp.channels();

			Image high1 = scaledecomp(im1, 2)[1];
			Image high2 = scaledecomp(im2, 2)[1];
			Image we1 = blendingweight(im1.width(), im1.height());
			Image we2 = blendingweight(im2.width(), im2.height());
			Image high_warp = Image(width, height, channels);

			high1 = stitchLinearBlending(high1, Image(im2.width(), im2.height(), 3), one_map_1, one_map_2, H);
			high2 = stitchLinearBlending(Image(im1.width(), im1.height(), 3), high2, one_map_1, one_map_2, H);
			we1 = stitchLinearBlending(we1, Image(im2.width(), im2.height(), 1), one_map_1, one_map_2, H);
			we2 = stitchLinearBlending(Image(im1.width(), im1.height(), 1), we2, one_map_1, one_map_2, H);  

			for (int i=0; i<width; ++i) 
				for (int j=0; j<height; ++j) {
					if (we1(i, j) >= we2(i, j)) {
						for (int c=0; c<channels; ++c) {
							high_warp(i, j, c) = high1(i, j, c);
						}
					}
					else {
						for (int c=0; c<channels; ++c) {
							high_warp(i, j, c) = high2(i, j, c);
						}
					}
				}
			return low_warp + high_warp;
		}
		return Image(1, 1, 1);
	}

// auto stitch
	Image autostitch(const Image &im1,
		const Image &im2,
		BlendType blend,
		float blurDescriptor,
		float radiusDescriptor) {
    // // --------- HANDOUT  PS07 ------------------------------
		vector<Point> corners1 = HarrisCorners(im1);
		vector<Point> corners2 = HarrisCorners(im2);
		vector<Feature> features1 = computeFeatures(im1, corners1,blurDescriptor, radiusDescriptor);
		vector<Feature> features2 = computeFeatures(im2, corners2,blurDescriptor, radiusDescriptor);

		vector<FeatureCorrespondence> correspondences = findCorrespondences(features1, features2);
		Matrix H = RANSAC(correspondences);

		Image output = stitchBlending(im1, im2, H, blend);

		return output;
	}

/************************************************************************
 * Tricks: mini planets.
 ************************************************************************/

	Image pano2planet(const Image &pano, int newImSize, bool clamp) {
    // // --------- HANDOUT  PS07 ------------------------------
		int width = pano.width();
		int height = pano.height();
		int channels = pano.channels();

		Image output = Image(newImSize, newImSize, channels);
		float center = (newImSize-1) / 2.0f;

		for (int i=0; i<newImSize; ++i) {
			for (int j=0; j<newImSize; ++j) {
				float radius = pow((i-center)*(i-center)+(j-center)*(j-center), 0.5);
				float angle = atan2(-(j-center), (i-center)) / 3.1415926 * 180;
				if (angle < 0) angle += 360;
				for (int c=0; c<channels; ++c) {
					output(i, j, c) = interpolateLin(pano, angle / 360 * (width-1), (height-1)-radius*(height-1)/center, c, clamp);
				}

			}
		}
		return output;
	}


/************************************************************************
 * 6.865: Stitch N images into a panorama
 ************************************************************************/

// Pset07-865. Compute sequence of N-1 homographies going from Im_i to Im_{i+1}
// Implement me!
	vector<Matrix> sequenceHs(const vector<Image> &ims,
		float blurDescriptor,
		float radiusDescriptor) {
    // // --------- HANDOUT  PS07 ------------------------------
		std::vector<Matrix> Hs = {};
		for (unsigned int i=0; i<ims.size()-1; ++i) {
			vector<Point> corners1 = HarrisCorners(ims[i]);
    		vector<Point> corners2 = HarrisCorners(ims[i+1]);
    		vector<Feature> features1 = computeFeatures(ims[i], corners1,blurDescriptor, radiusDescriptor);
    		vector<Feature> features2 = computeFeatures(ims[i+1], corners2,blurDescriptor, radiusDescriptor);

    		vector<FeatureCorrespondence> correspondences = findCorrespondences(features1, features2);
    		Hs.push_back(RANSAC(correspondences, 1000));
		}
		return Hs;
	}

// stack homographies:
//   transform a list of (N-1) homographies that go from I_i to I_i+1
//   to a list of N homographies going from I_i to I_refIndex.
	vector<Matrix> stackHomographies(const vector<Matrix> &Hs, int refIndex) {
    // // --------- HANDOUT  PS07 ------------------------------
		int N = Hs.size()+1;
		std::vector<Matrix> output(N, Matrix(3, 3));
		output[refIndex] = Matrix::Identity(3, 3);

		for (int i=refIndex-1; i>=0; --i) {
			output[i] = Hs[i] * output[i+1];
		}
		for (int i=refIndex+1; i<N; ++i) {
			output[i] = output[i-1] * Hs[i-1].inverse();
		}

		return output;
	}


// Pset07-865: compute bbox around N images given one main reference.
	BoundingBox bboxN(const vector<Matrix> &Hs, const vector<Image> &ims) {
    // // --------- HANDOUT  PS07 ------------------------------
		int N = ims.size();
		BoundingBox B_all = BoundingBox(0,0,0,0);
		for (int i=0; i<N; ++i) {
    		BoundingBox B = computeTransformedBBox(ims[i].width(), ims[i].height(), Hs[i]);
    		B_all = bboxUnion(B_all, B);
		}    	
		return B_all;
	}

// Pset07-865.
// Implement me!
	Image autostitchN(const vector<Image> &ims,
		int refIndex,
		float blurDescriptor,
		float radiusDescriptor) {
    // // --------- HANDOUT  PS07 ------------------------------
		vector<Matrix> Hs = sequenceHs(ims, blurDescriptor, radiusDescriptor);
		vector<Matrix> stackHs = stackHomographies(Hs, refIndex);
		BoundingBox B = bboxN(stackHs, ims);
		Matrix T = makeTranslation(B);
		int N = ims.size();

		Image output = Image(B.x2-B.x1+1, B.y2-B.y1+1, ims[0].channels());
		Image weight_sum = Image(B.x2-B.x1+1, B.y2-B.y1+1, 1);

		for (int i=0; i<N; ++i) {
			Image we = blendingweight(ims[i].width(), ims[i].height());
			Image one_map = Image(ims[i].width(), ims[i].height());
			one_map.set_color(1, 1, 1);

			applyhomographyBlend(ims[i], we, output, T*stackHs[i]);
			applyhomographyBlend(we, one_map, weight_sum, T*stackHs[i]);
		}

		for (int i=0; i<output.width(); ++i) 
			for (int j=0; j<output.height(); ++j) {
				if (weight_sum(i, j) != 0) {
					for (int c=0; c<output.channels(); ++c) {
						output(i, j, c) = output(i, j, c) / weight_sum(i, j);
					}
				}
			}
		return output;
	}


/******************************************************************************
 * Helper functions
 *****************************************************************************/

// copy a single-channeled image to several channels
	Image copychannels(const Image &im, int nChannels) {
    // image must have one channel
		assert(im.channels() == 1);
		Image oim(im.width(), im.height(), nChannels);

		for (int i = 0; i < im.width(); i++) {
			for (int j = 0; j < im.height(); j++) {
				for (int c = 0; c < nChannels; c++) {
					oim(i, j, c) = im(i, j);
				}
			}
		}
		return oim;
	}


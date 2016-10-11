/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2016 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 *FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/core/util/logcentral.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/tnm067lab1/processors/imageupsampler.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming
// scheme
const ProcessorInfo ImageUpsampler::processorInfo_{
    "org.inviwo.imageupsampler",  // Class identifier
    "Image Upsampler",            // Display name
    "TNM067",                     // Category
    CodeState::Experimental,      // Code state
    Tags::None,                   // Tags
};
const ProcessorInfo ImageUpsampler::getProcessorInfo() const { return processorInfo_; }

ImageUpsampler::ImageUpsampler()
    : Processor()
    , imageInport_("inport", false)
    , grayscaleImageOutport_("outport", DataFloat32::get(), false)
    , colorImageOutport_("coloroutport", DataUInt8::get(), false)
    , samplerSize_("samplerSize", "Sampler Size")
    , interpolationMethod_("interpolationMethod", "Interpolation Method")
    , pixelIntensityScaleFactor_("pixelIntensityScaleFactor", "IntensityScale Factor", 1.0, 0.001,
                                 2.0, 0.001) {
    addPort(imageInport_);
    addPort(grayscaleImageOutport_);
    addPort(colorImageOutport_);

    samplerSize_.addOption("1x1", "1X1", 1);
    samplerSize_.addOption("2x2", "2X2", 2);
    samplerSize_.addOption("4x4", "4X4", 4);
    samplerSize_.addOption("8x8", "8X8", 8);

    addProperty(samplerSize_);

    interpolationMethod_.addOption("piecewiseconstant", "Piecewise Constant", 1);
    interpolationMethod_.addOption("bilinear", "Bilinear", 2);
    interpolationMethod_.addOption("quadratic", "Quadratic", 3);
    interpolationMethod_.addOption("barycentric", "Barycentric", 4);
    interpolationMethod_.setCurrentStateAsDefault();
    addProperty(interpolationMethod_);

    addProperty(pixelIntensityScaleFactor_);
}

void ImageUpsampler::process() {
    auto imageIn = imageInport_.getData();
    auto inSize = imageInport_.getData()->getDimensions();
    auto outDim = inSize * size2_t(samplerSize_.get());

    auto grayscaleOutputImage =
        std::make_shared<Image>(outDim, imageInport_.getData()->getDataFormat());
    imageInport_.getData()->copyRepresentationsTo(grayscaleOutputImage.get());

    switch (interpolationMethod_.get()) {
        case 1:
            piecewiseConstant(imageIn.get(), grayscaleOutputImage.get());
            break;
        case 2:
            bilinearInterpolation(imageIn.get(), grayscaleOutputImage.get());
            break;
        case 3:
            quadraticInterpolation(imageIn.get(), grayscaleOutputImage.get());
            break;
        case 4:
            barycentricInterpolation(imageIn.get(), grayscaleOutputImage.get());
            break;
    };

    auto colorscaleImage = std::make_shared<Image>(outDim, DataVec4UInt8::get());
    applyColorMap(grayscaleOutputImage.get(), colorscaleImage.get());

    grayscaleImageOutport_.setData(grayscaleOutputImage);
    colorImageOutport_.setData(colorscaleImage);
}

void ImageUpsampler::piecewiseConstant(const Image *inputImage, Image *outputImage) {
    auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
    auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

    auto sampleSize = samplerSize_.get();
    auto inputSize = inputImage->getDimensions();
    auto outputSize = out_img->getDimensions();
	auto imgScaleX = (float)inputSize.x / (float)outputSize.x;
	auto imgScaleY = (float)inputSize.y / (float)outputSize.y;

    for (size_t i = 0; i < outputSize.x; i++) {
        for (size_t j = 0; j < outputSize.y; j++) {
            // TODO: Task 3: Updated this code to use piecewise constant interpolation
			// e.g set nearest neighbor
            //size2_t mappedIndex(i, j);
			size2_t mappedIndex((i*imgScaleX), (j*imgScaleY));
            mappedIndex = glm::clamp(mappedIndex, size2_t(0), inputSize - size_t(1));

            // get pixel from input image at pixel coordinate mappedIndex
            auto pixel_intensity = in_img->getAsNormalizedDouble(mappedIndex);

            out_img->setFromNormalizedDVec4(
                size2_t(i, j),
                dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
        }
    }
}

void ImageUpsampler::bilinearInterpolation(const Image *inputImage, Image *outputImage) {
    auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
    auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

    auto sampleSize = samplerSize_.get();
    auto inputSize = inputImage->getDimensions();
    auto outputSize = out_img->getDimensions();
	auto imgScaleX = (float)inputSize.x / (float)outputSize.x;
	auto imgScaleY = (float)inputSize.y / (float)outputSize.y;

    for (size_t i = 0; i < outputSize.x; i++) {
        for (size_t j = 0; j < outputSize.y; j++) {
            // TODO: Task 4: Updated this code to use bilinear interpolation
			//auto x = i*imgScaleX;
			//auto y = j*imgScaleY;
			float x = static_cast<float>(i) / (static_cast<float>(outputSize.x) - 1.f);
			float y = static_cast<float>(j) / (static_cast<float>(outputSize.y) - 1.f);

			x *= static_cast<float>(inputSize.x) - 1.f;
			y *= static_cast<float>(inputSize.y) - 1.f;
			auto r = floor(x);
			auto c = floor(y);
			size2_t mappedIndexQ11(r, c);
			size2_t mappedIndexQ12(r, c + 1);
            size2_t mappedIndexQ21(r + 1, c);
			size2_t mappedIndexQ22(r + 1, c + 1);

			auto mappedIndex1 = glm::clamp(mappedIndexQ11, size2_t(0), inputSize - size_t(1));
			auto mappedIndex2 = glm::clamp(mappedIndexQ12, size2_t(0), inputSize - size_t(1));
            auto mappedIndex3 = glm::clamp(mappedIndexQ21, size2_t(0), inputSize - size_t(1));
			auto mappedIndex4 = glm::clamp(mappedIndexQ22, size2_t(0), inputSize - size_t(1));

			auto xIntensity1 = ((r + 1 - x) / (r + 1 - r)) * in_img->getAsNormalizedDouble(mappedIndex1) + 
				((x - r) / (r + 1 - r)) * in_img->getAsNormalizedDouble(mappedIndex3);

			auto xIntensity2 = ((r + 1 - x) / (r + 1 - r)) * in_img->getAsNormalizedDouble(mappedIndex2) +
				((x - r) / (r + 1 - r)) * in_img->getAsNormalizedDouble(mappedIndex4);

            // get pixel from input image at pixel coordinate mappedIndex
            auto pixel_intensity = ((c + 1 - y) / (c + 1 - c)) * xIntensity1 +
				((y - c) / (c + 1 - c)) * xIntensity2;

            out_img->setFromNormalizedDVec4(
                size2_t(i, j),
                dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
        }
    }
}

void ImageUpsampler::quadraticInterpolation(const Image *inputImage, Image *outputImage) {
    auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
    auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

    auto sampleSize = samplerSize_.get();
    auto inputSize = inputImage->getDimensions();
    auto outputSize = out_img->getDimensions();
	auto imgScaleX = (float)inputSize.x / (float)outputSize.x;
	auto imgScaleY = (float)inputSize.y / (float)outputSize.y;

    for (size_t i = 0; i < outputSize.x; i++) {
        for (size_t j = 0; j < outputSize.y; j++) {
            // TODO: Task 5: Updated this code to use quadratic interpolation
			//auto x = i*imgScaleX;
			//auto y = j*imgScaleY;
			float x = static_cast<float>(i) / (static_cast<float>(outputSize.x) - 1.f);
			float y = static_cast<float>(j) / (static_cast<float>(outputSize.y) - 1.f);

			x *= static_cast<float>(inputSize.x) - 1.f;
			y *= static_cast<float>(inputSize.y) - 1.f;
			auto r = floor(x);
			auto c = floor(y);
			r = glm::clamp(r, 0.f, (float) inputSize.x);
			c = glm::clamp(c, 0.f, (float)inputSize.y);

			size2_t mappedIndex1(r - 1, c - 1);
			size2_t mappedIndex2(r, c - 1);
			size2_t mappedIndex3(r + 1, c - 1);
			size2_t mappedIndex4(r - 1, c);
			size2_t mappedIndex5(r, c);
			size2_t mappedIndex6(r + 1, c);
			size2_t mappedIndex7(r - 1, c + 1);
			size2_t mappedIndex8(r, c + 1);
			size2_t mappedIndex9(r + 1, c + 1);

			if (r - 1 < 0) {
				mappedIndex1 = mappedIndex2;
				mappedIndex4 = mappedIndex5;
				mappedIndex7 = mappedIndex8;

			} else if (c - 1 < 0) {
				mappedIndex2 = mappedIndex5;
				mappedIndex3 = mappedIndex6;
			}

			mappedIndex1 = glm::clamp(mappedIndex1, size2_t(0), inputSize - size_t(1));
			mappedIndex2 = glm::clamp(mappedIndex2, size2_t(0), inputSize - size_t(1));
			mappedIndex3 = glm::clamp(mappedIndex3, size2_t(0), inputSize - size_t(1));
			mappedIndex4 = glm::clamp(mappedIndex4, size2_t(0), inputSize - size_t(1));
			mappedIndex5 = glm::clamp(mappedIndex5, size2_t(0), inputSize - size_t(1));
			mappedIndex6 = glm::clamp(mappedIndex6, size2_t(0), inputSize - size_t(1));
			mappedIndex7 = glm::clamp(mappedIndex7, size2_t(0), inputSize - size_t(1));
			mappedIndex8 = glm::clamp(mappedIndex8, size2_t(0), inputSize - size_t(1));
			mappedIndex9 = glm::clamp(mappedIndex9, size2_t(0), inputSize - size_t(1));

			auto t = (1 + x - r) / 2;
			auto b1 = (1 - t) * (1 - (2 * t));
			auto b2 = (4 * t) * (1 - t);
			auto b3 = t * ((2 * t) - 1);
			auto f1 = b1 * in_img->getAsNormalizedDouble(mappedIndex1) + b2 * in_img->getAsNormalizedDouble(mappedIndex2) + b3 * in_img->getAsNormalizedDouble(mappedIndex3);
			auto f2 = b1 * in_img->getAsNormalizedDouble(mappedIndex4) + b2 * in_img->getAsNormalizedDouble(mappedIndex5) + b3 * in_img->getAsNormalizedDouble(mappedIndex6);
			auto f3 = b1 * in_img->getAsNormalizedDouble(mappedIndex7) + b2 * in_img->getAsNormalizedDouble(mappedIndex8) + b3 * in_img->getAsNormalizedDouble(mappedIndex9);
			
			// get pixel from input image at pixel coordinate mappedIndex
			t = (1 + y - c) / 2;
			b1 = (1 - t) * (1 - (2 * t));
			b2 = ((4 * t) * (1 - t));
			b3 = t * ((2 * t) - 1);

            auto pixel_intensity = (b1 * f1) + (b2 * f2) + (b3 * f3);
            
            out_img->setFromNormalizedDVec4(
                size2_t(i, j),
                dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
        }
    }
}
/*
void ImageUpsampler::barycentricInterpolation(const Image *inputImage, Image *outputImage) {
    auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
    auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

    auto sampleSize = samplerSize_.get();
    auto inputSize = inputImage->getDimensions();
    auto outputSize = out_img->getDimensions();
	auto imgScaleX = (float)inputSize.x / (float)outputSize.x;
	auto imgScaleY = (float)inputSize.y / (float)outputSize.y;

    for (size_t i = 0; i < outputSize.x; i++) {
        for (size_t j = 0; j < outputSize.y; j++) {
            // TODO: Task 6: Updated this code to use barycentric interpolation
			auto x = i*imgScaleX;
			auto y = j*imgScaleY;
			auto r = floor(x);
			auto c = floor(y);

			auto vecA = glm::vec3(r, c + 1, 0);
			auto vecB = glm::vec3(r, c, 0);
			auto vecC = glm::vec3(r + 1, c + 1, 0);
			auto vecD = glm::vec3(r + 1, c, 0);
			auto vecP = glm::vec3(x, y, 0);

			auto normal = glm::cross(vecB - vecC, vecA - vecC);

			auto areaABC = glm::length(glm::cross(vecB - vecA, vecC - vecA))/2.f;
			auto areaPBC = glm::length(glm::cross(vecP - vecB, vecC - vecB))/2.f;
			auto areaPCA = glm::length(glm::cross(vecC - vecP, vecA - vecB))/2.f;


			size2_t mappedIndexA(r, c + 1);
			size2_t mappedIndexB(r, c);
			size2_t mappedIndexC(r + 1, c + 1);
			size2_t mappedIndexD(r + 1, c);

			mappedIndexA = glm::clamp(mappedIndexA, size2_t(0), inputSize - size_t(1));
			mappedIndexB = glm::clamp(mappedIndexB, size2_t(0), inputSize - size_t(1));
			mappedIndexC = glm::clamp(mappedIndexC, size2_t(0), inputSize - size_t(1));
			mappedIndexD = glm::clamp(mappedIndexD, size2_t(0), inputSize - size_t(1));

			//Barycentric coordinates
			auto baryAlpha = areaPBC / areaABC;
			auto baryBeta = areaPCA / areaABC;
			auto baryGamma = 1 - baryAlpha - baryBeta;

			auto pixel_intensity = 0.0f;

			//if (baryAlpha >= 0 && baryAlpha <= 1 && baryBeta >= 0 && baryBeta <= 1 && baryAlpha + baryBeta <= 1) {
			if ((x <= r)  && (x <= r + 1.f) && (y < (x + c - r))) {
				// get pixel from input image at pixel coordinate mappedIndex
				pixel_intensity = in_img->getAsNormalizedDouble(mappedIndexA) * baryAlpha +
									   in_img->getAsNormalizedDouble(mappedIndexB) * baryBeta +
									   in_img->getAsNormalizedDouble(mappedIndexC) * baryGamma;
			}
			else {

				normal = glm::cross(vecD - vecC, vecB - vecC);
				auto areaBCD = glm::length(glm::cross(vecB - vecD, vecC - vecD))/2.f;
				auto areaPBC = glm::length(glm::cross(vecB - vecP, vecC - vecP))/2.f;
				auto areaPCD = glm::length(glm::cross(vecD - vecP, vecD - vecC))/2.f;

				//Barycentric coordinates
				baryAlpha = areaPBC / areaBCD;
				baryBeta = areaPCD / areaBCD;
				baryGamma = 1 - baryAlpha - baryBeta;

				// get pixel from input image at pixel coordinate mappedIndex
				pixel_intensity = in_img->getAsNormalizedDouble(mappedIndexA) * baryAlpha +
					in_img->getAsNormalizedDouble(mappedIndexB) * baryBeta +
					in_img->getAsNormalizedDouble(mappedIndexC) * baryGamma;

			}


            out_img->setFromNormalizedDVec4(
                size2_t(i, j),
                dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
        }
    }
}*/
void ImageUpsampler::barycentricInterpolation(const Image *inputImage, Image *outputImage) {
	auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
	auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

	auto sampleSize = samplerSize_.get();
	auto inputSize = inputImage->getDimensions();
	auto outputSize = out_img->getDimensions();
	auto imgScaleX = (float)inputSize.x / (float)outputSize.x;
	auto imgScaleY = (float)inputSize.y / (float)outputSize.y;

	for (size_t i = 0; i < outputSize.x; i++) {
		for (size_t j = 0; j < outputSize.y; j++) {
			// TODO: Task 6: Updated this code to use barycentric interpolation

			//float x = i*imgScaleX;
			//float y = j*imgScaleY;
			float x = static_cast<float>(i) / (static_cast<float>(outputSize.x) - 1.f);
			float y = static_cast<float>(j) / (static_cast<float>(outputSize.y) - 1.f);

			x *= static_cast<float>(inputSize.x) - 1.f;
			y *= static_cast<float>(inputSize.y) - 1.f;

			size2_t A(floor(x), floor(y));
			size2_t B(floor(x) + 1.f, floor(y));
			size2_t C(floor(x), floor(y) + 1.f);
			size2_t D(floor(x) + 1.f, floor(y) + 1.f);

			A = glm::clamp(A, size2_t(0), inputSize - size_t(1));
			B = glm::clamp(B, size2_t(0), inputSize - size_t(1));
			C = glm::clamp(C, size2_t(0), inputSize - size_t(1));
			D = glm::clamp(D, size2_t(0), inputSize - size_t(1));

			glm::vec3 vA = glm::vec3(floor(x), floor(y), 0);
			glm::vec3 vB = glm::vec3(floor(x) + 1.f, floor(y), 0);
			glm::vec3 vC = glm::vec3(floor(x), floor(y) + 1.f, 0);
			glm::vec3 vD = glm::vec3(floor(x) + 1.f, floor(y) + 1.f, 0);
			glm::vec3 vP = glm::vec3(x, y, 0);

			glm::vec3 u = vB - vA;
			glm::vec3 v = vC - vA;
			glm::vec3 w = vP - vA;

			glm::vec3 vCrossW = glm::cross(v, w);
			glm::vec3 vCrossU = glm::cross(v, u);
			glm::vec3 uCrossW = glm::cross(u, w);
			glm::vec3 uCrossV = glm::cross(u, v);
			float denom = glm::length(uCrossV);

			auto rtest = glm::dot(vCrossW, vCrossU);
			auto ttest = glm::dot(uCrossW, uCrossV);
			auto r = glm::length(vCrossW) / denom;
			auto t = glm::length(uCrossW) / denom;

			auto fA = in_img->getAsNormalizedDouble(A);
			auto fB = in_img->getAsNormalizedDouble(B);
			auto fC = in_img->getAsNormalizedDouble(C);

			// P is not in triangle
			if ((rtest < 0 && ttest < 0) || ((r + t) > 1)) {
				fA = in_img->getAsNormalizedDouble(D);
				glm::vec3 u = vB - vD;
				glm::vec3 v = vC - vD;
				glm::vec3 w = vP - vD;

				glm::vec3 vCrossW = glm::cross(v, w);
				glm::vec3 vCrossU = glm::cross(v, u);
				glm::vec3 uCrossW = glm::cross(u, w);
				glm::vec3 uCrossV = glm::cross(u, v);
				float denom = glm::length(uCrossV);

				rtest = glm::dot(vCrossW, vCrossU);
				ttest = glm::dot(uCrossW, uCrossV);
				r = glm::length(vCrossW) / denom;
				t = glm::length(uCrossW) / denom;
				if ((rtest < 0 && ttest < 0) || ((r + t) > 1)) {
					// Something is shitty
				}
			}

			auto pixel_intensity = (1 - r - t) * fA + r * fB + t * fC;

			out_img->setFromNormalizedDVec4(
				size2_t(i, j),
				dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
		}
	}
}

void ImageUpsampler::applyColorMap(const Image *inputImage, Image *outputImage) {
	auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
	auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

	auto samplesize = samplerSize_;
	auto inputSize = inputImage->getDimensions();
	auto outputSize = out_img->getDimensions();

	for (size_t i = 0; i < outputSize.x; i++) {
		for (size_t j = 0; j < outputSize.y; j++) {
			auto pixel_intensity =
				in_img->getAsNormalizedDouble(size2_t(i, j));  // get from input image
															   // scale factor for debugging
			pixel_intensity *= pixelIntensityScaleFactor_.get();

			auto scalarColor = scalarColorMapping_.sample(pixel_intensity);
			scalarColor.w = 1.0f;

			out_img->setFromNormalizedDVec4(size2_t(i, j),
				dvec4(scalarColor));  // set to output image
		}
	}
}

}  // namespace

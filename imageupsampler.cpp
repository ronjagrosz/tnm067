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
			auto x = i*imgScaleX;
			auto y = j*imgScaleY;
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
			/*auto x = i*imgScaleX;
			auto y = j*imgScaleY;
			auto t = ;
			auto b1 = (1 - t) * (1 - 2 * t);
			auto b2 = (4 * t) * (1 - t);
			auto b3 = t * (2 * t - 1);
			size2_t mappedIndexQ11(r, c);
			size2_t mappedIndexQ12(r, c + 1);
			size2_t mappedIndexQ21(r + 1, c);*/

            size2_t mappedIndex(i, j);
            mappedIndex = glm::clamp(mappedIndex, size2_t(0), inputSize - size_t(1));

            // get pixel from input image at pixel coordinate mappedIndex
            auto pixel_intensity = in_img->getAsNormalizedDouble(mappedIndex);

            out_img->setFromNormalizedDVec4(
                size2_t(i, j),
                dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
        }
    }
}

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

			auto p0x = r;
			auto p0y = c;

			auto p1x = r + 1;
			auto p1y = c;

			auto p2x = r;
			auto p2y = c + 1;
			auto areaABC = 0.5;
			auto areaPBC = sqrt(2) * ;
			auto areaPCA = (1 * (1 - (y - p0y))) / 2;

			if (y > ((x / c) + c * (1 - r))) {
				auto p2x = r + 1;
				auto p2y = c + 1;

				auto areaABC = 0.5;
				auto areaPBC = (1 * (p0x - x)) / 2;
				auto areaPCA = (1 * (1 - (y - p0y))) / 2;
			}
	
			//New points
			size2_t mappedIndexT1(p0x, p0y);
			size2_t mappedIndexT3(p1x, p1y);
			size2_t mappedIndexT2(p2x, p2y);

			//Barycentric coordinates
			auto baryX = areaPBC / areaABC;
			auto baryY = areaPCA / areaABC;
			auto baryZ = 1 - baryX - baryY;




			auto Area = 1 / 2 * (-p1y*p2x + p0y*(-p1x + p2x) + p0x*(p1y - p2y) + p1x*p2y);
			auto s = 1 / (2 * Area)*(p0y*p2x - p0x*p2y + (p2y - p0y)*x + (p0x - p2x)*y);
			auto t = 1 / (2 * Area)*(p0x*p1y - p0y*p1x + (p0y - p1y)*x + (p1x - p0x)*y);

            size2_t mappedIndex(i, j);
            mappedIndex = glm::clamp(mappedIndex, size2_t(0), inputSize - size_t(1));

            // get pixel from input image at pixel coordinate mappedIndex
            auto pixel_intensity = in_img->getAsNormalizedDouble(mappedIndex);

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
            double x = static_cast<double>(i) / outputSize.x;
            double y = static_cast<double>(j) / outputSize.y;

            size2_t mappedIndex(x * (inputSize.x - 1), y * (inputSize.y - 1));
            auto pixel_intensity =
                in_img->getAsNormalizedDouble(mappedIndex);  // get from input image

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

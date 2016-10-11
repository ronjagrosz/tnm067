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
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <modules/tnm067lab2/processors/hydrogengenerator.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/util/volumeramutils.h>
#include <modules/base/algorithm/volume/volumeminmax.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/datastructures/volume/volumeram.h>

#define PI 3.14159265359

namespace inviwo {

    const ProcessorInfo HydrogenGenerator::processorInfo_{
        "org.inviwo.HydrogenGenerator",  // Class identifier
        "Hydrogen Generator",            // Display name
        "TNM067",                        // Category
        CodeState::Experimental,         // Code state
        Tags::None,                      // Tags
    };

    const ProcessorInfo HydrogenGenerator::getProcessorInfo() const { return processorInfo_; }

    HydrogenGenerator::HydrogenGenerator()
        : Processor()
        , volume_("volume")
        , size_("size_", "Volume Size", 16, 4, 256)
    {
        addPort(volume_);
        addProperty(size_);
    }

    void HydrogenGenerator::process() {
        auto vol = std::make_shared<Volume>(size3_t(size_), DataFloat32::get());

        auto ram = vol->getEditableRepresentation<VolumeRAM>();
        auto data = static_cast<float *>(ram->getData());
        util::IndexMapper3D index(ram->getDimensions());

        util::forEachVoxel(*ram, [&](const size3_t &pos) {
            vec3 cartesian = idTOCartesian(pos);
            data[index(pos)] = eval(cartesian);
        });

        auto minMax = util::volumeMinMax(ram);
        vol->dataMap_.dataRange = vol->dataMap_.valueRange = dvec2(minMax.first.x, minMax.second.x);

        volume_.setData(vol);
    }

    inviwo::vec3 HydrogenGenerator::cartesianToSphereical(vec3 cartesian) {
        vec3 sph;
        //TODO implement this
		sph.x = glm::length(cartesian);
		if (sph.x == 0) {
			return vec3(0.f, 0.f, 0.f);
		}
		sph.y = acos(cartesian.z / sph.x);
		sph.z = atan2(cartesian.y, cartesian.x);
        return sph;
    }

    double HydrogenGenerator::eval(vec3 cartesian) {
        //TODO implement this
		vec3 sph = cartesianToSphereical(cartesian);
		double Z = 1;
		double a0 = 1;
		double yellow = 1 / (81 * sqrt(6.0*M_PI));
		double red = pow((Z / a0), 3.0 / 2.0);
		double blue = (pow(Z, 2) * pow(sph.x, 2)) / pow(a0, 2);
		double green = exp((-Z * sph.x) / (3 * a0));
		double pink = (3.0 * pow(cos(sph.y), 2) - 1);

        return pow(yellow * red * blue * green * pink, 2.0);
    }

    inviwo::vec3 HydrogenGenerator::idTOCartesian(size3_t pos) {
        vec3 p(pos);
        p /= size_ - 1;
        return p * (36.0f) - 18.0f;
    }

} // namespace


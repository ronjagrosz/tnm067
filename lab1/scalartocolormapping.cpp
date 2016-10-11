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

#define TASK_SOLUTION

#include <modules/tnm067lab1/utils/scalartocolormapping.h>

namespace inviwo {

ScalarToColorMapping::ScalarToColorMapping() {
    //addBaseColors(vec4(0, 0, 0, 1));
    //addBaseColors(vec4(1, 1, 1, 1));
	addBaseColors(vec4(1, 0, 0, 1));
	addBaseColors(vec4(1, 1, 1, 1));
	addBaseColors(vec4(0, 0, 1, 1));
}

ScalarToColorMapping::~ScalarToColorMapping() {}

void ScalarToColorMapping::clearColors() { baseColors_.clear(); }

void ScalarToColorMapping::addBaseColors(vec4 color) { baseColors_.push_back(color); }

vec4 ScalarToColorMapping::sample(float t) {
    if (baseColors_.size() == 0) return vec4(t);
    if (baseColors_.size() == 1) return vec4(baseColors_[0]);

    vec4 finalColor(t, t, t, 1);

	if (t >= 1) {
		return baseColors_[baseColors_.size() - 1];
	} else if (t < 0) {
		return baseColors_[0];
	}

    // Implement here:
    // Interpolate colors in baseColors_
    // return the right values
	float interpolatedPoint = t * (static_cast<float>(baseColors_.size()) - 1.f);
	int indexLeft = interpolatedPoint;

	vec4 colorLeft = baseColors_[indexLeft];
	vec4 colorRight = baseColors_[indexLeft + 1];

	finalColor = colorLeft + (colorRight - colorLeft) * (interpolatedPoint - indexLeft);

	return finalColor;
}
/*
auto colorsIndex = floor(baseColors_.size() * t) - 1;
auto p = t * ((baseColors_.size() - 1) / );

colorsIndex < 0 ? colorsIndex = 0 : colorsIndex = colorsIndex;

finalColor.r = baseColors_[colorsIndex].r * p + (baseColors_[colorsIndex + 1].r * (1.0 - p));
finalColor.g = baseColors_[colorsIndex].g * p + (baseColors_[colorsIndex + 1].g * (1.0 - p));
finalColor.b = baseColors_[colorsIndex].b * p + (baseColors_[colorsIndex + 1].b * (1.0 - p));
finalColor = glm::normalize(finalColor);


return finalColor;
*/
}  // namespace

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
 
#include "utils/structs.glsl"

uniform sampler2D vfColor;
uniform ImageParameters vfParameters;
in vec3 texCoord_;


vec4 passThrough(vec2 coord){
    return vec4( texture2D(vfColor,coord) );
}

vec2 calc( float x, float y){
	vec4 v = passThrough(vec2(x,y));
	return vec2(v.x, v.y);
}


vec4 magnitude( vec2 coord ){
    //TODO find the magnitude of the vectorfield at the postion coords
	vec2 v = calc(coord.x, coord.y);
	float V = sqrt(pow(v.x, 2) + pow(v.y, 2));
    return vec4(V);
}

vec4 divergence(vec2 coord){
    //TODO find the divergence of the vectorfield at the postion coords
	vec2 pixelSize = vfParameters.reciprocalDimensions;
	vec2 dVx = (calc(coord.x + pixelSize.x, coord.y) - calc(coord.x - pixelSize.x, coord.y) ) / (2 * pixelSize.x);
	vec2 dVy = (calc(coord.x, coord.y + pixelSize.y) - calc(coord.x, coord.y  - pixelSize.y) ) / (2 * pixelSize.y);

	float D = dVx.x + dVy.y;
	
    return vec4(D);
}

vec4 rotation(vec2 coord){
    //TODO find the curl of the vectorfield at the postion coords
    vec2 pixelSize = vfParameters.reciprocalDimensions;
	vec2 dVx = (calc(coord.x + pixelSize.x, coord.y) - calc(coord.x - pixelSize.x, coord.y) ) / (2 * pixelSize.x);
	vec2 dVy = (calc(coord.x, coord.y + pixelSize.y) - calc(coord.x, coord.y  - pixelSize.y) ) / (2 * pixelSize.y);

	float D = dVx.y - dVy.x;
	
    return vec4(D);
}





void main(void) {
    FragData0 = OUTPUT(texCoord_.xy);
}

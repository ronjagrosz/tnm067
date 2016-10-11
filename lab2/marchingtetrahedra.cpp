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

#include <modules/tnm067lab2/processors/marchingtetrahedra.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/assertion.h>

namespace inviwo {

size_t MarchingTetrahedra::HashFunc::max = 1;

const ProcessorInfo MarchingTetrahedra::processorInfo_{
    "org.inviwo.MarchingTetrahedra",  // Class identifier
    "Marching Tetrahedra",            // Display name
    "TNM067",                         // Category
    CodeState::Experimental,          // Code state
    Tags::None,                       // Tags
};
const ProcessorInfo MarchingTetrahedra::getProcessorInfo() const { return processorInfo_; }

MarchingTetrahedra::MarchingTetrahedra()
    : Processor()
    , volume_("volume")
    , mesh_("mesh")
    , isoValue_("isoValue", "ISO value", 0.1f, 0.0f, 1.0f) {
    addPort(volume_);
    addPort(mesh_);

    addProperty(isoValue_);

    volume_.onChange([&]() {
        if (!volume_.hasData()) {
            return;
        }
        NetworkLock lock(getNetwork());
        float iso = (isoValue_.get() - isoValue_.getMinValue()) /
                    (isoValue_.getMaxValue() - isoValue_.getMinValue());
        auto vr = volume_.getData()->dataMap_.valueRange;
        isoValue_.setMinValue(vr.x);
        isoValue_.setMaxValue(vr.y);
        isoValue_.setIncrement(glm::abs(vr.y - vr.x) / 50.0f);
        isoValue_.set(iso * (vr.y - vr.x) + vr.x);
        isoValue_.setCurrentStateAsDefault();
    });
}

void MarchingTetrahedra::process() {
    auto volume = volume_.getData()->getRepresentation<VolumeRAM>();
    MeshHelper mesh(volume_.getData());

    const auto dims = volume->getDimensions();
    MarchingTetrahedra::HashFunc::max = dims.x * dims.y * dims.z;

    float iso = isoValue_.get();

    util::IndexMapper3D index(dims);

    const static size_t tetrahedraIds[6][4] = {{0, 1, 2, 5}, {1, 3, 2, 5}, {3, 2, 5, 7},
                                               {0, 2, 4, 5}, {6, 4, 2, 5}, {6, 7, 5, 2}};

    size3_t pos;
    for (pos.z = 0; pos.z < dims.z - 1; ++pos.z) {
        for (pos.y = 0; pos.y < dims.y - 1; ++pos.y) {
            for (pos.x = 0; pos.x < dims.x - 1; ++pos.x) {
                // Step 1: create current cell
                // Use volume->getAsDouble to query values from the volume
                // Spatial position should be between 0 and 1
                // The voxel index should be the 1D-index for the voxel
                Cell c;
				auto voxIndex = 0; 
				//Loop through voxel 0 - 7
				for (int z = 0; z <= 1; z++) { //move on z axis

					for (int y = 0; y <= 1; y++) { //move on y axis

						for (int x = 0; x <= 1; x++) { //move on x axis

							//Create voxel
							Voxel voxel;

							//scale between 0 - 1

							voxel.pos.x = (pos.x + x) / (dims.x - 1.0f);
							voxel.pos.y = (pos.y + y) / (dims.y - 1.0f);
							voxel.pos.z = (pos.z + z) / (dims.z - 1.0f);

							voxel.value = volume->getAsDouble(size3_t(pos.x + x, pos.y + y, pos.z + z));
							voxel.index = index(size3_t(pos.x + x, pos.y + y, pos.z + z));

							//Store voxel in cell
							c.voxels[voxIndex++] = voxel;

						}
					}
				}

                // Step 2: Subdivide cell into tetrahedra (hint: use tetrahedraIds)
                std::vector<Tetrahedra> tetrahedras;
				for (int i = 0; i < 6; i++) {
					Tetrahedra tempTetrahedra;
					for (int j = 0; j < 4; j++) {
						tempTetrahedra.voxels[j] = c.voxels[tetrahedraIds[i][j]];
					}
					tetrahedras.push_back(tempTetrahedra);
				}

                for (const Tetrahedra& tetrahedra : tetrahedras) {
                    // Step three: Calculate for tetra case index
                    int caseId = 0;

					auto v0 = tetrahedra.voxels[0];
					auto v1 = tetrahedra.voxels[1];
					auto v2 = tetrahedra.voxels[2];
					auto v3 = tetrahedra.voxels[3];

					if (v0.value < isoValue_) caseId |= 1;
					if (v1.value < isoValue_) caseId |= 2;
					if (v2.value < isoValue_) caseId |= 4;
					if (v3.value < isoValue_) caseId |= 8;					
					
					switch (caseId) {
					case 0:
					case 15:
						break;

					case 1:
						calcTriangle(mesh, iso, v0, v1, v0, v3, v0, v2);
						break;
					case 14:
						calcTriangle(mesh, iso, v0, v2, v0, v3, v0, v1);
						break;


					case 2:
						calcTriangle(mesh, iso, v1, v0, v1, v2, v1, v3);
						break;
					case 13:
						calcTriangle(mesh, iso, v1, v3, v1, v2, v1, v0);
						break;

						
					case 3:
						calcTriangle(mesh, iso, v1, v2, v1, v3, v0, v3);
						calcTriangle(mesh, iso, v1, v2, v0, v3, v0, v2);
						break;
					case 12:
						calcTriangle(mesh, iso, v0, v3, v1, v3, v1, v2);
						calcTriangle(mesh, iso, v0, v2, v0, v3, v1, v2);
						break;
						

					case 4:
						calcTriangle(mesh, iso, v2, v3, v2, v1, v2, v0);
						break;
					case 11:
						
						calcTriangle(mesh, iso, v2, v0, v2, v1, v2, v3);
						break;
						

					case 5:
						calcTriangle(mesh, iso, v2, v1, v0, v1, v0, v3);
						calcTriangle(mesh, iso, v2, v3, v2, v1, v0, v3);
						
						break;
					case 10:
						calcTriangle(mesh, iso, v0, v3, v0, v1, v2, v1);
						calcTriangle(mesh, iso, v0, v3, v2, v1, v2, v3);
						break;

						
					case 6:
						calcTriangle(mesh, iso, v2, v0, v1, v3, v1, v0);
						calcTriangle(mesh, iso, v2, v0, v2, v3, v1, v3);
						
						break;
					case 9:
						calcTriangle(mesh, iso, v1, v0, v1, v3, v2, v0);
						calcTriangle(mesh, iso, v1, v3, v2, v3, v2, v0);
						break;

					case 7:
						calcTriangle(mesh, iso, v3, v1, v3, v0, v3, v2);
						break;
					case 8:
						calcTriangle(mesh, iso, v3, v2, v3, v0, v3, v1);
						break;
					}

                    // step four: Extract triangles

                }
            }
        }
    }

    mesh_.setData(mesh.toBasicMesh());
}

void MarchingTetrahedra::calcTriangle(MarchingTetrahedra::MeshHelper &mesh, float iso, MarchingTetrahedra::Voxel vox1, MarchingTetrahedra::Voxel vox2,
			MarchingTetrahedra::Voxel vox3, MarchingTetrahedra::Voxel vox4, MarchingTetrahedra::Voxel vox5, 
			MarchingTetrahedra::Voxel vox6) {

	auto posV0 = vox1.pos + ((vox2.pos - vox1.pos) * (iso - vox1.value)) / (vox2.value - vox1.value);
	auto posV1 = vox3.pos + ((vox4.pos - vox3.pos) * (iso - vox3.value)) / (vox4.value - vox3.value);
	auto posV2 = vox5.pos + ((vox6.pos - vox5.pos) * (iso - vox5.value)) / (vox6.value - vox5.value);

	size_t t0 = mesh.addVertex(posV0, vox1.index, vox2.index);
	size_t t1 = mesh.addVertex(posV1, vox3.index, vox4.index);
	size_t t2 = mesh.addVertex(posV2, vox5.index, vox6.index);

	mesh.addTriangle(t0, t1, t2);
}

MarchingTetrahedra::MeshHelper::MeshHelper(std::shared_ptr<const Volume> vol)
    : edgeToVertex_()
    , vertices_()
    , mesh_(std::make_shared<BasicMesh>())
    , indexBuffer_(mesh_->addIndexBuffer(DrawType::Triangles, ConnectivityType::None)) {
    mesh_->setModelMatrix(vol->getModelMatrix());
    mesh_->setWorldMatrix(vol->getWorldMatrix());
}

void MarchingTetrahedra::MeshHelper::addTriangle(size_t i0, size_t i1, size_t i2) {
    ivwAssert(i0 != i1, "i0 and i1 should not be the same value");
    ivwAssert(i0 != i2, "i0 and i2 should not be the same value");
    ivwAssert(i1 != i2, "i1 and i2 should not be the same value");

    indexBuffer_->add(static_cast<glm::uint32_t>(i0));
    indexBuffer_->add(static_cast<glm::uint32_t>(i1));
    indexBuffer_->add(static_cast<glm::uint32_t>(i2));

    auto a = vertices_[i0].pos;
    auto b = vertices_[i1].pos;
    auto c = vertices_[i2].pos;

    vec3 n = glm::normalize(glm::cross(b - a, c - a));
    vertices_[i0].normal += n;
    vertices_[i1].normal += n;
    vertices_[i2].normal += n;
}

std::shared_ptr<BasicMesh> MarchingTetrahedra::MeshHelper::toBasicMesh() {
    for (auto& vertex : vertices_) {
        vertex.normal = glm::normalize(vertex.normal);
    }
    mesh_->addVertices(vertices_);
    return mesh_;
}

std::uint32_t MarchingTetrahedra::MeshHelper::addVertex(vec3 pos, size_t i, size_t j) {
    ivwAssert(i != j, "i and j should not be the same value");
    if (j < i) {
        return addVertex(pos, j, i);
    }

    auto edge = std::make_pair(i, j);

    auto it = edgeToVertex_.find(edge);

    if (it == edgeToVertex_.end()) {
        edgeToVertex_[edge] = vertices_.size();
        vertices_.push_back({pos, vec3(0, 0, 0), pos, vec4(0.7f, 0.7f, 0.7f, 1.0f)});
        return vertices_.size() - 1;
    }

    return it->second;
}

}  // namespace

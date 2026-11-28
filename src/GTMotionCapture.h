#pragma once

#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"

namespace hdt::GTMotionCapture
{
	void pollControls(bool available);
	bool active();
	void beginFrame(const std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems,
		const btAlignedObjectArray<btTypedConstraint*>& constraints, const btVector3& worldOffset);
	bool tracingFrame();
	void beginSubstep(btScalar dt, int solverIterations);
	void sampleLimits();
	void endSubstep();
	void sample(const std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems);
	void finish();
	void flush(bool notify = true);
}

#pragma once

#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"

namespace hdt::GTMotionCapture
{
	void pollControls(bool available);
	bool active();
	void sample(const std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems);
	void finish();
	void flush(bool notify = true);
}

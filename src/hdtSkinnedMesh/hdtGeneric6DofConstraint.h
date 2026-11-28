#pragma once

#include "hdtBoneScaleConstraint.h"
#include "hdtBulletHelper.h"

namespace hdt
{
	class Generic6DofConstraint :
		public BoneScaleConstraint,
		public btGeneric6DofSpring2Constraint
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		Generic6DofConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, const btTransform& frameInA, const btTransform& frameInB);
		virtual ~Generic6DofConstraint() override = default;

		void scaleConstraint() override;
		void getInfo1(btConstraintInfo1* info) override;
		void getInfo2(btConstraintInfo2* info) override;
		float linearStiffnessScale() const { return m_linearStiffnessScale; }
		float angularStiffnessScale() const { return m_angularStiffnessScale; }
		float linearLimitScale() const { return m_linearLimitScale; }

		void configureProgressiveZ(btScalar projectedSigma, btScalar standingLocalZ, bool distal, btScalar blendWeight = 1);
		void clearProgressiveZ();
		bool progressiveZEnabled() const { return m_progressiveZ.sigma > 0; }
		btScalar progressiveZLastGain() const { return m_progressiveZ.lastGain; }
		btScalar progressiveZLastDamping() const { return m_progressiveZ.lastDamping; }

	private:
		struct ProgressiveZ {
			// Stored in unscaled joint coordinates so actor scaling follows the other linear limits.
			btScalar sigma = 0;
			btScalar standing = 0;
			btScalar rateLength = 0;
			btScalar blendWeight = 0;
			btScalar lastGain = 0;
			btScalar lastDamping = 0;
		};

		float m_linearStiffnessScale = 1.f;
		float m_angularStiffnessScale = 1.f;
		float m_linearLimitScale = 1.f;
		ProgressiveZ m_progressiveZ;
	};
}

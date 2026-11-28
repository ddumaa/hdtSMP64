#include "hdtGeneric6DofConstraint.h"
#include <cmath>

namespace hdt
{
	Generic6DofConstraint::Generic6DofConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, const btTransform& frameInA, const btTransform& frameInB) :
		BoneScaleConstraint(a, b, static_cast<btTypedConstraint*>(this)),
		btGeneric6DofSpring2Constraint(a->m_rig, b->m_rig, btTransform::getIdentity(), btTransform::getIdentity(), RO_XYZ)
	{
		auto fa = a->m_rigToLocal * frameInA;
		auto fb = b->m_rigToLocal * frameInB;
		setFrames(fa, fb);

		for (int i = 0; i < 6; ++i)
			enableSpring(i, true);

		//m_linearLimits.m_stopERP.setValue(0.1f, 0.1f, 0.1f);

		//m_oldLinearDiff.setZero();
		//m_oldAngularDiff.setZero();
		//m_linearBounce.setZero();
	}

	void Generic6DofConstraint::configureProgressiveZ(btScalar projectedSigma, btScalar standingLocalZ, bool distal,
		btScalar blendWeight)
	{
		clearProgressiveZ();
		if (!std::isfinite(projectedSigma) || projectedSigma <= 0 || !std::isfinite(standingLocalZ) ||
			!std::isfinite(blendWeight) || !std::isfinite(m_linearLimitScale) || m_linearLimitScale <= 0)
			return;

		m_progressiveZ.sigma = projectedSigma / m_linearLimitScale;
		m_progressiveZ.standing = standingLocalZ / m_linearLimitScale;
		m_progressiveZ.rateLength = distal ? 395.f : 150.f;
		m_progressiveZ.blendWeight = std::clamp(blendWeight, btScalar(0), btScalar(1));
	}

	void Generic6DofConstraint::clearProgressiveZ()
	{
		m_progressiveZ = {};
	}

	void Generic6DofConstraint::getInfo1(btConstraintInfo1* info)
	{
		if (progressiveZEnabled()) {
			m_progressiveZ.lastGain = 0;
			m_progressiveZ.lastDamping = 0;
		}
		btGeneric6DofSpring2Constraint::getInfo1(info);
	}

	void Generic6DofConstraint::getInfo2(btConstraintInfo2* info)
	{
		constexpr int axis = 2;
		if (!progressiveZEnabled() || m_progressiveZ.blendWeight <= 0 || !m_linearLimits.m_enableSpring[axis]) {
			btGeneric6DofSpring2Constraint::getInfo2(info);
			return;
		}

		// getInfo1 already calculated the joint coordinates in the current Bullet frames.
		const auto invMassA = getRigidBodyA().getInvMass();
		const auto invMassB = getRigidBodyB().getInvMass();
		const auto sigma = m_progressiveZ.sigma * m_linearLimitScale;
		const auto sigmaSquared = sigma * sigma;
		const auto standing = m_progressiveZ.standing * m_linearLimitScale;
		const auto delta = getRelativePivotPosition(axis) - standing;
		const auto deltaSquared = delta * delta;
		if (invMassA <= 0 || invMassB <= 0 || !std::isfinite(sigmaSquared) || sigmaSquared <= 0 ||
			!std::isfinite(deltaSquared) || deltaSquared <= 0 || !std::isfinite(info->fps) || info->fps <= 0) {
			btGeneric6DofSpring2Constraint::getInfo2(info);
			return;
		}

		constexpr btScalar onsetFraction = .33f;
		constexpr btScalar viscosity = 625.f;
		const auto dt = btScalar(1) / info->fps;
		const auto invMass = invMassA + invMassB;
		const auto weight = m_progressiveZ.blendWeight * deltaSquared /
			(deltaSquared + onsetFraction * onsetFraction * sigmaSquared);
		const auto gainScale = weight / (sigmaSquared * invMass);
		const auto rate = m_progressiveZ.rateLength;
		const auto extraStiffness = rate * rate * gainScale;
		const auto extraDamping = viscosity * gainScale;

		auto& stiffness = m_linearLimits.m_springStiffness[axis];
		auto& damping = m_linearLimits.m_springDamping[axis];
		auto& equilibrium = m_linearLimits.m_equilibriumPoint[axis];
		const auto savedStiffness = stiffness;
		const auto savedDamping = damping;
		const auto savedEquilibrium = equilibrium;
		const auto stiffnessCap = btScalar(1) / (16 * dt * dt * invMass);
		const auto cappedStiffness = m_linearLimits.m_springStiffnessLimited[axis];
		const auto baseStiffness = cappedStiffness ? btMin(savedStiffness, stiffnessCap) : savedStiffness;
		const auto totalStiffness = cappedStiffness ? btMin(baseStiffness + extraStiffness, stiffnessCap) :
			baseStiffness + extraStiffness;
		const auto totalDamping = savedDamping + extraDamping;
		const auto dampingCap = btScalar(1) / (dt * invMass);
		const auto cappedDamping = m_linearLimits.m_springDampingLimited[axis];
		if (!std::isfinite(totalStiffness) || !std::isfinite(totalDamping) || totalStiffness <= 0) {
			btGeneric6DofSpring2Constraint::getInfo2(info);
			return;
		}

		m_progressiveZ.lastGain = totalStiffness - baseStiffness;
		m_progressiveZ.lastDamping = cappedDamping ?
			btMin(totalDamping, dampingCap) - btMin(savedDamping, dampingCap) : extraDamping;
		// Preserve the base force and add a spring centered on the canonical standing coordinate.
		stiffness = totalStiffness;
		damping = totalDamping;
		equilibrium = (baseStiffness * savedEquilibrium + m_progressiveZ.lastGain * standing) / totalStiffness;
		btGeneric6DofSpring2Constraint::getInfo2(info);
		stiffness = savedStiffness;
		damping = savedDamping;
		equilibrium = savedEquilibrium;
	}

	void Generic6DofConstraint::scaleConstraint()
	{
		auto newScaleA = m_boneA->m_currentTransform.getScale();
		auto newScaleB = m_boneB->m_currentTransform.getScale();

		if (btFuzzyZero(newScaleA - m_scaleA) && btFuzzyZero(newScaleB - m_scaleB))
			return;

		float w0 = m_boneA->m_rig.getInvMass();
		float w1 = m_boneB->m_rig.getInvMass();
		auto factorA = newScaleA / m_scaleA;
		auto factorB = newScaleB / m_scaleB;
		auto factor = (factorA * w0 + factorB * w1) / (w0 + w1);
		auto factor2 = factor * factor;
		auto factor3 = factor2 * factor;
		auto factor5 = factor3 * factor2;

		getFrameOffsetA().setOrigin(getFrameOffsetA().getOrigin() * factorA);
		getFrameOffsetB().setOrigin(getFrameOffsetB().getOrigin() * factorB);
		m_linearLimits.m_equilibriumPoint *= factor;
		// target k = ma / x(kg/s^2)
		m_linearLimits.m_springStiffness *= factor3;
		m_linearStiffnessScale *= factor3;
		m_angularStiffnessScale *= factor5;
		m_linearLimitScale *= factor;
		m_linearLimits.m_upperLimit *= factor;
		m_linearLimits.m_lowerLimit *= factor;
		for (int i = 0; i < 3; ++i) {
			m_angularLimits[i].m_springStiffness *= factor5;
		}

		m_scaleA = newScaleA;
		m_scaleB = newScaleB;
	}
}

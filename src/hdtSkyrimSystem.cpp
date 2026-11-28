#include "hdtSkyrimSystem.h"
#include "hdtGTStandingReference.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"

#include "HavokUtils.h"
#include "Patterns/hdtPatternLibrary.h"
#include "Patterns/hdtXmlPatternExpander.h"
#include "XmlReader.h"
#include "hdtSkyrimPhysicsWorld.h"
#include <array>
#include <cmath>
#include <tbb/task_arena.h>

// F16C isn't supported on super old processors. AVX2+ (AVX processors can have it, but not guaranteed)
#if defined(__AVX2__) || defined(__AVX512F__)
#	include <immintrin.h>
#else
// float32
// Martin Kallman
//
// Fast half-precision to single-precision floating point conversion
//  - Supports signed zero and denormals-as-zero (DAZ)
//  - Does not support infinities or NaN
//  - Few, partially pipelinable, non-branching instructions,
//  - Core operations ~6 clock cycles on modern x86-64
static void __float32(float* __restrict out, const uint16_t in)
{
	uint32_t t1;
	uint32_t t2;
	uint32_t t3;

	t1 = in & 0x7fff;  // Non-sign bits
	t2 = in & 0x8000;  // Sign bit
	t3 = in & 0x7c00;  // Exponent

	t1 <<= 13;  // Align mantissa on MSB
	t2 <<= 16;  // Shift sign bit into position

	t1 += 0x38000000;  // Adjust bias

	t1 = (t3 == 0 ? 0 : t1);  // Denormals-as-zero

	t1 |= t2;  // Re-insert sign bit

	*((uint32_t*)out) = t1;
};
#endif

namespace hdt
{
	static constexpr float PI = 3.1415926535897932384626433832795f;

	btEmptyShape SkyrimSystemCreator::BoneTemplate::emptyShape[1];

	SkinnedMeshBone* SkyrimSystem::findBone(const RE::BSFixedString& name)
	{
		for (auto i : m_bones) {
			if (i->m_name == name) {
				return i.get();
			}
		}

		return nullptr;
	}

	SkinnedMeshBody* SkyrimSystem::findBody(const RE::BSFixedString& name)
	{
		for (auto i : m_meshes) {
			if (i->m_name == name) {
				return i.get();
			}
		}

		return nullptr;
	}

	int SkyrimSystem::findBoneIdx(const RE::BSFixedString& name)
	{
		for (int i = 0; i < m_bones.size(); ++i) {
			if (m_bones[i]->m_name == name) {
				return i;
			}
		}

		return -1;
	}

	SkyrimSystem::SkyrimSystem(RE::NiNode* skeleton) :
		m_skeleton(skeleton), m_oldRoot(nullptr)
	{
		m_oldRoot = m_skeleton;
	}

	float SkyrimSystem::prepareForRead(float timeStep)
	{
		auto newRoot = m_skeleton.get();
		while (newRoot->parent) {
			newRoot = newRoot->parent;
		}

		if (m_oldRoot != newRoot) {
			timeStep = RESET_PHYSICS;
		}

		if (!m_initialized) {
			timeStep = RESET_PHYSICS;
			m_initialized = true;
		}

		if (timeStep <= RESET_PHYSICS) {
			if (!this->block_resetting) {
				updateTransformUpDown(m_skeleton.get(), true);
			}

			m_lastRootRotation = convertNi(m_skeleton->world.rotate);
		} else if (m_skeleton->parent == RE::PlayerCharacter::GetSingleton()->Get3D2()) {
			if (SkyrimPhysicsWorld::get()->m_resetPc > 0) {
				timeStep = RESET_PHYSICS;
				updateTransformUpDown(m_skeleton.get(), true);
				m_lastRootRotation = convertNi(m_skeleton->world.rotate);
			} else if (!RE::PlayerCamera::GetSingleton()->GetRuntimeData2().isWeapSheathed || RE::PlayerCamera::GetSingleton()->currentState->id == RE::CameraState::kFirstPerson)  // isWeaponSheathed or potentially isCameraFree || cameraState is first person
			{
				m_lastRootRotation = convertNi(m_skeleton->world.rotate);
			} else {
				btQuaternion newRot = convertNi(m_skeleton->world.rotate);
				btVector3 rotAxis;
				float rotAngle;
				btTransformUtil::calculateDiffAxisAngleQuaternion(m_lastRootRotation, newRot, rotAxis, rotAngle);

				if (SkyrimPhysicsWorld::get()->m_clampRotations) {
					float limit = SkyrimPhysicsWorld::get()->m_rotationSpeedLimit * timeStep;

					if (rotAngle < -limit || rotAngle > limit) {
						rotAngle = btClamped(rotAngle, -limit, limit);
						btQuaternion clampedRot(rotAxis, rotAngle);
						m_lastRootRotation = clampedRot * m_lastRootRotation;
						m_skeleton->world.rotate = convertBt(m_lastRootRotation);

						const auto& children = m_skeleton->GetChildren();
						for (uint16_t i = 0; i < children.size(); ++i) {
							auto node = castNiNode(children[i].get());
							if (node) {
								updateTransformUpDown(node, true);
							}
						}
					}
				} else if (SkyrimPhysicsWorld::get()->m_unclampedResets) {
					float limit = SkyrimPhysicsWorld::get()->m_unclampedResetAngle * timeStep;

					if (rotAngle < -limit || rotAngle > limit) {
						timeStep = RESET_PHYSICS;
						updateTransformUpDown(m_skeleton.get(), true);
						m_lastRootRotation = convertNi(m_skeleton->world.rotate);
					}
				}
			}
		}

		m_oldRoot = hdt::make_nismart(newRoot);
		return timeStep;
	}

	SkyrimSystemCreator::SkyrimSystemCreator()
	{
	}

	namespace
	{
		enum class LiveZone { Chest, Glutes, Abdomen, Thighs, Unsupported };

		constexpr std::string_view geometryLiveFamily = "gt-geometry";
		constexpr size_t geometryBoneCount = 50;

		bool usesUniformGTTissue(XMLReader& reader)
		{
			return reader.getAttribute("shapePhysicsTissue", "") == "uniform-volume-v1";
		}

		bool isGeometryMovingBone(std::string_view name)
		{
			return name.size() == 9 && (name.starts_with("Soft_L") || name.starts_with("Soft_R")) &&
			       (name.substr(6) == "N00" ||
			        (name[6] >= '1' && name[6] <= '8' && name[7] == '0' && name[8] >= '1' && name[8] <= '3'));
		}

		bool is3BAChestBone(std::string_view name)
		{
			return name == "L Breast01" || name == "R Breast01" ||
			       name == "L Breast02" || name == "R Breast02" ||
			       name == "L Breast03" || name == "R Breast03";
		}

		LiveZone liveZone(const SkyrimSystem::LiveConstraint& link, LiveProfileKind kind)
		{
			const auto isPair = [&](std::string_view a, std::string_view b) {
				return (link.bodyA == a && link.bodyB == b) || (link.bodyA == b && link.bodyB == a);
			};

			if (kind == LiveProfileKind::GT) {
				const auto isChest = [](std::string_view name) {
					return name.starts_with("Soft_L") || name.starts_with("Soft_R");
				};
				if (isChest(link.bodyA) || isChest(link.bodyB))
					return LiveZone::Chest;
				if (isPair("HDT Belly", "Belly") || isPair("NPC Belly", "HDT Belly"))
					return LiveZone::Abdomen;
				if (isPair("NPC L FrontThigh", "NPC L PreFrontThigh") ||
				    isPair("NPC R FrontThigh", "NPC R PreFrontThigh") ||
				    isPair("NPC L RearThigh", "NPC L PreRearThigh") ||
				    isPair("NPC R RearThigh", "NPC R PreRearThigh"))
					return LiveZone::Thighs;
			} else if (kind == LiveProfileKind::ThreeBA) {
				if (isPair("L Breast01", "L Breast00") || isPair("R Breast01", "R Breast00") ||
				    isPair("L Breast02", "L Breast00") || isPair("R Breast02", "R Breast00") ||
				    isPair("L Breast02", "L Breast01") || isPair("R Breast02", "R Breast01") ||
				    isPair("L Breast03", "L Breast01") || isPair("R Breast03", "R Breast01") ||
				    isPair("L Breast03", "L Breast02") || isPair("R Breast03", "R Breast02"))
					return LiveZone::Chest;
				if (isPair("NPC Belly", "Belly"))
					return LiveZone::Abdomen;
				if (isPair("NPC L FrontThigh", "NPC L Thigh [LThg]") ||
				    isPair("NPC R FrontThigh", "NPC R Thigh [RThg]") ||
				    isPair("NPC L RearThigh", "NPC L Thigh [LThg]") ||
				    isPair("NPC R RearThigh", "NPC R Thigh [RThg]"))
					return LiveZone::Thighs;
			}

			if (isPair("NPC L Butt", "NPC L PreButt") || isPair("NPC R Butt", "NPC R PreButt"))
				return LiveZone::Glutes;
			return LiveZone::Unsupported;
		}

		bool isSafeLiveVector(const btVector3& vector, float lower = 0.f, float upper = 1.0e7f)
		{
			for (int axis = 0; axis < 3; ++axis) {
				if (!std::isfinite(vector[axis]) || vector[axis] < lower || vector[axis] > upper)
					return false;
			}
			return true;
		}

		bool isSafeLiveParameters(const SkyrimSystem::LiveConstraintParameters& p)
		{
			return isSafeLiveVector(p.linearStiffness) && isSafeLiveVector(p.angularStiffness) &&
			       isSafeLiveVector(p.linearDamping) && isSafeLiveVector(p.angularDamping) &&
			       isSafeLiveVector(p.linearLower, -1.0e7f) && isSafeLiveVector(p.linearUpper, -1.0e7f) &&
			       isSafeLiveVector(p.angularLower, -1.0e7f) && isSafeLiveVector(p.angularUpper, -1.0e7f) &&
			       isSafeLiveVector(p.linearEquilibrium, -1.0e7f);
		}

		bool isSafeNonHookeanFactors(const SkyrimSystem::LiveConstraintParameters& p)
		{
			// Bullet applies these multipliers to the ordinary coefficients.
			// The generated factors stay in this range, including negative values.
			return isSafeLiveVector(p.linearNonHookeanStiffness, -1.f, 1.f) &&
			       isSafeLiveVector(p.angularNonHookeanStiffness, -1.f, 1.f) &&
			       isSafeLiveVector(p.linearNonHookeanDamping, -1.f, 1.f) &&
			       isSafeLiveVector(p.angularNonHookeanDamping, -1.f, 1.f);
		}

		bool isSafeGravityFactor(float factor)
		{
			return std::isfinite(factor) && factor >= 0.f && factor <= 1.f;
		}

		bool isSafeLiveScale(float scale)
		{
			return std::isfinite(scale) && scale >= 1.0e-6f && scale <= 1.0e6f;
		}

		bool isSafeMass(const SkyrimSystem::LiveMassProperties& value)
		{
			return std::isfinite(value.mass) && value.mass >= 1.0e-9f && value.mass <= 1.0e7f &&
			       isSafeLiveVector(value.inertia, 1.0e-9f) && isSafeLiveVector(value.center, -1.0e7f);
		}

		bool hasUnrotatedCenter(const btTransform& transform)
		{
			const auto q = transform.getRotation();
			return btFuzzyZero(q.x()) && btFuzzyZero(q.y()) && btFuzzyZero(q.z()) &&
			       btFuzzyZero(btFabs(q.w()) - 1.f);
		}

		// Re-express the rigid body around the new COM while preserving the
		// skeleton transform and the velocity of every point attached to it.
		void updateBodyMass(SkinnedMeshBone& bone, const SkyrimSystem::LiveMassProperties& value,
			const std::vector<btTransform*>& frames)
		{
			auto& body = bone.m_rig;
			const float scale = bone.m_currentTransform.getScale();
			const float scaleSquared = scale * scale;
			const float scaleCubed = scaleSquared * scale;
			const auto previous = bone.m_localToRig.getOrigin();
			const auto center = value.center * scale;
			const auto shift = center - previous;
			auto world = body.getWorldTransform();
			auto interp = body.getInterpolationWorldTransform();
			const auto worldShift = world.getBasis() * shift;
			const auto interpShift = interp.getBasis() * shift;
			const auto velocity = body.getLinearVelocity() + body.getAngularVelocity().cross(worldShift);
			const auto interpVelocity = body.getInterpolationLinearVelocity() +
				body.getInterpolationAngularVelocity().cross(interpShift);
			const auto interpAngularVelocity = body.getInterpolationAngularVelocity();
			world.getOrigin() += worldShift;
			interp.getOrigin() += interpShift;
			bone.m_localToRig.setOrigin(center);
			bone.m_rigToLocal = bone.m_localToRig.inverse();
			for (auto* frame : frames)
				frame->getOrigin() -= shift;
			body.setMassProps(value.mass * scaleCubed, value.inertia * (scaleCubed * scaleSquared));
			body.setCenterOfMassTransform(world);
			body.setLinearVelocity(velocity);
			body.setInterpolationWorldTransform(interp);
			body.setInterpolationLinearVelocity(interpVelocity);
			body.setInterpolationAngularVelocity(interpAngularVelocity);
			body.updateInertiaTensor();
			body.activate();
		}

		// Bullet treats lower > upper as free. Keep the source limits on any
		// axis whose target changes that state (the solver row would appear or
		// disappear abruptly). Bounded-to-locked axes can narrow continuously.
		int keepFreeLimitMode(const btVector3& lower, const btVector3& upper, btVector3& targetLower, btVector3& targetUpper)
		{
			int kept = 0;
			for (int axis = 0; axis < 3; ++axis) {
				if ((lower[axis] > upper[axis]) != (targetLower[axis] > targetUpper[axis])) {
					targetLower[axis] = lower[axis];
					targetUpper[axis] = upper[axis];
					++kept;
				}
			}
			return kept;
		}
	}

	bool SkyrimSystem::LiveConstraint::matches(const LiveConstraint& target) const
	{
		return bodyA == target.bodyA && bodyB == target.bodyB && preset == target.preset &&
		       reversed == target.reversed && linearEnabled == target.linearEnabled &&
		       angularEnabled == target.angularEnabled && limitLinearStiffness == target.limitLinearStiffness &&
		       limitAngularStiffness == target.limitAngularStiffness && limitDamping == target.limitDamping &&
		       linearMotors == target.linearMotors && angularMotors == target.angularMotors &&
		       linearServos == target.linearServos && angularServos == target.angularServos;
	}

	bool SkyrimSystem::queueGTStandingReference(const LiveProfile* target)
	{
		if (m_gtReferenceLinks.empty())
			return false;
		auto job = std::make_shared<GTStandingReferenceJob>();
		job->worldGravity = -9.8f * scaleSkyrim;
		job->bodies.reserve(m_gtTraceBones.size());
		job->links.reserve(m_gtReferenceLinks.size());
		for (size_t i = 0; i < m_gtTraceBones.size(); ++i) {
			const auto& bone = *m_gtTraceBones[i];
			const auto& body = bone.m_rig;
			const auto scale = bone.m_currentTransform.getScale();
			if (!isSafeLiveScale(scale) || !hasUnrotatedCenter(bone.m_localToRig))
				return false;
			auto center = bone.m_localToRig.getOrigin() / scale;
			auto& input = job->bodies.emplace_back(GTStandingReferenceJob::Body{
				m_gtReferenceBones[i].asTransform(), btVector3(0, 0, 0), 0.f,
				body.getLinearDamping(), body.getAngularDamping(), bone.m_gravityFactor });
			if (i >= gtTraceAnchorCount) {
				const auto it = std::ranges::find(m_liveBones, &bone, &AttachedLiveBone::bone);
				if (it == m_liveBones.end())
					return false;
				const auto boneIndex = static_cast<size_t>(it - m_liveBones.begin());
				const auto inv = body.getInvInertiaDiagLocal();
				const auto scale3 = scale * scale * scale;
				const auto scale5 = scale3 * scale * scale;
				if (body.getInvMass() <= 0 || inv.x() <= 0 || inv.y() <= 0 || inv.z() <= 0)
					return false;
				const auto value = target ? target->mass[boneIndex] : LiveMassProperties{
					1.f / (body.getInvMass() * scale3),
					btVector3(1.f / inv.x(), 1.f / inv.y(), 1.f / inv.z()) / scale5, center };
				if (!isSafeMass(value))
					return false;
				input.mass = value.mass;
				input.inertia = value.inertia;
				center = value.center;
				if (target)
					input.gravityFactor = target->gravity[boneIndex].factor;
			}
			if (!isSafeGravityFactor(input.gravityFactor))
				return false;
			input.bind.getOrigin() += input.bind.getBasis() * center;
		}
		for (const auto& item : m_gtReferenceLinks) {
			const auto& attached = m_liveConstraints[item.attachment];
			const auto& source = target ? target->constraints[item.attachment] : attached.source;
			const auto& p = source.parameters;
			if (!isSafeLiveParameters(p) || source.linearMotors || source.angularMotors || source.linearServos || source.angularServos)
				return false;
			const auto frame = [&](size_t i, const btTransform& offset) {
				return job->bodies[i].bind.inverse() * m_gtReferenceBones[i].asTransform() * offset;
			};
			auto& link = job->links.emplace_back(GTStandingReferenceJob::Link{ item.bodyA, item.bodyB,
				frame(item.bodyA, item.frameA), frame(item.bodyB, item.frameB), {}, item.stage });
			const auto* linear = attached.constraint->getTranslationalLimitMotor();
			for (int axis = 0; axis < 3; ++axis) {
				const auto* angular = attached.constraint->getRotationalLimitMotor(axis);
				link.axes[axis] = { p.linearLower[axis], p.linearUpper[axis], p.linearStiffness[axis], p.linearDamping[axis],
					p.linearEquilibrium[axis], linear->m_bounce[axis], linear->m_stopERP[axis], linear->m_stopCFM[axis],
					linear->m_motorERP[axis], linear->m_motorCFM[axis], source.linearEnabled, source.limitLinearStiffness, source.limitDamping };
				link.axes[axis + 3] = { p.angularLower[axis], p.angularUpper[axis], p.angularStiffness[axis], p.angularDamping[axis],
					angular->m_equilibriumPoint, angular->m_bounce, angular->m_stopERP, angular->m_stopCFM,
					angular->m_motorERP, angular->m_motorCFM, source.angularEnabled, source.limitAngularStiffness, source.limitDamping };
			}
		}
		// At most one detached reference rig settles at a time. Queued work whose
		// owning system detached or took a newer target is skipped before solving.
		static tbb::task_arena worker(1, 0);
		m_gtStandingJob = job;
		worker.enqueue([job] {
			if (job.use_count() == 1)
				return;
			try {
				if (!job->solve())
					job->result.clear();
			} catch (...) {
				job->result.clear();
			}
			job->ready.store(true, std::memory_order_release);
		});
		return true;
	}

	void SkyrimSystem::clearGTStandingReference()
	{
		m_gtStandingJob.reset();
		m_gtProgressiveTransition = false;
		m_gtProgressiveActivation = 0.f;
		for (const auto& link : m_gtReferenceLinks)
			if (link.stage >= 0)
				m_liveConstraints[link.attachment].constraint->clearProgressiveZ();
	}

	bool SkyrimSystem::prepareGTStandingReference()
	{
		if (!m_gtStandingJob)
			return true;
		if (!m_gtStandingJob->ready.load(std::memory_order_acquire))
			return false;
		if (m_gtStandingJob->result.size() != gtProgressiveCount) {
			clearGTStandingReference();
			logger::warn("[ShapePhysicsLive] GT progressive standing reference could not be solved for {}", m_gtMotionXml);
			return true;
		}
		for (size_t i = 0; i < gtProgressiveCount; ++i) {
			const auto& ref = m_gtStandingJob->result[i];
			m_gtProgressiveTarget[i] = { ref.sigma, ref.position };
		}
		m_gtStandingJob.reset();
		m_gtProgressiveStart = m_gtProgressiveActivation > 0.f ? m_gtProgressiveCurrent : m_gtProgressiveTarget;
		m_gtProgressiveStartActivation = m_gtProgressiveActivation;
		m_gtProgressiveTransition = true;
		if (m_liveDuration <= 0.f) {
			m_liveElapsed = 0.f;
			m_liveDuration = .25f;
		}
		logger::info("[ShapePhysicsLive] GT progressive Z ready ({} radial links, canonical standing) for {}",
			gtProgressiveCount, m_liveTarget.empty() ? m_gtMotionXml : m_liveTarget);
		return true;
	}

	void SkyrimSystem::applyGTStandingReference(float weight)
	{
		if (!m_gtProgressiveTransition)
			return;
		m_gtProgressiveActivation = std::lerp(m_gtProgressiveStartActivation, 1.f, weight);
		size_t i = 0;
		for (const auto& link : m_gtReferenceLinks) {
			if (link.stage < 0)
				continue;
			const auto& start = m_gtProgressiveStart[i];
			const auto& target = m_gtProgressiveTarget[i];
			const GTProgressiveReference ref{ std::lerp(start.sigma, target.sigma, weight),
				std::lerp(start.standing, target.standing, weight) };
			m_gtProgressiveCurrent[i++] = ref;
			auto* constraint = m_liveConstraints[link.attachment].constraint;
			const auto scale = constraint->linearLimitScale();
			constraint->configureProgressiveZ(scale * ref.sigma, scale * ref.standing, link.stage == 1, m_gtProgressiveActivation);
		}
		m_gtProgressiveTransition = weight < 1.f;
	}

	bool SkyrimSystem::blendLiveProfile(const LiveProfile& target, std::string path, float seconds)
	{
		if (m_liveConstraints.empty() || m_liveConstraints.size() != target.constraints.size() ||
		    m_liveBones.size() != target.gravity.size() ||
		    (m_liveGeometryGT && (target.mass.size() != geometryBoneCount || m_liveBones.size() != geometryBoneCount)))
			return false;

		const bool is3BA = m_liveProfileKind == LiveProfileKind::ThreeBA;
		std::vector<LiveGravityTransition> gravity;
		std::vector<LiveMassTransition> mass;
		gravity.reserve(target.gravity.size());
		for (size_t index = 0; index < target.gravity.size(); ++index) {
			const auto& attached = m_liveBones[index];
			const auto& goal = target.gravity[index];
			if (attached.name != goal.bone || attached.preset != goal.preset || !isSafeGravityFactor(goal.factor))
				return false;
			auto* bone = attached.bone;
			if (!bone || bone->m_rig.isStaticOrKinematicObject() || (bone->m_rig.getFlags() & BT_DISABLE_WORLD_GRAVITY)) {
				if (m_liveGeometryGT)
					return false;
				continue;
			}
			if (!isSafeGravityFactor(bone->m_gravityFactor))
				return false;
			if (bone->m_gravityFactor != goal.factor)
				gravity.push_back({ bone, bone->m_gravityFactor, goal.factor });
			if (m_liveGeometryGT) {
				const auto& end = target.mass[index];
				const auto& body = bone->m_rig;
				const auto scale = bone->m_currentTransform.getScale();
				const auto inverse = body.getInvInertiaDiagLocal();
				if (!isSafeMass(end) || !isSafeLiveScale(scale) || body.getInvMass() <= 0 ||
				    inverse.x() <= 0 || inverse.y() <= 0 || inverse.z() <= 0 ||
				    !hasUnrotatedCenter(bone->m_localToRig) ||
				    body.getCollisionShape()->getShapeType() != EMPTY_SHAPE_PROXYTYPE)
					return false;
				const auto scale2 = scale * scale;
				const auto scale3 = scale2 * scale;
				const auto scale5 = scale3 * scale2;
				const LiveMassProperties start{ 1.f / (body.getInvMass() * scale3),
					btVector3(1.f / inverse.x(), 1.f / inverse.y(), 1.f / inverse.z()) / scale5,
					bone->m_localToRig.getOrigin() / scale };
				if (!isSafeMass(start))
					return false;
				if (start == end)
					continue;
				LiveMassTransition moving{ bone, start, end, {} };
				const auto bindFrame = [&](BoneScaleConstraint* constraint) {
					if (constraint->m_boneA != bone && constraint->m_boneB != bone)
						return true;
					auto* link = dynamic_cast<Generic6DofConstraint*>(constraint);
					if (!link)
						return start.center == end.center;
					if (link->m_boneA == bone)
						moving.frames.push_back(&link->getFrameOffsetA());
					if (link->m_boneB == bone)
						moving.frames.push_back(&link->getFrameOffsetB());
					return true;
				};
				for (const auto& link : m_constraints)
					if (!bindFrame(link.get()))
						return false;
				for (const auto& group : m_constraintGroups)
					for (const auto& link : group->m_constraints)
						if (!bindFrame(link.get()))
							return false;
				mass.push_back(std::move(moving));
			}
		}

		std::vector<LiveConstraintTransition> transition;
		transition.reserve(target.constraints.size());
		size_t matched = 0;
		std::array<size_t, 4> changedByZone{};
		int keptAxes = 0;
		for (size_t index = 0; index < target.constraints.size(); ++index) {
			const auto& attached = m_liveConstraints[index];
			const auto& goal = target.constraints[index];
			const auto zone = liveZone(goal, m_liveProfileKind);
			if (zone == LiveZone::Unsupported ||
			    attached.source.bodyA != goal.bodyA || attached.source.bodyB != goal.bodyB ||
			    attached.source.preset != goal.preset)
				return false;

			// The ordinary loader can skip a link whose endpoints are absent or kinematic.
			// Its XML identity must still match; absent runtime constraints are not created.
			auto* constraint = attached.constraint;
			if (!constraint && m_liveGeometryGT && target.uniformGTTissue && zone == LiveZone::Chest)
				return false;
			if (!constraint)
				continue;
			if (!attached.source.matches(goal))
				return false;
			if (m_liveGeometryGT && (attached.source.frameType != goal.frameType || !(attached.source.authoredFrame == goal.authoredFrame)))
				return false;

			const auto linearScale = constraint->linearStiffnessScale();
			const auto angularScale = constraint->angularStiffnessScale();
			const auto limitScale = constraint->linearLimitScale();
			if (!isSafeLiveScale(linearScale) || !isSafeLiveScale(angularScale) || !isSafeLiveScale(limitScale))
				return false;

			const auto* linear = constraint->getTranslationalLimitMotor();
			LiveConstraintParameters start;
			for (int axis = 0; axis < 3; ++axis) {
				// The motor initializes the three axes, not btVector3's fourth component.
				start.linearStiffness[axis] = linear->m_springStiffness[axis] / linearScale;
				start.linearDamping[axis] = linear->m_springDamping[axis];
				start.linearLower[axis] = linear->m_lowerLimit[axis] / limitScale;
				start.linearUpper[axis] = linear->m_upperLimit[axis] / limitScale;
				if (m_liveGeometryGT)
					start.linearEquilibrium[axis] = linear->m_equilibriumPoint[axis] / limitScale;
				const auto* angular = constraint->getRotationalLimitMotor(axis);
				start.angularStiffness[axis] = angular->m_springStiffness / angularScale;
				start.angularDamping[axis] = angular->m_springDamping;
				start.angularLower[axis] = angular->m_loLimit;
				start.angularUpper[axis] = angular->m_hiLimit;
				if (is3BA) {
					start.linearNonHookeanStiffness[axis] = linear->m_nonHookeanStiffness[axis];
					start.linearNonHookeanDamping[axis] = linear->m_nonHookeanDamping[axis];
					start.angularNonHookeanStiffness[axis] = angular->m_nonHookeanStiffness;
					start.angularNonHookeanDamping[axis] = angular->m_nonHookeanDamping;
				}
			}
			if (!isSafeLiveParameters(start) || (is3BA && !isSafeNonHookeanFactors(start)))
				return false;
			LiveConstraintParameters end = goal.parameters;
			keptAxes += keepFreeLimitMode(start.linearLower, start.linearUpper, end.linearLower, end.linearUpper);
			keptAxes += keepFreeLimitMode(start.angularLower, start.angularUpper, end.angularLower, end.angularUpper);
			++matched;
			if (start != end) {
				++changedByZone[static_cast<size_t>(zone)];
				transition.push_back({ constraint, start, end });
			}
		}
		if (matched == 0)
			return false;
		if (m_liveGeometryGT && target.uniformGTTissue) {
			clearGTStandingReference();
			logger::info("[ShapePhysicsLive] Using GT uniform tissue without progressive Z for target {}", path);
		} else if (m_liveGeometryGT && !m_gtReferenceLinks.empty() && (keptAxes || !queueGTStandingReference(&target))) {
			clearGTStandingReference();
			logger::warn("[ShapePhysicsLive] GT progressive Z unavailable for target {}", path);
		}

		const auto* profileName = liveProfileName(m_liveProfileKind);
		logger::info("[ShapePhysicsLive] Accepted {} parameter target {} ({} links changing: chest {}, glutes {}, abdomen {}, thighs {}; {} limit axes held; {} seconds)",
			profileName, path, transition.size(), changedByZone[0], changedByZone[1], changedByZone[2], changedByZone[3], keptAxes, seconds);
		if (!gravity.empty() || !mass.empty())
			logger::info("[ShapePhysicsLive] Blending {} {} gravity factors and {} bone masses/inertias/centers for {}",
				gravity.size(), profileName, mass.size(), path);
		m_liveTarget = std::move(path);
		m_liveConstraintTransitions = std::move(transition);
		m_liveGravityTransitions = std::move(gravity);
		m_liveMassTransitions = std::move(mass);
		m_liveElapsed = 0;
		m_liveDuration = seconds;
		if (m_liveConstraintTransitions.empty() && m_liveGravityTransitions.empty() && m_liveMassTransitions.empty() &&
		    !m_gtStandingJob) {
			m_liveAppliedTarget = m_liveTarget;
			logger::info("[ShapePhysicsLive] {} parameters already match {}", profileName, m_liveTarget);
		} else if (m_liveGeometryGT) {
			// A previously completed destination can be requested again during a reversal.
			// Its old name no longer describes the currently interpolated coefficients.
			m_liveAppliedTarget.clear();
		}
		return true;
	}

	void SkyrimSystem::readTransform(float timeStep)
	{
		SkinnedMeshSystem::readTransform(timeStep);
		if (block_resetting || !std::isfinite(timeStep) || timeStep <= 0)
			return;
		const bool referenceWasPending = static_cast<bool>(m_gtStandingJob);
		if (referenceWasPending && !prepareGTStandingReference())
			return;
		if (m_liveConstraintTransitions.empty() && m_liveGravityTransitions.empty() && m_liveMassTransitions.empty() &&
		    !m_gtProgressiveTransition) {
			if (referenceWasPending && m_liveAppliedTarget != m_liveTarget)
				m_liveAppliedTarget = m_liveTarget;
			return;
		}

		// The world holds its simulation mutex here. The base implementation
		// has already updated bone scales; Bullet has not started its step.
		m_liveElapsed = std::min(m_liveDuration, m_liveElapsed + timeStep);
		const float progress = m_liveElapsed / m_liveDuration;
		const float weight = progress * progress * (3.f - 2.f * progress);
		const bool is3BA = m_liveProfileKind == LiveProfileKind::ThreeBA;
		const auto validLink = [](const auto& link) {
			return isSafeLiveScale(link.constraint->linearStiffnessScale()) &&
			       isSafeLiveScale(link.constraint->angularStiffnessScale()) &&
			       isSafeLiveScale(link.constraint->linearLimitScale());
		};
		const auto validBone = [](const auto& entry) {
			return isSafeLiveScale(entry.bone->m_currentTransform.getScale());
		};
		if (!std::ranges::all_of(m_liveConstraintTransitions, validLink) ||
		    !std::ranges::all_of(m_liveMassTransitions, validBone)) {
			m_liveTarget.clear();
			m_liveAppliedTarget.clear();
			m_liveConstraintTransitions.clear();
			m_liveGravityTransitions.clear();
			m_liveMassTransitions.clear();
			clearGTStandingReference();
			logger::warn("[ShapePhysicsLive] Stopped {} parameter transition after an invalid bone scale", liveProfileName(m_liveProfileKind));
			return;
		}
		for (const auto& link : m_liveConstraintTransitions) {
			auto* constraint = link.constraint;
			const auto linearScale = constraint->linearStiffnessScale();
			const auto angularScale = constraint->angularStiffnessScale();
			const auto limitScale = constraint->linearLimitScale();
			constraint->setLinearLowerLimit(link.start.linearLower.lerp(link.target.linearLower, weight) * limitScale);
			constraint->setLinearUpperLimit(link.start.linearUpper.lerp(link.target.linearUpper, weight) * limitScale);
			constraint->setAngularLowerLimit(link.start.angularLower.lerp(link.target.angularLower, weight));
			constraint->setAngularUpperLimit(link.start.angularUpper.lerp(link.target.angularUpper, weight));
			const auto* linear = constraint->getTranslationalLimitMotor();
			for (int axis = 0; axis < 3; ++axis) {
				const auto* angular = constraint->getRotationalLimitMotor(axis);
				const auto startK = link.start.linearStiffness[axis];
				const auto targetK = link.target.linearStiffness[axis];
				const auto stiffness = std::lerp(startK, targetK, weight);
				constraint->setStiffness(axis, linearScale * stiffness, linear->m_springStiffnessLimited[axis]);
				if (m_liveGeometryGT) {
					// Blend preload (K * displacement), so the supported load and mass
					// change together even when the stiffness also changes.
					const auto preload = std::lerp(startK * link.start.linearEquilibrium[axis],
						targetK * link.target.linearEquilibrium[axis], weight);
					const auto equilibrium = stiffness > 0 ? preload / stiffness : 0.f;
					constraint->setEquilibriumPoint(axis, limitScale * equilibrium);
				}
				constraint->setStiffness(axis + 3, angularScale * std::lerp(link.start.angularStiffness[axis],
					link.target.angularStiffness[axis], weight), angular->m_springStiffnessLimited);
				constraint->setDamping(axis, std::lerp(link.start.linearDamping[axis],
					link.target.linearDamping[axis], weight), linear->m_springDampingLimited[axis]);
				constraint->setDamping(axis + 3, std::lerp(link.start.angularDamping[axis],
					link.target.angularDamping[axis], weight), angular->m_springDampingLimited);
				if (is3BA) {
					// These factors are dimensionless and the stock scaler leaves them unchanged.
					constraint->setNonHookeanStiffness(axis, std::lerp(link.start.linearNonHookeanStiffness[axis],
						link.target.linearNonHookeanStiffness[axis], weight));
					constraint->setNonHookeanStiffness(axis + 3, std::lerp(link.start.angularNonHookeanStiffness[axis],
						link.target.angularNonHookeanStiffness[axis], weight));
					constraint->setNonHookeanDamping(axis, std::lerp(link.start.linearNonHookeanDamping[axis],
						link.target.linearNonHookeanDamping[axis], weight));
					constraint->setNonHookeanDamping(axis + 3, std::lerp(link.start.angularNonHookeanDamping[axis],
						link.target.angularNonHookeanDamping[axis], weight));
				}
			}
		}
		for (const auto& bone : m_liveGravityTransitions)
			bone.bone->m_gravityFactor = std::lerp(bone.start, bone.target, weight);
		for (const auto& bone : m_liveMassTransitions)
			updateBodyMass(*bone.bone, { std::lerp(bone.start.mass, bone.target.mass, weight),
				bone.start.inertia.lerp(bone.target.inertia, weight),
				bone.start.center.lerp(bone.target.center, weight) }, bone.frames);
		applyGTStandingReference(weight);
		if (progress == 1.f) {
			m_liveAppliedTarget = m_liveTarget;
			m_liveConstraintTransitions.clear();
			m_liveGravityTransitions.clear();
			m_liveMassTransitions.clear();
			m_liveDuration = 0.f;
			if (!m_liveTarget.empty())
				logger::info("[ShapePhysicsLive] Finished {} parameter transition to {}", liveProfileName(m_liveProfileKind), m_liveTarget);
		}
	}

	SkyrimSystemCreator::LiveProfileFamily SkyrimSystemCreator::liveProfileFamily(std::string_view path)
	{
		constexpr std::string_view directory = "skse\\plugins\\hdtskinnedmeshconfigs\\shapephysicsregional\\";
		if (path.size() > 240)
			return {};
		std::string normalized(path);
		for (auto& c : normalized) {
			if (c == '/')
				c = '\\';
			else if (c >= 'A' && c <= 'Z')
				c = static_cast<char>(c - 'A' + 'a');
		}
		if (!normalized.starts_with(directory) || !normalized.ends_with(".xml"))
			return {};
		const std::string_view file(normalized.data() + directory.size(), normalized.size() - directory.size());
		constexpr std::string_view geometryPrefix = "gmr3_";
		constexpr std::string_view geometrySuffix = "_r_gt_geometry.xml";
		if (file.starts_with(geometryPrefix) && file.ends_with(geometrySuffix) &&
		    file.size() == geometryPrefix.size() + 12 + geometrySuffix.size()) {
			const auto hash = file.substr(geometryPrefix.size(), 12);
			if (std::ranges::all_of(hash, [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); }))
				return { LiveProfileKind::GT, std::string(geometryLiveFamily) };
		}
		const auto separator = file.find("__");
		if (separator == std::string_view::npos)
			return {};
		const auto id = file.substr(0, separator);
		const auto name = file.substr(separator + 2);
		auto kind = LiveProfileKind::GT;
		if (id == "gt_small") {
			if (name.find("__gt-growth1_gt_medium_gt_large_") == std::string_view::npos)
				return {};
		} else if (id == "gt_slot60" || id == "gt_slot60_extended") {
			if (name.find("__gt-regional4") == std::string_view::npos || name.find("_r4_") == std::string_view::npos)
				return {};
		} else if (id.starts_with("threeba_")) {
			auto variant = id.substr(8);
			if (variant.starts_with("full_"))
				variant.remove_prefix(5);
			if (variant.size() > 2 && variant[0] >= 'a' && variant[0] <= 'd' && variant[1] == '_')
				variant.remove_prefix(2);
			if ((variant != "base" && variant != "small" && variant != "large") ||
			    name.find("__3ba-smooth_q") == std::string_view::npos)
				return {};
			kind = LiveProfileKind::ThreeBA;
		} else {
			return {};
		}
		const auto stampEnd = name.find("__");
		if (stampEnd != 12 || name.find("..") != std::string_view::npos)
			return {};
		for (size_t i = 0; i < name.size(); ++i) {
			const auto c = name[i];
			const bool digit = c >= '0' && c <= '9';
			if (i < stampEnd) {
				if (!digit && !(c >= 'a' && c <= 'f'))
					return {};
			} else if (!digit && !(c >= 'a' && c <= 'z') && c != '_' && c != '-' && c != '.') {
				return {};
			}
		}
		return { kind, normalized.substr(0, directory.size() + separator + 2 + stampEnd) };
	}

	std::optional<SkyrimSystem::LiveProfile> SkyrimSystemCreator::readLiveProfile(const std::string& path, LiveProfileKind kind)
	{
		auto xml = readAllFile(path.c_str());
		if (xml.empty())
			return std::nullopt;
		XMLReader reader(reinterpret_cast<BYTE*>(xml.data()), xml.size());
		m_reader = &reader;
		reader.nextStartElement();
		if (reader.GetName() != "system")
			return std::nullopt;

		SkyrimSystem::LiveProfile profile;
		const bool is3BA = kind == LiveProfileKind::ThreeBA;
		const bool geometry = liveProfileFamily(path).key == geometryLiveFamily;
		profile.uniformGTTissue = geometry && usesUniformGTTissue(reader);
		std::unordered_map<std::string, float> gravityDefaults;
		const auto getGravityDefault = [&](const std::string& preset) {
			auto found = gravityDefaults.find(preset);
			if (found == gravityDefaults.end())
				found = gravityDefaults.find("");
			return found == gravityDefaults.end() ? 1.f : found->second;
		};
		const auto readGravity = [&](float initial) {
			// Read the same per-bone field as the regular loader without constructing shapes.
			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::EndTag)
					break;
				if (reader.GetInspected() != XMLReader::Inspected::StartTag)
					continue;
				if (reader.GetName() == "gravity-factor") {
					initial = reader.readFloat();
					if (std::isfinite(initial))
						initial = std::clamp(initial, 0.f, 1.f);
				} else {
					reader.skipCurrentElement();
				}
			}
			return initial;
		};
		try {
			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::EndTag)
					break;
				if (reader.GetInspected() != XMLReader::Inspected::StartTag)
					continue;

				const auto name = reader.GetName();
				if (is3BA && name == "bone-default") {
					const auto preset = reader.getAttribute("name", "");
					const auto extends = reader.getAttribute("extends", "");
					const auto factor = readGravity(getGravityDefault(extends));
					if (!isSafeGravityFactor(factor))
						return std::nullopt;
					gravityDefaults[preset] = factor;
				} else if (geometry && name == "bone") {
					auto bone = reader.getAttribute("name");
					if (!isGeometryMovingBone(bone)) {
						reader.skipCurrentElement();
						continue;
					}
					auto preset = reader.getAttribute("template", "");
					SkyrimSystem::LiveMassProperties value{0, btVector3(0, 0, 0), btVector3(0, 0, 0)};
					float gravity = -1.f;
					bool centerRead = false;
					while (reader.Inspect() && reader.GetInspected() != XMLReader::Inspected::EndTag) {
						if (reader.GetInspected() != XMLReader::Inspected::StartTag)
							continue;
						const auto field = reader.GetName();
						if (field == "mass")
							value.mass = reader.readFloat();
						else if (field == "inertia")
							value.inertia = reader.readVector3();
						else if (field == "centerOfMassTransform") {
							const auto center = reader.readTransform();
							if (!hasUnrotatedCenter(center))
								return std::nullopt;
							value.center = center.getOrigin();
							centerRead = true;
						} else if (field == "gravity-factor")
							gravity = reader.readFloat();
						else
							reader.skipCurrentElement();
					}
					if (!centerRead || !isSafeMass(value) || !isSafeGravityFactor(gravity))
						return std::nullopt;
					profile.mass.push_back(value);
					profile.gravity.push_back({ std::move(bone), std::move(preset), gravity });
				} else if (is3BA && name == "bone") {
					auto bone = reader.getAttribute("name");
					if (!is3BAChestBone(bone)) {
						reader.skipCurrentElement();
						continue;
					}
					auto preset = reader.getAttribute("template", "");
					const auto factor = readGravity(getGravityDefault(preset));
					if (!isSafeGravityFactor(factor))
						return std::nullopt;
					profile.gravity.push_back({ std::move(bone), std::move(preset), factor });
				} else if (name == "generic-constraint-default") {
					const auto className = reader.getAttribute("name", "");
					const auto extends = reader.getAttribute("extends", "");
					auto values = getGenericConstraintTemplate(extends);
					readGenericConstraintTemplate(values);
					m_genericConstraintTemplates[className] = values;
				} else if (name == "generic-constraint") {
					SkyrimSystem::LiveConstraint link{ reader.getAttribute("bodyA"), reader.getAttribute("bodyB"), reader.getAttribute("template", "") };
					if (liveZone(link, kind) == LiveZone::Unsupported) {
						reader.skipCurrentElement();
						continue;
					}
					auto values = getGenericConstraintTemplate(link.preset);
					readGenericConstraintTemplate(values);
					readLiveConstraintSettings(link, values, is3BA, geometry);
					if (!isSafeLiveParameters(link.parameters) || (is3BA && !isSafeNonHookeanFactors(link.parameters)))
						return std::nullopt;
					// Match the normalization applied by Bullet's angular limit setters.
					for (int axis = 0; axis < 3; ++axis) {
						link.parameters.angularLower[axis] = btNormalizeAngle(link.parameters.angularLower[axis]);
						link.parameters.angularUpper[axis] = btNormalizeAngle(link.parameters.angularUpper[axis]);
					}
					profile.constraints.push_back(std::move(link));
				} else {
					reader.skipCurrentElement();
				}
			}
		} catch (const std::string& error) {
			logger::warn("[ShapePhysicsLive] Invalid {} parameter target {}: {}", liveProfileName(kind), path, error);
			return std::nullopt;
		}
		if (profile.constraints.empty() || (geometry && profile.mass.size() != geometryBoneCount) ||
		    reader.GetErrorCode() != Xml::ErrorCode::None ||
		    reader.GetInspected() != XMLReader::Inspected::EndTag || reader.GetName() != "system")
			return std::nullopt;
		return profile;
	}

	void SkyrimSystemCreator::readLiveConstraintSettings(SkyrimSystem::LiveConstraint& link, const GenericConstraintTemplate& values,
		bool includeNonHookean, bool includeEquilibrium)
	{
		link.parameters = { values.linearStiffness, values.angularStiffness, values.linearDamping, values.angularDamping,
			values.linearLowerLimit, values.linearUpperLimit, values.angularLowerLimit, values.angularUpperLimit };
		if (includeEquilibrium)
		{
			link.parameters.linearEquilibrium = values.linearEquilibrium;
			link.frameType = static_cast<int>(values.frameType);
			link.authoredFrame = values.frame;
		}
		if (includeNonHookean) {
			link.parameters.linearNonHookeanStiffness = values.linearNonHookeanStiffness;
			link.parameters.angularNonHookeanStiffness = values.angularNonHookeanStiffness;
			link.parameters.linearNonHookeanDamping = values.linearNonHookeanDamping;
			link.parameters.angularNonHookeanDamping = values.angularNonHookeanDamping;
		}
		link.reversed = values.useLinearReferenceFrameA;
		link.linearEnabled = values.enableLinearSprings;
		link.angularEnabled = values.enableAngularSprings;
		link.limitLinearStiffness = values.linearStiffnessLimited;
		link.limitAngularStiffness = values.angularStiffnessLimited;
		link.limitDamping = values.springDampingLimited;
		link.linearMotors = values.linearMotors;
		link.angularMotors = values.angularMotors;
		link.linearServos = values.linearServoMotors;
		link.angularServos = values.angularServoMotors;
	}

	void SkyrimSystemCreator::indexBone(SkyrimBone* bone)
	{
		m_boneIndex.emplace(bone->m_name.data(), bone);
	}

	SkyrimBone* SkyrimSystemCreator::findBoneFromIndex(const RE::BSFixedString& name) const
	{
		auto it = m_boneIndex.find(name.data());
		return it != m_boneIndex.end() ? it->second : nullptr;
	}

	RE::NiNode* SkyrimSystemCreator::findObjectByName(const RE::BSFixedString& name)
	{
		// TODO check it's not a lurker skeleton
		return findNode(m_skeleton, name);
	}

	SkyrimBone* SkyrimSystemCreator::getOrCreateBone(const RE::BSFixedString& name)
	{
		auto bone = findBoneFromIndex(getRenamedBone(name));
		if (bone) {
			return bone;
		}

		logger::warn("Bone {} used before being created, trying to create it with current default values", name.c_str());
		// Create from the renamed name too: the lookup above used it, and under a rename
		// map (skeleton merge / DynamicHDT) the raw name's node typically no longer
		// exists — creating from the raw name would drop the reference or split the
		// bone into two identities. Constraint readers already create from renamed names.
		return createBoneFromNodeName(getRenamedBone(name));
	}

	RE::BSFixedString SkyrimSystemCreator::getRenamedBone(const RE::BSFixedString& name)
	{
		auto iter = m_renameMap.find(name);
		if (iter != m_renameMap.end())
			return iter->second;
		return name;
	}

	RE::BSTSmartPointer<SkyrimSystem> SkyrimSystemCreator::createOrUpdateSystem(RE::NiNode* skeleton, RE::NiAVObject* model, DefaultBBP::PhysicsFile_t* file, std::unordered_map<RE::BSFixedString, RE::BSFixedString>&& renameMap, SkyrimSystem* old_system)
	{
		auto path = file->first;
		if (path.empty()) {
			return nullptr;
		}

		auto loaded = readAllFile(path.c_str());
		if (loaded.empty()) {
			return nullptr;
		}

		m_renameMap = std::move(renameMap);
		m_skeleton = skeleton;
		m_model = model;
		m_filePath = path;

		// Expand any <pattern> macros before parsing, so the loader sees only ordinary SMP elements.
		// Shared definitions from the global patterns/ folder are visible alongside the file's own.
		// Fail closed on a malformed pattern: no physics for this item (the asset validator reports why).
		PatternOptions patternOpts;
		patternOpts.libraries = &getGlobalPatternLibraries();
		PatternExpansion expanded = expandPatterns(loaded, patternOpts);
		if (!expanded.ok) {
			// Fail-closed log is the only feedback a mod author gets, so include the diagnostic's line
			// (many carry one; 0 means the failure has no single source line, e.g. a whole-file parse error).
			if (expanded.diags.empty())
				logger::error("[SMP] pattern expansion failed for '{}': unknown error", path.c_str());
			else if (const PatternDiag& d = expanded.diags.front(); d.line > 0)
				logger::error("[SMP] pattern expansion failed for '{}' (line {}): {}", path.c_str(), d.line,
					d.message.c_str());
			else
				logger::error("[SMP] pattern expansion failed for '{}': {}", path.c_str(), d.message.c_str());
			if (!old_system)
				updateTransformUpDown(m_skeleton, true);
			return nullptr;
		}
		loaded = std::move(expanded.xml);

		XMLReader reader((uint8_t*)loaded.data(), loaded.size());
		m_reader = &reader;

		m_reader->nextStartElement();
		if (m_reader->GetName() != "system") {
			if (!old_system) {
				updateTransformUpDown(m_skeleton, true);
			}

			return nullptr;
		}

		auto meshNameMap = file->second;

		m_mesh = RE::make_smart<SkyrimSystem>(skeleton);
		const auto liveFamily = liveProfileFamily(path);
		m_mesh->m_liveProfileKind = liveFamily.kind;
		m_mesh->m_liveGeometryGT = liveFamily.key == geometryLiveFamily;
		const bool uniformGTTissue = m_mesh->m_liveGeometryGT && usesUniformGTTissue(reader);
		m_boneIndex.clear();

		// This forces the skeleton into a neutral reference pose, which avoids building invalid shape data
		// We pull the references directly from havok for the exact same reference data the engine uses
		std::vector<std::pair<RE::NiAVObject*, RE::NiTransform>> savedPoses;

		if (auto* userData = skeleton->GetUserData()) {
			if (auto* actor = userData->As<RE::Actor>()) {
				if (auto havokSkel = havok::getAnimationSkeleton(actor)) {
					savedPoses.reserve(havokSkel->bones.size());

					for (int32_t i = 0; i < static_cast<int32_t>(havokSkel->bones.size()); ++i) {
						if (auto boneNode = skeleton->GetObjectByName(RE::BSFixedString(havokSkel->bones[i].name.data()))) {
							savedPoses.emplace_back(boneNode, boneNode->local);

							RE::hkQsTransform refPose;
							if (havok::getReferencePoseByIndex(havokSkel, i, refPose))
								boneNode->local = havok::hKQsTransformToNiTransform(refPose);
						}
					}

					if (!savedPoses.empty()) {
						RE::NiUpdateData updateData;
						skeleton->Update(updateData);
					}
				}
			}
		}

		if (!old_system) {
			updateTransformUpDown(m_skeleton, true);
		}

		try {
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					const auto name = m_reader->GetName();
					if (name == "bone") {
						readOrUpdateBone();
					} else if (name == "bone-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultBoneInfo = getBoneTemplate(extends);
						readBoneTemplate(defaultBoneInfo);
						m_boneTemplates[clsname] = defaultBoneInfo;
					} else if (name == "per-vertex-shape") {
						auto shape = readPerVertexShape(meshNameMap);
						if (shape && shape->m_vertices.size()) {
							m_mesh->m_meshes.push_back(shape);
							shape->m_mesh = m_mesh.get();
						}
					} else if (name == "per-triangle-shape") {
						auto shape = readPerTriangleShape(&meshNameMap);
						if (shape && shape->m_vertices.size()) {
							m_mesh->m_meshes.push_back(shape);
							shape->m_mesh = m_mesh.get();
						}
					} else if (name == "constraint-group") {
						auto constraint = readConstraintGroup();
						if (constraint)
							m_mesh->m_constraintGroups.push_back(constraint);
					} else if (name == "generic-constraint") {
						auto constraint = readGenericConstraint();
						if (constraint)
							m_mesh->m_constraints.push_back(constraint);
					} else if (name == "stiffspring-constraint") {
						auto constraint = readStiffSpringConstraint();
						if (constraint)
							m_mesh->m_constraints.push_back(constraint);
					} else if (name == "conetwist-constraint") {
						auto constraint = readConeTwistConstraint();
						if (constraint)
							m_mesh->m_constraints.push_back(constraint);
					} else if (name == "generic-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultGenericConstraintTemplate = getGenericConstraintTemplate(extends);
						readGenericConstraintTemplate(defaultGenericConstraintTemplate);
						m_genericConstraintTemplates[clsname] = defaultGenericConstraintTemplate;
					} else if (name == "stiffspring-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultStiffSpringConstraintTemplate = getStiffSpringConstraintTemplate(extends);
						readStiffSpringConstraintTemplate(defaultStiffSpringConstraintTemplate);
						m_stiffSpringConstraintTemplates[clsname] = defaultStiffSpringConstraintTemplate;
					} else if (name == "conetwist-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultConeTwistConstraintTemplate = getConeTwistConstraintTemplate(extends);
						readConeTwistConstraintTemplate(defaultConeTwistConstraintTemplate);
						m_coneTwistConstraintTemplates[clsname] = defaultConeTwistConstraintTemplate;
					} else if (name == "shape") {
						auto attrName = m_reader->getAttribute("name");
						auto shape = readShape();
						if (shape) {
							m_shapeRefs.push_back(shape);
							m_shapes.insert(std::make_pair(attrName, shape));
						}
					} else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
		} catch (const std::string& err) {
			logger::error("xml parse error - {}", err.c_str());
			return nullptr;
		}

		if (m_deferredBuilds.size() > 2) {
			tbb::parallel_for_each(
				m_deferredBuilds.begin(), m_deferredBuilds.end(),
				[](const DeferredBuild& db) {
					if (db.vertexShape)
						db.vertexShape->autoGen();
					db.body->finishBuild();
				});
		} else if (!m_deferredBuilds.empty()) {
			for (const auto& db : m_deferredBuilds) {
				if (db.vertexShape)
					db.vertexShape->autoGen();
				db.body->finishBuild();
			}
		}

		m_deferredBuilds.clear();

		if (m_reader->GetErrorCode() != Xml::ErrorCode::None) {
			logger::error("xml parse error - {}", m_reader->GetErrorMessage());
			return nullptr;
		}

		m_mesh->m_skeleton = hdt::make_nismart(m_skeleton);
		std::array<SkyrimBone*, SkyrimSystem::gtMotionNames.size()> motionBones{};
		for (size_t i = 0; i < motionBones.size(); ++i) {
			auto* bone = findBoneFromIndex(getRenamedBone(SkyrimSystem::gtMotionNames[i]));
			if (!bone || bone->m_rig.isStaticOrKinematicObject())
				break;
			motionBones[i] = bone;
		}
		if (motionBones.back()) {
			m_mesh->m_gtMotionBones = motionBones;
			m_mesh->m_gtMotionXml = path;
			const auto& names = SkyrimSystem::gtTraceBoneNames();
			for (size_t i = 0; i < names.size(); ++i) {
				auto* bone = findBoneFromIndex(getRenamedBone(names[i].c_str()));
				if (!bone || bone->m_rig.isStaticOrKinematicObject() != (i < SkyrimSystem::gtTraceAnchorCount))
					break;
				m_mesh->m_gtTraceBones[i] = bone;
			}
		}
		if (m_mesh->m_liveGeometryGT) {
			// Keep the bind metadata available for a later blend back to the legacy geometry profile.
			if (!initializeGTStandingReference() || (!uniformGTTissue && !m_mesh->queueGTStandingReference())) {
				m_mesh->m_gtReferenceLinks.clear();
				if (!uniformGTTissue)
					logger::warn("[ShapePhysicsLive] GT progressive Z disabled: complete canonical Softbody rig unavailable for {}", path);
			}
			if (uniformGTTissue)
				logger::info("[ShapePhysicsLive] Using GT uniform tissue without progressive Z for {}", path);
		}
		m_mesh->m_shapeRefs.swap(m_shapeRefs);
		std::sort(m_mesh->m_bones.begin(), m_mesh->m_bones.end(), [](const auto& a, const auto& b) {
			return static_cast<SkyrimBone*>(a.get())->m_depth < static_cast<SkyrimBone*>(b.get())->m_depth;
		});

		// Restore the original pose to avoid a visual 1 Havok tick T-pose (only for visual reasons, it won't break anything otherwise)
		for (auto& [node, transform] : savedPoses) node->local = transform;
		if (!savedPoses.empty()) {
			RE::NiUpdateData updateData;
			skeleton->Update(updateData);
		}

		if (!m_mesh->valid())
			return nullptr;
		if (m_mesh->m_liveGeometryGT && m_mesh->m_liveBones.size() == geometryBoneCount &&
		    !m_mesh->m_liveConstraints.empty() && std::ranges::all_of(m_mesh->m_liveBones, [](const auto& item) {
			    return item.bone && !item.bone->m_rig.isStaticOrKinematicObject();
		    })) {
			// readConstraint records the XML entry before resolving its Bullet endpoints.
			// Require every authored chest link, including parallel links with different frames.
			bool foundChest = false;
			const bool complete = std::ranges::all_of(m_mesh->m_liveConstraints, [&](const auto& attached) {
				if (liveZone(attached.source, LiveProfileKind::GT) != LiveZone::Chest)
					return true;
				foundChest = true;
				return attached.constraint != nullptr;
			});
			if (foundChest && complete) {
				m_mesh->m_liveTarget = path;
				if (!m_mesh->m_gtStandingJob)
					m_mesh->m_liveAppliedTarget = path;
			}
		}
		return m_mesh;
	}

	const std::array<std::string, SkyrimSystem::gtTraceBoneCount>& SkyrimSystem::gtTraceBoneNames()
	{
		static const auto names = [] {
			std::array<std::string, gtTraceBoneCount> result;
			size_t i = 0;
			for (const char side : { 'L', 'R' })
				for (int chain = 1; chain <= 8; ++chain)
					result[i++] = fmt::format("Soft_{}{}00", side, chain);
			for (const char side : { 'L', 'R' }) {
				for (int chain = 1; chain <= 8; ++chain)
					for (int layer = 1; layer <= 3; ++layer)
						result[i++] = fmt::format("Soft_{}{}0{}", side, chain, layer);
				result[i++] = fmt::format("Soft_{}N00", side);
			}
			return result;
		}();
		return names;
	}

	bool SkyrimSystemCreator::initializeGTStandingReference()
	{
		const auto& bones = m_mesh->m_gtTraceBones;
		if (!bones.back())
			return false;
		const auto* shape = castBSTriShape(findObject(m_model, "Softbody"));
		if (!shape)
			return false;
		const auto& instance = shape->GetGeometryRuntimeData().skinInstance;
		if (!instance || !instance->skinData || !instance->bones)
			return false;
		const auto& data = instance->skinData;
		std::array<bool, SkyrimSystem::gtTraceBoneCount> found{};
		auto& bind = m_mesh->m_gtReferenceBones;
		for (uint32_t j = 0; j < data->GetBoneCount(); ++j) {
			for (size_t i = SkyrimSystem::gtTraceAnchorCount; i < bones.size(); ++i) {
				if (instance->bones[j] != bones[i]->m_node.get())
					continue;
				const auto pose = convertNi(data->GetBoneDataSkinToBone(j)).inverse();
				if (found[i] || !isSafeLiveScale(pose.getScale()) || btFabs(pose.getScale() - 1.f) > .001f)
					return false;
				bind[i] = pose;
				found[i] = true;
				break;
			}
		}
		const auto& names = SkyrimSystem::gtTraceBoneNames();
		for (size_t i = 0; i < SkyrimSystem::gtTraceAnchorCount; ++i) {
			auto child = names[i];
			child.back() = '1';
			const auto it = std::ranges::find(names, child);
			if (it == names.end())
				return false;
			const auto j = static_cast<size_t>(it - names.begin());
			if (!found[j] || bones[j]->m_node->parent != bones[i]->m_node.get())
				return false;
			// The unweighted anchor is the immediate parent of its authored 01
			// bone. Recover the anchor in skin bind space, without any world pose.
			bind[i] = bind[j] * convertNi(bones[j]->m_node->local).inverse();
			if (!isSafeLiveScale(bind[i].getScale()) || btFabs(bind[i].getScale() - 1.f) > .001f)
				return false;
			found[i] = true;
		}
		if (!std::ranges::all_of(found, [](bool present) { return present; }))
			return false;
		std::array<bool, SkyrimSystem::gtProgressiveCount> radial{};
		auto& links = m_mesh->m_gtReferenceLinks;
		for (size_t i = 0; i < m_mesh->m_liveConstraints.size(); ++i) {
			const auto& attached = m_mesh->m_liveConstraints[i];
			const auto& source = attached.source;
			const auto a = std::ranges::find(names, source.bodyA);
			const auto b = std::ranges::find(names, source.bodyB);
			if (a == names.end() || b == names.end())
				continue;
			if (!attached.constraint)
				return false;
			const auto ia = static_cast<size_t>(a - names.begin());
			const auto ib = static_cast<size_t>(b - names.begin());
			const auto low = std::min((*a)[8], (*b)[8]);
			const auto high = std::max((*a)[8], (*b)[8]);
			int stage = -1;
			if (a->compare(0, 8, *b, 0, 8) == 0 && low >= '1' && high <= '3' && high - low == 1) {
				stage = low - '1';
				const auto key = static_cast<size_t>(((*a)[5] == 'R' ? 16 : 0) + ((*a)[6] - '1') * 2 + stage);
				if (radial[key] || !source.linearEnabled || source.parameters.linearStiffness.z() <= 0.f)
					return false;
				radial[key] = true;
			}
			btTransform fa, fb;
			calcFrame(static_cast<FrameType>(source.frameType), source.authoredFrame, bind[ia], bind[ib], fa, fb);
			if (source.reversed)
				links.push_back({ i, ib, ia, fb, fa, stage });
			else
				links.push_back({ i, ia, ib, fa, fb, stage });
		}
		return (links.size() == 112 || links.size() == 128) &&
		       std::ranges::all_of(radial, [](bool present) { return present; });
	}

	RE::BSTSmartPointer<ConstraintGroup> SkyrimSystemCreator::readConstraintGroup()
	{
		RE::BSTSmartPointer<ConstraintGroup> ret = RE::make_smart<ConstraintGroup>();

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();

				if (name == "generic-constraint") {
					auto constraint = readGenericConstraint();
					if (constraint)
						ret->m_constraints.push_back(constraint);
				} else if (name == "stiffspring-constraint") {
					auto constraint = readStiffSpringConstraint();
					if (constraint)
						ret->m_constraints.push_back(constraint);
				} else if (name == "conetwist-constraint") {
					auto constraint = readConeTwistConstraint();
					if (constraint)
						ret->m_constraints.push_back(constraint);
				} else if (name == "generic-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultGenericConstraintTemplate = getGenericConstraintTemplate(extends);
					readGenericConstraintTemplate(defaultGenericConstraintTemplate);
					m_genericConstraintTemplates[clsname] = defaultGenericConstraintTemplate;
				} else if (name == "stiffspring-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultStiffSpringConstraintTemplate = getStiffSpringConstraintTemplate(extends);
					readStiffSpringConstraintTemplate(defaultStiffSpringConstraintTemplate);
					m_stiffSpringConstraintTemplates[clsname] = defaultStiffSpringConstraintTemplate;
				} else if (name == "conetwist-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultConeTwistConstraintTemplate = getConeTwistConstraintTemplate(extends);
					readConeTwistConstraintTemplate(defaultConeTwistConstraintTemplate);
					m_coneTwistConstraintTemplates[clsname] = defaultConeTwistConstraintTemplate;
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
		return ret;
	}

	void SkyrimSystemCreator::readBoneTemplate(BoneTemplate& cinfo)
	{
		bool clearCollide = true;
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (name == "mass")
					cinfo.m_mass = m_reader->readFloat();
				else if (name == "inertia")
					cinfo.m_localInertia = m_reader->readVector3();
				else if (name == "centerOfMassTransform")
					cinfo.m_centerOfMassTransform = m_reader->readTransform();
				else if (name == "linearDamping")
					cinfo.m_linearDamping = m_reader->readFloat();
				else if (name == "angularDamping")
					cinfo.m_angularDamping = m_reader->readFloat();
				else if (name == "friction")
					cinfo.m_friction = m_reader->readFloat();
				else if (name == "rollingFriction")
					cinfo.m_rollingFriction = m_reader->readFloat();
				else if (name == "restitution")
					cinfo.m_restitution = m_reader->readFloat();
				else if (name == "margin-multiplier")
					cinfo.m_marginMultipler = m_reader->readFloat();
				else if (name == "shape") {
					auto shape = readShape();
					if (shape) {
						m_shapeRefs.push_back(shape);
						cinfo.m_collisionShape = shape.get();
					} else
						cinfo.m_collisionShape = BoneTemplate::emptyShape;
				} else if (name == "collision-filter")
					cinfo.m_collisionFilter = m_reader->readInt();
				else if (name == "can-collide-with-bone") {
					if (clearCollide) {
						cinfo.m_canCollideWithBone.clear();
						cinfo.m_noCollideWithBone.clear();
						clearCollide = false;
					}
					cinfo.m_canCollideWithBone.push_back(m_reader->readText());
				} else if (name == "no-collide-with-bone") {
					if (clearCollide) {
						cinfo.m_canCollideWithBone.clear();
						cinfo.m_noCollideWithBone.clear();
						clearCollide = false;
					}
					cinfo.m_noCollideWithBone.push_back(m_reader->readText());
				} else if (name == "gravity-factor") {
					cinfo.m_gravityFactor = btClamped(m_reader->readFloat(), 0.0f, 1.0f);
				} else if (name == "wind-factor") {
					cinfo.m_windFactor = std::max(m_reader->readFloat(), 0.0f);
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	std::shared_ptr<btCollisionShape> SkyrimSystemCreator::readShape()
	{
		auto typeStr = m_reader->getAttribute("type");
		if (typeStr == "ref") {
			auto shapeName = m_reader->getAttribute("name");
			m_reader->skipCurrentElement();
			auto iter = m_shapes.find(shapeName);
			if (iter != m_shapes.end())
				return iter->second;
			logger::warn("unknown shape - {}", shapeName.c_str());
			return nullptr;
		}
		if (typeStr == "box") {
			btVector3 halfExtend(0, 0, 0);
			float margin = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "halfExtend")
						halfExtend = m_reader->readVector3();
					else if (name == "margin")
						margin = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			auto ret = std::make_shared<btBoxShape>(halfExtend);
			ret->setMargin(margin);
			return ret;
		}
		if (typeStr == "sphere") {
			float radius = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "radius")
						radius = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			return std::make_shared<btSphereShape>(radius);
		}
		if (typeStr == "capsule") {
			float radius = 0;
			float height = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "radius")
						radius = m_reader->readFloat();
					else if (name == "height")
						height = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			return std::make_shared<btCapsuleShape>(radius, height);
		}
		if (typeStr == "hull") {
			float margin = 0;
			auto ret = std::make_shared<btConvexHullShape>();
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "point")
						ret->addPoint(m_reader->readVector3(), false);
					else if (name == "margin")
						margin = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			ret->recalcLocalAabb();
			return ret->getNumPoints() ? ret : nullptr;
		}
		if (typeStr == "cylinder") {
			float height = 0;
			float radius = 0;
			float margin = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "height")
						height = m_reader->readFloat();
					else if (name == "radius")
						radius = m_reader->readFloat();
					else if (name == "margin")
						margin = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}

			if (radius >= 0 && height >= 0) {
				auto ret = std::make_shared<btCylinderShape>(btVector3(radius, height, radius));
				ret->setMargin(margin);
				return ret;
			}
			return nullptr;
		}
		if (typeStr == "compound") {
			auto ret = std::make_shared<btCompoundShape>();
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					if (m_reader->GetName() == "child") {
						btTransform tr;
						std::shared_ptr<btCollisionShape> shape;

						while (m_reader->Inspect()) {
							if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
								if (m_reader->GetName() == "transform") {
									tr = m_reader->readTransform();
								} else if (m_reader->GetName() == "shape") {
									shape = readShape();
								} else {
									logger::warn("unknown element - {}", m_reader->GetName().c_str());
									m_reader->skipCurrentElement();
								}
							} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
								break;
						}

						if (shape) {
							ret->addChildShape(tr, shape.get());
							m_shapeRefs.push_back(shape);
						}
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			return ret->getNumChildShapes() ? ret : nullptr;
		}
		logger::warn("Unknown shape type {}", typeStr.c_str());
		return nullptr;
	}

	void SkyrimSystemCreator::readOrUpdateBone()
	{
		const auto sourceName = m_reader->getAttribute("name");
		RE::BSFixedString name = getRenamedBone(sourceName);
		SkyrimSystem::AttachedLiveBone* attached = nullptr;
		if ((m_mesh->m_liveGeometryGT && isGeometryMovingBone(sourceName)) ||
		    (m_mesh->m_liveProfileKind == LiveProfileKind::ThreeBA && is3BAChestBone(sourceName))) {
			m_mesh->m_liveBones.push_back({ sourceName, m_reader->getAttribute("template", ""), nullptr });
			attached = &m_mesh->m_liveBones.back();
		}
		if (findBoneFromIndex(name)) {
			logger::warn("Bone {} already exists, skipped", name.c_str());
			m_reader->skipCurrentElement();
			return;
		}

		RE::BSFixedString cls = m_reader->getAttribute("template", "");
		auto* bone = createBoneFromNodeName(name, cls, true);
		if (!bone)
			m_reader->skipCurrentElement();
		if (attached) {
			attached->bone = bone;
			if (bone && m_mesh->m_liveGeometryGT)
				bone->m_localCollisionCenter = true;
		}
	}

	SkyrimBone* SkyrimSystemCreator::createBoneFromNodeName(const RE::BSFixedString& bodyName, const RE::BSFixedString& templateName, const bool readTemplate)
	{
		auto node = findObjectByName(bodyName);
		if (node) {
			logger::info("Found node named {}, creating bone", bodyName.c_str());
			auto boneTemplate = getBoneTemplate(templateName);
			if (readTemplate)
				readBoneTemplate(boneTemplate);
			auto bone = new SkyrimBone(node->name.c_str(), node, this->m_skeleton, boneTemplate);
			bone->m_localToRig = boneTemplate.m_centerOfMassTransform;
			bone->m_rigToLocal = boneTemplate.m_centerOfMassTransform.inverse();
			bone->m_marginMultipler = boneTemplate.m_marginMultipler;
			bone->m_gravityFactor = boneTemplate.m_gravityFactor;
			bone->m_windFactor = boneTemplate.m_windFactor;

			bone->readTransform(RESET_PHYSICS);

			m_mesh->m_bones.push_back(hdt::make_smart(bone));
			indexBone(bone);
			return bone;
		}
		logger::warn("Node named {} doesn't exist, skipped, no bone created", bodyName.c_str());
		return nullptr;
	}

	std::pair<RE::BSTSmartPointer<SkyrimBody>, SkyrimSystemCreator::VertexOffsetMap> SkyrimSystemCreator::generateMeshBody(const std::string name, DefaultBBP::NameSet_t* names)
	{
		RE::BSTSmartPointer<SkyrimBody> body = RE::make_smart<SkyrimBody>();
		body->m_name = name;

		int vertexStart = 0;
		int boneStart = 0;

		VertexOffsetMap vertexOffsetMap;

		for (auto& meshName : *names) {
			// We wouldn't find the trishape here without the ActorManager::fixArmorNameMaps() fix when the related bug happens
			// (for example when doing the smp reset).
			auto* triShape = castBSTriShape(findObject(m_model, meshName.c_str()));
			auto* dynamicShape = castBSDynamicTriShape(findObject(m_model, meshName.c_str()));
			if (!triShape) {
				continue;
			}

			if (!triShape->GetGeometryRuntimeData().skinInstance) {
				continue;
			}

			RE::NiSkinInstance* skinInstance = triShape->GetGeometryRuntimeData().skinInstance.get();
			RE::NiSkinData* skinData = skinInstance->skinData.get();
			for (uint32_t boneIdx = 0; boneIdx < skinData->GetBoneCount(); ++boneIdx) {
				auto node = skinInstance->bones[boneIdx];
				if (!node) {
					continue;
				}
				const auto& boneBound = skinData->GetBoneDataBound(boneIdx);
				auto boundingSphere = BoundingSphere(convertNi(boneBound.center), boneBound.radius);
				const RE::BSFixedString& boneName = node->name;
				auto bone = static_cast<SkinnedMeshBone*>(findBoneFromIndex(boneName));
				if (!bone) {
					auto defaultBoneInfo = getBoneTemplate("");
					auto newBone = new SkyrimBone(boneName, node->AsNode(), this->m_skeleton, defaultBoneInfo);
					m_mesh->m_bones.push_back(hdt::make_smart(newBone));
					indexBone(newBone);
					bone = newBone;
					logger::info("Created bone {} added to body {}, created without default values", boneName.c_str(), name);
				}

				body->addBone(bone, convertNi(skinData->GetBoneDataSkinToBone(boneIdx)), boundingSphere);
			}

			RE::NiSkinPartition* skinPartition = triShape->GetGeometryRuntimeData().skinInstance->skinPartition.get();
			body->m_vertices.resize(vertexStart + skinPartition->vertexCount);

			// vertices data are all the same in every partitions
			auto partition = skinPartition->partitions.data();
			auto vFlags = partition->vertexDesc.GetFlags();
			auto vSize = partition->vertexDesc.GetSize();
			auto vertexBlock = partition->buffData->rawVertexData;

			uint8_t* dynamicVData = nullptr;
			if (dynamicShape)
				dynamicVData = static_cast<uint8_t*>(dynamicShape->GetDynamicTrishapeRuntimeData().dynamicData);

			uint8_t boneOffset = 0;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_VERTEX)
				boneOffset += 16;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_UV)
				boneOffset += 4;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_UV_2)
				boneOffset += 4;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_NORMAL)
				boneOffset += 4;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_TANGENT)
				boneOffset += 4;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_COLORS)
				boneOffset += 4;

			for (uint32_t j = 0; j < skinPartition->vertexCount; ++j) {
				RE::NiPoint3* vertexPos;

				if (dynamicShape && dynamicVData)
					vertexPos = reinterpret_cast<RE::NiPoint3*>(&dynamicVData[j * 16]);
				else
					vertexPos = reinterpret_cast<RE::NiPoint3*>(&vertexBlock[j * vSize]);

				body->m_vertices[j + vertexStart].m_skinPos = convertNi(*vertexPos);

				SkyrimSystem::BoneData* boneData = reinterpret_cast<SkyrimSystem::BoneData*>(&vertexBlock[j * vSize + boneOffset]);

#if defined(__AVX2__) || defined(__AVX512F__)
				// batch convert all 4 bone weights FP16 to FP32 through F16C hardware instruction
				__m128i halfWeights = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(boneData->boneWeights));
				_mm_storeu_ps(body->m_vertices[j + vertexStart].m_weight, _mm_cvtph_ps(halfWeights));
				// cleanse garbage NIF data for unused bones
				for (int k = partition->bonesPerVertex; k < 4; ++k) {
					body->m_vertices[j + vertexStart].m_weight[k] = 0.0f;
				}
#else
				for (int k = 0; k < partition->bonesPerVertex && k < 4; ++k) {
					__float32(&body->m_vertices[j + vertexStart].m_weight[k], boneData->boneWeights[k]);
				}
#endif

				for (int k = 0; k < partition->bonesPerVertex && k < 4; ++k) {
					auto localBoneIndex = boneData->boneIndices[k];
					assert(localBoneIndex < body->m_skinnedBones.size());
					body->m_vertices[j + vertexStart].m_boneIdx[k] = localBoneIndex + boneStart;
				}
			}

			vertexOffsetMap.insert({ meshName, vertexStart });
			boneStart = static_cast<int>(body->m_skinnedBones.size());
			vertexStart = static_cast<int>(body->m_vertices.size());
		}

		if (0 == vertexStart) {
			m_reader->skipCurrentElement();
			return { nullptr, {} };
		}

		for (auto& i : body->m_vertices)
			i.sortWeight();

		return { body, vertexOffsetMap };
	}

	RE::BSTSmartPointer<SkyrimBody> SkyrimSystemCreator::readPerVertexShape(DefaultBBP::NameMap_t meshNameMap)
	{
		auto name = m_reader->getAttribute("name");
		auto it = meshNameMap.find(name);
		auto names = (it == meshNameMap.end()) ? DefaultBBP::NameSet_t({ name }) : it->second;

		auto body = generateMeshBody(name, &names).first;
		if (!body) {
			return nullptr;
		}

		auto shape = RE::make_smart<PerVertexShape>(body.get());

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto nodeName = m_reader->GetName();
				if (nodeName == "priority") {
					logger::warn("priority is deprecated and no longer used");
					m_reader->skipCurrentElement();
				} else if (nodeName == "margin") {
					shape->m_shapeProp.margin = m_reader->readFloat();
				} else if (nodeName == "shared") {
					auto str = m_reader->readText();
					if (str == "public") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					} else if (str == "internal") {
						body->m_shared = SkyrimBody::SharedType::SHARED_INTERNAL;
					} else if (str == "external") {
						body->m_shared = SkyrimBody::SharedType::SHARED_EXTERNAL;
					} else if (str == "private") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PRIVATE;
					} else {
						logger::warn("unknown shared value, use default value \"public\"");
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					}
				} else if (nodeName == "tag") {
					body->m_tags.push_back(m_reader->readText());
				} else if (nodeName == "can-collide-with-tag") {
					body->m_canCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "no-collide-with-tag") {
					body->m_noCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone)
						body->m_canCollideWithBones.insert(bone);
				} else if (nodeName == "no-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone)
						body->m_noCollideWithBones.insert(bone);
				} else if (nodeName == "weight-threshold") {
					auto boneName = m_reader->getAttribute("bone");
					float wt = m_reader->readFloat();
					for (int i = 0; i < body->m_skinnedBones.size(); ++i) {
						if (body->m_skinnedBones[i].ptr->m_name == getRenamedBone(boneName)) {
							body->m_skinnedBones[i].weightThreshold = wt;
							break;
						}
					}
				} else if (nodeName == "disable-tag") {
					body->m_disableTag = m_reader->readText();
				} else if (nodeName == "disable-priority") {
					body->m_disablePriority = m_reader->readInt();
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}

		m_deferredBuilds.push_back({ body.get(), shape.get() });

		return body;
	}

	RE::BSTSmartPointer<SkyrimBody> SkyrimSystemCreator::readPerTriangleShape(DefaultBBP::NameMap_t* meshNameMap)
	{
		auto name = m_reader->getAttribute("name");
		auto it = meshNameMap->find(name);
		auto names = (it == meshNameMap->end()) ? DefaultBBP::NameSet_t({ name }) : it->second;

		auto bodyData = generateMeshBody(name, &names);
		auto body = bodyData.first;
		auto vertexOffsetMap = bodyData.second;
		if (!body)
			return nullptr;

		auto shape = RE::make_smart<PerTriangleShape>(body.get());

		for (auto entry : vertexOffsetMap) {
			auto* g = castBSTriShape(findObject(m_model, entry.first.c_str()));
			if (!g) {
				continue;
			}
			if (g->GetGeometryRuntimeData().skinInstance) {
				int offset = entry.second;
				RE::NiSkinPartition* skinPartition = g->GetGeometryRuntimeData().skinInstance->skinPartition.get();
				for (int i = 0; i < skinPartition->partitions.size(); ++i) {
					auto& partition = skinPartition->partitions[i];
					for (int j = 0; j < partition.triangles; ++j)
						shape->addTriangle(partition.triList[j * 3] + offset, partition.triList[j * 3 + 1] + offset,
							partition.triList[j * 3 + 2] + offset);
				}
			} else {
				logger::warn("Shape {} has no skin data, skipped", entry.first.c_str());
				return nullptr;
			}
		}

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto nodeName = m_reader->GetName();
				if (nodeName == "priority") {
					logger::warn("priority is deprecated and no longer used");
					m_reader->skipCurrentElement();
				} else if (nodeName == "margin") {
					shape->m_shapeProp.margin = m_reader->readFloat();
				} else if (nodeName == "shared") {
					auto str = m_reader->readText();
					if (str == "public") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					} else if (str == "internal") {
						body->m_shared = SkyrimBody::SharedType::SHARED_INTERNAL;
					} else if (str == "external") {
						body->m_shared = SkyrimBody::SharedType::SHARED_EXTERNAL;
					} else if (str == "private") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PRIVATE;
					} else {
						logger::warn("unknown shared value, use default value \"public\"");
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					}
				} else if (nodeName == "prenetration" || nodeName == "penetration") {
					shape->m_shapeProp.penetration = m_reader->readFloat();
				} else if (nodeName == "tag") {
					body->m_tags.push_back(m_reader->readText());
				} else if (nodeName == "no-collide-with-tag") {
					body->m_noCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-tag") {
					body->m_canCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone)
						body->m_canCollideWithBones.insert(bone);
				} else if (nodeName == "no-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone)
						body->m_noCollideWithBones.insert(bone);
				} else if (nodeName == "weight-threshold") {
					auto boneName = m_reader->getAttribute("bone");
					float wt = m_reader->readFloat();
					for (int i = 0; i < body->m_skinnedBones.size(); ++i) {
						if (body->m_skinnedBones[i].ptr->m_name == getRenamedBone(boneName)) {
							body->m_skinnedBones[i].weightThreshold = wt;
						}
					}
				} else if (nodeName == "disable-tag") {
					body->m_disableTag = m_reader->readText();
				} else if (nodeName == "disable-priority") {
					body->m_disablePriority = m_reader->readInt();
				} else {
					logger::warn("unknown element - {}", nodeName.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}

		m_deferredBuilds.push_back({ body.get(), nullptr });

		return body;
	}

	void SkyrimSystemCreator::readFrameLerp(btTransform& tr)
	{
		tr.setIdentity();
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (name == "translationLerp")
					tr.getOrigin().setX(m_reader->readFloat());
				else if (name == "rotationLerp")
					tr.getOrigin().setY(m_reader->readFloat());
				else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	bool SkyrimSystemCreator::parseFrameType(const std::string& name, FrameType& frameType, btTransform& frame)
	{
		if (name == "frameInA") {
			frameType = FrameInA;
			frame = m_reader->readTransform();
		} else if (name == "frameInB") {
			frameType = FrameInB;
			frame = m_reader->readTransform();
		} else if (name == "frameInLerp") {
			frameType = FrameInLerp;
			readFrameLerp(frame);
		} else
			return false;
		return true;
	}

	void SkyrimSystemCreator::readGenericConstraintTemplate(GenericConstraintTemplate& dest)
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (parseFrameType(name, dest.frameType, dest.frame))
					;
				else if (name == "enableLinearSprings")
					dest.enableLinearSprings = m_reader->readBool();
				else if (name == "enableAngularSprings")
					dest.enableAngularSprings = m_reader->readBool();
				else if (name == "linearStiffnessLimited")
					dest.linearStiffnessLimited = m_reader->readBool();
				else if (name == "angularStiffnessLimited")
					dest.angularStiffnessLimited = m_reader->readBool();

				else if (name == "springDampingLimited")
					dest.springDampingLimited = m_reader->readBool();
				else if (name == "linearNonHookeanDamping")
					dest.linearNonHookeanDamping = m_reader->readVector3();
				else if (name == "angularNonHookeanDamping")
					dest.angularNonHookeanDamping = m_reader->readVector3();
				else if (name == "linearNonHookeanStiffness")
					dest.linearNonHookeanStiffness = m_reader->readVector3();
				else if (name == "angularNonHookeanStiffness")
					dest.angularNonHookeanStiffness = m_reader->readVector3();

				else if (name == "linearMotors")
					dest.linearMotors = m_reader->readBool();
				else if (name == "angularMotors")
					dest.angularMotors = m_reader->readBool();
				else if (name == "linearServoMotors")
					dest.linearServoMotors = m_reader->readBool();
				else if (name == "angularServoMotors")
					dest.angularServoMotors = m_reader->readBool();
				else if (name == "linearTargetVelocity")
					dest.linearTargetVelocity = m_reader->readVector3();
				else if (name == "angularTargetVelocity")
					dest.angularTargetVelocity = m_reader->readVector3();
				else if (name == "linearMaxMotorForce")
					dest.linearMaxMotorForce = m_reader->readVector3();
				else if (name == "angularMaxMotorForce")
					dest.angularMaxMotorForce = m_reader->readVector3();

				else if (name == "stopERP")
					dest.stopERP = m_reader->readFloat();
				else if (name == "stopCFM")
					dest.stopCFM = m_reader->readFloat();
				else if (name == "motorERP")
					dest.motorERP = m_reader->readFloat();
				else if (name == "motorCFM")
					dest.motorCFM = m_reader->readFloat();

				else if (name == "useLinearReferenceFrameA")
					dest.useLinearReferenceFrameA = m_reader->readBool();
				else if (name == "linearLowerLimit")
					dest.linearLowerLimit = m_reader->readVector3();
				else if (name == "linearUpperLimit")
					dest.linearUpperLimit = m_reader->readVector3();
				else if (name == "angularLowerLimit")
					dest.angularLowerLimit = m_reader->readVector3();
				else if (name == "angularUpperLimit")
					dest.angularUpperLimit = m_reader->readVector3();
				else if (name == "linearStiffness")
					dest.linearStiffness = m_reader->readVector3();
				else if (name == "angularStiffness")
					dest.angularStiffness = m_reader->readVector3();
				else if (name == "linearDamping")
					dest.linearDamping = m_reader->readVector3();
				else if (name == "angularDamping")
					dest.angularDamping = m_reader->readVector3();
				else if (name == "linearEquilibrium")
					dest.linearEquilibrium = m_reader->readVector3();
				else if (name == "angularEquilibrium")
					dest.angularEquilibrium = m_reader->readVector3();
				else if (name == "linearBounce")
					dest.linearBounce = m_reader->readVector3();
				else if (name == "angularBounce")
					dest.angularBounce = m_reader->readVector3();
				else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	bool SkyrimSystemCreator::findBones(const RE::BSFixedString& bodyAName, const RE::BSFixedString& bodyBName, SkyrimBone*& bodyA, SkyrimBone*& bodyB)
	{
		bodyA = findBoneFromIndex(bodyAName);
		bodyB = findBoneFromIndex(bodyBName);

		if (!bodyA) {
			logger::warn("constraint {} <-> {} : bone for bodyA doesn't exist, will try to create it", bodyAName.c_str(), bodyBName.c_str());
			bodyA = createBoneFromNodeName(bodyAName);
			if (!bodyA) {
				m_reader->skipCurrentElement();
				return false;
			}
		}
		if (!bodyB) {
			logger::warn("constraint {} <-> {} : bone for bodyB doesn't exist, will try to create it", bodyAName.c_str(), bodyBName.c_str());
			bodyB = createBoneFromNodeName(bodyBName);
			if (!bodyB) {
				m_reader->skipCurrentElement();
				return false;
			}
		}
		if (bodyA == bodyB) {
			logger::warn("constraint between same object {} <-> {}, skipped", bodyAName.c_str(), bodyBName.c_str());
			m_reader->skipCurrentElement();
			return false;
		}

		if (bodyA->m_rig.isKinematicObject() && bodyB->m_rig.isKinematicObject()) {
			logger::warn("constraint between two kinematic object {} <-> {}, skipped", bodyAName.c_str(), bodyBName.c_str());
			m_reader->skipCurrentElement();
			return false;
		}

		logger::info("OK: constraint between object {} <-> {}", bodyAName.c_str(), bodyBName.c_str());
		return true;
	}

	btQuaternion rotFromAtoB(const btVector3& a, const btVector3& b)
	{
		auto axis = a.cross(b);
		if (axis.fuzzyZero())
			return btQuaternion::getIdentity();
		float sinA = axis.length();
		float cosA = a.dot(b);
		float angle = btAtan2(cosA, sinA);
		return btQuaternion(axis, angle);
	}

	void SkyrimSystemCreator::calcFrame(FrameType type, const btTransform& frame, const btQsTransform& trA, const btQsTransform& trB, btTransform& frameA, btTransform& frameB)
	{
		btQsTransform frameInWorld;
		switch (type) {
		case FrameInA:
			frameA = frame;
			frameInWorld = trA * frame;
			frameB = (trB.inverse() * frameInWorld).asTransform();
			break;
		case FrameInB:
			frameB = frame;
			frameInWorld = trB * frameB;
			frameA = (trA.inverse() * frameInWorld).asTransform();
			break;
		case FrameInLerp:
			{
				auto trans = trA.getOrigin().lerp(trB.getOrigin(), frame.getOrigin().x());
				auto rot = trA.getBasis().slerp(trB.getBasis(), frame.getOrigin().y());
				frameInWorld = btQsTransform(rot, trans);
				frameA = (trA.inverse() * frameInWorld).asTransform();
				frameB = (trB.inverse() * frameInWorld).asTransform();
				break;
			}
		case AWithXPointToB:
			{
				btMatrix3x3 matr(trA.getBasis());
				frameInWorld = trA;
				auto old = matr.getColumn(0).normalized();
				auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
				auto q = rotFromAtoB(old, a2b);
				frameInWorld.getBasis() *= q;
				frameA = (trA.inverse() * frameInWorld).asTransform();
				frameB = (trB.inverse() * frameInWorld).asTransform();
				break;
			}
		case AWithYPointToB:
			{
				btMatrix3x3 matr(trA.getBasis());
				frameInWorld = trA;
				auto old = matr.getColumn(1).normalized();
				auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
				auto q = rotFromAtoB(old, a2b);
				frameInWorld.getBasis() *= q;
				frameA = (trA.inverse() * frameInWorld).asTransform();
				frameB = (trB.inverse() * frameInWorld).asTransform();
				break;
			}
		case AWithZPointToB:
			{
				btMatrix3x3 matr(trA.getBasis());
				frameInWorld = trA;
				auto old = matr.getColumn(2).normalized();
				auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
				auto q = rotFromAtoB(old, a2b);
				frameInWorld.getBasis() *= q;
				frameA = (trA.inverse() * frameInWorld).asTransform();
				frameB = (trB.inverse() * frameInWorld).asTransform();
				break;
			}
		}
	}

	RE::BSTSmartPointer<Generic6DofConstraint> SkyrimSystemCreator::readGenericConstraint()
	{
		SkyrimSystem::LiveConstraint link{ m_reader->getAttribute("bodyA"), m_reader->getAttribute("bodyB"), m_reader->getAttribute("template", "") };
		auto bodyAName = getRenamedBone(link.bodyA);
		auto bodyBName = getRenamedBone(link.bodyB);
		auto clsname = link.preset;
		const auto liveKind = m_mesh->m_liveProfileKind;
		SkyrimSystem::AttachedLiveConstraint* attached = nullptr;
		if (liveKind != LiveProfileKind::None && liveZone(link, liveKind) != LiveZone::Unsupported) {
			m_mesh->m_liveConstraints.push_back({ std::move(link) });
			attached = &m_mesh->m_liveConstraints.back();
		}

		SkyrimBone *bodyA, *bodyB;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB))
			return nullptr;

		auto trA = bodyA->m_currentTransform;
		auto trB = bodyB->m_currentTransform;

		auto cinfo = getGenericConstraintTemplate(clsname);
		readGenericConstraintTemplate(cinfo);
		btTransform frameA, frameB;
		calcFrame(cinfo.frameType, cinfo.frame, trA, trB, frameA, frameB);

		RE::BSTSmartPointer<Generic6DofConstraint> constraint;
		if (cinfo.useLinearReferenceFrameA) {
			constraint = RE::make_smart<Generic6DofConstraint>(bodyB, bodyA, frameB, frameA);
		} else {
			constraint = RE::make_smart<Generic6DofConstraint>(bodyA, bodyB, frameA, frameB);
		}

		constraint->setLinearLowerLimit(cinfo.linearLowerLimit);
		constraint->setLinearUpperLimit(cinfo.linearUpperLimit);
		constraint->setAngularLowerLimit(cinfo.angularLowerLimit);
		constraint->setAngularUpperLimit(cinfo.angularUpperLimit);
		for (int i = 0; i < 3; ++i) {
			constraint->setStiffness(i, cinfo.linearStiffness[i], cinfo.linearStiffnessLimited);
			constraint->setStiffness(i + 3, cinfo.angularStiffness[i], cinfo.angularStiffnessLimited);
			constraint->setDamping(i, cinfo.linearDamping[i], cinfo.springDampingLimited);
			constraint->setDamping(i + 3, cinfo.angularDamping[i], cinfo.springDampingLimited);

			constraint->setEquilibriumPoint(i, cinfo.linearEquilibrium[i]);
			constraint->setEquilibriumPoint(i + 3, cinfo.angularEquilibrium[i]);

			constraint->setNonHookeanDamping(i, cinfo.linearNonHookeanDamping[i]);
			constraint->setNonHookeanDamping(i + 3, cinfo.angularNonHookeanDamping[i]);
			constraint->setNonHookeanStiffness(i, cinfo.linearNonHookeanStiffness[i]);
			constraint->setNonHookeanStiffness(i + 3, cinfo.angularNonHookeanStiffness[i]);

			constraint->enableSpring(i, cinfo.enableLinearSprings);
			constraint->enableSpring(i + 3, cinfo.enableAngularSprings);

			constraint->enableMotor(i, cinfo.linearMotors);
			constraint->enableMotor(i + 3, cinfo.angularMotors);
			constraint->setServo(i, cinfo.linearServoMotors);
			constraint->setServo(i + 3, cinfo.angularServoMotors);
			// TODO: Test if servo motors go to [0, 0, 0], or whatever equilibrium is.  Provide option to set server motor target.  Hard coded to equilibrium right now.
			constraint->setServoTarget(i, cinfo.linearEquilibrium[i]);
			constraint->setServoTarget(i + 3, cinfo.angularEquilibrium[i]);
			constraint->setTargetVelocity(i, cinfo.linearTargetVelocity[i]);
			constraint->setTargetVelocity(i + 3, cinfo.angularTargetVelocity[i]);
			constraint->setMaxMotorForce(i, cinfo.linearMaxMotorForce[i]);
			constraint->setMaxMotorForce(i + 3, cinfo.angularMaxMotorForce[i]);

			constraint->setParam(BT_CONSTRAINT_ERP, cinfo.motorERP, i);
			constraint->setParam(BT_CONSTRAINT_CFM, cinfo.motorCFM, i);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP, cinfo.stopERP, i);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM, cinfo.stopCFM, i);

			auto rotMotor = constraint->getRotationalLimitMotor(i);
			if (rotMotor) {
				rotMotor->m_motorERP = cinfo.motorERP;
				rotMotor->m_motorCFM = cinfo.motorCFM;
				rotMotor->m_stopERP = cinfo.stopERP;
				rotMotor->m_stopCFM = cinfo.stopCFM;
			}
		}
		constraint->getTranslationalLimitMotor()->m_bounce = cinfo.linearBounce;
		constraint->getRotationalLimitMotor(0)->m_bounce = cinfo.angularBounce[0];
		constraint->getRotationalLimitMotor(1)->m_bounce = cinfo.angularBounce[1];
		constraint->getRotationalLimitMotor(2)->m_bounce = cinfo.angularBounce[2];
		/*constraint->getTranslationalLimitMotor()->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(0)->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(1)->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(2)->m_limitSoftness = 1;*/

		if (attached) {
			readLiveConstraintSettings(attached->source, cinfo, liveKind == LiveProfileKind::ThreeBA, m_mesh->m_liveGeometryGT);
			attached->constraint = constraint.get();
		}

		return constraint;
	}

	void SkyrimSystemCreator::readStiffSpringConstraintTemplate(StiffSpringConstraintTemplate& dest)
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (name == "minDistanceFactor")
					dest.minDistanceFactor = std::max(m_reader->readFloat(), 0.0f);
				else if (name == "maxDistanceFactor")
					dest.maxDistanceFactor = std::max(m_reader->readFloat(), 0.0f);
				else if (name == "stiffness")
					dest.stiffness = std::max(m_reader->readFloat(), 0.0f);
				else if (name == "damping")
					dest.damping = std::max(m_reader->readFloat(), 0.0f);
				else if (name == "equilibrium")
					dest.equilibriumFactor = btClamped(m_reader->readFloat(), 0.0f, 1.0f);
				else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	void SkyrimSystemCreator::readConeTwistConstraintTemplate(ConeTwistConstraintTemplate& dest)
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (parseFrameType(name, dest.frameType, dest.frame))
					;
				else if (name == "swingSpan1" || name == "coneLimit" || name == "limitZ")
					dest.swingSpan1 = std::max(m_reader->readFloat(), 0.f);
				else if (name == "swingSpan2" || name == "planeLimit" || name == "limitY")
					dest.swingSpan2 = std::max(m_reader->readFloat(), 0.f);
				else if (name == "twistSpan" || name == "twistLimit" || name == "limitX")
					dest.twistSpan = std::max(m_reader->readFloat(), 0.f);
				else if (name == "limitSoftness")
					dest.limitSoftness = btClamped(m_reader->readFloat(), 0.f, 1.f);
				else if (name == "biasFactor")
					dest.biasFactor = btClamped(m_reader->readFloat(), 0.f, 1.f);
				else if (name == "relaxationFactor")
					dest.relaxationFactor = btClamped(m_reader->readFloat(), 0.f, 1.f);
				else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	const SkyrimSystemCreator::BoneTemplate& SkyrimSystemCreator::getBoneTemplate(const RE::BSFixedString& name)
	{
		auto iter = m_boneTemplates.find(name);
		if (iter == m_boneTemplates.end())
			return m_boneTemplates[RE::BSFixedString()];
		return iter->second;
	}

	const SkyrimSystemCreator::GenericConstraintTemplate& SkyrimSystemCreator::getGenericConstraintTemplate(const RE::BSFixedString& name)
	{
		auto iter = m_genericConstraintTemplates.find(name);
		if (iter == m_genericConstraintTemplates.end())
			return m_genericConstraintTemplates[RE::BSFixedString()];
		return iter->second;
	}

	const SkyrimSystemCreator::StiffSpringConstraintTemplate& SkyrimSystemCreator::getStiffSpringConstraintTemplate(const RE::BSFixedString& name)
	{
		auto iter = m_stiffSpringConstraintTemplates.find(name);
		if (iter == m_stiffSpringConstraintTemplates.end())
			return m_stiffSpringConstraintTemplates[RE::BSFixedString()];
		return iter->second;
	}

	const SkyrimSystemCreator::ConeTwistConstraintTemplate& SkyrimSystemCreator::getConeTwistConstraintTemplate(const RE::BSFixedString& name)
	{
		auto iter = m_coneTwistConstraintTemplates.find(name);
		if (iter == m_coneTwistConstraintTemplates.end())
			return m_coneTwistConstraintTemplates[RE::BSFixedString()];
		return iter->second;
	}

	RE::BSTSmartPointer<StiffSpringConstraint> SkyrimSystemCreator::readStiffSpringConstraint()
	{
		auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
		auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
		auto clsname = m_reader->getAttribute("template", "");

		SkyrimBone *bodyA, *bodyB;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB))
			return nullptr;

		StiffSpringConstraintTemplate cinfo = getStiffSpringConstraintTemplate(clsname);
		readStiffSpringConstraintTemplate(cinfo);

		RE::BSTSmartPointer<StiffSpringConstraint> constraint = RE::make_smart<StiffSpringConstraint>(bodyA, bodyB);
		constraint->m_minDistance *= cinfo.minDistanceFactor;
		constraint->m_maxDistance *= cinfo.maxDistanceFactor;
		constraint->m_stiffness = cinfo.stiffness;
		constraint->m_damping = cinfo.damping;
		constraint->m_equilibriumPoint = constraint->m_minDistance * cinfo.equilibriumFactor + constraint->m_maxDistance * (1 - cinfo.equilibriumFactor);
		return constraint;
	}

	RE::BSTSmartPointer<ConeTwistConstraint> SkyrimSystemCreator::readConeTwistConstraint()
	{
		auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
		auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
		auto clsname = m_reader->getAttribute("template", "");

		SkyrimBone *bodyA = nullptr, *bodyB = nullptr;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB)) {
			return nullptr;
		}

		auto trA = bodyA->m_currentTransform;
		auto trB = bodyB->m_currentTransform;

		auto cinfo = getConeTwistConstraintTemplate(clsname);
		readConeTwistConstraintTemplate(cinfo);
		btTransform frameA, frameB;
		calcFrame(cinfo.frameType, cinfo.frame, trA, trB, frameA, frameB);

		RE::BSTSmartPointer<ConeTwistConstraint> constraint = RE::make_smart<ConeTwistConstraint>(bodyA, bodyB, frameA, frameB);
		constraint->setLimit(cinfo.swingSpan1, cinfo.swingSpan2, cinfo.twistSpan, cinfo.limitSoftness, cinfo.biasFactor, cinfo.relaxationFactor);

		return constraint;
	}
}

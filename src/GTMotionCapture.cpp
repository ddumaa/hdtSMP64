#include "GTMotionCapture.h"

#include "hdtSkyrimSystem.h"

#include <RE/S/SendHUDMessage.h>
#include <Windows.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <locale>
#include <unordered_map>

namespace hdt::GTMotionCapture
{
	namespace
	{
		using Clock = std::chrono::steady_clock;
		constexpr auto maxDuration = std::chrono::minutes(2);
		constexpr size_t maxFrames = 12000;
		constexpr size_t maxSteps = 12000;
		constexpr size_t maxHits = 400000;
		constexpr size_t maxConstraints = 256;
		constexpr float engagedStiffness = 0.001f;
		constexpr size_t anchorCount = SkyrimSystem::gtTraceAnchorCount;
		constexpr size_t boneCount = SkyrimSystem::gtTraceBoneCount;
		constexpr auto thirdPersonRequired = "GT recording needs an active, settled regular GT body in third person.";

		struct Pose
		{
			std::array<float, 3> position;
			std::array<float, 4> rotation;
			std::array<float, 3> comVelocity;
			std::array<float, 3> angularVelocity;
			float scale;
		};

		struct Frame
		{
			double seconds;
			btVector3 player;
			btVector3 chest;
			btQuaternion rotation;
			std::array<btVector3, SkyrimSystem::gtMotionNames.size()> points;
			std::array<Pose, anchorCount> targets;
		};

		struct Substep
		{
			size_t frame, number;
			float dt;
			double simTime;
			int iterations;
			std::uint16_t progressiveEligible = 0, progressiveEngaged = 0;
			float maxProgressiveStiffness = 0, maxProgressiveDamping = 0;
			std::array<Pose, anchorCount> pre;
			std::array<Pose, boneCount> post;
		};

		struct TrackedConstraint
		{
			Generic6DofConstraint* constraint;
			size_t a, b;
			unsigned lockedMask;
		};

		struct LimitHit
		{
			size_t step, constraint;
			unsigned lower, upper;
		};

		struct PendingFrame
		{
			bool ready = false, overflow = false, stepOpen = false;
			size_t firstStep, firstHit;
			double firstSimTime;
			btVector3 offset;
			std::array<Pose, anchorCount> targets;
		};

		bool recording = false;
		bool transitions = false;
		bool keyHeld = false;
		RE::BSTSmartPointer<SkyrimSystem> source;
		std::string xml;
		bool geometry = false;
		size_t progressiveLinks = 0;
		Clock::time_point start;
		std::vector<Frame> frames;
		std::vector<Substep> steps;
		std::vector<TrackedConstraint> tracked;
		std::vector<LimitHit> hits;
		std::array<Pose, boneCount - anchorCount> initialMoving;
		PendingFrame pending;
		double simTime = 0;

		bool belongsTo(const SkyrimSystem& system, const RE::NiAVObject* root)
		{
			for (auto* node = system.m_skeleton.get(); node; node = node->parent) {
				if (node == root)
					return true;
			}
			return false;
		}

		SkyrimSystem* findBody(const std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems,
			const RE::NiAVObject* root)
		{
			SkyrimSystem* result = nullptr;
			if (!root)
				return result;
			for (const auto& system : systems) {
				auto* candidate = dynamic_cast<SkyrimSystem*>(system.get());
				if (!candidate || !candidate->gtTraceBones().back() || !belongsTo(*candidate, root))
					continue;
				if (result)
					return nullptr;
				result = candidate;
			}
			return result;
		}

		std::array<float, 3> components(const btVector3& value)
		{
			return { value.x(), value.y(), value.z() };
		}

		Pose pose(const SkyrimBone& bone, const btTransform& world)
		{
			const auto q = world.getRotation();
			return { components(world.getOrigin()), { q.x(), q.y(), q.z(), q.w() },
				components(bone.m_rig.getLinearVelocity()), components(bone.m_rig.getAngularVelocity()),
				bone.m_currentTransform.getScale() };
		}

		Pose simulatedPose(const SkyrimBone& bone)
		{
			auto world = bone.m_rig.getWorldTransform() * bone.m_rigToLocal;
			world.getOrigin() += pending.offset;
			return pose(bone, world);
		}

		void discardPendingFrame()
		{
			if (!pending.ready)
				return;
			steps.resize(pending.firstStep);
			hits.resize(pending.firstHit);
			simTime = pending.firstSimTime;
			pending.ready = false;
		}

		struct RuntimeAxis { float lower, upper, equilibrium, stiffness, damping; bool enabled; };
		struct RuntimeJoint {
			size_t a, b;
			btTransform frameA, frameB;
			std::array<RuntimeAxis, 6> axis;
		};
		struct RuntimeBody {
			float inverseMass, gravityFactor, scale, linearDamping, angularDamping;
			btVector3 inverseInertia, gravity;
			btTransform localToRig;
		};
		std::vector<RuntimeJoint> runtimeJoints;
		std::array<RuntimeBody, boneCount> runtimeBodies;
		bool runtimeBodiesReady = false;

		bool lookupConstraints(const btAlignedObjectArray<btTypedConstraint*>& constraints)
		{
			std::unordered_map<const btRigidBody*, size_t> ids;
			for (size_t i = 0; i < boneCount; ++i)
				ids.emplace(&source->gtTraceBones()[i]->m_rig, i);
			for (int i = 0; i < constraints.size(); ++i) {
				auto* joint = dynamic_cast<Generic6DofConstraint*>(constraints[i]);
				if (!joint)
					continue;
				const auto a = ids.find(&joint->getRigidBodyA());
				const auto b = ids.find(&joint->getRigidBodyB());
				if (a == ids.end() || b == ids.end())
					continue;
				const auto* linear = joint->getTranslationalLimitMotor();
				auto& snap = runtimeJoints.emplace_back();
				snap.a = a->second;
				snap.b = b->second;
				snap.frameA = joint->getFrameOffsetA();
				snap.frameB = joint->getFrameOffsetB();
				unsigned locked = 0, ranged = 0;
				for (int axis = 0; axis < 3; ++axis) {
					const auto* angular = joint->getRotationalLimitMotor(axis);
					snap.axis[axis] = {linear->m_lowerLimit[axis], linear->m_upperLimit[axis],
						linear->m_equilibriumPoint[axis], linear->m_springStiffness[axis], linear->m_springDamping[axis],
						linear->m_enableSpring[axis]};
					snap.axis[axis + 3] = {angular->m_loLimit, angular->m_hiLimit, angular->m_equilibriumPoint,
						angular->m_springStiffness, angular->m_springDamping, angular->m_enableSpring};
					if (linear->m_lowerLimit[axis] == linear->m_upperLimit[axis]) locked |= 1u << axis;
					if (angular->m_loLimit == angular->m_hiLimit) locked |= 1u << (axis + 3);
					if (linear->m_lowerLimit[axis] < linear->m_upperLimit[axis]) ranged |= 1u << axis;
					if (angular->m_loLimit < angular->m_hiLimit) ranged |= 1u << (axis + 3);
				}
				if (!(locked || ranged))
					continue;
				if (tracked.size() == maxConstraints)
					return false;
				tracked.push_back({ joint, a->second, b->second, locked });
				if (geometry && joint->progressiveZEnabled())
					++progressiveLinks;
			}
			return !tracked.empty();
		}

		void setLimitBits(unsigned bit, int status, float lowError, float highError,
			unsigned& lower, unsigned& upper)
		{
			// Bullet reserves status 4 for a ranged axis even inside its interval.
			// Its signed errors indicate penetration at the lower and upper bounds.
			constexpr float epsilon = 1e-6f;
			if (status == 1 || (status == 4 && lowError < -epsilon)) lower |= bit;
			if (status == 2 || (status == 4 && highError > epsilon)) upper |= bit;
		}

		void writeVector(std::ostream& out, const btVector3& vector)
		{
			out << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
		}

		void writePose(std::ostream& out, size_t frame, int substep, const char* phase, size_t bone, const Pose& value)
		{
			out << frame << ',' << substep << ',' << phase << ',' << SkyrimSystem::gtTraceBoneNames()[bone]
				<< ',' << (bone < anchorCount ? "anchor" : "moving");
			for (const float field : value.position) out << ',' << field;
			for (const float field : value.rotation) out << ',' << field;
			for (const float field : value.comVelocity) out << ',' << field;
			for (const float field : value.angularVelocity) out << ',' << field;
			out << ',' << value.scale << '\n';
		}

		bool writeCsv(std::filesystem::path base, const char* suffix, const auto& write)
		{
			base += suffix;
			std::ofstream out(base, std::ios::binary | std::ios::out);
			if (!out)
				return false;
			out.imbue(std::locale::classic());
			out << std::setprecision(9);
			write(out);
			out.close();
			return static_cast<bool>(out);
		}

		bool saveTrace(const std::filesystem::path& base)
		{
			return writeCsv(base, "-bones.csv", [](std::ostream& out) {
				out << "frame,substep,phase,bone,kind,x,y,z,qx,qy,qz,qw,com_vx,com_vy,com_vz,wx,wy,wz,scale\n";
				size_t next = 0;
				for (size_t i = 0; i < frames.size(); ++i) {
					for (size_t bone = 0; bone < anchorCount; ++bone)
						writePose(out, i, -1, "target", bone, frames[i].targets[bone]);
					while (next < steps.size() && steps[next].frame == i) {
						const auto& step = steps[next];
						const auto number = static_cast<int>(step.number);
						for (size_t bone = 0; bone < anchorCount; ++bone)
							writePose(out, i, number, "pre", bone, step.pre[bone]);
						if (next == 0)
							for (size_t bone = anchorCount; bone < boneCount; ++bone)
								writePose(out, i, number, "pre", bone, initialMoving[bone - anchorCount]);
						for (size_t bone = 0; bone < boneCount; ++bone)
							writePose(out, i, number, "post", bone, step.post[bone]);
						++next;
					}
				}
			}) && writeCsv(base, "-steps.csv", [](std::ostream& out) {
				out << "frame,substep,dt_s,sim_time_s,solver_iterations\n";
				for (const auto& step : steps)
					out << step.frame << ',' << step.number << ',' << step.dt << ',' << step.simTime << ',' << step.iterations << '\n';
			}) && writeCsv(base, "-constraints.csv", [](std::ostream& out) {
				out << "constraint,body_a,body_b,locked_mask\n";
				const auto& names = SkyrimSystem::gtTraceBoneNames();
				for (size_t i = 0; i < tracked.size(); ++i)
					out << i << ',' << names[tracked[i].a] << ',' << names[tracked[i].b] << ',' << tracked[i].lockedMask << '\n';
			}) && writeCsv(base, "-limits.csv", [](std::ostream& out) {
				out << "frame,substep,constraint,lower_mask,upper_mask\n";
				for (const auto& hit : hits) {
					const auto& step = steps[hit.step];
					out << step.frame << ',' << step.number << ',' << hit.constraint << ',' << hit.lower << ',' << hit.upper << '\n';
				}
			});
		}

#include "GTTransitionCapture.inl"

		void saveProgressiveTrace(const std::filesystem::path& base)
		{
			if (!geometry)
				return;
			if (!progressiveLinks) {
				logger::info("[ShapePhysicsMotion] Progressive GT Z for {}: no configured links", xml);
				return;
			}
			size_t engagedSteps = 0;
			float peakStiffness = 0, peakDamping = 0;
			const bool saved = writeCsv(base, "-progressive.csv", [&](std::ostream& out) {
				out << "frame,substep,eligible_links,engaged_links,max_added_stiffness,max_added_damping\n";
				for (const auto& step : steps) {
					out << step.frame << ',' << step.number << ',' << step.progressiveEligible << ','
						<< step.progressiveEngaged << ',' << step.maxProgressiveStiffness << ',' << step.maxProgressiveDamping << '\n';
					engagedSteps += step.progressiveEngaged > 0;
					peakStiffness = std::max(peakStiffness, step.maxProgressiveStiffness);
					peakDamping = std::max(peakDamping, step.maxProgressiveDamping);
				}
			});
			if (!saved) {
				logger::warn("[ShapePhysicsMotion] Could not write progressive GT Z telemetry for {}", base.string());
				return;
			}
			logger::info("[ShapePhysicsMotion] Progressive GT Z for {}: {} configured links, engaged in {}/{} substeps "
				"(added stiffness > {}, peak added stiffness {:.4f}, peak added damping {:.4f})",
				xml, progressiveLinks, engagedSteps, steps.size(), engagedStiffness, peakStiffness, peakDamping);
		}

		bool save()
		{
			try {
				const auto log = logger::log_directory();
				if (!log)
					return false;
				const auto folder = *log / "ShapePhysicsMotion";
				std::filesystem::create_directories(folder);
				const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::system_clock::now().time_since_epoch()).count();
				const auto mode = geometry ? "geometry_gt" : "legacy_gt";
				const auto base = folder / fmt::format("{}-{}-{}", stamp, GetCurrentProcessId(), mode);
				if (!saveTrace(base)) {
					logger::error("[ShapePhysicsMotion] Could not write expanded telemetry for {}", base.string());
					return false;
				}
				saveProgressiveTrace(base);
				try {
					if (runtimeBodiesReady && !saveRuntimeBaseline(base))
						logger::warn("[ShapePhysicsMotion] Could not write the runtime rig snapshot for {}", base.string());
				} catch (const std::exception& error) {
					logger::warn("[ShapePhysicsMotion] Could not write the runtime rig snapshot: {}", error.what());
				}
				auto path = base;
				path += ".csv";
				std::ofstream out(path, std::ios::binary | std::ios::out);
				out.imbue(std::locale::classic());
				out << "time_s,gt_mode,applied_xml,speed_xy_su_s,player_world_x,player_world_y,player_world_z,"
					"chest_world_x,chest_world_y,chest_world_z,chest_qx,chest_qy,chest_qz,chest_qw";
				for (const auto name : SkyrimSystem::gtMotionNames)
					out << ',' << name << "_x," << name << "_y," << name << "_z";
				out << '\n' << std::setprecision(9);
				for (size_t i = 0; i < frames.size(); ++i) {
					const auto& frame = frames[i];
					out << frame.seconds << ',' << mode << ',' << std::quoted(xml, '"', '"') << ',';
					if (i > 0) {
						const auto delta = frame.player - frames[i - 1].player;
						const auto dt = frame.seconds - frames[i - 1].seconds;
						if (dt > 0.)
							out << std::hypot(delta.x(), delta.y()) / dt;
					}
					writeVector(out, frame.player);
					writeVector(out, frame.chest);
					out << ',' << frame.rotation.x() << ',' << frame.rotation.y() << ',' << frame.rotation.z()
						<< ',' << frame.rotation.w();
					for (const auto& point : frame.points)
						writeVector(out, point);
					out << '\n';
				}
				out.close();
				if (!out) {
					logger::error("[ShapePhysicsMotion] Could not write {}", path.string());
					return false;
				}
				logger::info("[ShapePhysicsMotion] Saved {} frames to {} ({} substeps, {} limit events)",
					frames.size(), path.string(), steps.size(), hits.size());
				return true;
			} catch (const std::exception& error) {
				logger::error("[ShapePhysicsMotion] Could not save CSV: {}", error.what());
				return false;
			}
		}
	}

	void pollControls(bool available)
	{
		const bool pressed = (GetAsyncKeyState(VK_CONTROL) & 0x8000) && (GetAsyncKeyState(VK_F10) & 0x8000);
		const bool toggle = pressed && !keyHeld;
		keyHeld = pressed;
		if (!available) {
			if (recording && transitions) transitionStop = "suspended_or_loading";
			finish();
			return;
		}
		if (!recording && !toggle)
			return;
		const auto* camera = RE::PlayerCamera::GetSingleton();
		const bool thirdPerson = camera && camera->IsInThirdPerson();
		if (recording && !thirdPerson) {
			if (transitions) transitionStop = "camera_left_third_person";
			finish();
			return;
		}
		if (recording && frames.empty() && transitionFrames.empty() && Clock::now() - start > std::chrono::seconds(2)) {
			RE::SendHUDMessage::ShowHUDMessage("GT recording found no active physics frames.");
			finish();
		}
		if (!toggle)
			return;
		DWORD foregroundProcess = 0;
		GetWindowThreadProcessId(GetForegroundWindow(), &foregroundProcess);
		if (foregroundProcess != GetCurrentProcessId())
			return;
		if (recording) {
			if (transitions) transitionStop = "hotkey";
			finish();
			return;
		}
		if (!thirdPerson) {
			RE::SendHUDMessage::ShowHUDMessage(thirdPersonRequired);
			return;
		}
		try {
			transitions = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
			if (transitions) {
				transitionFrames.reserve(maxFrames);
				transitionStop = "capture_closed";
			} else {
				frames.reserve(maxFrames);
				steps.reserve(maxSteps);
				hits.reserve(maxHits);
				tracked.reserve(maxConstraints);
			}
		} catch (const std::exception& error) {
			logger::error("[ShapePhysicsMotion] Could not allocate capture buffers: {}", error.what());
			RE::SendHUDMessage::ShowHUDMessage("GT motion recording could not allocate its buffers.");
			return;
		}
		start = Clock::now();
		recording = true;
	}

	bool active() { return recording; }

	void beginFrame(const std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems,
		const btAlignedObjectArray<btTypedConstraint*>& constraints, const btVector3& worldOffset)
	{
		if (transitions || !source)
			return;
		discardPendingFrame();
		if (!std::ranges::any_of(systems, [](const auto& system) { return system.get() == source.get(); }) ||
			source->gtMotionBlending() || xml != source->gtMotionXml()) {
			finish();
			return;
		}
		if (tracked.empty()) {
			try {
				if (!lookupConstraints(constraints)) {
					logger::warn("[ShapePhysicsMotion] Could not find the regular GT constraint set.");
					finish();
					return;
				}
			} catch (const std::exception& error) {
				logger::error("[ShapePhysicsMotion] Could not initialize constraint capture: {}", error.what());
				finish();
				return;
			}
		}
		pending.ready = true;
		pending.overflow = false;
		pending.stepOpen = false;
		pending.firstStep = steps.size();
		pending.firstHit = hits.size();
		pending.firstSimTime = simTime;
		pending.offset = worldOffset;
		for (size_t i = 0; i < anchorCount; ++i) {
			const auto* bone = source->gtTraceBones()[i];
			pending.targets[i] = pose(*bone, bone->m_currentTransform.asTransform());
		}
	}

	bool tracingFrame() { return pending.ready && !pending.overflow; }

	void beginSubstep(btScalar dt, int solverIterations)
	{
		if (!tracingFrame())
			return;
		if (steps.size() == maxSteps) {
			pending.overflow = true;
			return;
		}
		const size_t index = steps.size();
		if (!index)
			for (size_t i = anchorCount; i < boneCount; ++i)
				initialMoving[i - anchorCount] = simulatedPose(*source->gtTraceBones()[i]);
		auto& step = steps.emplace_back();
		step.frame = frames.size();
		step.number = index - pending.firstStep;
		step.dt = dt;
		step.simTime = simTime;
		step.iterations = solverIterations;
		for (size_t i = 0; i < anchorCount; ++i)
			step.pre[i] = simulatedPose(*source->gtTraceBones()[i]);
		simTime += dt;
		pending.stepOpen = true;
	}

	void sampleLimits()
	{
		if (!pending.stepOpen)
			return;
		auto& step = steps.back();
		for (size_t i = 0; i < tracked.size(); ++i) {
			auto* joint = tracked[i].constraint;
			const auto& a = joint->getRigidBodyA();
			const auto& b = joint->getRigidBodyB();
			if (!joint->isEnabled() || ((!a.isActive() || a.isStaticOrKinematicObject()) &&
				(!b.isActive() || b.isStaticOrKinematicObject())))
				continue;
			if (progressiveLinks && joint->progressiveZEnabled()) {
				const float stiffness = joint->progressiveZLastGain();
				++step.progressiveEligible;
				step.progressiveEngaged += stiffness > engagedStiffness;
				step.maxProgressiveStiffness = std::max(step.maxProgressiveStiffness, stiffness);
				step.maxProgressiveDamping = std::max(step.maxProgressiveDamping, joint->progressiveZLastDamping());
			}
			const auto* linear = joint->getTranslationalLimitMotor();
			unsigned lower = 0, upper = 0;
			for (int axis = 0; axis < 3; ++axis) {
				const auto* angular = joint->getRotationalLimitMotor(axis);
				setLimitBits(1u << axis, linear->m_currentLimit[axis], linear->m_currentLimitError[axis],
					linear->m_currentLimitErrorHi[axis], lower, upper);
				setLimitBits(1u << (axis + 3), angular->m_currentLimit, angular->m_currentLimitError,
					angular->m_currentLimitErrorHi, lower, upper);
			}
			lower &= ~tracked[i].lockedMask;
			upper &= ~tracked[i].lockedMask;
			if (!(lower || upper))
				continue;
			if (hits.size() == maxHits) {
				pending.overflow = true;
				return;
			}
			hits.push_back({ steps.size() - 1, i, lower, upper });
		}
	}

	void endSubstep()
	{
		if (!pending.stepOpen)
			return;
		if (!runtimeBodiesReady) {
			for (size_t i = 0; i < boneCount; ++i) {
				const auto& bone = *source->gtTraceBones()[i];
				const auto& rig = bone.m_rig;
				runtimeBodies[i] = {rig.getInvMass(), bone.m_gravityFactor, bone.m_currentTransform.getScale(),
					rig.getLinearDamping(), rig.getAngularDamping(), rig.getInvInertiaDiagLocal(), rig.getGravity(),
					bone.m_localToRig};
			}
			runtimeBodiesReady = true;
		}
		for (size_t i = 0; i < boneCount; ++i)
			steps.back().post[i] = simulatedPose(*source->gtTraceBones()[i]);
		pending.stepOpen = false;
	}

	void sample(const std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems)
	{
		const auto now = Clock::now();
		const auto* player = RE::PlayerCharacter::GetSingleton();
		const auto* camera = RE::PlayerCamera::GetSingleton();
		auto* body = player && camera && camera->IsInThirdPerson() ? findBody(systems, player->Get3D2()) : nullptr;
		static const RE::BSFixedString chestName("NPC Spine2 [Spn2]");
		const auto* chest = body ? findNode(body->m_skeleton.get(), chestName) : nullptr;
		if (transitions) {
			if (!chest || (source && source.get() != body)) {
				transitionStop = !body ? "player_body_absent_or_ambiguous" : !chest ? "chest_missing" : "physics_system_replaced";
				finish();
				return;
			}
			if (!source) {
				source.reset(body);
				start = now;
				transitionActor = player->GetFormID();
				RE::SendHUDMessage::ShowHUDMessage("GT transition trace started.");
			}
			sampleTransition(*body, convertNi(chest->world).asTransform(), now);
			if (now - start >= maxDuration || transitionFrames.size() >= maxFrames) {
				transitionStop = "capture_limit";
				finish();
			}
			return;
		}
		if (!chest || body->gtMotionBlending() || (source && (source.get() != body || xml != body->gtMotionXml()))) {
			if (frames.empty())
				RE::SendHUDMessage::ShowHUDMessage(thirdPersonRequired);
			finish();
			return;
		}
		if (!source) {
			source.reset(body);
			xml = body->gtMotionXml();
			geometry = body->gtMotionGeometry();
			RE::SendHUDMessage::ShowHUDMessage("GT motion recording started.");
			return;
		}
		if (!pending.ready)
			return;
		if (pending.overflow) {
			finish();
			return;
		}
		if (frames.empty())
			start = now;
		const auto world = convertNi(chest->world).asTransform();
		Frame frame{};
		frame.seconds = std::chrono::duration<double>(now - start).count();
		frame.player = convertNi(player->GetPosition());
		frame.chest = world.getOrigin();
		frame.rotation = world.getRotation();
		frame.targets = pending.targets;
		for (size_t i = 0; i < frame.points.size(); ++i) {
			const auto& point = body->gtMotionBones()[i]->m_node->world.translate;
			frame.points[i] = world.invXform(convertNi(point));
		}
		frames.push_back(frame);
		pending.ready = false;
		if (now - start >= maxDuration || frames.size() >= maxFrames)
			finish();
	}

	void finish()
	{
		discardPendingFrame();
		recording = false;
	}

	void flush(bool notify)
	{
		if (recording || (!transitionFrames.capacity() && !frames.capacity() && !steps.capacity() && !hits.capacity() && !tracked.capacity()))
			return;
		const bool hasFrames = !frames.empty() || !transitionFrames.empty();
		const bool saved = hasFrames && (transitions ? saveTransitions() : save());
		std::vector<TransitionFrame>().swap(transitionFrames);
		transitionStates.clear();
		runtimeJoints.clear();
		runtimeBodiesReady = false;
		transitions = false;
		std::vector<Frame>().swap(frames);
		std::vector<Substep>().swap(steps);
		std::vector<LimitHit>().swap(hits);
		std::vector<TrackedConstraint>().swap(tracked);
		simTime = 0;
		progressiveLinks = 0;
		source.reset();
		xml.clear();
		if (hasFrames && notify && !RE::UI::GetSingleton()->GameIsPaused())
			RE::SendHUDMessage::ShowHUDMessage(saved ? "GT motion CSV saved." : "GT motion CSV could not be saved. See hdtSMP64.log.");
	}
}

// Included inside GTMotionCapture's private namespace to keep the existing FSMP build target.
struct WritebackPose
{
	std::array<float, 3> p;
	std::array<float, 4> q;
};

WritebackPose capturePose(const btTransform& transform)
{
	const auto q = transform.getRotation();
	return { components(transform.getOrigin()), { q.x(), q.y(), q.z(), q.w() } };
}

void writeTransformFields(std::ostream& out, const WritebackPose& value)
{
	for (const auto field : value.p) out << ',' << field;
	for (const auto field : value.q) out << ',' << field;
}

bool saveRuntimeBaseline(const std::filesystem::path& capture)
{
	const auto folder = capture.parent_path() / "Runtime";
	std::filesystem::create_directories(folder);
	const auto base = folder / capture.filename();
	const auto& names = SkyrimSystem::gtTraceBoneNames();
	return writeCsv(base, "-bodies.csv", [&](std::ostream& out) {
		out << "bone,inverse_mass,inv_Ixx,inv_Iyy,inv_Izz,gravity_x,gravity_y,gravity_z,gravity_factor,"
			"scale,linear_damping,angular_damping,local_to_rig_x,local_to_rig_y,local_to_rig_z,"
			"local_to_rig_qx,local_to_rig_qy,local_to_rig_qz,local_to_rig_qw\n";
		for (std::size_t i = 0; i < runtimeBodies.size(); ++i) {
			const auto& b = runtimeBodies[i];
			out << names[i] << ',' << b.inverseMass;
			writeVector(out, b.inverseInertia);
			writeVector(out, b.gravity);
			out << ',' << b.gravityFactor << ',' << b.scale << ',' << b.linearDamping << ',' << b.angularDamping;
			writeTransformFields(out, capturePose(b.localToRig));
			out << '\n';
		}
	}) && writeCsv(base, "-joint-axes.csv", [&](std::ostream& out) {
		out << "joint,body_a,body_b,axis,enabled,lower,upper,equilibrium,stiffness,damping,"
			"a_x,a_y,a_z,a_qx,a_qy,a_qz,a_qw,b_x,b_y,b_z,b_qx,b_qy,b_qz,b_qw\n";
		for (std::size_t i = 0; i < runtimeJoints.size(); ++i) {
			const auto& j = runtimeJoints[i];
			for (std::size_t axis = 0; axis < j.axis.size(); ++axis) {
				const auto& v = j.axis[axis];
				out << i << ',' << names[j.a] << ',' << names[j.b] << ',' << axis << ',' << v.enabled << ','
					<< v.lower << ',' << v.upper << ',' << v.equilibrium << ',' << v.stiffness << ',' << v.damping;
				writeTransformFields(out, capturePose(j.frameA));
				writeTransformFields(out, capturePose(j.frameB));
				out << '\n';
			}
		}
	});
}

struct TransitionFrame
{
	double seconds;
	std::size_t state;
	bool blending;
	float elapsed, duration;
	WritebackPose chest;
	std::array<WritebackPose, boneCount - anchorCount> written, simulated;
};

struct TransitionState { std::string applied, target; };
std::vector<TransitionState> transitionStates;
std::vector<TransitionFrame> transitionFrames;
const char* transitionStop = "capture_closed";
std::uint32_t transitionActor = 0;

void sampleTransition(const SkyrimSystem& body, const btTransform& chest, Clock::time_point now)
{
	const auto applied = body.gtMotionXml();
	const auto& target = body.liveTarget();
	if (transitionStates.empty() || transitionStates.back().applied != applied || transitionStates.back().target != target)
		transitionStates.push_back({std::string(applied), target});

	auto& frame = transitionFrames.emplace_back();
	frame.seconds = std::chrono::duration<double>(now - start).count();
	frame.state = transitionStates.size() - 1;
	frame.blending = body.gtMotionBlending();
	frame.elapsed = body.gtMotionBlendElapsed();
	frame.duration = body.gtMotionBlendDuration();
	frame.chest = capturePose(chest);
	const auto inverse = chest.inverse();
	for (std::size_t i = anchorCount; i < boneCount; ++i) {
		const auto& bone = *body.gtTraceBones()[i];
		frame.written[i - anchorCount] = capturePose(inverse * convertNi(bone.m_node->world).asTransform());
		frame.simulated[i - anchorCount] = capturePose(inverse * (bone.m_rig.getWorldTransform() * bone.m_rigToLocal));
	}
}

bool saveTransitions()
{
	try {
		const auto log = logger::log_directory();
		if (!log) return false;
		const auto folder = *log / "ShapePhysicsMotion" / "Transitions";
		std::filesystem::create_directories(folder);
		const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
		const auto base = folder / fmt::format("{}-{}", stamp, GetCurrentProcessId());
		const auto saved = writeCsv(base, "-frames.csv", [](std::ostream& out) {
			out << "frame,time_s,actor_form_id,applied_xml,target_xml,blending,elapsed_sim_s,duration_sim_s,"
				"chest_x,chest_y,chest_z,chest_qx,chest_qy,chest_qz,chest_qw,stop_reason\n";
			for (std::size_t i = 0; i < transitionFrames.size(); ++i) {
				const auto& f = transitionFrames[i];
				const auto& state = transitionStates[f.state];
				out << i << ',' << f.seconds << ',' << transitionActor << ',' << std::quoted(state.applied, '"', '"') << ','
					<< std::quoted(state.target, '"', '"') << ',' << f.blending << ',' << f.elapsed << ',' << f.duration;
				writeTransformFields(out, f.chest);
				out << ',' << (i + 1 == transitionFrames.size() ? transitionStop : "") << '\n';
			}
		}) && writeCsv(base, "-poses.csv", [](std::ostream& out) {
			out << "frame,bone,x,y,z,qx,qy,qz,qw,bullet_x,bullet_y,bullet_z,bullet_qx,bullet_qy,bullet_qz,bullet_qw\n";
			const auto& names = SkyrimSystem::gtTraceBoneNames();
			for (std::size_t f = 0; f < transitionFrames.size(); ++f) {
				const auto& frame = transitionFrames[f];
				for (std::size_t j = 0; j < frame.written.size(); ++j) {
					out << f << ',' << names[anchorCount + j];
					writeTransformFields(out, frame.written[j]);
					writeTransformFields(out, frame.simulated[j]);
					out << '\n';
				}
			}
		});
		logger::info("[ShapePhysicsMotion] Transition trace {}, {} frames, stop {}, written={}",
			base.string(), transitionFrames.size(), transitionStop, saved);
		return saved;
	} catch (const std::exception& error) {
		logger::error("[ShapePhysicsMotion] Could not save transition trace: {}", error.what());
		return false;
	}
}

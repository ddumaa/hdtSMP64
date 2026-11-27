#include "GTMotionCapture.h"

#include "hdtSkyrimSystem.h"

#include <RE/S/SendHUDMessage.h>
#include <Windows.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>

namespace hdt::GTMotionCapture
{
	namespace
	{
		using Clock = std::chrono::steady_clock;
		constexpr auto maxDuration = std::chrono::minutes(2);
		constexpr size_t maxFrames = 30000;

		struct Frame
		{
			double seconds;
			btVector3 player;
			btVector3 chest;
			btQuaternion rotation;
			std::array<btVector3, SkyrimSystem::gtMotionNames.size()> points;
		};

		bool recording = false;
		bool keyHeld = false;
		const SkyrimSystem* source = nullptr;
		std::string xml;
		bool geometry = false;
		Clock::time_point start;
		std::vector<Frame> frames;

		bool belongsTo(const SkyrimSystem& system, const RE::NiAVObject* root)
		{
			for (auto* node = system.m_skeleton.get(); node; node = node->parent) {
				if (node == root)
					return true;
			}
			return false;
		}

		const SkyrimSystem* findBody(const std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems,
			const RE::NiAVObject* root)
		{
			const SkyrimSystem* result = nullptr;
			if (!root)
				return result;
			for (const auto& system : systems) {
				const auto* candidate = dynamic_cast<const SkyrimSystem*>(system.get());
				if (!candidate || !candidate->gtMotionBones().front() || !belongsTo(*candidate, root))
					continue;
				if (result)
					return nullptr;
				result = candidate;
			}
			return result;
		}

		void writeVector(std::ostream& out, const btVector3& vector)
		{
			out << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
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
				const auto path = folder / fmt::format("{}-{}-{}.csv", stamp, GetCurrentProcessId(), mode);
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
				logger::info("[ShapePhysicsMotion] Saved {} frames to {}", frames.size(), path.string());
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
			finish();
			return;
		}
		if (recording && frames.empty() && Clock::now() - start > std::chrono::seconds(2)) {
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
			finish();
			return;
		}
		frames.reserve(maxFrames);
		start = Clock::now();
		recording = true;
	}

	bool active() { return recording; }

	void sample(const std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems)
	{
		const auto now = Clock::now();
		const auto* player = RE::PlayerCharacter::GetSingleton();
		const auto* camera = RE::PlayerCamera::GetSingleton();
		const auto* body = player && camera && camera->IsInThirdPerson() ? findBody(systems, player->Get3D2()) : nullptr;
		static const RE::BSFixedString chestName("NPC Spine2 [Spn2]");
		const auto* chest = body ? findNode(body->m_skeleton.get(), chestName) : nullptr;
		if (!chest || body->gtMotionBlending() || (source && (source != body || xml != body->gtMotionXml()))) {
			if (frames.empty())
				RE::SendHUDMessage::ShowHUDMessage("GT recording needs an active, settled regular GT body in third person.");
			finish();
			return;
		}
		if (frames.empty()) {
			start = now;
			source = body;
			xml = body->gtMotionXml();
			geometry = body->gtMotionGeometry();
			RE::SendHUDMessage::ShowHUDMessage("GT motion recording started.");
		}
		const auto world = convertNi(chest->world).asTransform();
		Frame frame{};
		frame.seconds = std::chrono::duration<double>(now - start).count();
		frame.player = convertNi(player->GetPosition());
		frame.chest = world.getOrigin();
		frame.rotation = world.getRotation();
		for (size_t i = 0; i < frame.points.size(); ++i) {
			const auto& point = body->gtMotionBones()[i]->m_node->world.translate;
			frame.points[i] = world.invXform(convertNi(point));
		}
		frames.push_back(frame);
		if (now - start >= maxDuration || frames.size() >= maxFrames)
			finish();
	}

	void finish() { recording = false; }

	void flush(bool notify)
	{
		if (recording || frames.empty())
			return;
		const bool saved = save();
		frames.clear();
		source = nullptr;
		xml.clear();
		if (notify && !RE::UI::GetSingleton()->GameIsPaused())
			RE::SendHUDMessage::ShowHUDMessage(saved ? "GT motion CSV saved." : "GT motion CSV could not be saved. See hdtSMP64.log.");
	}
}

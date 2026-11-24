#pragma once

#include "hdtConvertNi.h"
#include "hdtDefaultBBP.h"
#include "hdtSkinnedMesh/hdtConeTwistConstraint.h"
#include "hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
#include "hdtSkinnedMesh/hdtStiffSpringConstraint.h"
#include "hdtSkyrimBody.h"
#include "hdtSkyrimBone.h"
#include <optional>
#include <string_view>

namespace hdt
{
	enum class LiveProfileKind { None, GT, ThreeBA };

	inline const char* liveProfileName(LiveProfileKind kind)
	{
		return kind == LiveProfileKind::ThreeBA ? "3BA SMP" : "GT";
	}

	class PerVertexShape;
	class SkyrimSystem : public SkinnedMeshSystem
	{
		friend class SkyrimSystemCreator;

	public:
		struct BoneData
		{
			uint16_t boneWeights[4];
			uint8_t boneIndices[4];
		};

		SkyrimSystem(RE::NiNode* skeleton);
		~SkyrimSystem() override = default;

		SkinnedMeshBone* findBone(const RE::BSFixedString& name);
		SkinnedMeshBody* findBody(const RE::BSFixedString& name);
		int findBoneIdx(const RE::BSFixedString& name);

		float prepareForRead(float timeStep) override;
		void readTransform(float timeStep) override;

		struct LiveConstraintParameters
		{
			btVector3 linearStiffness{ 0, 0, 0 };
			btVector3 angularStiffness{ 0, 0, 0 };
			btVector3 linearDamping{ 0, 0, 0 };
			btVector3 angularDamping{ 0, 0, 0 };
			btVector3 linearLower{ 0, 0, 0 };
			btVector3 linearUpper{ 0, 0, 0 };
			btVector3 angularLower{ 0, 0, 0 };
			btVector3 angularUpper{ 0, 0, 0 };
			btVector3 linearNonHookeanStiffness{ 0, 0, 0 };
			btVector3 angularNonHookeanStiffness{ 0, 0, 0 };
			btVector3 linearNonHookeanDamping{ 0, 0, 0 };
			btVector3 angularNonHookeanDamping{ 0, 0, 0 };

			bool operator==(const LiveConstraintParameters&) const = default;
		};

		struct LiveConstraint
		{
			std::string bodyA;
			std::string bodyB;
			std::string preset;
			LiveConstraintParameters parameters;
			bool reversed = false;
			bool linearEnabled = true;
			bool angularEnabled = true;
			bool limitLinearStiffness = true;
			bool limitAngularStiffness = true;
			bool limitDamping = true;
			bool linearMotors = false;
			bool angularMotors = false;
			bool linearServos = false;
			bool angularServos = false;

			bool matches(const LiveConstraint& target) const;
		};

		struct LiveBoneGravity
		{
			std::string bone;
			std::string preset;
			float factor;
		};

		struct LiveProfile
		{
			std::vector<LiveConstraint> constraints;
			std::vector<LiveBoneGravity> gravity;
		};

		// The accepted target and transition belong to this specific system.
		// Call these methods under the simulation lock.
		bool blendLiveProfile(const LiveProfile& target, std::string path, float seconds);
		const std::string& liveTarget() const { return m_liveTarget; }

		const std::vector<RE::BSTSmartPointer<SkinnedMeshBody>>& meshes() const { return m_meshes; }

		RE::NiPointer<RE::NiNode> m_skeleton;
		RE::NiPointer<RE::NiNode> m_oldRoot;
		bool m_initialized = false;
		float m_windFactor = 1.f;  // wind factor for the system (i.e., full actor/skeleton) (calculated based off obstructions)

		// angular velocity damper
		btQuaternion m_lastRootRotation;

	private:
		struct AttachedLiveConstraint
		{
			LiveConstraint source;
			Generic6DofConstraint* constraint = nullptr;  // Owned by this system's constraint lists.
		};

		struct LiveConstraintTransition
		{
			Generic6DofConstraint* constraint;
			LiveConstraintParameters start;
			LiveConstraintParameters target;
		};

		struct AttachedLiveBone
		{
			std::string name;
			std::string preset;
			SkinnedMeshBone* bone;  // Owned by this system's bone list.
		};

		struct LiveGravityTransition
		{
			SkinnedMeshBone* bone;
			float start;
			float target;
		};

		LiveProfileKind m_liveProfileKind = LiveProfileKind::None;
		std::vector<AttachedLiveConstraint> m_liveConstraints;
		std::vector<AttachedLiveBone> m_liveBones;
		std::vector<LiveConstraintTransition> m_liveConstraintTransitions;
		std::vector<LiveGravityTransition> m_liveGravityTransitions;
		std::string m_liveTarget;
		float m_liveElapsed = 0;
		float m_liveDuration = 0;
	};

	class XMLReader;

	class SkyrimSystemCreator
	{
	public:
		struct LiveProfileFamily
		{
			LiveProfileKind kind = LiveProfileKind::None;
			std::string key;
		};

		SkyrimSystemCreator();
		static LiveProfileFamily liveProfileFamily(std::string_view path);
		std::optional<SkyrimSystem::LiveProfile> readLiveProfile(const std::string& path, LiveProfileKind kind);

		RE::BSTSmartPointer<SkyrimSystem> createOrUpdateSystem(RE::NiNode* skeleton, RE::NiAVObject* model, DefaultBBP::PhysicsFile_t* file, std::unordered_map<RE::BSFixedString, RE::BSFixedString>&& renameMap, SkyrimSystem* old_system);

	protected:
		// O(1) bone lookup index. These are just to speed up the hashmap more since BSStrings are pooled
		struct PooledStringHash
		{
			size_t operator()(const char* p) const noexcept { return reinterpret_cast<size_t>(p); }
		};

		struct PooledStringEqual
		{
			bool operator()(const char* a, const char* b) const noexcept { return a == b; }
		};

		std::unordered_map<const char*, SkyrimBone*, PooledStringHash, PooledStringEqual> m_boneIndex;

		void indexBone(SkyrimBone* bone);
		SkyrimBone* findBoneFromIndex(const RE::BSFixedString& name) const;

		struct DeferredBuild
		{
			SkinnedMeshBody* body;
			PerVertexShape* vertexShape;
		};

		std::vector<DeferredBuild> m_deferredBuilds;

		struct BoneTemplate : public btRigidBody::btRigidBodyConstructionInfo
		{
			static btEmptyShape emptyShape[1];

			BoneTemplate() :
				btRigidBodyConstructionInfo(0, nullptr, emptyShape)
			{
				m_centerOfMassTransform = btTransform::getIdentity();
				m_marginMultipler = 1.f;
			}

			std::shared_ptr<btCollisionShape> m_shape;
			std::vector<RE::BSFixedString> m_canCollideWithBone;
			std::vector<RE::BSFixedString> m_noCollideWithBone;
			btTransform m_centerOfMassTransform;
			float m_marginMultipler;
			float m_gravityFactor = 1.0f;
			float m_windFactor = 1.0f;
			U32 m_collisionFilter = 0;
		};

		enum FrameType
		{
			FrameInA,
			FrameInB,
			FrameInLerp,
			AWithXPointToB,
			AWithYPointToB,
			AWithZPointToB
		};

		struct GenericConstraintTemplate
		{
			FrameType frameType = FrameInB;
			bool useLinearReferenceFrameA = false;
			btTransform frame = btTransform::getIdentity();
			btVector3 linearLowerLimit = btVector3(1, 1, 1);
			btVector3 linearUpperLimit = btVector3(-1, -1, -1);
			btVector3 angularLowerLimit = btVector3(1, 1, 1);
			btVector3 angularUpperLimit = btVector3(-1, -1, -1);
			btVector3 linearStiffness = btVector3(0, 0, 0);
			btVector3 angularStiffness = btVector3(0, 0, 0);
			btVector3 linearDamping = btVector3(0, 0, 0);
			btVector3 angularDamping = btVector3(0, 0, 0);
			btVector3 linearEquilibrium = btVector3(0, 0, 0);
			btVector3 angularEquilibrium = btVector3(0, 0, 0);
			btVector3 linearBounce = btVector3(0, 0, 0);
			btVector3 angularBounce = btVector3(0, 0, 0);
			bool enableLinearSprings = true;
			bool enableAngularSprings = true;
			bool linearStiffnessLimited = true;
			bool angularStiffnessLimited = true;
			bool springDampingLimited = true;
			bool linearMotors = false;
			bool angularMotors = false;
			// TODO: Test if servo motors go to [0, 0, 0], or whatever equilibrium is.  Provide option to set server motor target.  Hard coded to equilibrium right now.
			bool linearServoMotors = false;
			bool angularServoMotors = false;
			btVector3 linearNonHookeanDamping = btVector3(0, 0, 0);
			btVector3 angularNonHookeanDamping = btVector3(0, 0, 0);
			btVector3 linearNonHookeanStiffness = btVector3(0, 0, 0);
			btVector3 angularNonHookeanStiffness = btVector3(0, 0, 0);
			btVector3 linearTargetVelocity = btVector3(0, 0, 0);
			btVector3 angularTargetVelocity = btVector3(0, 0, 0);
			btVector3 linearMaxMotorForce = btVector3(0, 0, 0);
			btVector3 angularMaxMotorForce = btVector3(0, 0, 0);
			btScalar motorERP = 0.9f;
			btScalar motorCFM = 0;
			btScalar stopERP = 0.2f;
			btScalar stopCFM = 0;
		};

		struct StiffSpringConstraintTemplate
		{
			float minDistanceFactor = 1;
			float maxDistanceFactor = 1;
			float stiffness = 0;
			float damping = 0;
			float equilibriumFactor = 0.5;
		};

		struct ConeTwistConstraintTemplate
		{
			btTransform frame = btTransform::getIdentity();
			FrameType frameType = FrameInB;
			float swingSpan1 = 0;
			float swingSpan2 = 0;
			float twistSpan = 0;
			float limitSoftness = 1.0f;
			float biasFactor = 0.3f;
			float relaxationFactor = 1.0f;
		};

		using VertexOffsetMap = std::unordered_map<std::string, int>;

		RE::BSFixedString getRenamedBone(const RE::BSFixedString& name);

		RE::BSTSmartPointer<SkyrimSystem> m_mesh;
		RE::NiNode* m_skeleton;
		RE::NiAVObject* m_model;
		XMLReader* m_reader;
		std::unordered_map<RE::BSFixedString, RE::BSFixedString> m_renameMap;

		RE::NiNode* findObjectByName(const RE::BSFixedString& name);
		SkyrimBone* getOrCreateBone(const RE::BSFixedString& name);

		std::string m_filePath;

		std::unordered_map<RE::BSFixedString, BoneTemplate> m_boneTemplates;
		std::unordered_map<RE::BSFixedString, GenericConstraintTemplate> m_genericConstraintTemplates;
		std::unordered_map<RE::BSFixedString, StiffSpringConstraintTemplate> m_stiffSpringConstraintTemplates;
		std::unordered_map<RE::BSFixedString, ConeTwistConstraintTemplate> m_coneTwistConstraintTemplates;
		std::unordered_map<RE::BSFixedString, std::shared_ptr<btCollisionShape>> m_shapes;
		std::vector<std::shared_ptr<btCollisionShape>> m_shapeRefs;

		std::pair<RE::BSTSmartPointer<SkyrimBody>, VertexOffsetMap> generateMeshBody(const std::string name, DefaultBBP::NameSet_t* names);

		bool findBones(const RE::BSFixedString& bodyAName, const RE::BSFixedString& bodyBName, SkyrimBone*& bodyA, SkyrimBone*& bodyB);
		bool parseFrameType(const std::string& name, FrameType& type, btTransform& frame);
		static void calcFrame(FrameType type, const btTransform& frame, const btQsTransform& trA, const btQsTransform& trB, btTransform& frameA, btTransform& frameB);
		void readFrameLerp(btTransform& tr);
		void readBoneTemplate(BoneTemplate& dest);
		void readGenericConstraintTemplate(GenericConstraintTemplate& dest);
		void readStiffSpringConstraintTemplate(StiffSpringConstraintTemplate& dest);
		void readConeTwistConstraintTemplate(ConeTwistConstraintTemplate& dest);

		const BoneTemplate& getBoneTemplate(const RE::BSFixedString& name);
		const GenericConstraintTemplate& getGenericConstraintTemplate(const RE::BSFixedString& name);
		const StiffSpringConstraintTemplate& getStiffSpringConstraintTemplate(const RE::BSFixedString& name);
		const ConeTwistConstraintTemplate& getConeTwistConstraintTemplate(const RE::BSFixedString& name);

		SkyrimBone* createBoneFromNodeName(const RE::BSFixedString& bodyName, const RE::BSFixedString& templateName = "", const bool readTemplate = false);
		void readOrUpdateBone();
		RE::BSTSmartPointer<SkyrimBody> readPerVertexShape(DefaultBBP::NameMap_t meshNameMap);
		RE::BSTSmartPointer<SkyrimBody> readPerTriangleShape(DefaultBBP::NameMap_t* meshNameMap);
		RE::BSTSmartPointer<Generic6DofConstraint> readGenericConstraint();
		static void readLiveConstraintSettings(SkyrimSystem::LiveConstraint& link, const GenericConstraintTemplate& values,
			bool includeNonHookean);
		RE::BSTSmartPointer<StiffSpringConstraint> readStiffSpringConstraint();
		RE::BSTSmartPointer<ConeTwistConstraint> readConeTwistConstraint();
		RE::BSTSmartPointer<ConstraintGroup> readConstraintGroup();
		std::shared_ptr<btCollisionShape> readShape();
	};
}

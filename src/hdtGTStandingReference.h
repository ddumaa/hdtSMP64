#pragma once

#include <btBulletDynamicsCommon.h>

#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <vector>

namespace hdt
{
	struct GTStandingReferenceJob
	{
		struct Body
		{
			btTransform bind;
			btVector3 inertia;
			float mass, linearDamping, angularDamping, gravityFactor;
		};

		struct Axis
		{
			float lower, upper, stiffness, damping, equilibrium, bounce, stopERP, stopCFM, motorERP, motorCFM;
			bool enabled, capStiffness, capDamping;
		};

		struct Link
		{
			size_t bodyA, bodyB;
			btTransform frameA, frameB;
			std::array<Axis, 6> axes;
			int stage;
		};

		struct Reference
		{
			float sigma, position;
		};

		std::vector<Body> bodies;
		std::vector<Link> links;
		float worldGravity = 0.f;
		std::vector<Reference> result;
		std::atomic<bool> ready{ false };

		bool solve()
		{
			btDefaultCollisionConfiguration collision;
			btCollisionDispatcher dispatcher(&collision);
			btDbvtBroadphase broadphase;
			btSequentialImpulseConstraintSolver solver;
			btEmptyShape shape;
			// This world has only the authored GT chest constraints and fixed bind anchors.
			// It is independent of the player's active pose and the live simulation world.
			std::vector<std::unique_ptr<btRigidBody>> runtime;
			std::vector<std::unique_ptr<btGeneric6DofSpring2Constraint>> constraints;
			btDiscreteDynamicsWorld world(&dispatcher, &broadphase, &solver, &collision);
			world.setGravity(btVector3(0, 0, 0));
			auto& settings = world.getSolverInfo();
			settings.m_numIterations = 32;
			settings.m_solverMode = SOLVER_SIMD;
			settings.m_erp = .2f;
			settings.m_erp2 = .15f;
			settings.m_globalCfm = .001f;
			settings.m_leastSquaresResidualThreshold = .0001f;
			runtime.reserve(bodies.size());
			constraints.reserve(links.size());
			for (const auto& body : bodies) {
				btRigidBody::btRigidBodyConstructionInfo info(body.mass, nullptr, &shape, body.inertia);
				info.m_linearDamping = body.linearDamping;
				info.m_angularDamping = body.angularDamping;
				info.m_startWorldTransform = body.bind;
				auto rigid = std::make_unique<btRigidBody>(info);
				if (!body.mass)
					rigid->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
				rigid->setActivationState(DISABLE_DEACTIVATION);
				world.addRigidBody(rigid.get(), 0, 0);
				rigid->setGravity(btVector3(0, 0, worldGravity * body.gravityFactor));
				runtime.push_back(std::move(rigid));
			}
			for (const auto& link : links) {
				auto joint = std::make_unique<btGeneric6DofSpring2Constraint>(*runtime[link.bodyA], *runtime[link.bodyB],
					link.frameA, link.frameB, RO_XYZ);
				for (int axis = 0; axis < 6; ++axis) {
					const auto& p = link.axes[axis];
					joint->setLimit(axis, p.lower, p.upper);
					joint->enableSpring(axis, p.enabled);
					joint->setStiffness(axis, p.stiffness, p.capStiffness);
					joint->setDamping(axis, p.damping, p.capDamping);
					joint->setEquilibriumPoint(axis, p.equilibrium);
					joint->setBounce(axis, p.bounce);
					joint->setParam(BT_CONSTRAINT_STOP_ERP, p.stopERP, axis);
					joint->setParam(BT_CONSTRAINT_STOP_CFM, p.stopCFM, axis);
					joint->setParam(BT_CONSTRAINT_ERP, p.motorERP, axis);
					joint->setParam(BT_CONSTRAINT_CFM, p.motorCFM, axis);
				}
				world.addConstraint(joint.get());
				constraints.push_back(std::move(joint));
			}
			constexpr float dt = 1.f / 90.f;
			for (int step = 0; step < 360; ++step)
				world.stepSimulation(dt, 0);

			bool valid = true;
			result.reserve(32);
			for (size_t i = 0; i < links.size(); ++i) {
				const auto& link = links[i];
				if (link.stage < 0)
					continue;
				const auto axis = (bodies[link.bodyA].bind * link.frameA).getBasis().getColumn(2);
				float variance = 0.f;
				for (const auto index : { link.bodyA, link.bodyB }) {
					const auto& body = bodies[index];
					const auto local = body.bind.getBasis().transpose() * axis;
					const auto trace = (body.inertia.x() + body.inertia.y() + body.inertia.z()) * .5f;
					for (int j = 0; j < 3; ++j) {
						const auto component = (trace - body.inertia[j]) / body.mass;
						if (!std::isfinite(component) || component < -1.0e-5f)
							valid = false;
						variance += local[j] * local[j] * btMax(0.f, component);
					}
				}
				constraints[i]->calculateTransforms();
				const auto standing = constraints[i]->getRelativePivotPosition(2);
				if (!std::isfinite(variance) || variance <= 0.f || !std::isfinite(standing))
					valid = false;
				result.push_back({ std::sqrt(variance), standing });
			}
			for (const auto& link : constraints)
				world.removeConstraint(link.get());
			for (const auto& body : runtime)
				world.removeRigidBody(body.get());
			return valid && result.size() == 32;
		}
	};
}

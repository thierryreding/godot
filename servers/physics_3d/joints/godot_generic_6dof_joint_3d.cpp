/**************************************************************************/
/*  godot_generic_6dof_joint_3d.cpp                                       */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

/*
Adapted to Godot from the Bullet library.
*/

/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
2007-09-09
GodotGeneric6DOFJoint3D Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/

#include "godot_generic_6dof_joint_3d.h"

#define GENERIC_D6_DISABLE_WARMSTARTING 1

#define DEBUG(fmt, ...) \
	print_line(vformat(fmt, __VA_ARGS__))

//////////////////////////// GodotG6DOFRotationalLimitMotor3D ////////////////////////////////////

int GodotG6DOFRotationalLimitMotor3D::testLimitValue(real_t test_value) {
	if (m_loLimit > m_hiLimit) {
		m_currentLimit = 0; //Free from violation
		return 0;
	}

	if (test_value < m_loLimit) {
		m_currentLimit = 1; //low limit violation
		m_currentLimitError = test_value - m_loLimit;
		return 1;
	} else if (test_value > m_hiLimit) {
		m_currentLimit = 2; //High limit violation
		m_currentLimitError = test_value - m_hiLimit;
		return 2;
	};

	m_currentLimit = 0; //Free from violation
	return 0;
}

real_t GodotG6DOFRotationalLimitMotor3D::solveAngularLimits(
		real_t timeStep, Vector3 &axis, real_t jacDiagABInv,
		GodotBody3D *body0, GodotBody3D *body1, bool p_body0_dynamic, bool p_body1_dynamic) {
	if (!needApplyTorques()) {
		return 0.0f;
	}

	real_t target_velocity = m_targetVelocity;
	real_t maxMotorForce = m_maxMotorForce;

	//current error correction
	if (m_currentLimit != 0) {
		target_velocity = -m_ERP * m_currentLimitError / (timeStep);
		maxMotorForce = m_maxLimitForce;
	}

	maxMotorForce *= timeStep;

	// current velocity difference
	Vector3 vel_diff = body0->get_angular_velocity();
	if (body1) {
		vel_diff -= body1->get_angular_velocity();
	}

	real_t rel_vel = axis.dot(vel_diff);

	// correction velocity
	real_t motor_relvel = m_limitSoftness * (target_velocity - m_damping * rel_vel);

	if (Math::is_zero_approx(motor_relvel)) {
		return 0.0f; //no need for applying force
	}

	// correction impulse
	real_t unclippedMotorImpulse = (1 + m_bounce) * motor_relvel * jacDiagABInv;

	// clip correction impulse
	real_t clippedMotorImpulse;

	///@todo: should clip against accumulated impulse
	if (unclippedMotorImpulse > 0.0f) {
		clippedMotorImpulse = unclippedMotorImpulse > maxMotorForce ? maxMotorForce : unclippedMotorImpulse;
	} else {
		clippedMotorImpulse = unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce : unclippedMotorImpulse;
	}

	// sort with accumulated impulses
	real_t lo = real_t(-1e30);
	real_t hi = real_t(1e30);

	real_t oldaccumImpulse = m_accumulatedImpulse;
	real_t sum = oldaccumImpulse + clippedMotorImpulse;
	m_accumulatedImpulse = sum > hi ? real_t(0.) : (sum < lo ? real_t(0.) : sum);

	clippedMotorImpulse = m_accumulatedImpulse - oldaccumImpulse;

	Vector3 motorImp = clippedMotorImpulse * axis;

	if (p_body0_dynamic) {
		body0->apply_torque_impulse(motorImp);
	}
	if (body1 && p_body1_dynamic) {
		body1->apply_torque_impulse(-motorImp);
	}

	return clippedMotorImpulse;
}

//////////////////////////// GodotG6DOFTranslationalLimitMotor3D ////////////////////////////////////

void GodotG6DOFTranslationalLimitMotor3D::testLimitValue(int limitIndex, real_t value)
{
	real_t loLimit = m_lowerLimit[limitIndex];
	real_t hiLimit = m_upperLimit[limitIndex];

	if (loLimit > hiLimit) {
		m_currentLimitError[limitIndex] = 0;
		m_currentLimit[limitIndex] = 0;
	} else if (loLimit == hiLimit) {
		m_currentLimitError[limitIndex] = value - loLimit;
		m_currentLimit[limitIndex] = 3;
	} else {
		m_currentLimitError[limitIndex] = value - loLimit;
		m_currentLimitErrorHi[limitIndex] = value - hiLimit;
		m_currentLimit[limitIndex] = 4;
	}
}

void GodotG6DOFTranslationalLimitMotor3D::solveLinearAxis(
		real_t timeStep,
		real_t jacDiagABInv,
		GodotBody3D *body1, const Vector3 &pointInA,
		GodotBody3D *body2, const Vector3 &pointInB,
		bool p_body1_dynamic, bool p_body2_dynamic,
		int limit_index,
		const Vector3 &axis_normal_on_a,
		const Vector3 &anchorPos)
{
	///find relative velocity
	//    Vector3 rel_pos1 = pointInA - body1->get_transform().origin;
	//    Vector3 rel_pos2 = pointInB - body2->get_transform().origin;
	Vector3 rel_pos1 = anchorPos - body1->get_transform().origin;
	Vector3 rel_pos2 = anchorPos - body2->get_transform().origin;

	Vector3 vel1 = body1->get_velocity_in_local_point(rel_pos1);
	Vector3 vel2 = body2->get_velocity_in_local_point(rel_pos2);
	Vector3 vel = vel1 - vel2;

	real_t rel_vel = axis_normal_on_a.dot(vel);

	/// apply displacement correction

	if (enable_limit[limit_index] && isLimited(limit_index)) {
		//positional error (zeroth order error)
		real_t depth = -(pointInA - pointInB).dot(axis_normal_on_a);
		real_t lo = real_t(-1e30);
		real_t hi = real_t(1e30);

		real_t minLimit = m_lowerLimit[limit_index];
		real_t maxLimit = m_upperLimit[limit_index];

		//DEBUG("%s(axis: %d): limits: %d/%d", __func__, limit_index, minLimit, maxLimit);
		//if (limit_index == 1) {
		//	DEBUG("axis: %d, currentLimit: %d, error: %f/%f, diff: %f", limit_index,
		//	      m_currentLimit[limit_index], m_currentLimitError[limit_index],
		//	      m_currentLimitErrorHi[limit_index], m_currentLinearDiff[limit_index]);
		//}

		//handle the limits
		if (minLimit <= maxLimit) {
			if (depth > maxLimit) {
				depth -= maxLimit;
				lo = real_t(0.);
			} else if (depth < minLimit) {
				depth -= minLimit;
				hi = real_t(0.);
			} else {
				depth = 0.0;
			}
		} else {
			depth = 0.0;
		}

		real_t normalImpulse = m_limitSoftness[limit_index] * (m_restitution[limit_index] * depth / timeStep - m_damping[limit_index] * rel_vel) * jacDiagABInv;

		real_t oldNormalImpulse = m_accumulatedImpulse[limit_index];
		real_t sum = oldNormalImpulse + normalImpulse;
		m_accumulatedImpulse[limit_index] = sum > hi ? real_t(0.) : (sum < lo ? real_t(0.) : sum);
		normalImpulse = m_accumulatedImpulse[limit_index] - oldNormalImpulse;

		Vector3 impulse_vector = axis_normal_on_a * normalImpulse;

		if (p_body1_dynamic) {
			body1->apply_impulse(impulse_vector, rel_pos1);
		}

		if (p_body2_dynamic) {
			body2->apply_impulse(-impulse_vector, rel_pos2);
		}
	}

	if (m_enableMotor[limit_index]) {
		real_t desiredVelocity = -m_targetVelocity[limit_index];
		real_t motor_relvel = desiredVelocity + rel_vel;
		real_t normalImpulse = -motor_relvel; // * jacDiagABInv;

#if 1
		real_t new_acc = m_accumulatedMotorImpulse[limit_index] + Math::abs(normalImpulse);
		if (new_acc > m_maxMotorForce[limit_index]) {
			new_acc = m_maxMotorForce[limit_index];
		}

		real_t delta = new_acc - m_accumulatedMotorImpulse[limit_index];
		if (normalImpulse < 0.0) {
			normalImpulse = -delta;
		} else {
			normalImpulse = delta;
		}

		m_accumulatedMotorImpulse[limit_index] = new_acc;
#else
		// get motor factor
		real_t depth = -(pointInA - pointInB).dot(axis_normal_on_a);
		real_t limit_factor = 1.0;

		if (m_lowerLimit[limit_index] > m_upperLimit[limit_index]) {
			limit_factor = 1.0;
		} else if (m_lowerLimit[limit_index] == m_upperLimit[limit_index]) {
			limit_factor = 0.0;
		} else {
			real_t delta_max = desiredVelocity / timeStep;
			real_t pos = depth;
			real_t low = -m_maxMotorForce[limit_index];
			real_t high = m_maxMotorForce[limit_index];
			limit_factor = 1.0;

			if (delta_max < 0.0) {
				if ((pos >= low) && (pos < (low - delta_max))) {
					limit_factor = (low - pos) / delta_max;
				} else if (pos < low) {
					limit_factor = 0.0;
				} else {
					limit_factor = 1.0;
				}
			} else if (delta_max > 0.0) {
				if ((pos < high) && (pos > (high - delta_max))) {
					limit_factor = (high - pos) / delta_max;
				} else if (pos > high) {
					limit_factor = 0.0;
				} else {
					limit_factor = 1.0;
				}
			} else {
				limit_factor = 0.0;
			}
		}

		normalImpulse *= limit_factor;
#endif

		Vector3 impulse_vector = axis_normal_on_a * normalImpulse * timeStep;

		// XXX implement motor force limit

		if (p_body1_dynamic) {
			body1->apply_impulse(impulse_vector, rel_pos1);
		}

		if (p_body2_dynamic) {
			body2->apply_impulse(-impulse_vector, rel_pos2);
		}

#if 0
		if (m_accumulatedMotorImpulse[limit_index] < m_maxMotorForce[limit_index]) {
			real_t desiredVelocity = m_targetVelocity[limit_index];
			real_t motor_relvel = desiredVelocity + rel_vel;
			real_t normalImpulse = -motor_relvel * jacDiagABInv;
			// clamp accumulated pulse
			real_t new_acc = m_accumulatedMotorImpulse[limit_index] + Math::abs(normalImpulse);
			if (new_acc > m_maxMotorForce[limit_index]) {
				new_acc = m_maxMotorForce[limit_index];
			}
			real_t delta = new_acc - m_accumulatedMotorImpulse[limit_index];
			if (normalImpulse < 0.0) {
				normalImpulse = -delta;
			} else {
				normalImpulse = delta;
			}
			m_accumulatedMotorImpulse[limit_index] = new_acc;
			// apply clamped impulse
			Vector3 impulse_vector = axis_normal_on_a * normalImpulse;
			if (p_body1_dynamic) {
				body1->apply_impulse(impulse_vector, rel_pos1);
			}
			if (p_body2_dynamic) {
				body2->apply_impulse(-impulse_vector, rel_pos2);
			}
		}
#endif
	}

	if (m_enableSpring[limit_index]) {
		Vector3 position = pointInB - pointInA;
		DEBUG("position: %s", position);
		real_t error = position[limit_index] - m_springEquilibrium[limit_index];
		real_t ks = m_springStiffness[limit_index];
		real_t kd = m_springDamping[limit_index];

		real_t mA = 1.0 / body1->get_inv_mass();
		real_t mB = 1.0 / body2->get_inv_mass();
		real_t m;
		if (body1->get_inv_mass() == 0)
			m = mB;
		else if (body2->get_inv_mass() == 0)
			m = mA;
		else
			m = mA * mB / (mA + mB);

		/*
		// avoid damping that would blow up the spring
		if (kd * timeStep > m)
			kd = m / timeStep;
		*/

		real_t normalImpulse = rel_vel + (ks * error - kd * rel_vel) / m;
		Vector3 impulse_vector = axis_normal_on_a * normalImpulse * timeStep;

		if (p_body1_dynamic)
			body1->apply_impulse(impulse_vector, rel_pos1);

		if (p_body2_dynamic)
			body2->apply_impulse(-impulse_vector, rel_pos2);
	}
}

//////////////////////////// GodotGeneric6DOFJoint3D ////////////////////////////////////

GodotGeneric6DOFJoint3D::GodotGeneric6DOFJoint3D(GodotBody3D *rbA, GodotBody3D *rbB, const Transform3D &frameInA, const Transform3D &frameInB, bool useLinearReferenceFrameA) :
		GodotJoint3D(_arr, 2),
		m_frameInA(frameInA),
		m_frameInB(frameInB),
		m_useLinearReferenceFrameA(useLinearReferenceFrameA) {
	A = rbA;
	B = rbB;
	A->add_constraint(this, 0);
	B->add_constraint(this, 1);
}

void GodotGeneric6DOFJoint3D::calculateLinearInfo()
{
	m_calculatedLinearDiff = m_calculatedTransformB.origin - m_calculatedTransformA.origin;
	m_calculatedLinearDiff = m_calculatedTransformA.basis.inverse().xform(m_calculatedLinearDiff);

	for (int i = 0; i < 3; i++) {
		m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
		m_linearLimits.testLimitValue(i, m_calculatedLinearDiff[i]);
	}
}

void GodotGeneric6DOFJoint3D::calculateAngleInfo() {
	Basis relative_frame = m_calculatedTransformB.basis.inverse() * m_calculatedTransformA.basis;

	m_calculatedAxisAngleDiff = relative_frame.get_euler(EulerOrder::XYZ);

	// in euler angle mode we do not actually constrain the angular velocity
	// along the axes axis[0] and axis[2] (although we do use axis[1]) :
	//
	//    to get			constrain w2-w1 along		...not
	//    ------			---------------------		------
	//    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
	//    d(angle[1])/dt = 0	ax[1]
	//    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
	//
	// constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
	// to prove the result for angle[0], write the expression for angle[0] from
	// GetInfo1 then take the derivative. to prove this for angle[2] it is
	// easier to take the euler rate expression for d(angle[2])/dt with respect
	// to the components of w and set that to 0.

	Vector3 axis0 = m_calculatedTransformB.basis.get_column(0);
	Vector3 axis2 = m_calculatedTransformA.basis.get_column(2);

	m_calculatedAxis[1] = axis2.cross(axis0);
	m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
	m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);

	/*
	if(m_debugDrawer)
	{
		char buff[300];
		sprintf(buff,"\n X: %.2f ; Y: %.2f ; Z: %.2f ",
		m_calculatedAxisAngleDiff[0],
		m_calculatedAxisAngleDiff[1],
		m_calculatedAxisAngleDiff[2]);
		m_debugDrawer->reportErrorWarning(buff);
	}
	*/
}

void GodotGeneric6DOFJoint3D::calculateTransforms() {
	m_calculatedTransformA = A->get_transform() * m_frameInA;
	m_calculatedTransformB = B->get_transform() * m_frameInB;

	calculateLinearInfo();
	calculateAngleInfo();

	real_t miA = A->get_inv_mass();
	real_t miB = B->get_inv_mass();
	m_hasStaticBody = Math::is_zero_approx(miA) || Math::is_zero_approx(miB);
	real_t miS = miA + miB;

	if (miS > 0.0) {
		m_factA = miB / miS;
	} else {
		m_factA = 0.5;
	}

	m_factB = 1.0 - m_factA;
}

void GodotGeneric6DOFJoint3D::buildLinearJacobian(
		GodotJacobianEntry3D &jacLinear, const Vector3 &normalWorld,
		const Vector3 &pivotAInW, const Vector3 &pivotBInW) {
	memnew_placement(
			&jacLinear,
			GodotJacobianEntry3D(
					A->get_principal_inertia_axes().transposed(),
					B->get_principal_inertia_axes().transposed(),
					pivotAInW - A->get_transform().origin - A->get_center_of_mass(),
					pivotBInW - B->get_transform().origin - B->get_center_of_mass(),
					normalWorld,
					A->get_inv_inertia(),
					A->get_inv_mass(),
					B->get_inv_inertia(),
					B->get_inv_mass()));
}

void GodotGeneric6DOFJoint3D::buildAngularJacobian(
		GodotJacobianEntry3D &jacAngular, const Vector3 &jointAxisW) {
	memnew_placement(
			&jacAngular,
			GodotJacobianEntry3D(
					jointAxisW,
					A->get_principal_inertia_axes().transposed(),
					B->get_principal_inertia_axes().transposed(),
					A->get_inv_inertia(),
					B->get_inv_inertia()));
}

bool GodotGeneric6DOFJoint3D::testAngularLimitMotor(int axis_index) {
	real_t angle = m_calculatedAxisAngleDiff[axis_index];

	//test limits
	m_angularLimits[axis_index].testLimitValue(angle);
	return m_angularLimits[axis_index].needApplyTorques();
}

bool GodotGeneric6DOFJoint3D::setup(real_t p_timestep) {
	dynamic_A = (A->get_mode() > PhysicsServer3D::BODY_MODE_KINEMATIC);
	dynamic_B = (B->get_mode() > PhysicsServer3D::BODY_MODE_KINEMATIC);

	if (!dynamic_A && !dynamic_B) {
		return false;
	}

	// Clear accumulated impulses for the next simulation step
	m_linearLimits.m_accumulatedImpulse = Vector3(real_t(0.), real_t(0.), real_t(0.));
	m_linearLimits.m_accumulatedMotorImpulse = Vector3(0.0, 0.0, 0.0);
	int i;
	for (i = 0; i < 3; i++) {
		m_angularLimits[i].m_accumulatedImpulse = real_t(0.);
	}
	//calculates transform
	calculateTransforms();

	//  const Vector3& pivotAInW = m_calculatedTransformA.origin;
	//  const Vector3& pivotBInW = m_calculatedTransformB.origin;
	calcAnchorPos();
	Vector3 pivotAInW = m_AnchorPos;
	Vector3 pivotBInW = m_AnchorPos;

	// not used here
	//    Vector3 rel_pos1 = pivotAInW - A->get_transform().origin;
	//    Vector3 rel_pos2 = pivotBInW - B->get_transform().origin;

	Vector3 normalWorld;
	//linear part
	for (i = 0; i < 3; i++) {
		if (m_linearLimits.enable_limit[i] && m_linearLimits.isLimited(i)) {
			if (m_useLinearReferenceFrameA) {
				normalWorld = m_calculatedTransformA.basis.get_column(i);
			} else {
				normalWorld = m_calculatedTransformB.basis.get_column(i);
			}

			buildLinearJacobian(m_jacLinear[i], normalWorld,
					    pivotAInW, pivotBInW);
		}
	}

	// angular part
	for (i = 0; i < 3; i++) {
		//calculates error angle
		if (m_angularLimits[i].m_enableLimit && testAngularLimitMotor(i)) {
			normalWorld = this->getAxis(i);
			// Create angular atom
			buildAngularJacobian(m_jacAng[i], normalWorld);
		}
	}

	return true;
}

void GodotGeneric6DOFJoint3D::solve(real_t p_timestep) {
	m_timeStep = p_timestep;

	//calculateTransforms();

	int i;

	// angular
	Vector3 angular_axis;
	real_t angularJacDiagABInv;
	for (i = 0; i < 3; i++) {
		if (m_angularLimits[i].m_enableLimit && m_angularLimits[i].needApplyTorques()) {
			// get axis
			angular_axis = getAxis(i);

			angularJacDiagABInv = real_t(1.) / m_jacAng[i].getDiagonal();

			m_angularLimits[i].solveAngularLimits(m_timeStep, angular_axis, angularJacDiagABInv, A, B, dynamic_A, dynamic_B);
		}
	}

	// linear

	Vector3 pointInA = m_calculatedTransformA.origin;
	Vector3 pointInB = m_calculatedTransformB.origin;

	real_t jacDiagABInv;
	Vector3 linear_axis;
	for (i = 0; i < 3; i++) {
		jacDiagABInv = real_t(1.) / m_jacLinear[i].getDiagonal();
		//DEBUG("axis %d: jacobian: %f", i, jacDiagABInv);

		if (m_useLinearReferenceFrameA) {
			linear_axis = m_calculatedTransformA.basis.get_column(i);
		} else {
			linear_axis = m_calculatedTransformB.basis.get_column(i);
		}

		m_linearLimits.solveLinearAxis(
				m_timeStep,
				jacDiagABInv,
				A, pointInA,
				B, pointInB,
				dynamic_A, dynamic_B,
				i, linear_axis, m_AnchorPos);
	}
}

void GodotGeneric6DOFJoint3D::updateRHS(real_t timeStep) {
	(void)timeStep;
}

Vector3 GodotGeneric6DOFJoint3D::getAxis(int axis_index) const {
	return m_calculatedAxis[axis_index];
}

real_t GodotGeneric6DOFJoint3D::getAngle(int axis_index) const {
	return m_calculatedAxisAngleDiff[axis_index];
}

void GodotGeneric6DOFJoint3D::calcAnchorPos() {
	real_t imA = A->get_inv_mass();
	real_t imB = B->get_inv_mass();
	real_t weight;
	if (imB == real_t(0.0)) {
		weight = real_t(1.0);
	} else {
		weight = imA / (imA + imB);
	}
	const Vector3 &pA = m_calculatedTransformA.origin;
	const Vector3 &pB = m_calculatedTransformB.origin;
	m_AnchorPos = pA * weight + pB * (real_t(1.0) - weight);
}

void GodotGeneric6DOFJoint3D::set_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, real_t p_value) {
	ERR_FAIL_INDEX(p_axis, 3);
	switch (p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT:
			m_linearLimits.m_lowerLimit[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT:
			m_linearLimits.m_upperLimit[p_axis] = p_value;
			break;

		/*
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS:
			m_linearLimits.m_limitSoftness[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION:
			m_linearLimits.m_restitution[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING:
			m_linearLimits.m_damping[p_axis] = p_value;
			break;
		*/

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY:
			m_linearLimits.m_targetVelocity[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT:
			m_linearLimits.m_maxMotorForce[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
			m_linearLimits.m_springStiffness[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			m_linearLimits.m_springDamping[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			m_linearLimits.m_springEquilibrium[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT:
			m_angularLimits[p_axis].m_loLimit = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT:
			m_angularLimits[p_axis].m_hiLimit = p_value;
			break;

		/*
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS:
			m_angularLimits[p_axis].m_limitSoftness = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING:
			m_angularLimits[p_axis].m_damping = p_value;
			break;
		*/

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_RESTITUTION:
			m_angularLimits[p_axis].m_bounce = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT:
			m_angularLimits[p_axis].m_maxLimitForce = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP:
			m_angularLimits[p_axis].m_ERP = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY:
			m_angularLimits[p_axis].m_targetVelocity = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT:
			m_angularLimits[p_axis].m_maxLimitForce = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
			m_angularLimits[p_axis].m_springStiffness = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
			m_angularLimits[p_axis].m_springDamping = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
			m_angularLimits[p_axis].m_springEquilibrium = p_value;
			break;

		default:
			WARN_DEPRECATED_MSG("The parameter " + itos(p_param) + " is deprecated.");
			break;
	}
}

real_t GodotGeneric6DOFJoint3D::get_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const {
	ERR_FAIL_INDEX_V(p_axis, 3, 0);
	switch (p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT:
			return m_linearLimits.m_lowerLimit[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT:
			return m_linearLimits.m_upperLimit[p_axis];

		/*
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS:
			return m_linearLimits.m_limitSoftness[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION:
			return m_linearLimits.m_restitution[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING:
			return m_linearLimits.m_damping[p_axis];
		*/

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT:
			return m_angularLimits[p_axis].m_loLimit;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT:
			return m_angularLimits[p_axis].m_hiLimit;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY:
			return m_linearLimits.m_targetVelocity[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT:
			return m_linearLimits.m_maxMotorForce[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
			return m_linearLimits.m_springStiffness[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			return m_linearLimits.m_springDamping[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			return m_linearLimits.m_springEquilibrium[p_axis];

		/*
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS:
			return m_angularLimits[p_axis].m_limitSoftness;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING:
			return m_angularLimits[p_axis].m_damping;
		*/

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_RESTITUTION:
			return m_angularLimits[p_axis].m_bounce;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT:
			return m_angularLimits[p_axis].m_maxLimitForce;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP:
			return m_angularLimits[p_axis].m_ERP;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY:
			return m_angularLimits[p_axis].m_targetVelocity;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT:
			return m_angularLimits[p_axis].m_maxMotorForce;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
			return m_angularLimits[p_axis].m_springStiffness;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
			return m_angularLimits[p_axis].m_springDamping;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
			return m_angularLimits[p_axis].m_springEquilibrium;

		default:
			WARN_DEPRECATED_MSG("The parameter " + itos(p_param) + " is deprecated.");
			break;
	}

	return 0;
}

void GodotGeneric6DOFJoint3D::set_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_value) {
	ERR_FAIL_INDEX(p_axis, 3);

	switch (p_flag) {
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT:
			m_linearLimits.enable_limit[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT:
			m_angularLimits[p_axis].m_enableLimit = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR:
			m_angularLimits[p_axis].m_enableMotor = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR:
			DEBUG("%sabling linear motor for axis %d", p_value ? "en" : "dis", p_axis);
			m_linearLimits.m_enableMotor[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING:
			m_linearLimits.m_enableSpring[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING:
			m_angularLimits[p_axis].m_enableSpring = p_value;
			break;

		default:
			WARN_DEPRECATED_MSG("The flag " + itos(p_flag) + " is deprecated.");
			break;
	}
}

bool GodotGeneric6DOFJoint3D::get_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const {
	ERR_FAIL_INDEX_V(p_axis, 3, 0);
	switch (p_flag) {
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT:
			return m_linearLimits.enable_limit[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT:
			return m_angularLimits[p_axis].m_enableLimit;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR:
			return m_angularLimits[p_axis].m_enableMotor;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR:
			return m_linearLimits.m_enableMotor[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING:
			return m_linearLimits.m_enableSpring[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING:
			return m_angularLimits[p_axis].m_enableSpring;

		default:
			WARN_DEPRECATED_MSG("The flag " + itos(p_flag) + " is deprecated.");
			break;
	}

	return false;
}

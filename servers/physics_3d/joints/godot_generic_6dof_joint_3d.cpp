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
 * adapted to Godot from the Bullet library's btGeneric6DofSpring2Constraint
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

#include "godot_generic_6dof_joint_3d.h"

#define GENERIC_D6_DISABLE_WARMSTARTING 1

//////////////////////////// GodotG6DOFRotationalLimitMotor3D ////////////////////////////////////

void GodotGeneric6DOFRotationalLimitMotor3D::testLimitValue(real_t test_value)
{
	// we can't normalize the angles here because we would lost the sign that we use later, but it doesn't seem to be a problem
	if (m_loLimit > m_hiLimit) {
		m_currentLimit = 0;
		m_currentLimitError = 0.0;
	} else if (m_loLimit == m_hiLimit) {
		m_currentLimitError = test_value - m_loLimit;
		m_currentLimit = 3;
	} else {
		m_currentLimitError = test_value - m_loLimit;
		m_currentLimitErrorHi = test_value - m_hiLimit;
		m_currentLimit = 4;
	}
}

//////////////////////////// GodotG6DOFTranslationalLimitMotor3D ////////////////////////////////////

void GodotGeneric6DOFTranslationalLimitMotor3D::testLimitValue(int limitIndex, real_t test_value)
{
	real_t loLimit = m_lowerLimit[limitIndex];
	real_t hiLimit = m_upperLimit[limitIndex];

	if (loLimit > hiLimit) {
		m_currentLimitError[limitIndex] = 0.0;
		m_currentLimit[limitIndex] = 0;
	} else if (loLimit == hiLimit) {
		m_currentLimitError[limitIndex] = test_value - loLimit;
		m_currentLimit[limitIndex] = 3;
	} else {
		m_currentLimitError[limitIndex] = test_value - loLimit;
		m_currentLimitErrorHi[limitIndex] = test_value - hiLimit;
		m_currentLimit[limitIndex] = 4;
	}
}

//////////////////////////// GodotGeneric6DOFJoint3D ////////////////////////////////////

GodotGeneric6DOFJoint3D::GodotGeneric6DOFJoint3D(GodotBody3D *rbA, GodotBody3D *rbB, const Transform3D &frameInA, const Transform3D &frameInB, bool useLinearReferenceFrameA, RotateOrder rotateOrder) : GodotJoint3D(_arr, 2), m_frameInA(frameInA), m_frameInB(frameInB), m_useLinearReferenceFrameA(useLinearReferenceFrameA), m_rotateOrder(rotateOrder)
{
	A = rbA;
	B = rbB;
	A->add_constraint(this, 0);
	B->add_constraint(this, 1);

	calculateTransforms();
}

bool GodotGeneric6DOFJoint3D::setup(real_t p_step)
{
	int i;

	print_line(vformat("> GodotGeneric6DOFJoint3D::%s(p_step=%f)", __func__, p_step));

	calculateTransforms();

	// test linear limits
	for (i = 0; i < 3; i++) {
		//if (m_linearLimits.m_currentLimit[i] == 4)
		//	info->m_numConstraintRows += 2;
		//else if (m_linearLimits.m_currentLimit[i] != 0)
		//	info->m_numConstraintRows += 1;

		//if (m_linearLimits.m_enableMotor[i])
		//	info->m_numConstraintRows += 1;

		//if (m_linearLimits.m_enableSpring[i])
		//	info->m_numConstraintRows += 1;
	}

	// test angular limits
	for (i = 0; i < 3; i++) {
		testAngularLimitMotor(i);

		//if (m_angularLimits[i].m_currentLimit == 4)
		//	info->m_numConstraintRows += 2;
		//else if (m_angularLimits[i].m_currentLimit[i] != 0)
		//	info->m_numConstraintRows += 1;

		//if (m_angularLimits[i].m_enableMotor)
		//	info->m_numConstraintRows += 1;

		//if (m_angularLimits[i].m_enableSpring)
		//	info->m_numConstraintRows += 1;
	}

	print_line(vformat("< GodotGeneric6DOFJoint3D::%s()", __func__));
	return true;
}

int GodotGeneric6DOFJoint3D::get_limit_motor_info(GodotGeneric6DOFRotationalLimitMotor3D *motor,
						  const Transform3D &transA,
						  const Transform3D &transB,
						  const Vector3 &linVelA,
						  const Vector3 &linVelB,
						  const Vector3 &angVelA,
						  const Vector3 &angVelB,
						  GodotGeneric6DOFConstraintInfo *info,
						  int row, Vector3 &axis, bool rotational,
						  bool rotationAllowed)
{
	return 0;
}

void GodotGeneric6DOFJoint3D::solve(real_t p_step)
{
	GodotGeneric6DOFConstraintInfo info;
	const Transform3D &transA = A->get_transform();
	const Transform3D &transB = B->get_transform();
	const Vector3 &linVelA = A->get_linear_velocity();
	const Vector3 &linVelB = B->get_linear_velocity();
	const Vector3 &angVelA = A->get_angular_velocity();
	const Vector3 &angVelB = B->get_angular_velocity();

	print_line(vformat("> GodotGeneric6DOFJoint3D::%s(p_step=%f)", __func__, p_step));

	memset(&info, 0, sizeof(info));

	// for stability, better to solve angular limits first
	// solve angular limits
	int index[] = { 0, 1, 2 };
	int i, row = 0;

	switch (m_rotateOrder) {
		case RO_XYZ:
			index[0] = 0;
			index[1] = 1;
			index[2] = 2;
			break;

		case RO_XZY:
			index[0] = 0;
			index[1] = 2;
			index[2] = 1;
			break;

		case RO_YXZ:
			index[0] = 1;
			index[1] = 0;
			index[2] = 2;
			break;

		case RO_YZX:
			index[0] = 1;
			index[1] = 2;
			index[2] = 0;
			break;

		case RO_ZXY:
			index[0] = 2;
			index[1] = 0;
			index[2] = 1;
			break;

		case RO_ZYX:
			index[0] = 2;
			index[1] = 1;
			index[2] = 0;
			break;
	}

	for (i = 0; i < 3; i++) {
		int j = index[i];

		if (m_angularLimits[j].m_currentLimit || m_angularLimits[j].m_enableMotor || m_angularLimits[i].m_enableSpring) {
			int flags = m_flags >> ((i + 3) * GODOT_6DOF_FLAGS_AXIS_SHIFT);
			Vector3 axis = getAxis(i);

			if ((flags & GODOT_6DOF_FLAGS_CFM_STOP) == 0)
				m_angularLimits[i].m_stopCFM = info.cfm[0];

			if ((flags & GODOT_6DOF_FLAGS_ERP_STOP) == 0)
				m_angularLimits[i].m_stopERP = info.erp;

			if ((flags & GODOT_6DOF_FLAGS_CFM_MOTO) == 0)
				m_angularLimits[i].m_motorCFM = info.cfm[0];

			if ((flags & GODOT_6DOF_FLAGS_ERP_MOTO) == 0)
				m_angularLimits[i].m_motorERP = info.erp;

			row += get_limit_motor_info(&m_angularLimits[i], transA, transB, linVelA, linVelB, angVelA, angVelB, &info, row, axis, true);
		}

	}

	// solve linear limits
	GodotGeneric6DOFRotationalLimitMotor3D motor;

	for (i = 0; i < 3; i++) {
		if (m_linearLimits.m_currentLimit[i] || m_linearLimits.m_enableMotor[i] || m_linearLimits.m_enableSpring[i]) {
			motor.m_bounce = m_linearLimits.m_bounce[i];
			motor.m_currentLimit = m_linearLimits.m_currentLimit[i];
		}
	}

	print_line(vformat("< GodotGeneric6DOFJoint3D::%s()", __func__));
}

#if 0
//////////////////////////// GodotG6DOFRotationalLimitMotor3D ////////////////////////////////////

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

real_t GodotG6DOFTranslationalLimitMotor3D::solveLinearAxis(
		real_t timeStep,
		real_t jacDiagABInv,
		GodotBody3D *body1, const Vector3 &pointInA,
		GodotBody3D *body2, const Vector3 &pointInB,
		bool p_body1_dynamic, bool p_body2_dynamic,
		int limit_index,
		const Vector3 &axis_normal_on_a,
		const Vector3 &anchorPos) {
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

	//positional error (zeroth order error)
	real_t depth = -(pointInA - pointInB).dot(axis_normal_on_a);
	real_t lo = real_t(-1e30);
	real_t hi = real_t(1e30);

	real_t minLimit = m_lowerLimit[limit_index];
	real_t maxLimit = m_upperLimit[limit_index];

	//handle the limits
	if (minLimit < maxLimit) {
		{
			if (depth > maxLimit) {
				depth -= maxLimit;
				lo = real_t(0.);

			} else {
				if (depth < minLimit) {
					depth -= minLimit;
					hi = real_t(0.);
				} else {
					return 0.0f;
				}
			}
		}
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
	return normalImpulse;
}
#endif

void GodotGeneric6DOFJoint3D::calculateLinearInfo()
{
	m_calculatedLinearDiff = m_calculatedTransformB.origin - m_calculatedTransformA.origin;
	m_calculatedLinearDiff = m_calculatedTransformA.basis.xform_inv(m_calculatedLinearDiff);

	for (int i = 0; i < 3; i++) {
		m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
		m_linearLimits.testLimitValue(i, m_calculatedLinearDiff[i]);
	}
}

void GodotGeneric6DOFJoint3D::calculateAngleInfo()
{
	Basis relative_frame = m_calculatedTransformB.basis.inverse() * m_calculatedTransformA.basis;

	m_calculatedAxisAngleDiff = relative_frame.get_euler(EulerOrder::XYZ);

	switch (m_rotateOrder) {
		case RO_XYZ:
			//matrixToEulerXYZ(relative_frame, m_calculatedAxisAngleDiff);
			break;

		case RO_XZY:
			//matrixToEulerXZY(relative_frame, m_calculatedAxisAngleDiff);
			break;

		case RO_YXZ:
			//matrixToEulerYXZ(relative_frame, m_calculatedAxisAngleDiff);
			break;

		case RO_YZX:
			//matrixToEulerYZX(relative_frame, m_calculatedAxisAngleDiff);
			break;

		case RO_ZXY:
			//matrixToEulerZXY(relative_frame, m_calculatedAxisAngleDiff);
			break;

		case RO_ZYX:
			//matrixToEulerZYX(relative_frame, m_calculatedAxisAngleDiff);
			break;
	}

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

	switch (m_rotateOrder) {
		case RO_XYZ: {
			// Is this the "line of nodes" calculation choosing planes YZ (B coordinate system) and xy (A coordinate system)? (http://en.wikipedia.org/wiki/Euler_angles)
			// The two planes are non-homologous, so this is a Tait Bryan angle formalism and not a proper Euler
			// Extrinsic rotations are equal to the reversed order intrinsic rotations so the above xyz extrinsic rotations (axes are fixed) are the same as the zy'X" intrinsic rotations (axes are refreshed after each rotation)
			// that is why xy and YZ planes are chosen (this will describe a zy'x" intrinsic rotation) (see the figure on the left at http://en.wikipedia.org/wiki/Euler_angles under Tait Bryan angles)
			// x' = Nperp = N.cross(axis2)
			// y' = N = axis2.cross(axis0)
			// z' = z
			//
			// x" = X
			// y" = y'
			// z" = ??
			// in other words:
			// first rotate around z
			// second rotate around y' = z.cross(X)
			// third rotate around x" = X
			// Original XYZ extrinsic rotation order.
			// Planes: xy and YZ normals: z, X.  Plane intersection (N) is z.cross(X)
			Vector3 axis0 = m_calculatedTransformB.basis.get_column(0);
			Vector3 axis2 = m_calculatedTransformA.basis.get_column(2);

			m_calculatedAxis[1] = axis2.cross(axis0);
			m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
			m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);
			break;
		}

		case RO_XZY: {
			// planes: xz,ZY normals: y, X
			// first rotate around y
			// second rotate around z' = y.cross(X)
			// third rotate around x" = X
			Vector3 axis0 = m_calculatedTransformB.basis.get_column(0);
			Vector3 axis1 = m_calculatedTransformA.basis.get_column(1);

			m_calculatedAxis[2] = axis0.cross(axis1);
			m_calculatedAxis[0] = axis1.cross(m_calculatedAxis[2]);
			m_calculatedAxis[1] = m_calculatedAxis[2].cross(axis0);
			break;
		}

		case RO_YXZ: {
			// planes: yx, XZ normals: z, Y
			// first rotate around z
			// second rotate around x' = z.cross(Y)
			// third rotate around y" = Y
			Vector3 axis1 = m_calculatedTransformB.basis.get_column(1);
			Vector3 axis2 = m_calculatedTransformA.basis.get_column(2);

			m_calculatedAxis[0] = axis1.cross(axis2);
			m_calculatedAxis[1] = axis1.cross(m_calculatedAxis[0]);
			m_calculatedAxis[2] = m_calculatedAxis[0].cross(axis1);
			break;
		}

		case RO_YZX: {
			// planes: yz,ZX normals: x, Y
			// first rotate around x
			// second rotate around z' = x.cross(Y)
			// third rotate around y" = Y
			Vector3 axis0 = m_calculatedTransformA.basis.get_column(0);
			Vector3 axis1 = m_calculatedTransformB.basis.get_column(1);

			m_calculatedAxis[2] = axis0.cross(axis1);
			m_calculatedAxis[0] = axis1.cross(m_calculatedAxis[2]);
			m_calculatedAxis[1] = m_calculatedAxis[2].cross(axis0);
			break;
		}

		case RO_ZXY: {
			// planes: zx,XY normals: y, Z
			// first rotate around y
			// second rotate around x' = y.cross(Z)
			// third rotate around z" = Z
			Vector3 axis1 = m_calculatedTransformA.basis.get_column(1);
			Vector3 axis2 = m_calculatedTransformB.basis.get_column(2);

			m_calculatedAxis[0] = axis1.cross(axis2);
			m_calculatedAxis[1] = axis2.cross(m_calculatedAxis[0]);
			m_calculatedAxis[2] = m_calculatedAxis[0].cross(axis1);
			break;
		}

		case RO_ZYX: {
			// planes: zy,YX normals: x, Z
			// first rotate around x
			// second rotate around y' = x.cross(Z)
			// third rotate around z" = Z
			Vector3 axis0 = m_calculatedTransformA.basis.get_column(0);
			Vector3 axis2 = m_calculatedTransformB.basis.get_column(2);

			m_calculatedAxis[1] = axis2.cross(axis0);
			m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
			m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);
			break;
		}
	}

	m_calculatedAxis[0].normalize();
	m_calculatedAxis[1].normalize();
	m_calculatedAxis[2].normalize();
}

void GodotGeneric6DOFJoint3D::calculateTransforms()
{
	m_calculatedTransformA = A->get_transform() * m_frameInA;
	m_calculatedTransformB = B->get_transform() * m_frameInB;

	calculateLinearInfo();
	calculateAngleInfo();

	real_t miA = A->get_inv_mass();
	real_t miB = B->get_inv_mass();
	real_t miS = miA + miB;

	if (Math::is_zero_approx(miA) || Math::is_zero_approx(miB))
		m_hasStaticBody = true;
	else
		m_hasStaticBody = false;

	if (miS > 0.0)
		m_factA = miB / miS;
	else
		m_factA = 0.5;

	m_factB = 1.0 - m_factA;
}

static real_t AdjustAngleToLimits(real_t angle, real_t lowerLimit, real_t upperLimit)
{
	if (lowerLimit >= upperLimit)
		return angle;

	if (angle < lowerLimit) {
		real_t diffLo = Math::abs(Math::normalize_angle(lowerLimit - angle));
		real_t diffHi = Math::abs(Math::normalize_angle(upperLimit - angle));
		return (diffLo < diffHi) ? angle : (angle + Math_PI * 2);
	}

	if (angle > upperLimit) {
		real_t diffLo = Math::abs(Math::normalize_angle(angle - upperLimit));
		real_t diffHi = Math::abs(Math::normalize_angle(angle - lowerLimit));
		return (diffLo < diffHi) ? (angle - Math_PI * 2) : angle;
	}

	return angle;
}

void GodotGeneric6DOFJoint3D::testAngularLimitMotor(int axis_index)
{
	real_t angle = m_calculatedAxisAngleDiff[axis_index];
	angle = AdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit);

	m_angularLimits[axis_index].m_currentPosition = angle;
	m_angularLimits[axis_index].testLimitValue(angle);
}

#if 0
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

bool GodotGeneric6DOFJoint3D::setup(real_t p_timestep) {
	dynamic_A = (A->get_mode() > PhysicsServer3D::BODY_MODE_KINEMATIC);
	dynamic_B = (B->get_mode() > PhysicsServer3D::BODY_MODE_KINEMATIC);

	if (!dynamic_A && !dynamic_B) {
		return false;
	}

	if (m_useSolveConstraintObsolete) {
		// Clear accumulated impulses for the next simulation step
		m_linearLimits.m_accumulatedImpulse = Vector3(real_t(0.), real_t(0.), real_t(0.));
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

				buildLinearJacobian(
					m_jacLinear[i], normalWorld,
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
	}

	return true;
}

void GodotGeneric6DOFJoint3D::solve(real_t p_timestep) {
	m_timeStep = p_timestep;

	//calculateTransforms();

	int i;

	// linear

	Vector3 pointInA = m_calculatedTransformA.origin;
	Vector3 pointInB = m_calculatedTransformB.origin;

	real_t jacDiagABInv;
	Vector3 linear_axis;
	for (i = 0; i < 3; i++) {
		if (m_linearLimits.enable_limit[i] && m_linearLimits.isLimited(i)) {
			jacDiagABInv = real_t(1.) / m_jacLinear[i].getDiagonal();

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
#endif

void GodotGeneric6DOFJoint3D::setParam(int num, real_t value, int axis)
{
}

real_t GodotGeneric6DOFJoint3D::getParam(int num, int axis) const
{
	return 0.0;
}

void GodotGeneric6DOFJoint3D::set_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, real_t p_value)
{
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
		*/

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION:
			m_linearLimits.m_bounce[p_axis] = p_value;
			break;

		/*
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING:
			m_linearLimits.m_damping[p_axis] = p_value;
			break;
		*/

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

		/*
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT:
			m_angularLimits[p_axis].m_maxLimitForce = p_value;
			break;
		*/

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP:
			m_angularLimits[p_axis].m_stopERP = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY:
			m_angularLimits[p_axis].m_targetVelocity = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT:
			m_angularLimits[p_axis].m_maxMotorForce = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY:
			print_line(vformat("XXX: setting linear target velocity for axis %d to %f", p_axis, p_value));
			m_linearLimits.m_targetVelocity[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT:
			print_line(vformat("XXX: setting linear force limit for axis %d to %f", p_axis, p_value));
			m_linearLimits.m_maxMotorForce[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
			print_line(vformat("XXX: setting linear spring stiffness for axis %d to %f", p_axis, p_value));
			m_linearLimits.m_springStiffness[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			print_line(vformat("XXX: setting linear spring damping for axis %d to %f", p_axis, p_value));
			m_linearLimits.m_springDamping[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			print_line(vformat("XXX: setting linear spring equilibrium point for axis %d to %f", p_axis, p_value));
			m_linearLimits.m_equilibriumPoint[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
			print_line(vformat("XXX: setting angular spring stiffness for axis %d to %f", p_axis, p_value));
			m_angularLimits[p_axis].m_springStiffness = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
			print_line(vformat("XXX: setting angular spring damping for axis %d to %f", p_axis, p_value));
			m_angularLimits[p_axis].m_springDamping = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
			print_line(vformat("XXX: setting angular spring equilibrium point for axis %d to %f", p_axis, p_value));
			m_angularLimits[p_axis].m_equilibriumPoint = p_value;
			break;

		default:
			WARN_DEPRECATED_MSG("The parameter " + itos(p_param) + " is deprecated.");
			break;
	}
}

real_t GodotGeneric6DOFJoint3D::get_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const
{
	ERR_FAIL_INDEX_V(p_axis, 3, 0);

	switch (p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT:
			return m_linearLimits.m_lowerLimit[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT:
			return m_linearLimits.m_upperLimit[p_axis];

		/*
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS:
			return m_linearLimits.m_limitSoftness[p_axis];
		*/

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION:
			return m_linearLimits.m_bounce[p_axis];

		/*
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING:
			return m_linearLimits.m_damping[p_axis];
		*/

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT:
			return m_angularLimits[p_axis].m_loLimit;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT:
			return m_angularLimits[p_axis].m_hiLimit;

		/*
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS:
			return m_angularLimits[p_axis].m_limitSoftness;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING:
			return m_angularLimits[p_axis].m_damping;
		*/

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_RESTITUTION:
			return m_angularLimits[p_axis].m_bounce;

		/*
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT:
			return m_angularLimits[p_axis].m_maxLimitForce;
		*/

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP:
			return m_angularLimits[p_axis].m_stopERP;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY:
			return m_angularLimits[p_axis].m_targetVelocity;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT:
			return m_angularLimits[p_axis].m_maxMotorForce;

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY:
			return m_linearLimits.m_targetVelocity[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT:
			return m_linearLimits.m_maxMotorForce[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
			return m_linearLimits.m_springStiffness[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			return m_linearLimits.m_springDamping[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			return m_linearLimits.m_equilibriumPoint[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
			return m_angularLimits[p_axis].m_springStiffness;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
			return m_angularLimits[p_axis].m_springDamping;

		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
			return m_angularLimits[p_axis].m_equilibriumPoint;

		default:
			WARN_DEPRECATED_MSG("The parameter " + itos(p_param) + " is deprecated.");
			break;
	}

	return 0;
}

void GodotGeneric6DOFJoint3D::set_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_value)
{
	ERR_FAIL_INDEX(p_axis, 3);

	switch (p_flag) {
		/*
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT:
			m_linearLimits.enable_limit[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT:
			m_angularLimits[p_axis].m_enableLimit = p_value;
			break;
		*/

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR:
			m_angularLimits[p_axis].m_enableMotor = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR:
			print_line(vformat("XXX: %s linear motor for axis %d", p_value ? "enabling" : "disabling", p_axis));
			m_linearLimits.m_enableMotor[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING:
			print_line(vformat("XXX: %s linear spring for axis %d", p_value ? "enabling" : "disabling", p_axis));
			m_linearLimits.m_enableSpring[p_axis] = p_value;
			break;

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING:
			print_line(vformat("XXX: %s angular spring for axis %d", p_value ? "enabling" : "disabling", p_axis));
			m_angularLimits[p_axis].m_enableSpring = p_value;
			break;

		default:
			WARN_DEPRECATED_MSG("The flag " + itos(p_flag) + " is deprecated.");
			break;
	}
}

bool GodotGeneric6DOFJoint3D::get_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const
{
	ERR_FAIL_INDEX_V(p_axis, 3, 0);

	switch (p_flag) {
		/*
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT:
			return m_linearLimits.enable_limit[p_axis];

		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT:
			return m_angularLimits[p_axis].m_enableLimit;
		*/

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

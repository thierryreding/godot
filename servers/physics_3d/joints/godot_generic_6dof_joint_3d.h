/**************************************************************************/
/*  godot_generic_6dof_joint_3d.h                                         */
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

#ifndef GODOT_GENERIC_6DOF_JOINT_3D_H
#define GODOT_GENERIC_6DOF_JOINT_3D_H

/*
 * Adapted to Godot from the Bullet library.
 */

#include "servers/physics_3d/godot_joint_3d.h"
#include "servers/physics_3d/joints/godot_jacobian_entry_3d.h"

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

enum RotateOrder {
	RO_XYZ = 0,
	RO_XZY,
	RO_YXZ,
	RO_YZX,
	RO_ZXY,
	RO_ZYX,
};

class GodotG6DOFRotationalLimitMotor3D {
public:
	// upper < lower means free
	// upper == lower means locked
	// upper > lower means limited
	real_t m_loLimit = -1e30;
	real_t m_hiLimit = 1e30;
	real_t m_bounce = 0.0;
	real_t m_stopERP = 0.5;
	real_t m_stopCFM;
	real_t m_motorERP;
	real_t m_motorCFM;
	bool m_enableMotor;
	real_t m_targetVelocity = 0.0; //!< target motor velocity
	real_t m_maxMotorForce = 0.1; //!< max force on motor
	bool m_servoMotor;
	real_t m_servoTarget;
	bool m_enableSpring;
	real_t m_springStiffness;
	bool m_springStiffnessLimited;
	real_t m_springDamping;
	bool m_springDampingLimited;
	real_t m_equilibriumPoint;

	real_t m_currentLimitError;
	real_t m_currentLimitErrorHi;
	real_t m_currentPosition;
	int m_currentLimit;

	GodotG6DOFRotationalLimitMotor3D()
	{
		m_loLimit = 1.0;
		m_hiLimit = -1.0;
		m_bounce = 0.0;
		m_stopERP = 0.2;
		m_stopCFM = 0.0;
		m_motorERP = 0.9;
		m_motorCFM = 0.0;
		m_enableMotor = false;
		m_targetVelocity = 0.0;
		m_maxMotorForce = 6.0;
		m_servoMotor = false;
		m_servoTarget = 0.0;
		m_enableSpring = false;
		m_springStiffness = 0.0;
		m_springStiffnessLimited = false;
		m_springDamping = 0.0;
		m_springDampingLimited = false;
		m_equilibriumPoint = 0.0;

		m_currentLimitError = 0.0;
		m_currentLimitErrorHi = 0.0;
		m_currentPosition = 0.0;
		m_currentLimit = 0;
	}

	bool isLimited()
	{
		return m_loLimit < m_hiLimit;
	}

	void testLimitValue(real_t test_value);
};

class GodotG6DOFTranslationalLimitMotor3D {
public:
	// upper < lower means free
	// upper == lower means locked
	// upper > lower means limited
	Vector3 m_lowerLimit;
	Vector3 m_upperLimit;
	Vector3 m_bounce;
	Vector3 m_stopERP;
	Vector3 m_stopCFM;
	Vector3 m_motorERP;
	Vector3 m_motorCFM;
	bool m_enableMotor[3];
	bool m_servoMotor[3];
	bool m_enableSpring[3];
	Vector3 m_servoTarget;
	Vector3 m_springStiffness;
	bool m_springStiffnessLimited[3];
	Vector3 m_springDamping;
	bool m_springDampingLimited[3];
	Vector3 m_equilibriumPoint;
	Vector3 m_targetVelocity;
	Vector3 m_maxMotorForce;

	Vector3 m_currentLimitError;
	Vector3 m_currentLimitErrorHi;
	Vector3 m_currentLinearDiff;
	int m_currentLimit[3];

	GodotG6DOFTranslationalLimitMotor3D()
	{
		m_lowerLimit = Vector3(0.0, 0.0, 0.0);
		m_upperLimit = Vector3(0.0, 0.0, 0.0);
		m_bounce = Vector3(0.0, 0.0, 0.0);
		m_stopERP = Vector3(0.2, 0.2, 0.2);
		m_stopCFM = Vector3(0.0, 0.0, 0.0);
		m_motorERP = Vector3(0.9, 0.9, 0.9);
		m_motorCFM = Vector3(0.0, 0.0, 0.0);

		m_currentLimitError = Vector3(0.0, 0.0, 0.0);
		m_currentLimitErrorHi = Vector3(0.0, 0.0, 0.0);
		m_currentLinearDiff = Vector3(0.0, 0.0, 0.0);

		for (int i = 0; i < 3; i++) {
			m_enableMotor[i] = false;
			m_servoMotor[i] = false;
			m_enableSpring[i] = false;
			m_servoTarget[i] = 0.0;
			m_springStiffness[i] = 0.0;
			m_springStiffnessLimited[i] = false;
			m_springDamping[i] = 0.0;
			m_springDampingLimited[i] = false;
			m_equilibriumPoint[i] = 0.0;
			m_targetVelocity[i] = 0.0;
			m_maxMotorForce[i] = 0.0;

			m_currentLimit[i] = 0;
		}
	}

	inline bool isLimited(int limitIndex)
	{
		return m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex];
	}

	void testLimitValue(int limitIndex, real_t test_value);
};

enum Godot6DOFFlags {
	GODOT_6DOF_FLAGS_CFM_STOP = 1,
	GODOT_6DOF_FLAGS_ERP_STOP = 2,
	GODOT_6DOF_FLAGS_CFM_MOTO = 4,
	GODOT_6DOF_FLAGS_ERP_MOTO = 8,
	GODOT_6DOF_FLAGS_USE_INFINITE_ERROR = 1 << 16,
};

#define GODOT_6DOF_FLAGS_AXIS_SHIFT 4 // bits per axis

class GodotGeneric6DOFJoint3D : public GodotJoint3D {
protected:
	union {
		struct {
			GodotBody3D *A;
			GodotBody3D *B;
		};

		GodotBody3D *_arr[2] = { nullptr, nullptr };
	};

	Transform3D m_frameInA;
	Transform3D m_frameInB;

	GodotJacobianEntry3D m_jacLinear[3];
	GodotJacobianEntry3D m_jacAng[3];

	GodotG6DOFTranslationalLimitMotor3D m_linearLimits;
	GodotG6DOFRotationalLimitMotor3D m_angularLimits[3];

	bool m_useLinearReferenceFrameA;
	RotateOrder m_rotateOrder;

protected:
	Transform3D m_calculatedTransformA;
	Transform3D m_calculatedTransformB;
	Vector3 m_calculatedAxisAngleDiff;
	Vector3 m_calculatedAxis[3];
	Vector3 m_calculatedLinearDiff;
	real_t m_factA;
	real_t m_factB;
	bool m_hasStaticBody;
	int m_flags;

	GodotGeneric6DOFJoint3D(GodotGeneric6DOFJoint3D const &) = delete;
	void operator = (GodotGeneric6DOFJoint3D const &) = delete;

	int setAngularLimits();
	int setLinearLimits();

	void calculateLinearInfo();
	void calculateAngleInfo();
	void testAngularLimitMotor(int axis_index);

public:
	GodotGeneric6DOFJoint3D(GodotBody3D *rbA, GodotBody3D *rbB, const Transform3D &frameInA, const Transform3D &frameInB, bool useLinearReferenceFrameA, RotateOrder rotateOrder = RO_XYZ);

	virtual PhysicsServer3D::JointType get_type() const override
	{
		return PhysicsServer3D::JOINT_TYPE_6DOF;
	}

	virtual bool setup(real_t p_step) override;
	virtual void solve(real_t p_step) override;

	const GodotBody3D *getRigidBodyA() const
	{
		return A;
	}

	const GodotBody3D *getRigidBodyB() const
	{
		return B;
	}

	void set_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, real_t p_value);
	real_t get_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const;

	void set_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_value);
	bool get_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const;

	GodotG6DOFRotationalLimitMotor3D *getRotationalLimitMotor(int index)
	{
		return &m_angularLimits[index];
	}

	GodotG6DOFTranslationalLimitMotor3D *getTranslationalLimitMotor()
	{
		return &m_linearLimits;
	}

	// Calculates the global transform for the joint offset for body A an B, and also calculates the angle differences between the bodies.
	void calculateTransforms(const Transform3D &transA, const Transform3D &transB);
	void calculateTransforms();

	// Gets the global transform of the offset for body A.
	const Transform3D &getCalculatedTransformA() const
	{
		return m_calculatedTransformA;
	}

	// Gets the global transform of the offset for body B.
	const Transform3D &getCalculatedTransformB() const
	{
		return m_calculatedTransformB;
	}

	const Transform3D &getFrameOffsetA() const
	{
		return m_frameInA;
	}

	const Transform3D &getFrameOffsetB() const
	{
		return m_frameInB;
	}

	Transform3D &getFrameOffsetA()
	{
		return m_frameInA;
	}

	Transform3D &getFrameOffsetB()
	{
		return m_frameInB;
	}

	// Get the rotation axis in global coordinates (GodotGeneric6DOFJoint3D::calculateTransforms() must be called previously)
	Vector3 getAxis(int axis_index) const
	{
		return m_calculatedAxis[axis_index];
	}

	// Get the relative Euler angle (GodotGeneric6DOFJoint3D::calculateTransforms() must be called previously)
	real_t getAngle(int axis_index) const
	{
		return m_calculatedAxisAngleDiff[axis_index];
	}

	// Get the relative position of the constraint pivot (GodotGeneric6DOFJoint3D::calculateTransforms() must be called previously)
	real_t getRelativePivotPosition(int axis_index)
	{
		return m_calculatedLinearDiff[axis_index];
	}

	void setFrames(const Transform3D &frameA, const Transform3D &frameB);

	void setLinearLowerLimit(const Vector3 &linearLower)
	{
		m_linearLimits.m_lowerLimit = linearLower;
	}

	void getLinearLowerLimit(Vector3 &linearLower)
	{
		linearLower = m_linearLimits.m_lowerLimit;
	}

	void setLinearUpperLimit(const Vector3 &linearUpper)
	{
		m_linearLimits.m_upperLimit = linearUpper;
	}

	void getLinearUpperLimit(Vector3 &linearUpper)
	{
		linearUpper = m_linearLimits.m_upperLimit;
	}

	void setAngularLowerLimit(const Vector3 &angularLower)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[0].m_loLimit = Math::normalize_angle(angularLower[i]);
	}

	void setAngularLowerLimitReversed(const Vector3 &angularLower)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[0].m_hiLimit = Math::normalize_angle(-angularLower[i]);
	}

	void getAngularLowerLimit(Vector3 &angularLower)
	{
		for (int i = 0; i < 3; i++)
			angularLower[i] = m_angularLimits[i].m_loLimit;
	}

	void getAngularLowerLimitReversed(Vector3 &angularLower)
	{
		for (int i = 0; i < 3; i++)
			angularLower[i] = -m_angularLimits[i].m_hiLimit;
	}

	void setAngularUpperLimit(const Vector3 &angularUpper)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[0].m_hiLimit = Math::normalize_angle(angularUpper[i]);
	}

	void setAngularUpperLimitReversed(const Vector3 &angularUpper)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[0].m_loLimit = Math::normalize_angle(-angularUpper[i]);
	}

	void getAngularUpperLimit(Vector3 &angularUpper)
	{
		for (int i = 0; i < 3; i++)
			angularUpper[i] = m_angularLimits[i].m_hiLimit;
	}

	void getAngularUpperLimitReversed(Vector3 &angularUpper)
	{
		for (int i = 0; i < 3; i++)
			angularUpper[i] = -m_angularLimits[i].m_loLimit;
	}

	// first 3 are linear, next 3 are angular
	void setLimit(int axis, real_t lo, real_t hi)
	{
		if (axis < 3) {
			m_linearLimits.m_lowerLimit[axis] = lo;
			m_linearLimits.m_upperLimit[axis] = hi;
		} else {
			m_angularLimits[axis - 3].m_loLimit = Math::normalize_angle(lo);
			m_angularLimits[axis - 3].m_hiLimit = Math::normalize_angle(hi);
		}
	}

	void setLimitReversed(int axis, real_t lo, real_t hi)
	{
		if (axis < 3) {
			m_linearLimits.m_lowerLimit[axis] = lo;
			m_linearLimits.m_upperLimit[axis] = hi;
		} else {
			m_angularLimits[axis - 3].m_loLimit = -Math::normalize_angle(hi);
			m_angularLimits[axis - 3].m_hiLimit = -Math::normalize_angle(lo);
		}
	}

	bool isLimited(int limitIndex)
	{
		if (limitIndex < 3)
			return m_linearLimits.isLimited(limitIndex);

		return m_angularLimits[limitIndex - 3].isLimited();
	}

	void setRotationOrder(RotateOrder rotateOrder)
	{
		m_rotateOrder = rotateOrder;
	}

	RotateOrder getRotationOrder()
	{
		return m_rotateOrder;
	}

	void setAxis(const Vector3 &axis1, const Vector3 &axis2);

	void setBounce(int index, real_t bounce);

	void enableMotor(int index, bool enable);
	void setServo(int index, bool enable); // set the type of the motor (servo or not) (the motor has to be turned on for servo also)
	void setTargetVelocity(int index, real_t velocity);
	void setServoTarget(int index, real_t target);
	void setMaxMotorForce(int index, real_t force);

	void enableSpring(int index, bool enable);
	void setStiffness(int index, real_t stiffness, bool limitIfNeeded = true); // if limitIfNeeded is true, the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely
	void setDamping(int index, real_t damping, bool limitIfNeeded = true); // if limitIfNeeded is true, the system will automatically limit the damping in necessary situations where otherwise the spring would blow up
	void setEquilibriumPoint(); // set the current constraint position/orientation as an equilibrium point for all DOF
	void setEquilibriumPoint(int index); // set the current constraint position/orientation as an equilibrium point for a given DOF
	void setEquilibriumPoint(int index, real_t value);

	// Override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5).
	// If no axis is provided, it uses the default axis for this constraint.
	virtual void setParam(int num, real_t value, int axis = -1);
	virtual real_t getParam(int num, int axis = -1) const;


};

#endif // GODOT_GENERIC_6DOF_JOINT_3D_H

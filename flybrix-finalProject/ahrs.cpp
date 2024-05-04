/*
*  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
*
*  http://www.flybrix.com
*/

#include "ahrs.h"

#include <cmath>
#include "quickmath.h"
#include "debug.h"

void se_mahony_ahrs_update_imu_with_mag(Vector3<float> g, Vector3<float> a, Vector3<float> m, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q);

void se_mahony_ahrs_update_imu(Vector3<float> g, Vector3<float> a, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q);

namespace {
	inline Quaternion<float> madgwickStepA(const Quaternion<float>& q, Vector3<float> a) {
		quick::normalize(a);
		// q* * [0 gx gy gz] * q; g = [0 0 1]
		Vector3<float> fa{
			2.f * (q.x * q.z - q.w * q.y),        // X
			2.f * (q.w * q.x + q.y * q.z),        // Y
			2.f * (0.5f - q.x * q.x - q.y * q.y)  // Z
		};
		// Subtract accelerometer measure from local frame
		fa -= a;
		// Multiply with transposed Jacobian
		return Quaternion<float>{
			-2.f * q.y * fa.x + 2.f * q.x * fa.y,                     // w
				2.f * q.z * fa.x + 2.f * q.w * fa.y - 4.f * q.x * fa.z,   // x
				-2.f * q.w * fa.x + 2.f * q.z * fa.y - 4.f * q.y * fa.z,  // y
				2.f * q.x * fa.x + 2.f * q.y * fa.y                       // z
		};
	}

	inline Quaternion<float> madgwickStepM(const Quaternion<float>& q, Vector3<float> m) {
		if (m.isZero()) {
			return Quaternion<float>{0, 0, 0, 0};
		}
		quick::normalize(m);
		Quaternion<float> h = q * m * q.conj();
		if (h.isZero()) {
			return Quaternion<float>{0, 0, 0, 0};
		}
		float b1{ 2 / quick::invSqrt(h.x * h.x + h.y * h.y) };
		float b2{ 2 * h.z };
		Vector3<float> fm{
			b1 * (0.5f - q.y * q.y - q.z * q.z) + b2 * (q.x * q.z - q.w * q.y),  // X
			b1 * (q.x * q.y - q.w * q.z) + b2 * (q.w * q.x + q.y * q.z),         // Y
			b1 * (q.w * q.y + q.x * q.z) + b2 * (0.5f - q.x * q.x - q.y * q.y)   // Z
		};
		fm -= m;
		return Quaternion<float>{
			(-b2 * q.y) * fm.x + (-b1 * q.z + b2 * q.x) * fm.y + (b1 * q.y) * fm.z,                                   // W
				(b2 * q.z) * fm.x + (b1 * q.y + b2 * q.w) * fm.y + (b1 * q.z - 2.f * b2 * q.x) * fm.z,                    // X
				(-2.f * b1 * q.y - b2 * q.w) * fm.x + (b1 * q.x + b2 * q.z) * fm.y + (b1 * q.w - 2.f * b2 * q.y) * fm.z,  // Y
				(-2.f * b1 * q.z + b2 * q.x) * fm.x + (-b1 * q.w + b2 * q.y) * fm.y + (b1 * q.x) * fm.z                   // Z
		};
	}

	inline Quaternion<float> madgwick(Quaternion<float> q, float beta, float dt, const Vector3<float>& g, const Vector3<float>& a, const Vector3<float>& m = { 0, 0, 0 }) {
		if (a.isZero()) {
			return q;
		}
		Quaternion<float> step = madgwickStepA(q, a) + madgwickStepM(q, m);
		quick::normalize(step);
		Quaternion<float> q_dot = q * (g * 0.5) - step * beta;
		q += q_dot * dt;
		quick::normalize(q);
		return q;
	}

	inline void mahony(Quaternion<float>& q, Vector3<float>& ifb, float ki, float kp, float dt, const Vector3<float>& g, const Vector3<float>& a, const Vector3<float>& m) {
		se_mahony_ahrs_update_imu_with_mag(g, a, m, dt, ki, kp, ifb, q);
	}

	inline void mahony(Quaternion<float>& q, Vector3<float>& ifb, float ki, float kp, float dt, const Vector3<float>& g, const Vector3<float>& a) {
		se_mahony_ahrs_update_imu(g, a, dt, ki, kp, ifb, q);
	}
}

void Ahrs::update(ClockTime timestamp) {
	if (!accelerometer_.ready || !gyroscope_.ready) {
		return;
	}
	uint32_t delta = timestamp - last_update_timestamp_;
	last_update_timestamp_ = timestamp;

	if (ClockTime::isNotReasonable(delta)) {
		return;
	}

	float dt = delta / 1000000.0f;

	if (dt > max_delta_time_) {
		dt = max_delta_time_;
	}

	accelerometer_.consume();
	gyroscope_.consume();


	if (magnetometer_.ready) {
		switch (type_) {
		case Type::Madgwick: {
			pose_ = madgwick(pose_, parameter_1_, dt, gyroscope_.value, accelerometer_.value, magnetometer_.value);
		} break;
		case Type::Mahony: {
			mahony(pose_, integral_feedback_, parameter_1_, parameter_2_, dt, gyroscope_.value, accelerometer_.value, magnetometer_.value);
		} break;
		case Type::UAV_Class: {
			se_euler_ahrs_update_imu(dt, pose_, magnetometer_.value, 0);
		} break;

		}
		magnetometer_.consume();
	}
	else {
		switch (type_) {
		case Type::Madgwick: {
			pose_ = madgwick(pose_, parameter_1_, dt, gyroscope_.value, accelerometer_.value, { 0, 0, 0 });
		} break;
		case Type::Mahony: {
			mahony(pose_, integral_feedback_, parameter_1_, parameter_2_, dt, gyroscope_.value, accelerometer_.value);
		} break;
		case Type::UAV_Class: {
			se_euler_ahrs_update_imu(dt, pose_, magnetometer_.value, 0);
		}break;
		}
	}
}

/* IMU algorithm update */

void se_mahony_ahrs_update_imu_with_mag(Vector3<float> g, Vector3<float> a, Vector3<float> m, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q) {
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	/*
	* Use IMU algorithm if magnetometer measurement is invalid
	* (avoids NaN in magnetometer normalization)
	*/
	if ((m.x == 0.0f) && (m.y == 0.0f) && (m.z == 0.0f)) {
		se_mahony_ahrs_update_imu(g, a, delta_time, ki_2, kp_2, fb_i, q);
		return;
	}

	/*
	* Compute feedback only if accelerometer measurement is valid
	* (avoids NaN in accelerometer normalization)
	*/
	if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {
		/* Normalize accelerometer measurement */
		quick::normalize(a);

		/* Normalize magnetometer measurement */
		quick::normalize(m);

		/* Auxiliary variables to avoid repeated arithmetic */
		q0q0 = q.w * q.w;
		q0q1 = q.w * q.x;
		q0q2 = q.w * q.y;
		q0q3 = q.w * q.z;
		q1q1 = q.x * q.x;
		q1q2 = q.x * q.y;
		q1q3 = q.x * q.z;
		q2q2 = q.y * q.y;
		q2q3 = q.y * q.z;
		q3q3 = q.z * q.z;

		/* Reference direction of Earth's magnetic field */
		hx = 2.0f * (m.x * (0.5f - q2q2 - q3q3) + m.y * (q1q2 - q0q3) + m.z * (q1q3 + q0q2));
		hy = 2.0f * (m.x * (q1q2 + q0q3) + m.y * (0.5f - q1q1 - q3q3) + m.z * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (m.x * (q1q3 - q0q2) + m.y * (q2q3 + q0q1) + m.z * (0.5f - q1q1 - q2q2));

		/* Estimated direction of gravity and magnetic field */
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		/*
		* Error is sum of cross product between estimated direction and measured
		* direction of field vectors
		*/
		halfex = (a.y * halfvz - a.z * halfvy) + (m.y * halfwz - m.z * halfwy);
		halfey = (a.z * halfvx - a.x * halfvz) + (m.z * halfwx - m.x * halfwz);
		halfez = (a.x * halfvy - a.y * halfvx) + (m.x * halfwy - m.y * halfwx);

		/* Compute and apply integral feedback if enabled */
		if (ki_2 > 0.0f) {
			fb_i.x += ki_2 * halfex * delta_time; /* integral error scaled by Ki */
			fb_i.y += ki_2 * halfey * delta_time;
			fb_i.z += ki_2 * halfez * delta_time;
			g += fb_i; /* apply integral feedback */
		}
		else {
			fb_i = Vector3<float>(); /* prevent integral windup */
		}

		/* Apply proportional feedback */
		g.x += kp_2 * halfex;
		g.y += kp_2 * halfey;
		g.z += kp_2 * halfez;
	}

	/* Integrate rate of change of quaternion */
	g *= 0.5f * delta_time; /* pre-multiply common factors */
	qa = q.w;
	qb = q.x;
	qc = q.y;
	q.w += -qb * g.x - qc * g.y - q.z * g.z;
	q.x += qa * g.x + qc * g.z - q.z * g.y;
	q.y += qa * g.y - qb * g.z + q.z * g.x;
	q.z += qa * g.z + qb * g.y - qc * g.x;

	/* Normalize quaternion */
	quick::normalize(q);
}

void se_mahony_ahrs_update_imu(Vector3<float> g, Vector3<float> a, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q) {
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	/*
	*Compute feedback only if accelerometer measurement valid
	*(avoids NaN in accelerometer normalisation)
	*/
	if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {
		/* Normalize accelerometer measurement */
		quick::normalize(a);

		/*
		* Estimated direction of gravity and vector perpendicular to magnetic flux
		*/
		halfvx = q.x * q.z - q.w * q.y;
		halfvy = q.w * q.x + q.y * q.z;
		halfvz = q.w * q.w - 0.5f + q.z * q.z;

		/*
		* Error is sum of cross product between estimated and measured direction
		* of gravity
		*/
		halfex = (a.y * halfvz - a.z * halfvy);
		halfey = (a.z * halfvx - a.x * halfvz);
		halfez = (a.x * halfvy - a.y * halfvx);

		/* Compute and apply integral feedback if enabled */
		if (ki_2 > 0.0f) {
			/* integral error scaled by Ki */
			fb_i.x += ki_2 * halfex * delta_time;
			fb_i.y += ki_2 * halfey * delta_time;
			fb_i.z += ki_2 * halfez * delta_time;
			/* apply integral feedback */
			g += fb_i;
		}
		else {
			/* prevent integral windup */
			fb_i.x = 0.0f;
			fb_i.y = 0.0f;
			fb_i.z = 0.0f;
		}

		/* Apply proportional feedback */
		g.x += kp_2 * halfex;
		g.y += kp_2 * halfey;
		g.z += kp_2 * halfez;
	}

	/* Integrate rate of change of quaternion */
	/* pre-multiply common factors */
	g *= 0.5f * delta_time;
	qa = q.w;
	qb = q.x;
	qc = q.y;
	q.w += -qb * g.x - qc * g.y - q.z * g.z;
	q.x += qa * g.x + qc * g.z - q.z * g.y;
	q.y += qa * g.y - qb * g.z + q.z * g.x;
	q.z += qa * g.z + qb * g.y - qc * g.x;

	// Normalize quaternion
	quick::normalize(q);
}

// Class State Estimation Functions
// ========================================================================================================
// ========================================================================================================
// ========================================================================================================

Ahrs::Ahrs() {
	// Constants
	g = 9.81;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			I3(i, j) = 0.0;
			if (i == j) {
				I3(i, j) = 1.0;
			}
		}
	}
	// Initialize State
	x_hat.x = 0.0;
	x_hat.y = 0.0;
	x_hat.z = 0.0;

	// Initialize Covariance
	P = I3;
	P(0, 0) = PI / 2 / 3;
	P(1, 1) = PI / 2 / 3;
	P(2, 2) = PI / 3;

	// Initialize Measurement Noise Covariance Matrix
	// Initialize Measurement Noise
	sig_acc.x = 0.0; // Set and scale these appropriately
	sig_acc.y = 0.0; // Set and scale these appropriately
	sig_acc.z = 0.0; // Set and scale these appropriately

	R_acc = I3;

	R_acc(0, 0) = sig_acc.x*sig_acc.x;
	R_acc(1, 1) = sig_acc.y*sig_acc.y;
	R_acc(2, 2) = sig_acc.z*sig_acc.z;

	// Initialize Process Noise Covariance Matrix
	// Initialize Process Noise
	sig_gyro.x = 0.0; // Set and scale these appropriately
	sig_gyro.y = 0.0; // Set and scale these appropriately
	sig_gyro.z = 0.0; // Set and scale these appropriately

	Q = I3;
	Q(0, 0) = sig_gyro.x*sig_gyro.x;
	Q(1, 1) = sig_gyro.y*sig_gyro.y;
	Q(2, 2) = sig_gyro.z*sig_gyro.z;

	ZeroMatTemp(0, 0) = 0.0;ZeroMatTemp(0, 1) = 0.0;ZeroMatTemp(0, 2) = 0.0;
	ZeroMatTemp(1, 0) = 0.0;ZeroMatTemp(1, 1) = 0.0;ZeroMatTemp(1, 2) = 0.0;
	ZeroMatTemp(2, 0) = 0.0;ZeroMatTemp(2, 1) = 0.0;ZeroMatTemp(2, 2) = 0.0;
	ZeroVecTemp.isZero();
}

Ahrs::~Ahrs() {

}

void Ahrs::se_euler_ahrs_update_imu(float delta_time, Quaternion<float>& q, Vector3<float> m, int use_mag)
{
	rotateAccelToClassCoords();
	rotateGyroToClassCoords();
	rotateMagToClassCoords();

	// Convert Flybrix RFU Euler Angles to FRD Euler Angles used in class
	// =======================================================================
	Vector3<float> frd_euler;
	frd_euler = rfuTofrdEulerAngles(q.roll(), q.pitch(), q.yaw());
	// Convert Accel measurements to FRD coordinates
	rotateAccelToClassCoords();
	// Convert Gyro measurements to FRD coordinates
	rotateGyroToClassCoords();

	double phi_hat = frd_euler.x;
	double theta_hat = frd_euler.y;
	double psi_hat = frd_euler.z;
	double g_p = gyroscopeRot_.value.x;
	double g_q = gyroscopeRot_.value.y;
	double g_r = gyroscopeRot_.value.z;
	double a_x = accelerometerRot_.value.x;
	double a_y = accelerometerRot_.value.y;
	double a_z = accelerometerRot_.value.z;
	// Perform Kalman Functions
	// ==================================================================


	// Propagate states
	// ================
	Vector3<float> x_dot;
	x_dot.x = 0.0;
	x_dot.y = 0.0;
	x_dot.z = 0.0;

	x_hat = ZeroVecTemp;

	phi_hat = x_hat.x;
	theta_hat = x_hat.y;
	psi_hat = x_hat.z;

	// Compute the jacobian of f
	Matrix<float, 3, 3> A;
	A(0, 0) = 0.0;
	A(0, 1) = 0.0;
	A(0, 2) = 0.0;

	A(1, 0) = 0.0;
	A(1, 1) = 0.0;
	A(1, 2) = 0.0;

	A(2, 0) = 0.0;
	A(2, 1) = 0.0;
	A(2, 2) = 0.0;

	// Propagate Covariance Matrix
	Matrix<float, 3, 3> P_dot;
	P_dot += ZeroMatTemp;
	P_dot += ZeroMatTemp;
	P_dot += ZeroMatTemp;

	P += ZeroMatTemp;

	// Measurement Update
	// ==================

	// ACCELEROMETER

	// Compute the jacobian of h for accel
	Matrix<float, 3, 3> C_acc;
	C_acc(0, 0) = 0.0;
	C_acc(0, 1) = 0.0;
	C_acc(0, 2) = 0.0;

	C_acc(1, 0) = 0.0;
	C_acc(1, 1) = 0.0;
	C_acc(1, 2) = 0.0;

	C_acc(2, 0) = 0.0;
	C_acc(2, 1) = 0.0;
	C_acc(2, 2) = 0.0;

	// Compute the Kalman gain matrix
	Matrix<float, 3, 3> den;
	Matrix<float, 3, 3> L_acc;

	den = ZeroMatTemp;
	den += ZeroMatTemp;
	L_acc = ZeroMatTemp;

	// Compute the predicted measurement
	Vector3<float> y_hat;
	y_hat.x = 0.0;
	y_hat.y = 0.0;
	y_hat.z = 0.0;

	// Compute the updated state
	Vector3<float> resid_acc;
	resid_acc.x = 0.0;
	resid_acc.y = 0.0;
	resid_acc.z = 0.0;

	x_hat.x += 0.0;
	x_hat.y += 0.0;
	x_hat.z += 0.0;

	Matrix<float, 3, 3> P0 = I3;
	P0 -= ZeroMatTemp;
	P = ZeroMatTemp;

	frd_euler.x = x_hat.x;
	frd_euler.y = x_hat.y;
	frd_euler.z = x_hat.z;

	// Convert FRD Euler Angles back to RFU Euler angles
	Vector3<float> rfu_euler;
	rfu_euler = frdTorfuEulerAngles(frd_euler.x, frd_euler.y, frd_euler.z);
	float roll = rfu_euler.x;
	float pitch = rfu_euler.y;
	float yaw = rfu_euler.z;

	DebugPrintf("pitch: %f6 \n", pitch * 180 / PI);
	// Convert Euler angles back to quaternions
	pose_.toQuaternion(roll, pitch, yaw);

	// Normalize quaternion
	quick::normalize(pose_);
}

// Class Support functions
void Ahrs::rotateAccelToClassCoords() {
	accelerometerRot_ = accelerometer_;
	accelerometerRot_.value.x = accelerometer_.value.y;
	accelerometerRot_.value.y = accelerometer_.value.x;
	accelerometerRot_.value.z = -accelerometer_.value.z;
}

void Ahrs::rotateGyroToClassCoords() {
	gyroscopeRot_ = gyroscope_;
	gyroscopeRot_.value.x = gyroscope_.value.y;
	gyroscopeRot_.value.y = gyroscope_.value.x;
	gyroscopeRot_.value.z = -gyroscope_.value.z;
}

void Ahrs::rotateMagToClassCoords() {
	magnetometerRot_ = magnetometer_;
	magnetometerRot_.value.x = magnetometer_.value.y;
	magnetometerRot_.value.y = magnetometer_.value.x;
	magnetometerRot_.value.z = -magnetometer_.value.z;
}

Vector3<float> Ahrs::frdTorfuEulerAngles(float roll, float pitch, float yaw) {
	// Convert Yaw-Pitch-Roll Forward-Right-Down to NED euler angles to 
	// Roll-Yaw-Pitch Right-Forward-Up to ENU euler angles
	float phi_rfu, theta_rfu, psi_rfu;

	phi_rfu = asin(cos(pitch)*sin(roll));
	theta_rfu = atan2(sin(pitch), cos(pitch)*cos(roll));
	psi_rfu = atan2(cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw), cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw));

	Vector3<float> euler_out;
	euler_out.x = phi_rfu;
	euler_out.y = theta_rfu;
	euler_out.z = psi_rfu;

	return euler_out;
}

Vector3<float> Ahrs::rfuTofrdEulerAngles(float roll, float pitch, float yaw) {
	// Convert Roll-Yaw-Pitch Right-Forward-Up to ENU euler angles to
	// Yaw-Pitch-Roll Forward-Right-Down to NED euler angles

	float phi_frd, theta_frd, psi_frd;

	phi_frd = atan2(sin(roll), cos(pitch)*cos(roll));
	theta_frd = asin(cos(roll)*sin(pitch));
	psi_frd = atan2(cos(yaw)*sin(pitch)*sin(roll) - cos(pitch)*sin(yaw), cos(pitch)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw));


	Vector3<float> euler_out;
	euler_out.x = phi_frd;
	euler_out.y = theta_frd;
	euler_out.z = psi_frd;

	return euler_out;
}

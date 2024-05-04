/*
*  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
*
*  http://www.flybrix.com
*/

#ifndef SE_AHRS_H_
#define SE_AHRS_H_

#include <cstdint>
#include "utility/clock.h"
#include "utility/quaternion.h"
#include "utility/vector3.h"
#include "utility/linalg.h"

class Ahrs final {
public:
	Ahrs();
	~Ahrs();

	enum class Type {
		Madgwick = 0,
		Mahony = 1,
		UAV_Class = 2,
	};
	Ahrs& setType(Type type) {
		type_ = type;
		return *this;
	}
	Ahrs& setParameters(float p1, float p2) {
		parameter_1_ = p1;
		parameter_2_ = p2;
		return *this;
	}
	Ahrs& setMaxDeltaTime(float mdt) {
		max_delta_time_ = mdt;
		return *this;
	}
	Ahrs& setTimestamp(ClockTime time) {
		last_update_timestamp_ = time;
		return *this;
	}
	Ahrs& setAccelerometer(float x, float y, float z) {
		return setAccelerometer(Vector3<float>(x, y, z));
	}
	Ahrs& setAccelerometer(const Vector3<float>& v) {
		accelerometer_.value = v;
		accelerometer_.ready = true;
		return *this;
	}
	Ahrs& setGyroscope(float x, float y, float z) {
		return setGyroscope(Vector3<float>(x, y, z));
	}
	Ahrs& setGyroscope(const Vector3<float>& v) {
		gyroscope_.value = v;
		gyroscope_.ready = true;
		return *this;
	}
	Ahrs& setMagnetometer(float x, float y, float z) {
		return setMagnetometer(Vector3<float>(x, y, z));
	}
	Ahrs& setMagnetometer(const Vector3<float>& v) {
		if (v.x != 0.0f || v.y != 0.0f || v.z != 0.0f) {
			magnetometer_.value = v;
			magnetometer_.ready = true;
		}
		return *this;
	}
	Quaternion<float>& pose() {
		return pose_;
	}

	const Type& ahrsType() const {
		return type_;
	}

	Vector3<float> gravity() {
		return Vector3<float>(2.0f * (pose_.x * pose_.z - pose_.w * pose_.y), 2.0f * (pose_.y * pose_.z + pose_.w * pose_.x), 1.0f - 2.0f * (pose_.x * pose_.x + pose_.y * pose_.y));
	}

	const Quaternion<float>& pose() const {
		return pose_;
	}

	Vector3<float> frdTorfuEulerAngles(float roll, float pitch, float yaw);
	Vector3<float> rfuTofrdEulerAngles(float roll, float pitch, float yaw);

	void update(ClockTime timestamp);


private:
	struct Measurement {
		bool ready{ false };
		Vector3<float> value;
		void consume() {
			ready = false;
		}
	};

	float max_delta_time_{ 0 };
	float parameter_1_{ 0 };
	float parameter_2_{ 0 };
	ClockTime last_update_timestamp_{ ClockTime::zero() };

	// Class Support Functions & Variables
	// =========================================
	void rotateAccelToClassCoords();
	void rotateGyroToClassCoords();
	void rotateMagToClassCoords();


	Quaternion<float> pose_;
	Vector3<float> integral_feedback_;
	Type type_{ Type::Madgwick };
	Measurement accelerometer_;
	Measurement gyroscope_;
	Measurement magnetometer_;

	Measurement accelerometerRot_;
	Measurement gyroscopeRot_;
	Measurement magnetometerRot_;

	// Class Kalman Filter Variables
	// ====================================
	void se_euler_ahrs_update_imu(float delta_time, Quaternion<float>& q, Vector3<float> m, int use_mag);
	Vector3<float> x_hat;
	Matrix<float, 3, 3> P;
	Matrix<float, 3, 3> Q;
	Matrix<float, 3, 3> R_acc;
	Matrix<float, 3, 3> R_mag;

	Matrix<float, 3, 3> Rx_phi;
	Matrix<float, 3, 3> Ry_theta;
	Matrix<float, 3, 3> Rz_psi;

	Vector3<float> sig_mag;
	Vector3<float> sig_acc;
	Vector3<float> sig_gyro;

	// Constants
	Matrix<float, 3, 3> I3;
	float g;
	float delta_inc;
	float delta_dec;

	Matrix<float, 3, 3> ZeroMatTemp;
	Vector3<float> ZeroVecTemp;




};

#endif /* end of include guard: SE_AHRS_H_ */

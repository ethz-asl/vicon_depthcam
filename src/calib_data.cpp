/*
 * simple class to hold the data from the calibration
 */


#include <Eigen/Dense>

class Stored_Transformation {
public:
	Eigen::Matrix<double, 3, 1> getTranslation() {
		return translation;
	}
	Eigen::Quaternion<double> getQuaternion() {
		return quaternion;
	}

	Stored_Transformation() {

		location = 2;
		if (location == 1) //in the LEE J-Floor
				{
			translation << -0.017, -0.011, 0.032;
			0.713859, temp_rot_matrix << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
			//quaternion=Eigen::Quaterniond(-0.714343,0.049333,0.025629,-0.697584); //w,x,y,z
			quaternion = Eigen::Quaterniond(0.713566, -0.0500134, 0.025629,
					0.698194); //w,x,y,z
			quaternion.normalize();
			ROS_INFO("stored quaternion");
			ROS_INFO_STREAM(quaternion.coeffs());
		}

		else if (location == 2) {
			translation << -0.0316981, 0.0303474, -0.0558053;

			//from pos1
			//quaternion=Eigen::Quaterniond(-0.135559,0.523541,-0.819119,0.191239); //w,x,y,z

			//from moving chessboard
			quaternion = Eigen::Quaterniond(0.136118, -0.525403, 0.817856,
					-0.191143); //w,x,y,z
			quaternion.normalize();
			ROS_INFO("stored quaternion");
			ROS_INFO_STREAM(quaternion.coeffs());
		}

	}
private:
	int location;
	Eigen::Matrix<double, 3, 1> translation;
	Eigen::Matrix<double, 3, 3> temp_rot_matrix;
	Eigen::Quaternion<double> quaternion;

};

//old calibration data from LEE J-Floor via rotation matrix

//from calib2
/*
 temp_translation << -0.01404, -0.00931766, 0.0313022;

 temp_rot_matrix << 	0.0197269,  -0.994185,  -0.104248,
 0.999355,   0.0168538,   0.0277549,
 -0.02586,   -0.104753,   0.993891;
 */

//calib centered
/*
 temp_translation << -0.020179, -0.00554317, 0.0314023;

 temp_rot_matrix << 	 0.0181471, -0.994698,   -0.10054,
 0.999459,   0.0157294,   0.0248558,
 -0.0231407, -0.100965,    0.994458;
 */

//static
/*
 temp_translation << -0.017, -0.011, 0.032;
 temp_rot_matrix << 	 0.024, -0.994,   -0.108,
 0.999,   0.0205,   0.325,
 0.0305, -0.109,    0.994;
 */

//quaternion=Eigen::Quaterniond(temp_rot_matrix);


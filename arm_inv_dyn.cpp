#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>
#include <pseudo_inversion.h>
#include <MovingMedianFilter.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Wrench.h>
#include <kdl/chainjnttojacdotsolver.hpp>

using namespace KDL;
using namespace std;

bool VERBOSE =  false;

#define MAX_STIFF 0.3
// #define arm_cubes_n 6
// #define use_addon true
#define K_I 2.8531 // constant to compute torque from current Nm/A

Frame 				ref_frame_copy;
Frame 				ref_frame;
Eigen::VectorXd		ref_twist(6);
double				stiffn;
int 				arm_side;
int 				arm_cubes_n ;
int 				AlterEgoVersion;

std::string 		ns;
Eigen::Matrix4d		T_b_0;
Eigen::Matrix3d 	R_offset;
Eigen::Matrix3d 	R_p_2_r;
Eigen::Matrix3d		R_o_b;
double 				hand_cl;
ros::Time 			cmd_time, cmd_time_old;
double 				arm_l(0.432);
double 				shoulder_l(0.2);
KDL::JntArray 		meas_cube_m1;
KDL::JntArray 		meas_cube_m2;
KDL::JntArray 		meas_cube_shaft;
double 				meas_shaft_addon_init(0.0);
KDL::JntArray meas_cube_shaft_grav;
string 				side;

bool arm_cb = false;
bool powerbooster = false;
bool use_addon = false;

/*---------------------------------------------------------------------*
* POSTURE CALLBACK                                                     *
*                                                                      *
*----------------------------------------------------------------------*/
void posture__Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) //Questo tipo di messaggio contiene informazioni sulla posizione e sull'orientamento di un oggetto nello spazio tridimensionale.
{
	Eigen::Quaterniond	ref_quat;
	static Eigen::Quaterniond	old_quat;
	double sign_check;
	Eigen::Vector3d		r_p; //posizione di riferimento
	Eigen::Matrix3d		r_M; //conterrà il valore di riferimento del quaternione
	static tf::TransformBroadcaster br;
  	tf::Transform transform;

	r_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
	r_p = r_p*arm_l;
	r_p.y() += shoulder_l;
	ref_quat.x() = msg->pose.orientation.x;
	ref_quat.y() = msg->pose.orientation.y;
	ref_quat.z() = msg->pose.orientation.z;
	ref_quat.w() = msg->pose.orientation.w;

	sign_check = ref_quat.w() * old_quat.w() + ref_quat.x() * old_quat.x() + ref_quat.y() * old_quat.y() + ref_quat.z() * old_quat.z();
	if(sign_check < 0.0){
		ref_quat.w() = -ref_quat.w(); 
		ref_quat.vec() = -ref_quat.vec(); 
	}
	old_quat = ref_quat;
	r_M = ref_quat;

	r_M =  r_M * R_p_2_r;
	
	// Viene creata una variabile ref_frame di tipo KDL::Frame utilizzando la matrice di rotazione r_M e il vettore di posizione r_p
	ref_frame = Frame( Rotation(r_M(0,0), r_M(0,1), r_M(0,2), r_M(1,0), r_M(1,1), r_M(1,2), r_M(2,0), r_M(2,1), r_M(2,2)), Vector(r_p(0), r_p(1), r_p(2)));
	cmd_time_old = cmd_time;
	cmd_time = ros::Time::now();
}

/*---------------------------------------------------------------------*
* STIFFNESS CALLBACK                                                   *
*                                                                      *
*----------------------------------------------------------------------*/
void arm_stiffness__Callback(const std_msgs::Float64::ConstPtr& msg)
{
	stiffn = msg->data*MAX_STIFF;

	if (stiffn > MAX_STIFF)
		stiffn = MAX_STIFF;

	if (stiffn < 0)
		stiffn = 0;
}


void cubes_m1__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for(int i = 0; i < msg->data.size(); i++) 
	{
		meas_cube_m1(i) = msg->data[i];
	}


	if (AlterEgoVersion == 3)
	{

		meas_cube_m1(0) = -meas_cube_m1(0); // to control for the left one
		meas_cube_m1(1) = meas_cube_m1(1) / 2;
	}
}

void cubes_m2__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for(int i = 0; i < msg->data.size(); i++) 
	{
		meas_cube_m2(i) = msg->data[i];
	}


	if (AlterEgoVersion == 3)
	{

		meas_cube_m2(0) = -meas_cube_m2(0);
		meas_cube_m2(1) = meas_cube_m2(1) / 2;
	}
}

void cubes_shaft__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for(int i = 0; i < msg->data.size(); i++) 
	{
		meas_cube_shaft(i) = msg->data[i];
	}
	if (AlterEgoVersion==3)
	{

		meas_cube_shaft(1) = (meas_cube_shaft(1)) / 2.086;

		if (!arm_cb)
			meas_shaft_addon_init = meas_cube_shaft(1);

		meas_cube_shaft(1) = (meas_cube_shaft(1) - meas_shaft_addon_init); // 19° -> 0.33rad flange offset w.r.t. cube zero position

	}
	arm_cb = true;
}

void powerbooster__Callback(const std_msgs::Bool::ConstPtr &msg)
{

	powerbooster = msg->data;
}

void hand_closure__Callback(const std_msgs::Float64::ConstPtr& msg)
{
	hand_cl = msg->data;
}

int sgn(double d)
{
    return d<0? -1 : d>0; 
}

//MAIN
int main(int argc, char **argv)
{
	// ------------------------------------------------------------------------------------- Init node
  	ros::init(argc, argv, "arm_inv_dyn");
  	ros::NodeHandle n;
	ns = ros::this_node::getNamespace();

	if(ns.find("left") != std::string::npos){
		side = ns.substr(ns.find("left"));
	}
	else if(ns.find("right") != std::string::npos){
		side = ns.substr(ns.find("right"));
	}

	std::string robot_name = std::getenv("ROBOT_NAME");
	n.getParam("/"+robot_name+"/arm_cubes_n", arm_cubes_n);

// ------------------------------------------------------------------------------------- Init Var
	// --- node param ---
	string 				chain_topic;
	string 				pose_ref_topic;
	string 				stiff_ref_topic;
	string 				hand_cl_topic;
	string 				phantom_arm_topic;
	string 				ref_eq_arm_topic;
	string 				ref_pr_arm_topic;
	string 				ref_hand_topic;
	string 				cubes_m1_topic;
	string 				cubes_m2_topic;
	string 				cubes_shaft_topic;
	string 				power_booster_topic;
	
	string 				ready_pilot_topic;
	
  	ros::Subscriber 	sub_posture;
  	ros::Subscriber 	sub_stiffness;
  	ros::Subscriber 	sub_hand_cl;
  	ros::Subscriber 	sub_cubes_m1;
  	ros::Subscriber 	sub_cubes_m2;
  	ros::Subscriber 	sub_cubes_shaft;
	ros::Subscriber 	sub_powerbooster;

	
	ros::Publisher    	pub_cart_ref;
	ros::Publisher    	pub_phantom;
	ros::Publisher    	pub_ref_eq_arm_eq;
	ros::Publisher    	pub_ref_pr_arm_eq;
	ros::Publisher    	pub_ref_hand_eq;
	ros::Publisher    	pub_wrench;
	ros::Publisher 		ready_for_pilot;

	geometry_msgs::Pose 				cart_ref_msg;
	sensor_msgs::JointState 			phantom_msg;
	std_msgs::Float64MultiArray 		arm_eq_ref_msg;
	std_msgs::Float64MultiArray 		arm_pr_ref_msg;
	std_msgs::Float64 					hand_ref_msg;
	std_msgs::Float64 					weight;
	unsigned long int 					msg_seq(0);

	char buffer [50];

	// --- kinematics ---
	std::vector<double>		R_p2r;
	std::vector<double>		T_h2fwk;
	std::vector<double>		DH;
	std::vector<double>		DH_Xtr;
	std::vector<double>		DH_Xrot;
	std::vector<double>		DH_Ztr;
	std::vector<double>		stiffn_vec(arm_cubes_n);
	std::vector<double> 	stiffn_vec_no_power;
	std::vector<double> 	stiffn_vec_power;
	std::vector<double>		DH_Zrot;
	std::vector<double>		T_t2s;
	std::vector<double> 	R_5to6;
	std::vector<double>		T_o2t;
	std::vector<double>		R_o2b;
	std::vector<double>		q_min;
	std::vector<double>		q_max;
	std::vector<int>		qbmove_tf_ids;
	int						softhand_tf_id;
	KDL::Chain 				chain;
	KDL::JntSpaceInertiaMatrix	B;
	KDL::JntArray				C;	
	boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver; //Questo oggetto è un risolutore cinematico diretto per la posizione della catena cinematica del robot.
	//Questo risolutore può essere utilizzato per calcolare la posizione e l'orientamento dell'end-effector per qualsiasi parte del robot o del sistema senza essere limitato 
	//a un componente specifico come il "shaft".
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver; // Questo oggetto è un risolutore che calcola il Jacobiano per la catena cinematica del robot.
	boost::scoped_ptr<KDL::ChainDynParam> 		dyn_solver;
	boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver;

	boost::scoped_ptr<KDL::ChainFkSolverPos> shaft_to_pose_solver;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> shaft_to_jac_solver;


	Eigen::VectorXd qMax_left(arm_cubes_n);
	Eigen::VectorXd qMin_left(arm_cubes_n);
	Eigen::VectorXd qMax_right(arm_cubes_n);
	Eigen::VectorXd qMin_right(arm_cubes_n);

	KDL::JntArray  		q;
	KDL::JntArray		q_dot;
	KDL::JntArray		q_ddot;
	KDL::JntArrayVel 	Vel( q, q_dot); 	
	KDL::Jacobian  		JA_kdl;
	KDL::Jacobian  		J_wrench_kdl;
	KDL::Frame 			act_frame, frame_grav, shaft_frame;
	KDL::Vector     		error;
	Eigen::VectorXd		err_post(6);
	Eigen::VectorXd		x_post(6);
	Eigen::MatrixXd 	JA(6,arm_cubes_n);
	Eigen::MatrixXd 	JA_pinv(arm_cubes_n,6);
	Eigen::MatrixXd 	P(arm_cubes_n,6);
	KDL::Jacobian 		jacobian_dot(chain.getNrOfJoints());

	
	Eigen::VectorXd 	eq_dot(arm_cubes_n);
	Eigen::VectorXd 	theta_s(arm_cubes_n);
	Eigen::VectorXd 	theta_d_dot(arm_cubes_n);
	Eigen::VectorXd 	theta_d(arm_cubes_n);
	Eigen::VectorXd 	eq_dot_0(arm_cubes_n);
	Eigen::VectorXd 	eq(arm_cubes_n);
	Eigen::VectorXd 	q_eigen_f(arm_cubes_n);
	Eigen::VectorXd 	q_preset(arm_cubes_n);
	Eigen::VectorXd 	q_send(arm_cubes_n);
	Eigen::VectorXd 	qd(arm_cubes_n);
	Eigen::VectorXd 	qd_dot(arm_cubes_n);
	Eigen::VectorXd 	qd_ddot(arm_cubes_n);
	Eigen::VectorXd 	e(arm_cubes_n);
	Eigen::VectorXd 	e_dot(arm_cubes_n);
	Eigen::VectorXd 	e_ddot(arm_cubes_n);
	Eigen::VectorXd 	tau_computed(arm_cubes_n);
	Eigen::VectorXd 	tau(arm_cubes_n);
	Eigen::VectorXd 	tau_elastica(arm_cubes_n);
	Eigen::VectorXd 	d_tau(arm_cubes_n);
	Eigen::VectorXd 	v(arm_cubes_n);
	Eigen::VectorXd 	l(arm_cubes_n);
	Eigen::VectorXd 	h(arm_cubes_n);
	Eigen::DiagonalMatrix<double,Eigen::Dynamic> L(6);

	Eigen::Quaterniond	act_quat;
	Eigen::Quaterniond ref_shaft_quat, act_shaft_quat, first_cube_quat;

	Vector 				gravity_v;
	double 				cube_m; //massa del cubo
	double 				cube_wrist_m(0.713);
	double 				cube_m_addon(0.65);
	double 				hand_m(0.5);
	RigidBodyInertia 	inert_Q_0, inert_Q_1, inert_Q_2, inert_Q_3, inert_Q_4, inert_Q_5, inert_Q_6;
	KDL::JntArray 		G;
	KDL::JntArray 		g_wrench_comp;
	double				a_mot, k_mot;

    ros::Duration 		max_cmd_time = ros::Duration(1);
    ros::Duration 		filt_time = ros::Duration(5);
    ros::Duration 		max_cmd_latency = ros::Duration(1);
	ros::Time 			start_raise;
    ros::Time 			start_f_time;
    int 				act_bp(1);
	bool 				flag_pilot_out_ = true;
    double 				alpha(1);

    Eigen::MatrixXd Jac_wrench, Jac_trans_pinv_wrench;
    Eigen::VectorXd Grav_wrench;
    Eigen::VectorXd tau_meas;
    Eigen::VectorXd wrench(6); //Vettore di forze o coppie (solitamente a 6 gradi di libertà).
    geometry_msgs::Wrench wrench_msg; //Un messaggio ROS per rappresentare le forze o le coppie.





	// --- Rviz ---
	tf::TransformBroadcaster ik_br, ik_ref, ik_shaft; // è una classe in ROS che consente di trasmettere trasformazioni TF tra diversi frame di riferimento
	tf::Transform ik_tf, ik_tf_ref, ik_tf_shaft;
	

	// ------------------------------------------------------------------------------------- Check/save args
	n.getParam("arm_l", arm_l); //0.532
	n.getParam("shoulder_l", shoulder_l); //0.26 Left ; -0.26 Right
	n.getParam("DH_table", DH); //[0, -1.57, 0.110, 0, 0, -1.57, 0, 1.57, 0, -1.57, 0.213, 1.57, 0, 1.57, 0, 0, 0.012, -1.57, 0.231, 1.57, -0.06, 0, 0, 1.57] a valori di 4
	n.getParam("DH_Xtr", DH_Xtr);//[0, 0, 0, 0, -0.012, -0.088] 
	n.getParam("DH_Xrot", DH_Xrot); //[-1.57, -1.57, -1.57, 1.57, -1.57, 0]
	n.getParam("DH_Ztr", DH_Ztr); //[0.1528, 0, 0.213, 0, 0.231, 0.032]
	n.getParam("DH_Zrot", DH_Zrot); //[0, 1.57, 1.57, 0, 1.57, 1.57]
	n.getParam("T_t2s", T_t2s); //[0, -0.1736, 0.9848, 0.008, 0.9848, -0.1710,-0.0301, 0.108, 0.1736, 0.9698, 0.1710, 0.009, 0, 0, 0, 1] #Torso to Shoulder ( fixed transformation )
	//trasformazione tra torso e spalla fissata
	n.getParam("R_p2r", R_p2r); //[0, 0, -1, 0, 1, 0, 1, 0, 0] 
	n.getParam("T_o2t", T_o2t); // [0, 0, 1, 0.034, 0, -1, 0, 0.2561, 1, 0, 0, 0.035, 0, 0, 0, 1]  
	// #oculus to Torso ( fixed transformation : transform the commanded position from the oculus accordingly with the distance of the shoulder wrt the torso frame)
	n.getParam("R_o2b", R_o2b); //[-0.1736, 0, -0.9848,-0.9698,-0.1736, 0.1710, -0.1710, 0.9848, 0.0301]
	n.getParam("q_min", q_min); //q_min: [-2.3, -0.45, -1.57, -2.09, -1.57, -1.53]
	n.getParam("q_max", q_max); //q_max: [0.4, 1.37, 1.57, 0, 1.57, 1.3]
	n.getParam("qbmove_tf_ids", qbmove_tf_ids); //[1, 2, 3, 4, 5, 6]
	n.getParam("softhand_tf_id", softhand_tf_id); //7
	n.getParam("pose_ref_topic", pose_ref_topic); //"hand_pos"
	n.getParam("stiff_ref_topic", stiff_ref_topic); //"stiff_ref_pose"
	n.getParam("phantom_arm_topic", phantom_arm_topic); //"phantom_left/ego_arm_left_phantom/control/joint_states"
	n.getParam("ref_eq_arm_topic", ref_eq_arm_topic); //"ref_cubes_eq"
	n.getParam("ref_pr_arm_topic", ref_pr_arm_topic); //"ref_cubes_preset"
	n.getParam("ref_hand_topic", ref_hand_topic); //"ref_hand"
	n.getParam("cubes_m1_topic", cubes_m1_topic); //"meas_arm_m1"
	n.getParam("cubes_m2_topic", cubes_m2_topic); //"meas_arm_m2"
	n.getParam("cubes_shaft_topic", cubes_shaft_topic); //"meas_arm_shaft"
	n.getParam("hand_cl_topic", hand_cl_topic); //"ind"
	n.getParam("cube_mass", cube_m); //0.5
	n.getParam("cube_wrist_mass", cube_wrist_m); //0.713
	n.getParam("cube_addon_mass", cube_m_addon); //0.650
	n.getParam("hand_mass", hand_m); //0.5
	n.getParam("T_h2fwk", T_h2fwk); //[0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1] trasformazione sistema di riferimento mano e 'end-effector' 
	n.getParam("a_mot", a_mot); //11
	n.getParam("k_mot", k_mot); //0.02
	n.getParam("stiffness_vec", stiffn_vec); //[0.0, 0.0, 0.1, 0.1, 0.1, 0.1]
	n.getParam("ready_pilot_topic", ready_pilot_topic); //"ready_for_pilot"
	n.getParam("active_back_pos", act_bp); //1
	n.getParam("stiffness_vec", stiffn_vec_no_power); //[0.0, 0.0, 0.1, 0.1, 0.1, 0.1]
	n.getParam("stiffness_vec_power", stiffn_vec_power); //[0.0, 0.81, 0.8, 0.6, 0.6, 0.4]
	n.getParam("R_5to6", R_5to6); //[0, 0, 1, 0, 1, 0, -1, 0, 0]
	n.getParam("/"+robot_name+"/power_booster_topic", power_booster_topic); // /AlterEgo_sim/power_booster
	n.getParam("/"+robot_name+"/AlterEgoVersion", AlterEgoVersion); // /AlterEgo_sim/3
	n.getParam("/"+robot_name+"/use_addon", use_addon); // /AlterEgo_sim/true

	int run_freq = 50;
	n.getParam("/"+robot_name+"/arms_frequency", run_freq);	// Override if configured /AlterEgo_sim/50
	ros::Rate loop_rate(run_freq);

	Eigen::MatrixXd		K_v(6,6);
	Eigen::MatrixXd		K_d(6,6);
	Eigen::MatrixXd		K_c(6,6);
	Eigen::MatrixXd		K_p(6,6); //guadagno utilizzato nel computed torque
	Eigen::MatrixXd		K_0(arm_cubes_n,arm_cubes_n); //6x6

	K_p <<  2*Eigen::MatrixXd::Identity(6,6); //matrice di guadagni Kd = 0.1 di dimensione 6x6
	K_c <<  10*Eigen::MatrixXd::Identity(6,6); //matrice di guadagni Kc
	P <<  10*Eigen::MatrixXd::Identity(6,6); //matrice di guadagni 
	K_0 <<  Eigen::MatrixXd::Zero(arm_cubes_n,arm_cubes_n); //matrice K0 nulla di dimensione 6x6
	K_d <<  0.1*Eigen::MatrixXd::Identity(6,6);
	theta_s << Eigen::VectorXd::Zero(6,1);
	theta_d_dot = 0.1*theta_d_dot.setOnes();
	theta_d += theta_d_dot/run_freq;
	Eigen::MatrixXd W(6, 6); 
	W << Eigen::MatrixXd::Identity(6, 6); //dichiarazione e inizializzazione matrice W identità di dimensione 6x6

	
	if(AlterEgoVersion==2){
		
		K_v << 1500/run_freq*Eigen::MatrixXd::Identity(6,6);
		K_0(2,2) = 0.1;
		K_0(4,4) = 0.1;
	}
	else if(AlterEgoVersion == 3){
		K_v << 12 * Eigen::MatrixXd::Identity(6, 6);
		K_0(2, 2) = 0.3;
		K_0(4, 4) = 0.3;
		
		// W(0, 0) = 10;

		// if (ns.find("left") != std::string::npos)
		// {
		// 	W(1, 1) = 10;
		// }
	}

	if(AlterEgoVersion == 2){
		inert_Q_0 = KDL::RigidBodyInertia(0, KDL::Vector(0.0, 0.0, 0.0)); 					
		inert_Q_1 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.0)); 				
		inert_Q_2 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, (0.09 - 0.007)));
		inert_Q_3 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.0));
		inert_Q_4 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, (0.09 - 0.013)));
		inert_Q_5 = KDL::RigidBodyInertia(0.35, KDL::Vector(0.0, 0.0, 0.0));				


		chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_t2s[0],T_t2s[4],T_t2s[8],T_t2s[1],T_t2s[5],T_t2s[9],T_t2s[2],T_t2s[6],T_t2s[10]),
		Vector(T_t2s[3],T_t2s[7],T_t2s[11]))));
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr[0], DH_Xrot[0], DH_Ztr[0], DH_Zrot[0]),inert_Q_1));
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr[1], DH_Xrot[1], DH_Ztr[1], DH_Zrot[1]),inert_Q_2));
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr[2], DH_Xrot[2], DH_Ztr[2], DH_Zrot[2]),inert_Q_3));
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr[3], DH_Xrot[3], DH_Ztr[3], DH_Zrot[3]),inert_Q_4));
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr[4], DH_Xrot[4], DH_Ztr[4], DH_Zrot[4]),inert_Q_5));
		chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_h2fwk[0],T_h2fwk[4],T_h2fwk[8],T_h2fwk[1],T_h2fwk[5],T_h2fwk[9],T_h2fwk[2],T_h2fwk[6],T_h2fwk[10]),
		Vector(T_h2fwk[3],T_h2fwk[7],T_h2fwk[11]))));

	}
	else if(AlterEgoVersion == 3){
		if(VERBOSE)std::cout<<"----------[VERBOSE]---------- Kinematics: Version "<<AlterEgoVersion<<std::endl;
		inert_Q_0 = KDL::RigidBodyInertia(0, KDL::Vector(0.0, 0.0, 0.0));			 
		inert_Q_1 = KDL::RigidBodyInertia(cube_m_addon, KDL::Vector(0.0, 0.0, 0.0)); 
		inert_Q_2 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
		inert_Q_3 = KDL::RigidBodyInertia(cube_m + 0.1, KDL::Vector(0.0, 0.0, 0.0));
		inert_Q_4 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
		inert_Q_5 = KDL::RigidBodyInertia(cube_wrist_m, KDL::Vector(0.0, 0.0, 0.0)); 
		inert_Q_6 = KDL::RigidBodyInertia(hand_m, KDL::Vector(0.0, 0.0, 0.0));		 

		chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_t2s[0], T_t2s[4], T_t2s[8], T_t2s[1], T_t2s[5], T_t2s[9], T_t2s[2], T_t2s[6], T_t2s[10]),
														Vector(T_t2s[3], T_t2s[7], T_t2s[11]))));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[0], DH_Xrot[0], DH_Ztr[0], DH_Zrot[0]), inert_Q_1));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[1], DH_Xrot[1], DH_Ztr[1], DH_Zrot[1]), inert_Q_2));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[2], DH_Xrot[2], DH_Ztr[2], DH_Zrot[2]), inert_Q_3));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[3], DH_Xrot[3], DH_Ztr[3], DH_Zrot[3]), inert_Q_4));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[4], DH_Xrot[4], DH_Ztr[4], DH_Zrot[4]), inert_Q_5));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[5], DH_Xrot[5], DH_Ztr[5], DH_Zrot[5]), inert_Q_6));
		chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_h2fwk[0],T_h2fwk[4],T_h2fwk[8],T_h2fwk[1],T_h2fwk[5],T_h2fwk[9],T_h2fwk[2],T_h2fwk[6],T_h2fwk[10]),
		Vector(T_h2fwk[3],T_h2fwk[7],T_h2fwk[11]))));
	}


	G.resize(chain.getNrOfJoints());
	KDL::SetToZero(G);
	g_wrench_comp.resize(chain.getNrOfJoints());

	tau_meas.resize(chain.getNrOfJoints());
	Jac_wrench.resize(6, chain.getNrOfJoints());
	Jac_trans_pinv_wrench.resize(6, chain.getNrOfJoints());

	gravity_v = KDL::Vector(0,0,-9.81);

	jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
	jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
	shaft_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
	shaft_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
	dyn_solver.reset(new KDL::ChainDynParam(chain,gravity_v));
	jac_dot_solver.reset(new KDL::ChainJntToJacDotSolver(chain));
	
	q.resize(chain.getNrOfJoints());
	q_dot.resize(chain.getNrOfJoints());
	q_ddot.resize(chain.getNrOfJoints());
	JA_kdl.resize(chain.getNrOfJoints());
	KDL::SetToZero(q);
	KDL::SetToZero(q_dot);
	KDL::SetToZero(q_ddot);

	meas_cube_m1.resize(chain.getNrOfJoints());
	meas_cube_m2.resize(chain.getNrOfJoints());
	meas_cube_shaft.resize(chain.getNrOfJoints());
	meas_cube_shaft_grav.resize(chain.getNrOfJoints());
	J_wrench_kdl.resize(chain.getNrOfJoints());
	Grav_wrench.resize(chain.getNrOfJoints());
	double defl_max = 0.6;

	shaft_to_pose_solver->JntToCart(meas_cube_shaft, shaft_frame); // La funzione JntToCart prende in ingresso le posizioni dei giunti (meas_cube_shaft) 
	//e calcola la pose dell'end-effector associato allo "shaft".

	//// Dando in ingresso q, si ottiene la posizione dell'end effector in un oggetto di tipo "Frame" da cui poi si estrapola il vettore di traslazione
	jnt_to_pose_solver->JntToCart(q, ref_frame); 
	KDL::Vector	pos_end_effector = ref_frame.p; //postura dell'e-e desiderata (braccio in posizione di equilibrio)

	//A partire da q viene restituito un oggetto di tipo "Jacobian" 6x6
	jnt_to_jac_solver->JntToJac(q, JA_kdl); 

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < arm_cubes_n; j++)
			if (fabs(JA_kdl(i, j)) > 0.000001)
				JA(i, j) = JA_kdl(i, j);
			else
				JA(i, j) = 0;
	}

	if (AlterEgoVersion == 2){
		JA_pinv 	= JA.transpose()*((JA*JA.transpose() +K_d).inverse());
	}
	else if(AlterEgoVersion == 3){
		JA_pinv = W.inverse() * JA.transpose() * ((JA * W.inverse() * JA.transpose() + K_d).inverse());
	}	
	ref_twist << 0, 0, 0, 0, 0, 0;
	dyn_solver->JntToMass(q,B); //Inertia Matrix B 
	dyn_solver->JntToCoriolis(q,q_dot,C); // Coriolis Matrix C
	dyn_solver->JntToGravity(q, G); //Gravity vector G

	KDL::ChainDynParam dyn_param(chain,gravity_v);
	eq << Eigen::VectorXd::Zero(arm_cubes_n);
	q_eigen_f << Eigen::VectorXd::Zero(arm_cubes_n);


	hand_cl = 0;
	R_p_2_r << R_p2r[0], R_p2r[1], R_p2r[2], 
		R_p2r[3], R_p2r[4], R_p2r[5],
		R_p2r[6], R_p2r[7], R_p2r[8];

	// ------------------------------------------------------------------------------------- Subscribe to topics
  	sub_posture		= n.subscribe(pose_ref_topic, 1, posture__Callback);
  	sub_stiffness	= n.subscribe(stiff_ref_topic, 1, arm_stiffness__Callback);
	sub_hand_cl		= n.subscribe(hand_cl_topic, 1, hand_closure__Callback);
  	sub_cubes_m1	= n.subscribe(cubes_m1_topic, 1, cubes_m1__Callback);
  	sub_cubes_m2	= n.subscribe(cubes_m2_topic, 1, cubes_m2__Callback);
  	sub_cubes_shaft	= n.subscribe(cubes_shaft_topic, 1, cubes_shaft__Callback);
	sub_powerbooster = n.subscribe("/"+robot_name+"/"+power_booster_topic, 1, powerbooster__Callback);


	// ------------------------------------------------------------------------------------- Published topics 
	//ros::Publisher    	pub_inv_kin		= n.advertise<qb_interface::cubeEq_Preset>(target_chain + "_eq_pre", 1000);
	// pub_cart_ref	= n.advertise<geometry_msgs::Pose>(chain_topic, 1);
	// pub_phantom			= n.advertise<sensor_msgs::JointState>(phantom_arm_topic, 1);
	pub_ref_eq_arm_eq	= n.advertise<std_msgs::Float64MultiArray>(ref_eq_arm_topic, 1);
	pub_ref_pr_arm_eq	= n.advertise<std_msgs::Float64MultiArray>(ref_pr_arm_topic, 1);
	pub_ref_hand_eq		= n.advertise<std_msgs::Float64>(ref_hand_topic, 1);
	ready_for_pilot		= n.advertise<std_msgs::Bool>(ready_pilot_topic, 1);


	cmd_time = ros::Time::now();
	cmd_time_old = ros::Time::now();

	Eigen::VectorXd q_dot_eigen(q_dot.rows());
		for (int i = 0; i < q_dot.rows(); ++i) {
    		q_dot_eigen(i) = q_dot(i);
		}

		Eigen::VectorXd q_eigen(q.rows());
		for (int i = 0; i < q.rows(); ++i) {
    		q_eigen(i) = q(i);
		}

		Eigen::VectorXd q_ddot_eigen(q_ddot.rows());
		for (int i = 0; i < q_ddot.rows(); ++i) {
    		q_ddot_eigen(i) = q_ddot(i);
		}

		Eigen::VectorXd G_eigen(G.rows());
		for (int i = 0; i < G.rows(); ++i) {
    		G_eigen(i) = G(i);
		}

		Eigen::MatrixXd jacobian_dot_eigen(jacobian_dot.rows(), jacobian_dot.columns());

        for (int i = 0; i < jacobian_dot.rows(); ++i) {
    		for (int j = 0; j < jacobian_dot.columns(); ++j) {
        		jacobian_dot_eigen(i, j) = jacobian_dot(i, j);
    		}
		}

		Eigen::MatrixXd B_eigen(B.rows(), B.columns());
		for (int i = 0; i < B.rows(); ++i) {
    		for (int j = 0; j < B.columns(); ++j) {
        		B_eigen(i, j) = B(i, j);
    		}
		}

		Eigen::MatrixXd C_eigen(C.rows(), C.columns());
		for (int i = 0; i < C.rows(); ++i) {
    		for (int j = 0; j < C.columns(); ++j) {
        		C_eigen(i, j) = C(i, j);
    		}
		}

	while (ros::ok())
  	{
		// Direct Kinematic
  		jnt_to_pose_solver->JntToCart(q, act_frame); 
		KDL::Vector	pos_end_effector_act = act_frame.p; // si ottiene la posizione attuale dell'end effector
		jnt_to_jac_solver->JntToJac(q, JA_kdl); //Calcola il Jacobiano delle articolazioni basato sulle posizioni delle articolazioni correnti (q) del robot. 

		for (int i = 0; i<6; i++){
			for (int j = 0; j< chain.getNrOfJoints(); j++)
				if(fabs(JA_kdl(i,j)) > 0.000001)
					JA(i,j) = JA_kdl(i,j);
				else
					JA(i,j) = 0;
		}
		jac_dot_solver->JntToJacDot (Vel,jacobian_dot);
		error = pos_end_effector - pos_end_effector_act; //err_twist è calcolato come la differenza tra due frame, act_frame e ref_frame. 
		//Questo rappresenta l'errore tra la posizione e l'orientamento desiderati (ref_frame) e la posizione e l'orientamento attuali (act_frame) dell'end-effector del robot.

		err_post 	<< error[0], error[1], error[2], error[3], error[4], error[5];
		err_post = K_v*err_post;

		if (AlterEgoVersion == 2){
			JA_pinv = JA.transpose()*((JA*JA.transpose() +K_d).inverse());
		}
		else if(AlterEgoVersion == 3){
			JA_pinv = W.inverse() * JA.transpose() * ((JA * W.inverse() * JA.transpose() + K_d).inverse());
		}

		//Kinematic Reference (Calcolo di qd_dot e qd_ddot)
		qd_dot = JA_pinv*err_post;
		qd_dot = JA_pinv*err_post; //K_v = Lambda
		qd_ddot = -JA_pinv*K_v*JA*q_dot_eigen-JA_pinv*jacobian_dot_eigen*JA_pinv*err_post;
		qd += qd_dot/run_freq;
		e = qd-q_eigen;
		e_dot = qd_dot - q_dot_eigen;
		e_ddot = qd_ddot - q_ddot_eigen;
		tau_computed = B_eigen*qd_ddot + C_eigen*q_dot_eigen + G_eigen + B_eigen*K_p*e +B_eigen*K_v*e_dot;
		Eigen::Matrix<double, 12, 6> g;
    	g.setZero();
    	g.block<6, 6>(6, 0) = -B_eigen.completeOrthogonalDecomposition().pseudoInverse(); // Assegna l'inverso di B alle ultime 6 righe



		Eigen::ArrayXd operand1 = (a_mot * (q_eigen - theta_s));
		Eigen::ArrayXd operand2 = (a_mot * theta_d);
		h[0] = 2*k_mot*(a_mot*cosh(operand1[0])*q_dot_eigen[0]*cosh(operand2[0])+sinh(operand2[0])*theta_d_dot[0]*a_mot*sinh(operand1[0]));
		h[1] = 2*k_mot*(a_mot*cosh(operand1[1])*q_dot_eigen[1]*cosh(operand2[1])+sinh(operand2[1])*theta_d_dot[1]*a_mot*sinh(operand1[1]));
		h[2] = 2*k_mot*(a_mot*cosh(operand1[2])*q_dot_eigen[2]*cosh(operand2[2])+sinh(operand2[2])*theta_d_dot[2]*a_mot*sinh(operand1[2]));
		h[3] = 2*k_mot*(a_mot*cosh(operand1[3])*q_dot_eigen[3]*cosh(operand2[3])+sinh(operand2[3])*theta_d_dot[3]*a_mot*sinh(operand1[3]));
		h[4] = 2*k_mot*(a_mot*cosh(operand1[4])*q_dot_eigen[4]*cosh(operand2[4])+sinh(operand2[4])*theta_d_dot[4]*a_mot*sinh(operand1[4]));
		h[5] = 2*k_mot*(a_mot*cosh(operand1[5])*q_dot_eigen[5]*cosh(operand2[5])+sinh(operand2[5])*theta_d_dot[5]*a_mot*sinh(operand1[5]));


		
		l[0] = -2*a_mot*k_mot*cosh(operand1[0])*cosh(operand2[0]);
		l[1] = -2*a_mot*k_mot*cosh(operand1[1])*cosh(operand2[1]);
		l[2] = -2*a_mot*k_mot*cosh(operand1[2])*cosh(operand2[2]);
		l[3] = -2*a_mot*k_mot*cosh(operand1[3])*cosh(operand2[3]);
		l[4] = -2*a_mot*k_mot*cosh(operand1[4])*cosh(operand2[4]);
		l[5] = -2*a_mot*k_mot*cosh(operand1[5])*cosh(operand2[5]);
		L.diagonal() << l[0],l[1],l[2],l[3],l[4],l[5],l[6];


		d_tau = -C_eigen*e_ddot+B_eigen*K_v*e_ddot+B_eigen*K_p*e_dot;
		Eigen::VectorXd e_trasposto_kp;
		e_trasposto_kp = e.transpose()*K_p;
		Eigen::VectorXd Vx1(6+6);
		Vx1.head(6) = e_trasposto_kp;
		Vx1.tail(6) = e_dot.transpose();
		tau_elastica[0] = 2*k_mot*(cosh(operand2[0])*sinh(operand1[0]));
		tau_elastica[1] = 2*k_mot*(cosh(operand2[1])*sinh(operand1[1]));
		tau_elastica[2] = 2*k_mot*(cosh(operand2[2])*sinh(operand1[2]));
		tau_elastica[3] = 2*k_mot*(cosh(operand2[3])*sinh(operand1[3]));
		tau_elastica[4] = 2*k_mot*(cosh(operand2[4])*sinh(operand1[4]));
		tau_elastica[5] = 2*k_mot*(cosh(operand2[5])*sinh(operand1[5]));


		v = d_tau + P.inverse()*(K_c*(tau_computed-tau_elastica)-g.transpose()*Vx1);
		tau = L.inverse()*(v-h); // uscita del controllore
		theta_s += v/run_freq; 
		q_ddot_eigen = B_eigen.inverse()*(tau_elastica-C_eigen*q_dot_eigen - G_eigen);
		q_dot_eigen += q_ddot_eigen/run_freq;
		q_eigen += q_dot_eigen/run_freq;

		if (act_bp == 1 && (ros::Time::now() - cmd_time > max_cmd_time))
		{
			eq << Eigen::VectorXd::Zero(arm_cubes_n);
			start_f_time = ros::Time::now();
			alpha = 1;
			flag_pilot_out_ = true;
		}
		else
			flag_pilot_out_ = false;

		// Filtering position
		if (ros::Time::now()-start_f_time < filt_time){
			alpha -= 1/(filt_time.toSec() *run_freq);
			if (alpha < 0)
				alpha = 0;
			q_eigen_f = alpha*q_eigen_f + (1-alpha)*q_eigen;
		}
		else{
			q_eigen_f = q_eigen;
		}

		 // controllo che ognuno di questi gdl sia in un range di 0.3 dallo 0. per (1) considero che ha un offset di +- 0.33
		if (flag_pilot_out_ && (std::fabs(meas_cube_shaft(0)) < 0.1) && ((std::fabs(meas_cube_shaft(1)) > 0.25) && (std::fabs(meas_cube_shaft(1)) < 0.35)) && (std::fabs(meas_cube_shaft(3)) < 0.1))
		{
			std_msgs::Bool msg;
			msg.data = true;
			ready_for_pilot.publish(msg);
		} //Se tutte queste condizioni sono vere, il braccio robotico è considerato "pronto per il pilota" 
		// e viene inviato un messaggio booleano tramite il topic ready_for_pilot.
		

	// Questo ciclo assicura che le posizioni delle articolazioni del braccio robotico siano limitate ai valori massimi e minimi consentiti, 
		//a seconda della versione del robot e delle condizioni specifiche per alcune articolazioni.
		for (int i = 0; i<arm_cubes_n; i++) {
			
			if(AlterEgoVersion==2)
			{
				if (q_eigen_f(i) > q_max[i])
				{
					q_eigen_f(i) = q_max[i];
				}
				if (q_eigen_f(i) < q_min[i])
				{
					q_eigen_f(i) = q_min[i];
				}
			}
			else if(AlterEgoVersion == 3 )
			{
				if (i == 1)
				{
					if (ns.find("left") != std::string::npos) //distringuo tra braccio sinistro e destro
					{
						if (q_eigen_f(i) > q_max[i] + 0.33)
						{
							q_eigen_f(i) = q_max[i] + 0.33;
						}
						if (q_eigen_f(i) < q_min[i] + 0.33)
						{
							q_eigen_f(i) = q_min[i] + 0.33;
						}
					}
					else
					{
						if (q_eigen_f(i) > q_max[i] - 0.33)
						{
							q_eigen_f(i) = q_max[i] - 0.33;
						}
						if (q_eigen_f(i) < q_min[i] - 0.33)
						{
							q_eigen_f(i) = q_min[i] - 0.33;
						}
					}
				}
				else
				{
					if (q_eigen_f(i) > q_max[i])
					{
						q_eigen_f(i) = q_max[i];
					}
					if (q_eigen_f(i) < q_min[i])
					{
						q_eigen_f(i) = q_min[i];
					}
				}
			}
			q(i)	= q_eigen_f(i);
		}

		if (powerbooster)
		{
			stiffn_vec = stiffn_vec_power;
		}
		else
		{
			stiffn_vec = stiffn_vec_no_power;
		}

		for (int i = 0; i < 6; ++i) {
        	q_dot.data[i] = q_dot_eigen(i);
    		}	


		act_frame.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());

		// est. wrench
		jnt_to_jac_solver->JntToJac(meas_cube_shaft, J_wrench_kdl);
		dyn_solver->JntToGravity(meas_cube_shaft, g_wrench_comp);

		//from KDL to Eig right
	    for(int i = 0; i < 6 ; i++)
	    {
	      for(int j = 0; j < chain.getNrOfJoints() ; j++)
	      {
	        Jac_wrench(i, j) = J_wrench_kdl(i, j);
	      }
	    }


		//calcolo di tau
	    for(int i = 0; i < chain.getNrOfJoints(); i++)
	    {
	      Grav_wrench(i) = g_wrench_comp(i); //from KDL to Eig

	      double defl_1_R = meas_cube_shaft(i) - meas_cube_m1(i);	//altrimenti quando calcolo tau la tangente va a inf
	      double defl_2_R = meas_cube_shaft(i) - meas_cube_m2(i);

	      if (fabs(defl_1_R) > defl_max) defl_1_R = sgn(defl_1_R) * defl_max;
	      if (fabs(defl_2_R) > defl_max) defl_2_R = sgn(defl_2_R) * defl_max;

	      tau_meas(i) = k_mot * tan(a_mot * (defl_1_R)) + k_mot * tan(a_mot * (defl_2_R));
	      
	    }

	    pseudo_inverse(Jac_wrench.transpose(), Jac_trans_pinv_wrench, true);
	    wrench =  Jac_trans_pinv_wrench * (Grav_wrench + tau_meas);

	    wrench_msg.force.x = wrench(0);
	    wrench_msg.force.y = wrench(1);
	    wrench_msg.force.z = wrench(2);
	    wrench_msg.torque.x = wrench(3);
	    wrench_msg.torque.y = wrench(4);
	    wrench_msg.torque.z = wrench(5);
	    pub_wrench.publish(wrench_msg);

		// --- set all messages ---
		arm_eq_ref_msg.data.clear();
		arm_pr_ref_msg.data.clear();
		for (int i=0; i<chain.getNrOfJoints(); i++){
			arm_eq_ref_msg.data.push_back(q_eigen(i));
			arm_pr_ref_msg.data.push_back(stiffn_vec[i] + (2 * stiffn));
		}
		hand_ref_msg.data = hand_cl;



		ref_frame.M.GetQuaternion(ref_shaft_quat.x(), ref_shaft_quat.y(), ref_shaft_quat.z(), ref_shaft_quat.w());
		ik_tf_ref.setRotation(tf::Quaternion(ref_shaft_quat.x(), ref_shaft_quat.y(), ref_shaft_quat.z(), ref_shaft_quat.w()));
		ik_tf_ref.setOrigin(tf::Vector3(ref_frame.p[0], ref_frame.p[1], ref_frame.p[2]));
		ik_ref.sendTransform(tf::StampedTransform(ik_tf_ref, ros::Time::now(), "torso", side + "_hand_ref"));

		shaft_to_pose_solver->JntToCart(meas_cube_shaft, shaft_frame);
		shaft_frame.M.GetQuaternion(act_shaft_quat.x(), act_shaft_quat.y(), act_shaft_quat.z(), act_shaft_quat.w());
		ik_tf_shaft.setOrigin(tf::Vector3(shaft_frame.p[0], shaft_frame.p[1], shaft_frame.p[2]));
		ik_tf_shaft.setRotation(tf::Quaternion(act_shaft_quat.x(), act_shaft_quat.y(), act_shaft_quat.z(), act_shaft_quat.w()));
		ik_shaft.sendTransform(tf::StampedTransform(ik_tf_shaft, ros::Time::now(), "torso", side + "_hand_curr"));

		// Rviz model:
		phantom_msg.name.resize(chain.getNrOfJoints() + 1);
		phantom_msg.position.resize(chain.getNrOfJoints() + 1);
		for (int i=0; i<chain.getNrOfJoints(); i++){
			 sprintf (buffer, "phantom_cube%d_shaft_joint", qbmove_tf_ids[i]);
			 phantom_msg.name[i] = buffer;
			phantom_msg.position[i] = q(i);
		}
		sprintf (buffer, "phantom_hand%d_synergy_joint", softhand_tf_id);
		phantom_msg.name[5] = buffer;
		phantom_msg.position[5] = hand_cl;


		// --- publish all messages ---
		pub_ref_eq_arm_eq.publish(arm_eq_ref_msg);
		pub_ref_pr_arm_eq.publish(arm_pr_ref_msg);
		pub_ref_hand_eq.publish(hand_ref_msg);
		// pub_phantom.publish(phantom_msg);


		// --- cycle ---
  		ros::spinOnce();
    	loop_rate.sleep();
	}
}


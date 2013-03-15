/************************************************************************/

#define XX 0
#define YY 1
#define ZZ 2
#define N_XYZ 3

// joint types - need all revolute together
#define REVOLUTE 1
#define REVOLUTE_X 2
#define REVOLUTE_Y 3
#define REVOLUTE_Z 4
#define REVOLUTE_M_X 5
#define REVOLUTE_M_Y 6
#define REVOLUTE_M_Z 7
#define WELD 8
#define FREE 9

// parameters
#define P0 0
#define V0 1
#define A0 2
#define PT 3
#define VT 4
#define AT 5
#define N_PARAMETERS 6

// this defines the number of strands
// thread: as defined by pthread
// strand: N_THREADS*8
#define N_BLOCKS 1 // we really mean gangs here, but reuse the term BLOCK
#define N_STRANDS_PER_BLOCK 1
#define N_STRANDS (N_BLOCKS*N_STRANDS_PER_BLOCK)

#define MAX_N_CHILDREN 5

/************************************************************************/

/*
1) Assumption is that joints and links come in pairs, with the joint being
the parent joint of that link.
*/

typedef struct kin_dyn
{
  // stuff that needs to be initialized
  char *joint_name;
  char *link_name;
  int index;
  int parent; // parent link/joint of this link/joint, 0 is root;
  double joint_offset_b[3]; // offset from previous joint to this joint,
                            // in previous body's body coordinates.
  int joint_type; // what type of joint is it?
  double axis_b[3]; // joint axis, in this and previous body's 
                    // body coordinates.
  double mass; // mass of this link.
  double com_offset_b[3]; // com offset, in this link's coordinates.
  int moi_diagonal; // is Moment of Inertia diagonal?
  double moi_v[3]; // diagonal moment of inertia
  double moi[3][3]; // moment of inertia
  double angle_limits[2];
  double velocity_limits[2];
  double torque_limits[2];
  int children[MAX_N_CHILDREN];

  // joint state
  double angle;
  double angled;
  double angledd;

  // other stuff computed from joint state.
  double sine;   // rotation represented with sine and cosine (FK, ID)
  double cosine; // for single axis rotation (FK, ID)
  double rotation[3][3]; // rotation represented with full matrix (FK, ID)
  double position_w[3]; // joint position in world coordinates (FK)
  double com_w[3]; // COM position in world coordinates (FK)
  double orientation[3][3]; // current pose of link in world coordinates (FK)
  double angular_velocity_b[3];
  double angular_acceleration_b[3];
  double joint_acceleration_b[3];
  double link_acceleration_b[3];
  double tmp_joint_force_b[3];
  double joint_force_b[3];
  double joint_torque_b[3];
  double link_force_b[3];
  double link_torque_b[3];
  double joint_force_w[3];
  double joint_torque_w[3];
  double link_force_w[3];
  double link_torque_w[3];
  double j_offset[3];
  double joint_offset_w[3];
  double com_offset_w[3]; // offset of COM in world coordinates.
  double the_joint_torque;

  // dynamics tmp variables
  double w_x_off[3];
  double w_x_com[3];
  double acc_tmp[3];
  double Iw_b[3];
  double angled_x_axis_b[3];

}
  KIN_DYN;

/************************************************************************/
/*
#define N_LINKS 6
#define N_JOINTS 5

// We know assume these indices reflect the order in which joints are
// created.
#define A1P 0
#define K1P 1
#define H1P 2
#define H2P 3
#define K2P 4
// it is convenient to tack on ground force indices
#define FX 5
#define FZ 6

#define WEIGHT_A1P 1.0f
#define WEIGHT_K1P 1.0f
#define WEIGHT_H1P 1.0f
#define WEIGHT_H2P 1.0f
#define WEIGHT_K2P 1.0f
#define WEIGHT_GX 1.0f
#define WEIGHT_GZ 1.0f
*/
/************************************************************************/

void generate_rotation_matrix_from_q( double q[4], double r[3][3] );
void init_kin_dyn();
void forward_kinematics( KIN_DYN *r, int n );

void id_pass1_r( KIN_DYN *r, int i );
void id_pass1_x( KIN_DYN *r, int i );
void id_pass1_y( KIN_DYN *r, int i );
void id_pass1_z( KIN_DYN *r, int i );
void id_pass1_mx( KIN_DYN *r, int i );
void id_pass1_my( KIN_DYN *r, int i );
void id_pass1_mz( KIN_DYN *r, int i );

void id_pass2( KIN_DYN *r, int i );
void id_pass2_no_sum( KIN_DYN *r, int i );
void id_pass2_null_link( KIN_DYN *r, int i );

void m3_identity( double m[3][3] );
void mtv3_multiply( double m[3][3], double v1[3], double v2[3] );
void mv3_multiply( double m[3][3], double v1[3], double v2[3] );
void v3_copy( const double * v1, double * v2 );

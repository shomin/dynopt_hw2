/************************************************************************/
/*
kin-dyn.c: do kinematics and dynamics for some robot.

Problems:
1) want to set ankle position (joint 1, not 0) in atlas_statics
*/
/************************************************************************/
/*
*/
/************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "kin-dyn.h"

/************************************************************************/
/* DEFINES */

/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/

static inline void v3_zero( double * v )
{
  v[0] = 0.0;
  v[1] = 0.0;
  v[2] = 0.0;
}

/************************************************************************/

void v3_copy( const double * v1,
	      double * v2 )
{
  v2[0] = v1[0];
  v2[1] = v1[1];
  v2[2] = v1[2];
}

/************************************************************************/

static inline void v3_scale( const double s,
			     const double * v1,
			     double * v2 )
{
  v2[0] = s*v1[0];
  v2[1] = s*v1[1];
  v2[2] = s*v1[2];
}

/************************************************************************/

static inline void v3_scale_acc( const double s,
				 const double * v1,
				 double * v2 )
{
  v2[0] += s*v1[0];
  v2[1] += s*v1[1];
  v2[2] += s*v1[2];
}

/************************************************************************/

static inline void v3_add( const double * v1,
			   const double * v2,
			   double * v3 )
{
  v3[0] = v1[0] + v2[0];
  v3[1] = v1[1] + v2[1];
  v3[2] = v1[2] + v2[2];
}

/************************************************************************/

static inline void v3_acc( const double * v1,
			   double * v2 )
{
  v2[0] += v1[0];
  v2[1] += v1[1];
  v2[2] += v1[2];
}

/************************************************************************/

static void vxv3( double v1[3], double v2[3], double v3[3] )
{
  v3[ZZ] = v1[XX]*v2[YY] - v1[YY]*v2[XX];
  v3[YY] = -v1[XX]*v2[ZZ] + v1[ZZ]*v2[XX];
  v3[XX] = v1[YY]*v2[ZZ] - v1[ZZ]*v2[YY];
}

/************************************************************************/

static void vxv3_acc( double v1[3], double v2[3], double v3[3] )
{
  v3[ZZ] += v1[XX]*v2[YY] - v1[YY]*v2[XX];
  v3[YY] += -v1[XX]*v2[ZZ] + v1[ZZ]*v2[XX];
  v3[XX] += v1[YY]*v2[ZZ] - v1[ZZ]*v2[YY];
}

/************************************************************************/

static void m3_zero( double m[3][3] )
{
  int i, j;
 
  for ( i = 0; i < 3; i++ )
    for ( j = 0; j < 3; j++ )
      m[i][j] = 0;
}

/************************************************************************/

void m3_identity( double m[3][3] )
{
  int i, j;
 
  for ( i = 0; i < 3; i++ )
    for ( j = 0; j < 3; j++ )
      m[i][j] = 0;
  m[0][0] = 1;
  m[1][1] = 1;
  m[2][2] = 1;
}

/************************************************************************/

void mv3_multiply( double m[3][3], double v1[3], double v2[3] )
{
  int i, j;
  double sum;

  for ( i = 0; i < 3; i++ )
    {
      sum = 0;
      for ( j = 0; j < 3; j++ )
	sum += m[i][j]*v1[j];
      v2[i] = sum;
    }
}

/************************************************************************/

static void mv3_multiply_acc( double m[3][3], double v1[3], double v2[3] )
{
  int i, j;
  double sum;

  for ( i = 0; i < 3; i++ )
    {
      sum = 0;
      for ( j = 0; j < 3; j++ )
	sum += m[i][j]*v1[j];
      v2[i] += sum;
    }
}

/************************************************************************/
// multiply the transpose of the matrix m times a vector: m'*v

void mtv3_multiply( double m[3][3], double v1[3], double v2[3] )
{
  int i, j;
  double sum;

  for ( i = 0; i < 3; i++ )
    {
      sum = 0;
      for ( j = 0; j < 3; j++ )
	sum += m[j][i]*v1[j];
      v2[i] = sum;
    }
}

/************************************************************************/
// multiply the transpose of the matrix m times a vector: m'*v

static void mtv3_multiply_acc( double m[3][3], double v1[3], double v2[3] )
{
  int i, j;
  double sum;

  for ( i = 0; i < 3; i++ )
    {
      sum = 0;
      for ( j = 0; j < 3; j++ )
	sum += m[j][i]*v1[j];
      v2[i] += sum;
    }
}

/************************************************************************/

static void m3_multiply( double m1[3][3], double m2[3][3], double m3[3][3] )
{
  int i, j, k;
  double sum;

  for ( i = 0; i < 3; i++ )
    {
      for ( j = 0; j < 3; j++ )
	{
	  sum = 0;
	  for ( k = 0; k < 3; k++ )
	    sum += m1[i][k]*m2[k][j];
	  m3[i][j] = sum;
	}
    }
}

/************************************************************************/

static void generate_axis_angle_rotation_matrix( double axis[3],
						 double cosine,
						 double sine,
						 double r[3][3] )
{
  int i;
  double c = cosine;
  double s = sine;
  double t = 1 - c;
  double axis_length_2 = 0;
  double norm_factor;
  double x, y, z;

  for ( i = 0; i < 3; i++ )
    axis_length_2 += axis[i]*axis[i];

  norm_factor = 1.0/sqrt( axis_length_2 );

  x = axis[0]*norm_factor;
  y = axis[1]*norm_factor;
  z = axis[2]*norm_factor;

  // printf( "axis: %g %g %g; angle: %g\n", x, y, z, angle );

  r[0][0] = t*x*x + c;
  r[0][1] = t*x*y - z*s;
  r[0][2] = t*x*z + y*s;

  r[1][0] = t*x*y + z*s;
  r[1][1] = t*y*y + c;
  r[1][2] = t*y*z - x*s;

  r[2][0] = t*x*z - y*s;
  r[2][1] = t*y*z + x*s;
  r[2][2] = t*z*z + c;

  /*
  printf( "%g %g %g\n%g %g %g\n%g %g %g\n",
	  r[0][0], r[0][1], r[0][2], 
	  r[1][0], r[1][1], r[1][2], 
	  r[2][0], r[2][1], r[2][2] );
  */
}

/************************************************************************/

void generate_rotation_matrix_from_q( double q[4], double r[3][3] )
{
  double X = q[0];
  double Y = q[1];
  double Z = q[2];
  double W = q[3];

  double xx      = X * X;
  double xy      = X * Y;
  double xz      = X * Z;
  double xw      = X * W;

  double yy      = Y * Y;
  double yz      = Y * Z;
  double yw      = Y * W;

  double zz      = Z * Z;
  double zw      = Z * W;

  r[0][0]  = 1 - 2 * ( yy + zz );
  r[0][1]  =     2 * ( xy - zw );
  r[0][2]  =     2 * ( xz + yw );

  r[1][0]  =     2 * ( xy + zw );
  r[1][1]  = 1 - 2 * ( xx + zz );
  r[1][2]  =     2 * ( yz - xw );

  r[2][0]  =     2 * ( xz - yw );
  r[2][1]  =     2 * ( yz + xw );
  r[2][2]  = 1 - 2 * ( xx + yy );
}

/************************************************************************/

// product of elements
static inline void duv_by_element( const double * v1,
				   const double * v2,
				   double * v3 )
{
  v3[0] = v1[0] * v2[0];
  v3[1] = v1[1] * v2[1];
  v3[2] = v1[2] * v2[2];
}

/************************************************************************/

static inline void w_x( const double * w,
			const double * v,
			double * result )
{
  result[XX] = w[YY]*v[ZZ] - w[ZZ]*v[YY];
  result[YY] = w[ZZ]*v[XX] - w[XX]*v[ZZ];
  result[ZZ] = w[XX]*v[YY] - w[YY]*v[XX];
}

/************************************************************************/

static inline void w_x_acc( const double * w,
			    const double * v,
			    double * result )
{
  result[XX] += w[YY]*v[ZZ] - w[ZZ]*v[YY];
  result[YY] += w[ZZ]*v[XX] - w[XX]*v[ZZ];
  result[ZZ] += w[XX]*v[YY] - w[YY]*v[XX];
}

/************************************************************************/

static inline void w_v_u_x( const double * w,
			    const double * v,
			    double * result )
{
  result[XX] = w[YY]*v[ZZ] - w[ZZ]*v[YY];
  result[YY] = w[ZZ]*v[XX] - w[XX]*v[ZZ];
  result[ZZ] = w[XX]*v[YY] - w[YY]*v[XX];
}

/************************************************************************/

static inline void w_v_u_x_acc( const double * w,
				const double * v,
				double * result )
{
  result[XX] += w[YY]*v[ZZ] - w[ZZ]*v[YY];
  result[YY] += w[ZZ]*v[XX] - w[XX]*v[ZZ];
  result[ZZ] += w[XX]*v[YY] - w[YY]*v[XX];
}

/************************************************************************/

static inline void w_u_v_x( const double * w,
			    const double * v,
			    double * result )
{
  result[XX] = w[YY]*v[ZZ] - w[ZZ]*v[YY];
  result[YY] = w[ZZ]*v[XX] - w[XX]*v[ZZ];
  result[ZZ] = w[XX]*v[YY] - w[YY]*v[XX];
}

/************************************************************************/

static inline void w_u_v_x_acc( const double * w,
				const double * v,
				double * result )
{
  result[XX] += w[YY]*v[ZZ] - w[ZZ]*v[YY];
  result[YY] += w[ZZ]*v[XX] - w[XX]*v[ZZ];
  result[ZZ] += w[XX]*v[YY] - w[YY]*v[XX];
}

/************************************************************************/
/************************************************************************/
/************************************************************************/

static inline void Rv_x_v_mult( const double cosine,
				 const double sine,
				 const double * v1,
				 double * v2 )
{
  v2[0] = v1[0];
  v2[1] = v1[1] * cosine - v1[2] * sine;
  v2[2] = v1[1] * sine + v1[2] * cosine;
}

/************************************************************************/

static inline void Rv_x_v_mult_acc( const double cosine,
				     const double sine,
				     const double * v1,
				     double * v2 )
{
  v2[0] += v1[0];
  v2[1] += v1[1] * cosine - v1[2] * sine;
  v2[2] += v1[1] * sine + v1[2] * cosine;
}

/************************************************************************/

static inline void Rv_y_v_mult( const double cosine,
				 const double sine,
				 const double * v1,
				 double * v2 )
{
  v2[0] = v1[0] * cosine + v1[2] * sine;
  v2[1] = v1[1];
  v2[2] = -v1[0] * sine + v1[2] * cosine;
}

/************************************************************************/

static inline void Rv_y_v_mult_acc( const double cosine,
				     const double sine,
				     const double * v1,
				     double * v2 )
{
  v2[0] += v1[0] * cosine + v1[2] * sine;
  v2[1] += v1[1];
  v2[2] += -v1[0] * sine + v1[2] * cosine;
}

/************************************************************************/

static inline void Rv_z_v_mult( const double cosine,
				 const double sine,
				 const double * v1,
				 double * v2 )
{
  v2[0] = v1[0] * cosine - v1[1] * sine;
  v2[1] = v1[0] * sine + v1[1] * cosine;
  v2[2] = v1[2];
}

/************************************************************************/

static inline void Rv_z_v_mult_acc( const double cosine,
				     const double sine,
				     const double * v1,
				     double * v2 )
{
  v2[0] += v1[0] * cosine - v1[1] * sine;
  v2[1] += v1[0] * sine + v1[1] * cosine;
  v2[2] += v1[2];
}

/************************************************************************/
/************************************************************************/
/************************************************************************/

void init_kin_dyn()
{
	//printf( "\nKIN_DYN initialized.\n" );
}

/************************************************************************/
/*
1) We assume joint angles set in each joint.
2) We assume position_w and orientation set for joint/link 0.
Something like
  position[0][0] = 0;
  position[0][1] = 0.089;
  position[0][2] = 0;

  m3_zero( orientation[0] );
  orientation[0][0][0] = 1;
  orientation[0][1][1] = 1;
  orientation[0][2][2] = 1;
  -or-
  generate_rotation_matrix_from_q( q, orientation[0] );
*/

void forward_kinematics( KIN_DYN *r, int n )
{
  int i;
  int p;
  double total_mass;
  double com_total[3];

  mv3_multiply( r[0].orientation, r[0].com_offset_b, r[0].com_offset_w );
  v3_add( r[0].position_w, r[0].com_offset_w, r[0].com_w );

  total_mass = r[0].mass;
  v3_scale( r[0].mass, r[0].com_w, com_total );

  for ( i = 1; i < n; i++ )
    {
      p = r[i].parent;
      generate_axis_angle_rotation_matrix( r[i].axis_b, r[i].cosine,
					   r[i].sine, r[i].rotation );
      m3_multiply( r[p].orientation, r[i].rotation, r[i].orientation );
      mv3_multiply( r[p].orientation, r[i].joint_offset_b,
		    r[i].joint_offset_w );
      v3_add( r[p].position_w, r[i].joint_offset_w, r[i].position_w );

      mv3_multiply( r[i].orientation, r[i].com_offset_b, r[i].com_offset_w );
      v3_add( r[i].position_w, r[i].com_offset_w, r[i].com_w );

      total_mass += r[i].mass;
      v3_scale_acc( r[i].mass, r[i].com_w, com_total );
    }

  v3_scale( 1.0/total_mass, com_total, com_total );
  //printf( "atlas mass: %g\n", total_mass );
}

/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/

static void id_pass1_part2( KIN_DYN *r, int i )
{

  // COM acceleration
  v3_copy( r[i].joint_acceleration_b, r[i].link_acceleration_b );
  // w x COM_offset
  w_v_u_x( r[i].angular_velocity_b, r[i].com_offset_b, r[i].w_x_com );
  // xdd += w x (w x COM_offset)
  w_x_acc( r[i].angular_velocity_b, r[i].w_x_com, r[i].link_acceleration_b );
  // xdd += wd x COM_offset
  w_v_u_x_acc( r[i].angular_acceleration_b, r[i].com_offset_b,
               r[i].link_acceleration_b );

  /*
printf( "ja[%d]: %g %g %g\n", i, r[i].joint_acceleration_b[XX],
          r[i].joint_acceleration_b[YY], r[i].joint_acceleration_b[ZZ] );
printf( "la[%d]: %g %g %g\n", i, r[i].link_acceleration_b[XX],
          r[i].link_acceleration_b[YY], r[i].link_acceleration_b[ZZ] );
	*/
}

  /*
  printf( "av: %g %g %g\n", r[i].angular_velocity_b[XX],
          r[i].angular_velocity_b[YY],
          r[i].angular_velocity_b[ZZ] );
  printf( "aa: %g %g %g\n", r[i].angular_acceleration_b[XX],
          r[i].angular_acceleration_b[YY],
          r[i].angular_acceleration_b[ZZ] );

  printf( "joint offset: %g %g %g\n", r[i].joint_offset_b[XX],
          r[i].joint_offset_b[YY],
          r[i].joint_offset_b[ZZ] );
  */
  /*
  */

  /*
  printf( "com: %g %g %g\n", r[i].com_offset_b[XX],
          r[i].com_offset_b[YY],
          r[i].com_offset_b[ZZ] );
  printf( "wxcom: %g %g %g\n", r[i].w_x_com[XX],
          r[i].w_x_com[YY],
          r[i].w_x_com[ZZ] );
  */

/************************************************************************/
// rotation about arbitrary axis

void id_pass1_r( KIN_DYN *r, int i )
{
  int ii;
  int p; // parent

  p = r[i].parent;

  // generate rotation matrix
  generate_axis_angle_rotation_matrix( r[i].axis_b, r[i].cosine,
				       r[i].sine, r[i].rotation );


  // propagate angular velocity outwards in body coordinates
  mtv3_multiply( r[i].rotation, r[p].angular_velocity_b,
		r[i].angular_velocity_b );
  // add joint angular velocity
  for ( ii = 0; ii < 3; ii++ )
    r[i].angular_velocity_b[ii] += r[i].angled*r[i].axis_b[ii];

  // propagate angular acceleration outwards in body coordinates
  mtv3_multiply( r[i].rotation, r[p].angular_acceleration_b,
		r[i].angular_acceleration_b );
  // add joint angular acceleration
  for ( ii = 0; ii < 3; ii++ )
    r[i].angular_acceleration_b[ii] += r[i].angledd*r[i].axis_b[ii];

  // handle Coriolis effect (w1 x w2)
  v3_scale( r[i].angled, r[i].axis_b, r[i].angled_x_axis_b );
  vxv3_acc( r[i].angular_velocity_b, r[i].angled_x_axis_b,
	    r[i].angular_acceleration_b );

  // joint acceleration
  v3_copy( r[p].joint_acceleration_b, r[i].acc_tmp );
  // w x offset
  w_v_u_x( r[p].angular_velocity_b, r[i].joint_offset_b, r[i].w_x_off );
  // xdd += w x (w x offset)
  w_x_acc( r[p].angular_velocity_b, r[i].w_x_off, r[i].acc_tmp );
  // xdd += wd x offset
  w_v_u_x_acc( r[p].angular_acceleration_b, r[i].joint_offset_b,
	       r[i].acc_tmp );
  mtv3_multiply( r[i].rotation, r[i].acc_tmp, r[i].joint_acceleration_b );

  id_pass1_part2( r, i );
}

/************************************************************************/
// rotation about 1 0 0 axis

void id_pass1_x( KIN_DYN *r, int i )
{
  int p; // parent

  p = r[i].parent;

  // propagate angular velocity outwards in body coordinates
  Rv_x_v_mult( r[i].cosine, -r[i].sine, r[p].angular_velocity_b,
	       r[i].angular_velocity_b );
  // add joint angular velocity
  r[i].angular_velocity_b[XX] += r[i].angled;

  // propagate angular acceleration outwards in body coordinates
  Rv_x_v_mult( r[i].cosine, -r[i].sine, r[p].angular_acceleration_b,
	       r[i].angular_acceleration_b );
  // add joint angular acceleration
  r[i].angular_acceleration_b[XX] += r[i].angledd;

  // handle Coriolis effect (w1 x w2)
  r[i].angular_acceleration_b[YY] += r[i].angled*r[i].angular_velocity_b[ZZ];
  r[i].angular_acceleration_b[ZZ] -= r[i].angled*r[i].angular_velocity_b[YY];

  // joint acceleration
  v3_copy( r[p].joint_acceleration_b, r[i].acc_tmp );
  // w x offset
  w_v_u_x( r[p].angular_velocity_b, r[i].joint_offset_b, r[i].w_x_off );
  // xdd += w x (w x offset)
  w_x_acc( r[p].angular_velocity_b, r[i].w_x_off, r[i].acc_tmp );
  // xdd += wd x offset
  w_v_u_x_acc( r[p].angular_acceleration_b, r[i].joint_offset_b,
	       r[i].acc_tmp );
  Rv_x_v_mult( r[i].cosine, -r[i].sine, r[i].acc_tmp,
	       r[i].joint_acceleration_b );

  id_pass1_part2( r, i );
}

/************************************************************************/
// rotation about -1 0 0 axis

void id_pass1_mx( KIN_DYN *r, int i )
{
  int p; // parent

  p = r[i].parent;

  // propagate angular velocity outwards in body coordinates
  Rv_x_v_mult( r[i].cosine, r[i].sine, r[p].angular_velocity_b,
	       r[i].angular_velocity_b );
  // add joint angular velocity
  r[i].angular_velocity_b[XX] -= r[i].angled;

  // propagate angular acceleration outwards in body coordinates
  Rv_x_v_mult( r[i].cosine, r[i].sine, r[p].angular_acceleration_b,
	       r[i].angular_acceleration_b );
  // add joint angular acceleration
  r[i].angular_acceleration_b[XX] -= r[i].angledd;

  // handle Coriolis effect (w1 x w2)
  r[i].angular_acceleration_b[YY] -= r[i].angled*r[i].angular_velocity_b[ZZ];
  r[i].angular_acceleration_b[ZZ] += r[i].angled*r[i].angular_velocity_b[YY];

  //printf( "mx: %d %d %g %g %g\n", i, p, r[p].joint_acceleration_b[XX],
	//	  r[p].joint_acceleration_b[YY], r[p].joint_acceleration_b[ZZ] );

  // joint acceleration
  v3_copy( r[p].joint_acceleration_b, r[i].acc_tmp );

  //printf( "mx acc_tmp: %g %g %g\n", r[i].acc_tmp[XX],
  //        r[i].acc_tmp[YY], r[i].acc_tmp[ZZ] );

  // w x offset
  w_v_u_x( r[p].angular_velocity_b, r[i].joint_offset_b, r[i].w_x_off );
  // xdd += w x (w x offset)
  w_x_acc( r[p].angular_velocity_b, r[i].w_x_off, r[i].acc_tmp );
  // xdd += wd x offset
  w_v_u_x_acc( r[p].angular_acceleration_b, r[i].joint_offset_b,
	       r[i].acc_tmp );
  Rv_x_v_mult( r[i].cosine, r[i].sine, r[i].acc_tmp,
	       r[i].joint_acceleration_b );

  //printf( "mx ja[%d]: %g %g %g\n", i, r[i].joint_acceleration_b[XX],
  //        r[i].joint_acceleration_b[YY], r[i].joint_acceleration_b[ZZ] );

  id_pass1_part2( r, i );
}

/************************************************************************/
// rotation about 0 1 0 axis

void id_pass1_y( KIN_DYN *r, int i )
{
  int p; // parent

  p = r[i].parent;

  // propagate angular velocity outwards in body coordinates
  Rv_y_v_mult( r[i].cosine, -r[i].sine, r[p].angular_velocity_b,
	       r[i].angular_velocity_b );
  // add joint angular velocity
  r[i].angular_velocity_b[YY] += r[i].angled;

  // propagate angular acceleration outwards in body coordinates
  Rv_y_v_mult( r[i].cosine, -r[i].sine, r[p].angular_acceleration_b,
	       r[i].angular_acceleration_b );
  // add joint angular acceleration
  r[i].angular_acceleration_b[YY] += r[i].angledd;

  // handle Coriolis effect (w1 x w2)
  r[i].angular_acceleration_b[XX] -= r[i].angled*r[i].angular_velocity_b[ZZ];
  r[i].angular_acceleration_b[ZZ] += r[i].angled*r[i].angular_velocity_b[XX];

  // joint acceleration
  v3_copy( r[p].joint_acceleration_b, r[i].acc_tmp );
  // w x offset
  w_v_u_x( r[p].angular_velocity_b, r[i].joint_offset_b, r[i].w_x_off );
  // xdd += w x (w x offset)
  w_x_acc( r[p].angular_velocity_b, r[i].w_x_off, r[i].acc_tmp );
  // xdd += wd x offset
  w_v_u_x_acc( r[p].angular_acceleration_b, r[i].joint_offset_b,
	       r[i].acc_tmp );
  Rv_y_v_mult( r[i].cosine, -r[i].sine, r[i].acc_tmp,
	       r[i].joint_acceleration_b );

  id_pass1_part2( r, i );
}

/************************************************************************/
// rotation about 0 -1 0 axis

void id_pass1_my( KIN_DYN *r, int i )
{
  int p; // parent

  p = r[i].parent;

  // propagate angular velocity outwards in body coordinates
  Rv_y_v_mult( r[i].cosine, r[i].sine, r[p].angular_velocity_b,
	       r[i].angular_velocity_b );
  // add joint angular velocity
  r[i].angular_velocity_b[YY] -= r[i].angled;

  // propagate angular acceleration outwards in body coordinates
  Rv_y_v_mult( r[i].cosine, r[i].sine, r[p].angular_acceleration_b,
	       r[i].angular_acceleration_b );
  // add joint angular acceleration
  r[i].angular_acceleration_b[YY] -= r[i].angledd;

  // handle Coriolis effect (w1 x w2)
  r[i].angular_acceleration_b[XX] += r[i].angled*r[i].angular_velocity_b[ZZ];
  r[i].angular_acceleration_b[ZZ] -= r[i].angled*r[i].angular_velocity_b[XX];

  // joint acceleration
  v3_copy( r[p].joint_acceleration_b, r[i].acc_tmp );
  // w x offset
  w_v_u_x( r[p].angular_velocity_b, r[i].joint_offset_b, r[i].w_x_off );
  // xdd += w x (w x offset)
  w_x_acc( r[p].angular_velocity_b, r[i].w_x_off, r[i].acc_tmp );
  // xdd += wd x offset
  w_v_u_x_acc( r[p].angular_acceleration_b, r[i].joint_offset_b,
	       r[i].acc_tmp );
  Rv_y_v_mult( r[i].cosine, r[i].sine, r[i].acc_tmp,
	       r[i].joint_acceleration_b );

  id_pass1_part2( r, i );
}

/************************************************************************/
// rotation about 0 0 1 axis

void id_pass1_z( KIN_DYN *r, int i )
{
  int p; // parent

  p = r[i].parent;

  // propagate angular velocity outwards in body coordinates
  Rv_z_v_mult( r[i].cosine, -r[i].sine, r[p].angular_velocity_b,
	       r[i].angular_velocity_b );
  // add joint angular velocity
  r[i].angular_velocity_b[ZZ] += r[i].angled;

  // propagate angular acceleration outwards in body coordinates
  Rv_z_v_mult( r[i].cosine, -r[i].sine, r[p].angular_acceleration_b,
	       r[i].angular_acceleration_b );
  // add joint angular acceleration
  r[i].angular_acceleration_b[ZZ] += r[i].angledd;

  // handle Coriolis effect (w1 x w2)
  r[i].angular_acceleration_b[XX] += r[i].angled*r[i].angular_velocity_b[YY];
  r[i].angular_acceleration_b[YY] -= r[i].angled*r[i].angular_velocity_b[XX];

  // joint acceleration
  v3_copy( r[p].joint_acceleration_b, r[i].acc_tmp );
  // w x offset
  w_v_u_x( r[p].angular_velocity_b, r[i].joint_offset_b, r[i].w_x_off );
  // xdd += w x (w x offset)
  w_x_acc( r[p].angular_velocity_b, r[i].w_x_off, r[i].acc_tmp );
  // xdd += wd x offset
  w_v_u_x_acc( r[p].angular_acceleration_b, r[i].joint_offset_b,
	       r[i].acc_tmp );
  Rv_z_v_mult( r[i].cosine, -r[i].sine, r[i].acc_tmp,
	       r[i].joint_acceleration_b );

  id_pass1_part2( r, i );
}

/************************************************************************/
// rotation about 0 0 -1 axis

void id_pass1_mz( KIN_DYN *r, int i )
{
  int p; // parent

  p = r[i].parent;

  // propagate angular velocity outwards in body coordinates
  Rv_z_v_mult( r[i].cosine, r[i].sine, r[p].angular_velocity_b,
	       r[i].angular_velocity_b );
  // add joint angular velocity
  r[i].angular_velocity_b[ZZ] -= r[i].angled;

  // propagate angular acceleration outwards in body coordinates
  Rv_z_v_mult( r[i].cosine, r[i].sine, r[p].angular_acceleration_b,
	       r[i].angular_acceleration_b );
  // add joint angular acceleration
  r[i].angular_acceleration_b[ZZ] -= r[i].angledd;

  // handle Coriolis effect (w1 x w2)
  r[i].angular_acceleration_b[XX] -= r[i].angled*r[i].angular_velocity_b[YY];
  r[i].angular_acceleration_b[YY] += r[i].angled*r[i].angular_velocity_b[XX];

  // joint acceleration
  v3_copy( r[p].joint_acceleration_b, r[i].acc_tmp );
  // w x offset
  w_v_u_x( r[p].angular_velocity_b, r[i].joint_offset_b, r[i].w_x_off );
  // xdd += w x (w x offset)
  w_x_acc( r[p].angular_velocity_b, r[i].w_x_off, r[i].acc_tmp );
  // xdd += wd x offset
  w_v_u_x_acc( r[p].angular_acceleration_b, r[i].joint_offset_b,
	       r[i].acc_tmp );
  Rv_z_v_mult( r[i].cosine, r[i].sine, r[i].acc_tmp,
	       r[i].joint_acceleration_b );

  id_pass1_part2( r, i );
}

/************************************************************************/
/************************************************************************/
/************************************************************************/

static void sum_child_forces( KIN_DYN *r, int i )
{
  int ic, c;

  v3_zero( r[i].joint_force_b );
  v3_zero( r[i].joint_torque_b );

  for ( ic = 0; ic < MAX_N_CHILDREN; ic++ )
    {
      c = r[i].children[ ic ];
      if ( c < 0 )
	break;
      /*
      /printf( "%d processing child: %d %d %g %g\n", i, c, r[c].joint_type,
	      r[c].cosine, r[c].sine );


      printf( "jfc: %g %g %g\n", r[c].joint_force_b[XX],
	      r[c].joint_force_b[YY],
	      r[c].joint_force_b[ZZ] );
      */

      switch( r[c].joint_type )
	{
	case REVOLUTE_X:
	  Rv_x_v_mult( r[c].cosine, r[c].sine, r[c].joint_force_b,
		       r[i].tmp_joint_force_b );
	  Rv_x_v_mult_acc( r[c].cosine, r[c].sine, r[c].joint_torque_b,
			   r[i].joint_torque_b );
	  break;
	case REVOLUTE_Y:
	  Rv_y_v_mult( r[c].cosine, r[c].sine, r[c].joint_force_b,
		       r[i].tmp_joint_force_b );
	  Rv_y_v_mult_acc( r[c].cosine, r[c].sine, r[c].joint_torque_b,
			   r[i].joint_torque_b );
	  break;
	case REVOLUTE_Z:
	  Rv_z_v_mult( r[c].cosine, r[c].sine, r[c].joint_force_b,
		       r[i].tmp_joint_force_b );
	  Rv_z_v_mult_acc( r[c].cosine, r[c].sine, r[c].joint_torque_b,
			   r[i].joint_torque_b );
	  break;
	case REVOLUTE_M_X:
	  Rv_x_v_mult( r[c].cosine, -r[c].sine, r[c].joint_force_b,
		       r[i].tmp_joint_force_b );
	  Rv_x_v_mult_acc( r[c].cosine, -r[c].sine, r[c].joint_torque_b,
			   r[i].joint_torque_b );
	  break;
	case REVOLUTE_M_Y:
	  Rv_y_v_mult( r[c].cosine, -r[c].sine, r[c].joint_force_b,
		       r[i].tmp_joint_force_b );
	  Rv_y_v_mult_acc( r[c].cosine, -r[c].sine, r[c].joint_torque_b,
			   r[i].joint_torque_b );
	  break;
	case REVOLUTE_M_Z:
	  Rv_z_v_mult( r[c].cosine, -r[c].sine, r[c].joint_force_b,
		       r[i].tmp_joint_force_b );
	  Rv_z_v_mult_acc( r[c].cosine, -r[c].sine, r[c].joint_torque_b,
			   r[i].joint_torque_b );
	  break;
	case REVOLUTE:
	  /* this was already done
	  generate_axis_angle_rotation_matrix( r[i].axis_b, r[i].cosine,
					       r[i].sine, r[c].rotation );
	  */
	  mv3_multiply( r[c].rotation, r[c].joint_force_b,
			 r[i].tmp_joint_force_b );
	  mv3_multiply_acc( r[c].rotation, r[c].joint_torque_b,
			     r[i].joint_torque_b );
	  break;
	default:
	  fprintf( stderr,
		   "Unknown joint(%d) type = %d in sum_child_forces2\n",
		   c, r[c].joint_type );
	  exit( -1 );
	}

      v3_acc( r[i].tmp_joint_force_b, r[i].joint_force_b );

      w_u_v_x_acc( r[c].joint_offset_b, r[i].tmp_joint_force_b,
		   r[i].joint_torque_b );
    }
}

/************************************************************************/

void id_pass2( KIN_DYN *r, int i )
{
  int ii;

  //printf( "\n" );

  sum_child_forces( r, i );

  /*
    printf( "tmp: %g %g %g\n", r[i].tmp_joint_force_b[XX],
    r[i].tmp_joint_force_b[YY],
    r[i].tmp_joint_force_b[ZZ] );
  */

  /*
    printf( "jf: %g %g %g\n", r[i].joint_force_b[XX],
    r[i].joint_force_b[YY],
    r[i].joint_force_b[ZZ] );
  */


  /*
    printf( "jt: %g %g %g\n", r[i].joint_torque_b[XX],
    r[i].joint_torque_b[YY],
    r[i].joint_torque_b[ZZ] );
  */
	
  /*
  printf( "jf: %g %g %g\n", r[i].joint_force_b[XX],
          r[i].joint_force_b[YY],
          r[i].joint_force_b[ZZ] );
  printf( "jt: %g %g %g\n", r[i].joint_torque_b[XX],
          r[i].joint_torque_b[YY],
          r[i].joint_torque_b[ZZ] );
  */

  /*
	printf( "la: %g %g %g\n", r[i].link_acceleration_b[XX],
          r[i].link_acceleration_b[YY],
          r[i].link_acceleration_b[ZZ] );
	*/
  // m*qdd                                                                    
  v3_scale( r[i].mass, r[i].link_acceleration_b, r[i].link_force_b );
  // add I*wdot to joint F/T                                                  
  // add w x I*w to joint F/T                                                 
  if ( r[i].moi_diagonal )
    {
      duv_by_element( r[i].moi_v, r[i].angular_acceleration_b,
		      r[i].link_torque_b );
      duv_by_element( r[i].moi_v, r[i].angular_velocity_b, r[i].Iw_b );
    }
  else
    {
      mv3_multiply( r[i].moi, r[i].angular_acceleration_b,
		    r[i].link_torque_b );
      mv3_multiply( r[i].moi, r[i].angular_velocity_b, r[i].Iw_b );
    }
  w_x_acc( r[i].angular_velocity_b, r[i].Iw_b, r[i].link_torque_b );

  v3_acc( r[i].link_force_b, r[i].joint_force_b );
  v3_acc( r[i].link_torque_b, r[i].joint_torque_b );
  w_u_v_x_acc( r[i].com_offset_b, r[i].link_force_b, r[i].joint_torque_b );

  r[i].the_joint_torque = 0;
  if ( r[i].joint_type >= REVOLUTE &&
       r[i].joint_type <= REVOLUTE_M_Z )
    for ( ii = 0; ii < 3; ii++ )
      r[i].the_joint_torque += r[i].axis_b[ii] * r[i].joint_torque_b[ii];

	/*
  printf( "jf[%d]: %g %g %g\n", i, r[i].joint_force_b[XX],
          r[i].joint_force_b[YY], r[i].joint_force_b[ZZ] );

  printf( "jt[%d]: %g %g %g; %g\n", i, r[i].joint_torque_b[XX],
          r[i].joint_torque_b[YY], r[i].joint_torque_b[ZZ],
	  r[i].the_joint_torque );
	*/
}

/**********************************/

void id_pass2_no_sum( KIN_DYN *r, int i )
{
  int ii;

  //printf( "\n" );

  /*
    printf( "tmp: %g %g %g\n", r[i].tmp_joint_force_b[XX],
    r[i].tmp_joint_force_b[YY],
    r[i].tmp_joint_force_b[ZZ] );
  */

  /*
    printf( "jf: %g %g %g\n", r[i].joint_force_b[XX],
    r[i].joint_force_b[YY],
    r[i].joint_force_b[ZZ] );
  */


  /*
    printf( "jt: %g %g %g\n", r[i].joint_torque_b[XX],
    r[i].joint_torque_b[YY],
    r[i].joint_torque_b[ZZ] );
  */
	
  /*
  printf( "jf: %g %g %g\n", r[i].joint_force_b[XX],
          r[i].joint_force_b[YY],
          r[i].joint_force_b[ZZ] );
  printf( "jt: %g %g %g\n", r[i].joint_torque_b[XX],
          r[i].joint_torque_b[YY],
          r[i].joint_torque_b[ZZ] );
  */

  /*
	printf( "la: %g %g %g\n", r[i].link_acceleration_b[XX],
          r[i].link_acceleration_b[YY],
          r[i].link_acceleration_b[ZZ] );
	*/
  // m*qdd                                                                    
  v3_scale( r[i].mass, r[i].link_acceleration_b, r[i].link_force_b );
  // add I*wdot to joint F/T                                                  
  // add w x I*w to joint F/T                                                 
  if ( r[i].moi_diagonal )
    {
      duv_by_element( r[i].moi_v, r[i].angular_acceleration_b,
		      r[i].link_torque_b );
      duv_by_element( r[i].moi_v, r[i].angular_velocity_b, r[i].Iw_b );
    }
  else
    {
      mv3_multiply( r[i].moi, r[i].angular_acceleration_b,
		    r[i].link_torque_b );
      mv3_multiply( r[i].moi, r[i].angular_velocity_b, r[i].Iw_b );
    }
  w_x_acc( r[i].angular_velocity_b, r[i].Iw_b, r[i].link_torque_b );

  v3_acc( r[i].link_force_b, r[i].joint_force_b );
  v3_acc( r[i].link_torque_b, r[i].joint_torque_b );
  w_u_v_x_acc( r[i].com_offset_b, r[i].link_force_b, r[i].joint_torque_b );

  r[i].the_joint_torque = 0;
  if ( r[i].joint_type >= REVOLUTE &&
       r[i].joint_type <= REVOLUTE_M_Z )
    for ( ii = 0; ii < 3; ii++ )
      r[i].the_joint_torque += r[i].axis_b[ii] * r[i].joint_torque_b[ii];

	/*
  printf( "jf[%d]: %g %g %g\n", i, r[i].joint_force_b[XX],
          r[i].joint_force_b[YY], r[i].joint_force_b[ZZ] );

  printf( "jt[%d]: %g %g %g; %g\n", i, r[i].joint_torque_b[XX],
          r[i].joint_torque_b[YY], r[i].joint_torque_b[ZZ],
	  r[i].the_joint_torque );
	*/
}


/************************************************************************/

void id_pass2_null_link( KIN_DYN *r, int i )
{
  int ic, c;
  int ii;

  //printf( "\n" );

  sum_child_forces( r, i );

  /*
    printf( "tmp: %g %g %g\n", r[i].tmp_joint_force_b[XX],
    r[i].tmp_joint_force_b[YY],
    r[i].tmp_joint_force_b[ZZ] );
  */

  /*
    printf( "jf: %g %g %g\n", r[i].joint_force_b[XX],
    r[i].joint_force_b[YY],
    r[i].joint_force_b[ZZ] );
  */

  /*
    printf( "jt: %g %g %g\n", r[i].joint_torque_b[XX],
    r[i].joint_torque_b[YY],
    r[i].joint_torque_b[ZZ] );
  */
	
  /*
  printf( "jf: %g %g %g\n", r[i].joint_force_b[XX],
          r[i].joint_force_b[YY],
          r[i].joint_force_b[ZZ] );
  printf( "jt: %g %g %g\n", r[i].joint_torque_b[XX],
          r[i].joint_torque_b[YY],
          r[i].joint_torque_b[ZZ] );


  printf( "la: %g %g %g\n", r[i].link_acceleration_b[XX],
          r[i].link_acceleration_b[YY],
          r[i].link_acceleration_b[ZZ] );
  */

  r[i].the_joint_torque = 0;
  if ( r[i].joint_type >= REVOLUTE &&
       r[i].joint_type <= REVOLUTE_M_Z )
    for ( ii = 0; ii < 3; ii++ )
      r[i].the_joint_torque += r[i].axis_b[ii] * r[i].joint_torque_b[ii];

	/*  
			printf( "jf[%d]: %g %g %g\n", i, r[i].joint_force_b[XX],
          r[i].joint_force_b[YY], r[i].joint_force_b[ZZ] );

  printf( "jt[%d]: %g %g %g; %g\n", i, r[i].joint_torque_b[XX],
          r[i].joint_torque_b[YY], r[i].joint_torque_b[ZZ],
	  r[i].the_joint_torque );
	*/
}

/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/


#ifdef COMMENT
/************************************************************************/
// Assume trees given in reverse order in indices array
// forces and torques expressed in world coordinates.
// if you want actual joint torques, you need to rotate these vectors
// into body coordinates. Base is in world coordinates. Or just penalize
// all directions in world coordinates.

void propagate_forces( int n_links, int *indices,
			int children[][MAX_N_CHILDREN+1] )
{
  int index, child_index;

  for ( index = 0; index < n_links; index++ )
    {
      // zero joint forces and torques
      v3_zero( joint_force_w[ index ] );
      v3_zero( joint_torque_w[ index ] );

      // sum over all children
      for ( child_index = 0;
	    children[ index ][ child_index ] != 0;
	    child_index++ )
	{
	  // aggregate forces and torques in joint F/T
	  v3_acc( joint_force_w[ child_index], joint_force_w[ index ] );
	  vxv3_acc( R_offset[ child_index ], joint_force_w[ child_index ],
		    joint_torque_w[ index ] );
	  v3_acc( joint_torque_w[ child_index], joint_torque_w[ index ] );
	}

      // add in gravity
      joint_force_w[ index ][ ZZ ] += G*masses[i];
      joint_torque_w[ index ][ XX ] += -R_com[i][YY]*G*masses[i];
      joint_torque_w[ index ][ YY ] += R_com[i][XX]*G*masses[i];

      mtv3_multiply( orientation[ index ], joint_force_w[ index ],
		     joint_force_b[ index ] );
      mtv3_multiply( orientation[ index ], joint_torque_w[ index ],
		     joint_torque_b[ index ] );
    }
}

/************************************************************************/
// Figure out how each joint moves j_position.

void compute_jacobian( int n_links, int *indices, double *j_position,
		       double j_matrix[][6] )
{
  int index;

  for ( index = 0; index < n_links; index++ )
    {

      mtv3_multiply( orientation[ index ], axis[ index ], axis_w[ index ] );
      v3_copy( axis_w[ index ], &(j_matrix[ index ][3] );
      v3_subtract( j_position, position[ index ], j_offset[ index ] );
      vxv3( axis_w[ index ], j_offset[ index ], &(j_matrix[ index ][0] ) );
    }
}

/************************************************************************/
/************************************************************************/

#endif

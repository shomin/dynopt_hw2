#include "atlas.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
	if(nrhs!=3) {
		mexErrMsgIdAndTxt("kindyn:nrhs","3 inputs required: angles[28], anglesd[28], anglesdd[28] ");
	}

  int j, i;
  KIN_DYN r[ N_LINKS ];
  KIN_DYN lleg[ LLEG_LINKS ];
  KIN_DYN rleg[ RLEG_LINKS ];
  double q[4];
  double g_vector[3] = { 0.0, 0.0, 9.81 };
  double root_force_w[3];
  double root_torque_w[3];
  double v1[3];
  double w[3];

  init_kin_dyn();

  init_robot( r );
  init_lleg( lleg );
  init_rleg( rleg );


	double *input_angle;
	double *input_angled;
	double *input_angledd;
	input_angle = mxGetPr(prhs[0]);
	input_angled = mxGetPr(prhs[1]);
	input_angledd = mxGetPr(prhs[2]);

	/*
	double *pelvis_pos;
	double *pelvis_quat;
	pelvis_pos = mxGetPr(prhs[3]);
	pelvis_quat = mxGetPr(prhs[4]);
	*/

  for ( i = 0; i < 3; i++ )
    {
      r[0].position_w[i] = 0;
      r[0].angular_velocity_b[i] = 0;
      r[0].angular_acceleration_b[i] = 0;
      r[0].joint_acceleration_b[i] = 0;
      r[0].link_acceleration_b[i] = 0;
    }

  // default state for rest of robot
  for ( i = 0; i < N_LINKS; i++ )
    {
      r[i].angle = input_angle[i];
      r[i].angled = input_angled[i];
      r[i].angledd = input_angledd[i];
    }
  // set up ground joint
  //r[0].position_w[XX] = -0.0484;
  //r[0].position_w[YY] = 0;
  //r[0].position_w[ZZ] = 0.9271;


	// Put left foot on the ground and go up to find the pelvis
	lleg[0].angle = 0;
  lleg[0].angle = 0;
  lleg[0].sine = 0;
  lleg[0].cosine = 1;
  m3_identity( lleg[0].orientation );
  lleg[0].angled = 0;
  lleg[0].angledd = 0;
  lleg[0].position_w[XX] = 0;
  lleg[0].position_w[YY] = 0.089;
  lleg[0].position_w[ZZ] = 0;

	for(i=1; i < LLEG_JOINTS; i++){
		lleg[i].angle = r[11-i].angle;
		lleg[i].angled = r[11-i].angled;
		lleg[i].angled = r[11-i].angled;
		lleg[i].sine = sin( lleg[i].angle );
		lleg[i].cosine = cos( lleg[i].angle );
	}
	//FK
	forward_kinematics(lleg, LLEG_LINKS);

	double pelvis_offset[3];
	pelvis_offset[0] = 0;
	pelvis_offset[1] = -0.089;
	pelvis_offset[2] = 0;

	mv3_multiply(lleg[6].orientation,pelvis_offset,r[0].position_w);

	for(i=0;i<3;i++)
		r[0].position_w[i] += lleg[6].position_w[i];

	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			r[0].orientation[i][j] = lleg[6].orientation[i][j];

  r[0].angle = 0;
  r[0].sine = 0;
  r[0].cosine = 1;


  for ( i = 1; i < N_LINKS; i++ ){
		r[i].sine = sin( r[i].angle );
		r[i].cosine = cos( r[i].angle );
	}
  
  forward_kinematics( r, N_LINKS );

  //printf( "COM\n" );
	double total_com[3];
	double total_mass;
	total_com[XX] = 0;	total_com[YY] = 0;	total_com[ZZ] = 0;
	total_mass = 0;

  for ( i = 0; i < N_LINKS; i++ ){
    //printf( "%d: %g %g %g\n", i, r[i].com_w[XX], r[i].com_w[YY],r[i].com_w[ZZ] );
		total_com[XX] += (r[i].mass*r[i].com_w[XX]);
		total_com[YY] += (r[i].mass*r[i].com_w[YY]);
		total_com[ZZ] += (r[i].mass*r[i].com_w[ZZ]);
		total_mass += r[i].mass;
	}

	total_com[XX] = total_com[XX]/total_mass;
	total_com[YY] = total_com[YY]/total_mass;
	total_com[ZZ] = total_com[ZZ]/total_mass;

	/*
  for ( i = 0; i < N_LINKS; i++ )
    {
      printf( "%d: %g %g %g\n", i, r[i].orientation[0][0],
	      r[i].orientation[0][1], r[i].orientation[0][2] );
      printf( "    %g %g %g\n", r[i].orientation[1][0],
	      r[i].orientation[1][1], r[i].orientation[1][2] );
      printf( "    %g %g %g\n", r[i].orientation[2][0],
	      r[i].orientation[2][1], r[i].orientation[2][2] );
    }
	*/

  // implement gravity
  mtv3_multiply( r[0].orientation, g_vector, r[0].joint_acceleration_b );
  v3_copy( r[0].joint_acceleration_b, r[0].link_acceleration_b );

  inverse_dynamics( r );
	/*
  printf( "Angular Velocity\n" );
  for ( i = 0; i < N_LINKS; i++ )
    {
      v3_copy( r[i].angular_velocity_b, v1 );
      // mv3_multiply( r[i].orientation, r[i].angular_velocity_b, v1 );
      printf( "%d: %g %g %g\n", i, v1[XX], v1[YY], v1[ZZ] );
    }

  printf( "Angular Acceleration\n" );
  for ( i = 0; i < N_LINKS; i++ )
    {
      v3_copy( r[i].angular_acceleration_b, v1 );
      // mv3_multiply( r[i].orientation, r[i].angular_acceleration_b, v1 );
      printf( "%d: %g %g %g\n", i, v1[XX], v1[YY], v1[ZZ] );
    }
	*/
  mv3_multiply( r[0].orientation, r[0].joint_force_b, root_force_w );
  mv3_multiply( r[0].orientation, r[0].joint_torque_b, root_torque_w );

	/*
  printf( "joint_force: %g %g %g\n", r[0].joint_force_b[0], r[0].joint_force_b[1],
	  r[0].joint_force_b[2] );
  printf( "joint torque: %g %g %g\n", r[0].joint_torque_b[0], r[0].joint_torque_b[1],
	  r[0].joint_torque_b[2] );

  printf( "root_force: %g %g %g\n", root_force_w[0], root_force_w[1],
	  root_force_w[2] );
  printf( "root torque: %g %g %g\n", root_torque_w[0], root_torque_w[1],
	  root_torque_w[2] );
	*/
	/*
  printf( "Torques\n" );
  for ( j = 1; j < N_JOINTS; j++ )
    printf( "%d: %g\n", j, r[j].the_joint_torque );
	*/

	// Each leg inv dyn w/ added pelvis forces

	//r/l leg:
	//0 = ground
	//6 = l_leg_uhz

	//rleg:
	//11 = r_leg_uhz
	//16 = r_leg_lax 

	//body:
	//5  = l_leg_uhz
	//10 = l_leg_lax

	
	lleg[0].angle = 0;
  lleg[0].angle = 0;
  lleg[0].sine = 0;
  lleg[0].cosine = 1;
  m3_identity( lleg[0].orientation );
  lleg[0].angled = 0;
  lleg[0].angledd = 0;
  lleg[0].position_w[YY] = 0.089;
	for(i=0;i<3;i++){
		lleg[0].angular_velocity_b[i] = 0.0;
		lleg[0].angular_acceleration_b[i] = 0.0;
		lleg[0].joint_acceleration_b[i] = 0.0;
		lleg[0].link_acceleration_b[i] = 0.0;
	}

	for(i=1; i < LLEG_JOINTS; i++){
		lleg[i].angle = r[11-i].angle;
		lleg[i].angled = r[11-i].angled;
		lleg[i].angled = r[11-i].angled;
		lleg[i].sine = sin( lleg[i].angle );
		lleg[i].cosine = cos( lleg[i].angle );
	}

	rleg[0].angle = 0;
  rleg[0].angle = 0;
  rleg[0].sine = 0;
  rleg[0].cosine = 1;
  m3_identity( rleg[0].orientation );
  rleg[0].angled = 0;
  rleg[0].angledd = 0;
  rleg[0].position_w[YY] = -0.089;
	for(i=0;i<3;i++){
		rleg[0].angular_velocity_b[i] = 0.0;
		rleg[0].angular_acceleration_b[i] = 0.0;
		rleg[0].joint_acceleration_b[i] = 0.0;
		rleg[0].link_acceleration_b[i] = 0.0;
	}

	for(i=1; i < RLEG_JOINTS; i++){
		rleg[i].angle = r[17-i].angle;
		rleg[i].angled = r[17-i].angled;
		rleg[i].angled = r[17-i].angled;
		rleg[i].sine = sin( rleg[i].angle );
		rleg[i].cosine = cos( rleg[i].angle );
	}

	//FK
	forward_kinematics(lleg, LLEG_LINKS);
	forward_kinematics(rleg, RLEG_LINKS);

	/*
	printf("\nlleg[6].position:\n %f %f %f\n\n",lleg[6].position_w[0],lleg[6].position_w[1],lleg[6].position_w[2]);

	printf("lleg[6].orientation:\n%f %f %f\n%f %f %f\n%f %f %f\n",lleg[6].orientation[0][0],lleg[6].orientation[0][1],lleg[6].orientation[0][2],lleg[6].orientation[1][0],lleg[6].orientation[1][1],lleg[6].orientation[1][2],lleg[6].orientation[2][0],lleg[6].orientation[2][1],lleg[6].orientation[2][2]);

	printf("\nrleg[6].position:\n %f %f %f\n\n",rleg[6].position_w[0],rleg[6].position_w[1],rleg[6].position_w[2]);

	printf("rleg[6].orientation:\n%f %f %f\n%f %f %f\n%f %f %f\n\n",rleg[6].orientation[0][0],rleg[6].orientation[0][1],rleg[6].orientation[0][2],rleg[6].orientation[1][0],rleg[6].orientation[1][1],rleg[6].orientation[1][2],rleg[6].orientation[2][0],rleg[6].orientation[2][1],rleg[6].orientation[2][2]);
	*/

	
	// put force on the pelvis - for now we'll just try an equal split
	// no gravity because these legs are massless

	double split_force[3];
	double split_torque[3];
	for (i=0;i<3;i++){
		split_force[i] = root_force_w[i]/2.0;
		split_torque[i] = root_torque_w[i]/2.0;
	}

	lleg[0].link_acceleration_b[ZZ] = 9.81;
	lleg[0].joint_acceleration_b[ZZ] = 9.81;

	rleg[0].link_acceleration_b[ZZ] = 9.81;
	rleg[0].joint_acceleration_b[ZZ] = 9.81;

	
	lleg_id(lleg,split_force,split_torque);
	rleg_id(rleg,split_force,split_torque);

	for(i=1; i < RLEG_JOINTS; i++){
		r[17-i].the_joint_torque = rleg[i].the_joint_torque;
	}

	for(i=1; i < LLEG_JOINTS; i++){
		r[11-i].the_joint_torque = lleg[i].the_joint_torque;
	}



	
	double *output_torques;
	plhs[0] = mxCreateDoubleMatrix(1,N_LINKS,mxREAL);
	output_torques = mxGetPr(plhs[0]);

	for (i=0; i<N_LINKS;i++){
		output_torques[i] = r[i].the_joint_torque;
	}

	double *output_pos;
	plhs[1] = mxCreateDoubleMatrix(3,N_LINKS,mxREAL);
	output_pos = mxGetPr(plhs[1]);	

	for (i=0; i<N_LINKS;i++){
		for (j=0; j<3; j++){
			output_pos[(3*i)+j] = r[i].position_w[j];
		}
	}

	double *output_l_force;
	plhs[2] = mxCreateDoubleMatrix(1,3,mxREAL);
	output_l_force = mxGetPr(plhs[2]);	
	mv3_multiply( lleg[0].orientation, lleg[0].joint_force_b, output_l_force );

	double *output_l_torque;
	plhs[3] = mxCreateDoubleMatrix(1,3,mxREAL);
	output_l_torque = mxGetPr(plhs[3]);	
	mv3_multiply( lleg[0].orientation, lleg[0].joint_torque_b, output_l_torque );


	double *output_r_force;
	plhs[4] = mxCreateDoubleMatrix(1,3,mxREAL);
	output_r_force = mxGetPr(plhs[4]);	
	mv3_multiply( rleg[0].orientation, rleg[0].joint_force_b, output_r_force );

	double *output_r_torque;
	plhs[5] = mxCreateDoubleMatrix(1,3,mxREAL);
	output_r_torque = mxGetPr(plhs[5]);	
	mv3_multiply( rleg[0].orientation, rleg[0].joint_torque_b, output_r_torque );

	/*
	double *l_torques;
	plhs[6] = mxCreateDoubleMatrix(1,7,mxREAL);
	l_torques = mxGetPr(plhs[6]);	
	for (i=0; i < LLEG_JOINTS ;i++){
			l_torques[i] = lleg[i].the_joint_torque;
	}

	double *r_torques;
	plhs[7] = mxCreateDoubleMatrix(1,7,mxREAL);
	r_torques = mxGetPr(plhs[7]);	
	for (i=0; i < RLEG_JOINTS ;i++){
			r_torques[i] = rleg[i].the_joint_torque;
	}
	*/
	double *r_orientation;
	plhs[6] = mxCreateDoubleMatrix(3,3,mxREAL);
	r_orientation = mxGetPr(plhs[6]);	
	for (i=0;i<3;i++){
		for(j=0;j<3;j++){
			r_orientation[(3*i)+j] = r[16].orientation[i][j];
		}
	}

	double *l_orientation;
	plhs[7] = mxCreateDoubleMatrix(3,3,mxREAL);
	l_orientation = mxGetPr(plhs[7]);	
	for (i=0;i<3;i++){
		for(j=0;j<3;j++){
			l_orientation[(3*i)+j] = r[10].orientation[i][j];
		}
	}

	double *com_ret;
	plhs[8] = mxCreateDoubleMatrix(1,3,mxREAL);
	com_ret = mxGetPr(plhs[8]);	
	for(j=0;j<3;j++){
		com_ret[j] = total_com[j];
	}


	//  return 0;
}


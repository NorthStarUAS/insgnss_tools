/*! \file	matrix.h
 *	\brief	matrix mathematics header file
 *
 *	\details
 *	by:	    ko shu pui, patrick
 *	date:	24 nov 91	v0.1b
 *	revi:   Rodney Teo and Jung Soon Jang
 *	ref:
 *       [1] Mary L.Boas, "Mathematical Methods in the Physical Sciene,"
 *	John Wiley & Sons, 2nd Ed., 1983. Chap 3.
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: matrix.h 752 2011-12-21 20:14:23Z murch $
 */

#ifndef MATRIX_H_
#define MATRIX_H_
#include <stdio.h>
/*
*-----------------------------------------------------------------------------
*	internal matrix structure
*-----------------------------------------------------------------------------
*/

typedef struct {
	int	row;
	int	col;
	}	MATHEAD;

typedef struct {
	MATHEAD	head;
	/*
	* only the starting address of the following will be
	* returned to the C programmer, like malloc() concept
	*/
	double	*matrix; // double
	}	MATBODY;

typedef	double	**MATRIX;// double

#define	Mathead(a)	((MATHEAD *)((MATHEAD *)(a) - 1))
#define MatRow(a)	(Mathead(a)->row)
#define	MatCol(a)	(Mathead(a)->col)

/*
*----------------------------------------------------------------------------
*	mat_errors definitions
*----------------------------------------------------------------------------
*/
#define	MAT_MALLOC	1
#define MAT_FNOTOPEN	2
#define	MAT_FNOTGETMAT	3

/*
*----------------------------------------------------------------------------
*	matrice types
*----------------------------------------------------------------------------
*/
#define UNDEFINED	-1
#define ZERO_MATRIX	0
#define	UNIT_MATRIX	1
#define	ONES_MATRIX	2


/* prototypes of matrix package */

//MATRIX mat_error	(int);
MATRIX _mat_creat	(int, int);
MATRIX mat_creat	(int, int, int);
MATRIX mat_fill		(MATRIX, int);
int mat_free		(MATRIX);
MATRIX mat_copy		(MATRIX, MATRIX);
MATRIX mat_copy1	(MATRIX, MATRIX);
MATRIX mat_colcopy1	(MATRIX, MATRIX, int, int);
int fgetmat		(MATRIX, FILE *fp);
MATRIX mat_dump     (MATRIX);
MATRIX mat_dumpf    (MATRIX, char *);
MATRIX mat_fdump	(MATRIX, FILE *fp);
MATRIX mat_fdumpf   (MATRIX, char *, FILE *fp);
MATRIX mat_add		(MATRIX, MATRIX, MATRIX);
MATRIX mat_sub		(MATRIX, MATRIX, MATRIX);
MATRIX mat_mul		(MATRIX, MATRIX,MATRIX);
MATRIX mat_transmul	(MATRIX A,MATRIX B, MATRIX C);
MATRIX mat_mymul	(MATRIX A,MATRIX B, MATRIX C, short m);
MATRIX mat_mymul1	(MATRIX A,MATRIX B, MATRIX C, short m);
MATRIX mat_mymul2	(MATRIX A,MATRIX B, MATRIX C, short m);
MATRIX mat_mymul3	(MATRIX A,MATRIX B, MATRIX C, short m);
MATRIX mat_mymul4	(MATRIX A,MATRIX B, MATRIX C, short m);
MATRIX mat_mymul5	(MATRIX A,MATRIX B, MATRIX C, short m);
double mat_diagmul	(MATRIX);
MATRIX mat_tran		(MATRIX, MATRIX);
MATRIX mat_inv		(MATRIX,MATRIX);
MATRIX mat_SymToeplz	(MATRIX, MATRIX);
int mat_lu		(	MATRIX, MATRIX);
MATRIX mat_backsubs1	(MATRIX, MATRIX, MATRIX, MATRIX, int);
MATRIX mat_lsolve	(MATRIX, MATRIX,MATRIX);
MATRIX mat_submat	(MATRIX, int, int, MATRIX);
double mat_cofact	(MATRIX, int, int);
double mat_det		(MATRIX);
double mat_minor	(MATRIX, int, int);
MATRIX mat_durbin       (MATRIX,MATRIX);
MATRIX mat_lsolve_durbin(MATRIX, MATRIX,MATRIX);
double mat_norm (MATRIX X, int column);
MATRIX mat_subcopy(MATRIX A, int row, int col, MATRIX B);
MATRIX mat_T321 (double pitch, double roll, double yaw, MATRIX);
MATRIX mat_round (MATRIX X, MATRIX C);
double mat_dot (MATRIX X, MATRIX Y);
MATRIX mat_scalMult (MATRIX X,double A, MATRIX C);
MATRIX mat_scalMul(MATRIX X,double A, MATRIX C);
#endif


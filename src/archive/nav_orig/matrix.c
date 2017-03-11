/*! \file	matrix.c
 *	\brief	matrix mathematics source file
 *
 *	\details
 *
 *	 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: matrix.c 929 2012-10-29 16:50:59Z joh07594 $
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include "matrix.h"


/////////////////////////////////// comment out for SIL sim
#include <unistd.h>
#include <termios.h>
//////////////////////////////////////////////////////////////

/*
*-----------------------------------------------------------------------------
*	MATRIX FUNCTIONS
*	For those functions which return MATRIX values, the correct
* 	size has to be created before calling the function and the
*	address of which has to be passed in as usually the last
*	variable of the function. The result will be accessible through
*	last variable and through the returned value. Always use the
*	same variable as the last input variable to be assigned to this
*	returned value. eg. C = mat_add( A, B, C ); This prevents confusion.
*
*	These functions are written way so that:
*	i. equations involving a few of these functions can be written in
*		one line
*	ii. malloc does not have to be called within functions and freed outside
*		functions because the result has to be "returned".
*
*	Note: All matrix functions don't create memory without clearing it
* 			EXCEPT mat_creat(). Every use of mat_creat MUST therefore
*			be followed by mat_free()
*
*
*-----------------------------------------------------------------------------
*/

/*
*-----------------------------------------------------------------------------
*	funct:	mat_add
*	desct:	addition of two matrices
*	given:	A, B = Compatible matrice to be added
*   returns: C which has to be right dimensions
*	comen:
*-----------------------------------------------------------------------------
*/
MATRIX mat_add(MATRIX A, MATRIX B, MATRIX C)
{
	int		i, j;

	// if dimensions of C is wrong
	//if ( MatRow(C) != MatRow(A) || MatCol(C) != MatCol(B) ) {
	//	printf("mat_add error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {
		for (i=0; i<MatRow(A); i++) {
			for (j=0; j<MatCol(A); j++) {
				C[i][j] = A[i][j] + B[i][j];
			}
		}
	//}
	return(C);
}


MATRIX _mat_creat(int row, int col)
{
	MATBODY	*mat;
	int 	i;

	if ((mat = (MATBODY *)malloc( sizeof(MATHEAD) + sizeof(double *) * row)) == NULL){}
	//	return (mat_error( MAT_MALLOC ));

	for (i=0; i<row; i++)
	{
	if ((*((double **)(&mat->matrix) + i) = (double *)malloc(sizeof(double) * col)) == NULL){}
	//	return (mat_error( MAT_MALLOC ));
	}

	mat->head.row = row;
	mat->head.col = col;

	return (&(mat->matrix));
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_creat
*	desct:	create a matrix
*	given:  row, col = dimension, type = which kind of matrix
*	retrn:	allocated matrix (use mat_free() to free memory)
*-----------------------------------------------------------------------------
*/
MATRIX	mat_creat(int row,int col,int type)
{
	MATRIX	A;

	if(row==0 || col==0)
		return(NULL);

	if ((A =_mat_creat( row, col )) != NULL)
		{
		return (mat_fill(A, type));
		}
	else
		return (NULL);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_fill
*	desct:	form a special matrix
*	given:  A = matrix, type = which kind of matrix
*	retrn:	A
*-----------------------------------------------------------------------------
*/
MATRIX mat_fill(MATRIX A,int type )
{
	int		i, j;

	switch (type) {
		case UNDEFINED:
			break;
		case ONES_MATRIX:
		case ZERO_MATRIX:
		case UNIT_MATRIX:
			for (i=0; i<MatRow(A); i++)
			for (j=0; j<MatCol(A); j++)
				{
				if (type == ONES_MATRIX)
					{
					A[i][j] = 1.0;
					continue;
					}
				if (type == UNIT_MATRIX)
					{
					if (i==j)
						{
						A[i][j] = 1.0;
						continue;
						}
					}
				A[i][j] = 0.0;
				}
			break;
	}
	return (A);
}


/*
*-----------------------------------------------------------------------------
*	funct:	mat_free
*	desct:	free an allocated matrix
*	given:  A = matrix
*	retrn:	nothing <actually 0 = NULL A passed, 1 = normal exit>
*-----------------------------------------------------------------------------
*/
int mat_free(MATRIX A)
{
	int i;

	if (A == NULL)
		return (0);
	for (i=0; i<MatRow(A); i++)
		{
		free( A[i] );
		}
	free( Mathead(A) );
	return (1);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_copy
*	desct:	duplicate a matrix
*	given:	A = matrice to duplicated
*	retrn:	C = A
*	comen:
*-----------------------------------------------------------------------------
*/
MATRIX mat_copy(MATRIX A, MATRIX C)
{
	int		i, j;

	// if dimensions of C is wrong
	//if ( MatRow(A) != MatRow(C) || MatCol(A) != MatCol(C) ) {
	//	printf("mat_copy error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {
		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatCol(A); j++) {
			C[i][j] = A[i][j];
		}
	//}
	return(C);
}

MATRIX mat_copy1(MATRIX A, MATRIX C)
{
	int		i, j;

	// if dimensions of C is wrong
	//if ( MatRow(A) != MatRow(C) || MatCol(A) != MatCol(C) ) {
	//	printf("mat_copy error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {
		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatCol(A); j++) {
			C[i][j] = -A[i][j];
		}
	//}
	return(C);
}


MATRIX mat_colcopy1(MATRIX A,MATRIX B,int cola,int colb)
{
	int	i, n;

	n = MatRow(A);
	for (i=0; i<n; i++)
		{
		A[i][cola] = B[i][colb];
		}
	return (A);
}

int fgetmat(MATRIX A, FILE *fp )
{
	int 	i, j, k=0;

	for (i=0; i<MatRow(A); i++)
	for (j=0; j<MatCol(A); j++)
		{
/*
*	to avoid a bug in TC
*/
#ifdef	__TURBOC__
		{
		double	temp;
		k += fscanf( fp, "%lf", &temp );
		A[i][j] = temp;
		}
#else
		k += fscanf( fp, "%lf", &A[i][j] );
#endif

		}

	return (k);
}
/*
*-----------------------------------------------------------------------------
*	file:	matdet.c
*	desc:	determinant calculations
*	by:	ko shu pui, patrick
*	date:	21 may 92 v0.3
*	revi:
*	ref:
*       [1] Mary L.Boas, "Mathematical Methods in the Physical Sciene,"
*	John Wiley & Sons, 2nd Ed., 1983. Chap 3.
*
*-----------------------------------------------------------------------------
*/

static double signa[2] = {1.0, -1.0};

/*
*-----------------------------------------------------------------------------
*	funct:	mat_minor
*	desct:	find minor
*	given:	A = a square matrix,
*		i=row, j=col
*	retrn:	the minor of Aij
*-----------------------------------------------------------------------------
*/
double mat_minor(MATRIX A,int i,int j)
{
	MATRIX	S;
	double	result;

	if ( ( S = mat_creat(MatRow(A)-1, MatCol(A)-1, UNDEFINED) ) == NULL )
		return -1.0;
	mat_submat(A, i, j, S);
	result = mat_det( S );
	mat_free(S);

	return (result);

}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_cofact
*	desct:	find cofactor
*	given:	A = a square matrix,
*		i=row, j=col
*	retrn:	the cofactor of Aij
*-----------------------------------------------------------------------------
*/
double mat_cofact(MATRIX A,int i,int j)
{
	double	result;

	result = signa[(i+j)%2] * A[i][j] * mat_minor(A, i, j);

	return (result);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_det
*	desct:	find determinant
*	given:	A = matrix
*	retrn:	the determinant of A
*	comen:
*-----------------------------------------------------------------------------
*/
double mat_det(MATRIX a)
{
	MATRIX	A, P;
	int	i, j, n;
	double	result;

	n = MatRow(a);
	if ( (	A = mat_creat(n, n, UNDEFINED) ) == NULL )
		return -1.0;
	mat_copy(a, A);
	if ( (	P = mat_creat(n, 1, UNDEFINED) ) == NULL )
		return -1.0;


	/*
	* take a LUP-decomposition
	*/
	i = mat_lu(A, P);
	switch (i)
		{
		/*
		* case for singular matrix
		*/
		case -1:
		result = 0.0;
		break;

		/*
		* normal case: |A| = |L||U||P|
		* |L| = 1,
		* |U| = multiplication of the diagonal
		* |P| = +-1
		*/
		default:
		result = 1.0;
		for (j=0; j<MatRow(A); j++)
			{
            result *= A[(int)P[j][0]][j];
			}
		result *= signa[i%2];
		break;
		}

	mat_free(A);
	mat_free(P);
	return (result);
}
/*
*-----------------------------------------------------------------------------
*	funct:	mat_dump
*	desct:	dump a matrix
*	given:	A = matrice to dumped
*	retrn:	nothing
*	comen:	matrix a dumped to standard output
*-----------------------------------------------------------------------------
*/
MATRIX mat_dump( MATRIX A )
//MATRIX A;
{
	if (A==NULL)
		return(NULL);
	else
    	return(mat_fdumpf(A, "%f ", stdout));
}


/*
*-----------------------------------------------------------------------------
*	funct:	mat_dumpf
*   desct:  dump a matrix with format string to standard output
*	given:	A = matrice to dumped
*	retrn:	nothing
*	comen:	matrix a dumped to standard output
*-----------------------------------------------------------------------------
*/
MATRIX mat_dumpf(MATRIX A,char *s)
{
    return (mat_fdumpf(A, s, stdout));
}

MATRIX mat_fdump(MATRIX A,FILE *fp)
{
    return (mat_fdumpf(A, "%f ", fp));
}

MATRIX mat_fdumpf(MATRIX A,char *s,FILE *fp)
{
	int	i, j;

	for (i=0; i<MatRow(A); i++)
		{
		for (j=0; j<MatCol(A); j++)
			{
            fprintf( fp, s, A[i][j] );
			}
        fprintf( fp, "\n" );
		}

	return (A);
}


/*
*-----------------------------------------------------------------------------
*	funct:	mat_durbin
*	desct:	Levinson-Durbin algorithm
*
*		This function solve the linear eqns Ax = B:
*
*		|  v0   v1   v2  .. vn-1 | |  a1   |    |  v1   |
*		|  v1   v0   v1  .. vn-2 | |  a2   |    |  v2   |
*		|  v2   v1   v0  .. vn-3 | |  a3   |  = |  ..   |
*		|  ...                   | |  ..   |    |  ..   |
*		|  vn-1 vn-2 ..  .. v0   | |  an   |    |  vn   |
*
*		where A is a symmetric Toeplitz matrix and B
*		in the above format (related to A)
*
*	given:	R = autocorrelated matrix (v0, v1, ... vn) (dim (n+1) x 1)
*	retrn:	x (of Ax = B)
*-----------------------------------------------------------------------------
*/
MATRIX mat_durbin(MATRIX R, MATRIX X)
{
	int		i, i1, j, ji, p;
	MATRIX	W, E, K, A;

	// if dimensions of X is wrong
	p = MatRow(R) - 1;
	if (  MatRow(X) != p || MatCol(X) != 1 ) {
		printf("mat_durbin error: incompatible output matrix size\n");
		_exit(-1);
	// if dimensions of X is correct
	} else {
		if ( (	W = mat_creat( p+2, 1, UNDEFINED ) ) == NULL )
			return (NULL);
		if ( (	E = mat_creat( p+2, 1, UNDEFINED ) ) == NULL )
			return (NULL);
		if ( (	K = mat_creat( p+2, 1, UNDEFINED ) ) == NULL )
			return (NULL);
		if ( (	A = mat_creat( p+2, p+2, UNDEFINED ) ) == NULL )
			return (NULL);

		W[0][0] = R[1][0];
		E[0][0] = R[0][0];

		for (i=1; i<=p; i++) {
			K[i][0] = W[i-1][0] / E[i-1][0];
			E[i][0] = E[i-1][0] * (1.0 - K[i][0] * K[i][0]);

			A[i][i] = -K[i][0];

			i1 = i-1;
			if (i1 >= 1) {
				for (j=1; j<=i1; j++) {
					ji = i - j;
					A[j][i] = A[j][i1] - K[i][0] * A[ji][i1];
				}
			}

			if (i != p) {
				W[i][0] = R[i+1][0];
				for (j=1; j<=i; j++)
					W[i][0] += A[j][i] * R[i-j+1][0];
			}
		}

		for (i=0; i<p; i++) {
			X[i][0] = -A[i+1][p];
		}
	}

	mat_free( A );
	mat_free( W );
	mat_free( K );
	mat_free( E );

	return(X);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_lsolve_durbin
*	desct:	Solve simultaneous linear eqns using
*		Levinson-Durbin algorithm
*
*		This function solve the linear eqns Ax = B:
*
*		|  v0   v1   v2  .. vn-1 | |  a1   |    |  v1   |
*		|  v1   v0   v1  .. vn-2 | |  a2   |    |  v2   |
*		|  v2   v1   v0  .. vn-3 | |  a3   |  = |  ..   |
*		|  ...                   | |  ..   |    |  ..   |
*		|  vn-1 vn-2 ..  .. v0   | |  an   |    |  vn   |
*
*	domain:	where A is a symmetric Toeplitz matrix and B
*		in the above format (related to A)
*
*	given:	A, B
*	retrn:	x (of Ax = B)
*
*-----------------------------------------------------------------------------
*/
MATRIX mat_lsolve_durbin(MATRIX A,MATRIX B,MATRIX X)
{
	MATRIX	R;
	int		i, n;

	n = MatRow(A);
	if ( (	R = mat_creat(n+1, 1, UNDEFINED) ) == NULL )
		return (NULL);
	for (i=0; i<n; i++)
		{
		R[i][0] = A[i][0];
		}
	R[n][0] = B[n-1][0];

	mat_durbin( R, X );
	mat_free( R );
	return(X);
}
/*
*-----------------------------------------------------------------------------
*	file:	materr.c
*	desc:	matrix error handler
*	by:	ko shu pui, patrick
*	date:	24 nov 91 v0.1
*	revi:
*	ref:
*       [1] Mary L.Boas, "Mathematical Methods in the Physical Sciene,"
*	John Wiley & Sons, 2nd Ed., 1983. Chap 3.
*
*	[2] Kendall E.Atkinson, "An Introduction to Numberical Analysis,"
*	John Wiley & Sons, 1978.
*
*-----------------------------------------------------------------------------
*/

MATRIX mat_error(int errno)
{
	switch( errno )
		{
		case MAT_MALLOC:
			printf("mat: malloc error\n" );
			break;
		case MAT_FNOTOPEN:
			printf("mat: fileopen error\n" );
			break;
		case MAT_FNOTGETMAT:
			printf("fgetmat: matrix read error\n");
			break;
		}

	return (NULL);
}
/*
*-----------------------------------------------------------------------------
*	funct:	mat_inv
*	desct:	find inverse of a matrix
*	given:	a = square matrix a
*	retrn:	square matrix Inverse(A)
*		NULL = fails, singular matrix, or malloc() fails
*		1 = success
*-----------------------------------------------------------------------------
*/
MATRIX mat_inv(MATRIX a, MATRIX C)
{
	MATRIX	A, B, P;
	int		i, n;

	n = MatCol(a);
	if ( (	A = mat_creat( n, n, UNDEFINED ) ) == NULL )
		return (NULL);
	mat_copy(a,A);
	if ( ( B = mat_creat( n, 1, UNDEFINED ) ) == NULL )
		return (NULL);
	if ( ( P = mat_creat( n, 1, UNDEFINED ) ) == NULL )
		return (NULL);

	// if dimensions of C is wrong
	if ( MatRow(a) != MatRow(C) || MatCol(a) != MatCol(C) ) {
		printf("mat_inv error: incompatible output matrix size\n");
		_exit(-1);
	// if dimensions of C is correct
	} else {
		/*
		*	- LU-decomposition -
		*	also check for singular matrix
		*/
		if (mat_lu(A, P) == -1) {
			mat_free(A);
			mat_free(B);
			mat_free(C);
			mat_free(P);
			printf("mat_inv error: failed to invert\n");
			return (NULL);
		}

		for (i=0; i<n; i++) {
			mat_fill(B, ZERO_MATRIX);
			B[i][0] = 1.0;
			mat_backsubs1( A, B, C, P, i );
		}
	}

	mat_free(A);
	mat_free(B);
	mat_free(P);

	if (C==NULL) {
			printf("mat_inv error: failed to invert\n");
			return(NULL);
	} else {
		return (C);
	}
}
/*
*-----------------------------------------------------------------------------
*	funct:	mat_mul
*	desct:	multiplication of two matrices
*	given:	A, B = compatible matrice to be multiplied
*	retrn:	C
*	comen:
*-----------------------------------------------------------------------------
*/
MATRIX mat_mul(MATRIX A,MATRIX B, MATRIX C)
{
	int		i, j, k;

	// if dimensions of C is wrong
	//if ( MatRow(C) != MatRow(A) || MatCol(C) != MatCol(B) ) {
	//	printf("mat_mul error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatCol(B); j++)
		for (k=0, C[i][j]=0.0; k<MatCol(A); k++) {
			C[i][j] += A[i][k] * B[k][j];
		}
	//}
	return(C);
}

MATRIX mat_transmul(MATRIX A,MATRIX B, MATRIX C)
{
/* computes C = A * B' */

	int		i, j, k;

		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatRow(B); j++)
		for (k=0, C[i][j]=0.0; k<MatCol(A); k++) {
			C[i][j] += A[i][k] * B[j][k];
		}

	return(C);
}

MATRIX mat_mymul(MATRIX A,MATRIX B, MATRIX C, short m)
{
	int		i, j, k;

	// if dimensions of C is wrong
	//if ( MatRow(C) != MatRow(A) || MatCol(C) != MatCol(B) ) {
	//	printf("mat_mul error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatCol(B); j++)
		for (k=0, C[i][j]=0.0; k<MatCol(A)-m; k++) {
			C[i][j] += A[i][k] * B[k][j];
		}
	//}
	return(C);
}

MATRIX mat_mymul1(MATRIX A,MATRIX B, MATRIX C, short m)
{
	int		i, j, k;

	// if dimensions of C is wrong
	//if ( MatRow(C) != MatRow(A) || MatCol(C) != MatCol(B) ) {
	//	printf("mat_mul error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatCol(B)-m; j++)
		for (k=0, C[i][j]=0.0; k<MatCol(A); k++) {
			C[i][j] += A[i][k] * B[k][j];
		}
	//}
	return(C);
}

MATRIX mat_mymul2(MATRIX A,MATRIX B, MATRIX C, short m)
{
	int		i, j, k;

	// if dimensions of C is wrong
	//if ( MatRow(C) != MatRow(A) || MatCol(C) != MatCol(B) ) {
	//	printf("mat_mul error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(A)-m; i++)
		for (j=0; j<MatCol(B); j++)
		for (k=0, C[i][j]=0.0; k<MatCol(A); k++) {
			C[i][j] += A[i][k] * B[k][j];
		}
	//}

	for(i=4;i<7;i++)
	   for(j=0;j<7;j++)
	      C[i][j] = B[i][j];

	return(C);
}

MATRIX mat_mymul3(MATRIX A,MATRIX B, MATRIX C, short m)
{
/* Note: This function finds C = A * B' */
	int		i, j, k;

	// if dimensions of C is wrong
	//if ( MatRow(C) != MatRow(A) || MatCol(C) != MatCol(B) ) {
	//	printf("mat_mul error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {
	for (i=0; i<MatRow(A); i++)
	for (j=0; j<MatCol(B)-m; j++)
        for (k=0, C[i][j]=0.0; k<MatCol(A); k++) {
		C[i][j] += A[i][k] * B[j][k];
	}
	//}
	for(i=0;i<7;i++)
	   for(j=4;j<7;j++)
	      C[i][j] = A[i][j];

	return(C);
}

MATRIX mat_mymul4(MATRIX A,MATRIX B, MATRIX C, short m)
{
/* Note: This function finds C = A * B' */

	int		i, j, k;

	// if dimensions of C is wrong
	//if ( MatRow(C) != MatRow(A) || MatCol(C) != MatCol(B) ) {
	//	printf("mat_mul error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatRow(B); j++)
		for (k=0, C[i][j]=0.0; k<MatCol(A)-m; k++) {
			C[i][j] += A[i][k] * B[j][k];
		}
	//}
	return(C);
}

MATRIX mat_mymul5(MATRIX A,MATRIX B, MATRIX C, short m)
{
	int		i, j, k;



	// if dimensions of C is wrong
	//if ( MatRow(C) != MatRow(A) || MatCol(C) != MatCol(B) ) {
	//	printf("mat_mul error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatCol(B); j++)
		for (k=0, C[i][j]=0.0; k<MatCol(A)-m; k++) {
			C[i][j] += A[i][k] * B[k][j];
		}
	//}
	        for(i=m+1;i<MatRow(A);i++)
	        for(j=0;j<MatCol(B);j++)
	           C[i][j] += B[i][j];
	return(C);
}

double mat_diagmul(MATRIX A)
{
	int i;
	double result = 1.0;

	for (i=0; i<MatRow(A); i++)
		{
		result *= A[i][i];
		}
	return (result);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_lu
*	desct:	in-place LU decomposition with partial pivoting
*	given:	!! A = square matrix (n x n) !ATTENTION! see commen
*		P = permutation vector (n x 1)
*	retrn:	number of permutation performed
*		-1 means suspected singular matrix
*	comen:	A will be overwritten to be a LU-composite matrix
*
*	note:	the LU decomposed may NOT be equal to the LU of
*		the orignal matrix a. But equal to the LU of the
*		rows interchanged matrix.
*-----------------------------------------------------------------------------
*/
int mat_lu(MATRIX A,MATRIX P)
{
	int		i, j, k, n;
	int	maxi;
	double tmp;
	double	c, c1;
	int		p;

	n = MatCol(A);

	for (p=0,i=0; i<n; i++)
		{
		P[i][0] = i;
		}

	for (k=0; k<n; k++)
	{
	/*
	* --- partial pivoting ---
	*/
	for (i=k, maxi=k, c=0.0; i<n; i++)
		{
        c1 = fabs( A[(int)P[i][0]][k] );
		if (c1 > c)
			{
			c = c1;
			maxi = i;
			}
		}

	/*
	*	row exchange, update permutation vector
	*/
	if (k != maxi)
		{
		p++;
		tmp = P[k][0];
		P[k][0] = P[maxi][0];
		P[maxi][0] = tmp;
		}

	/*
	*	suspected singular matrix
	*/
    if ( A[(int)P[k][0]][k] == 0.0 )
		return (-1);

	for (i=k+1; i<n; i++)
		{
		/*
		* --- calculate m(i,j) ---
		*/
        A[(int)P[i][0]][k] = A[(int)P[i][0]][k] / A[(int)P[k][0]][k];

		/*
		* --- elimination ---
		*/
		for (j=k+1; j<n; j++)
			{
            A[(int)P[i][0]][j] -= A[(int)P[i][0]][k] * A[(int)P[k][0]][j];
			}
		}
	}

	return (p);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_backsubs1
*	desct:	back substitution
*	given:	A = square matrix A (LU composite)
*		!! B = column matrix B (attention!, see comen)
*		!! X = place to put the result of X
*		P = Permutation vector (after calling mat_lu)
*		xcol = column of x to put the result
*	retrn:	column matrix X (of AX = B)
*	comen:	B will be overwritten
*-----------------------------------------------------------------------------
*/
MATRIX mat_backsubs1(MATRIX A,MATRIX B,MATRIX X,MATRIX P,int xcol)
{
	int	i, j, k, n;
	double	sum;

	n = MatCol(A);

	for (k=0; k<n; k++)
		{
		for (i=k+1; i<n; i++)
            B[(int)P[i][0]][0] -= A[(int)P[i][0]][k] * B[(int)P[k][0]][0];
		}

    X[n-1][xcol] = B[(int)P[n-1][0]][0] / A[(int)P[n-1][0]][n-1];
	for (k=n-2; k>=0; k--)
		{
		sum = 0.0;
		for (j=k+1; j<n; j++)
			{
            sum += A[(int)P[k][0]][j] * X[j][xcol];
			}
        X[k][xcol] = (B[(int)P[k][0]][0] - sum) / A[(int)P[k][0]][k];
		}

	return (X);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_lsolve
*	desct:	solve linear equations
*	given:	a = square matrix A
*		b = column matrix B
*	retrn:	column matrix X (of AX = B)
*-----------------------------------------------------------------------------
*/
MATRIX mat_lsolve(MATRIX a,MATRIX b,MATRIX X)
{
	MATRIX	A, B, P;
	int n;

	n = MatCol(a);
	if ( ( A = mat_creat(n, n, UNDEFINED) ) == NULL )
		return (NULL);
	if ( ( B = mat_creat(n, 1, UNDEFINED) ) == NULL )
		return (NULL);
	mat_copy(a,A);
	mat_copy(b,B);
	if ( ( P = mat_creat(n, 1, UNDEFINED) ) == NULL )
		return (NULL);

	// if dimensions of C is wrong
	if ( MatRow(X) != n || MatCol(X) != 1 ) {
		printf("mat_lsolve error: incompatible output matrix size\n");
	    _exit(-1);
	// if dimensions of C is correct
	} else {

		mat_lu( A, P );
		mat_backsubs1( A, B, X, P, 0 );
	}
	mat_free(A);
	mat_free(B);
	mat_free(P);

	return(X);
}
/*
*-----------------------------------------------------------------------------
*	funct:	mat_sub
*	desct:	subtraction of two matrice
*	given:	A, B = compatible matrice to be added
*	retrn:	NULL if malloc() fails
*		else allocated matrix of A - B
*	comen:
*-----------------------------------------------------------------------------
*/
MATRIX mat_sub(MATRIX A, MATRIX B, MATRIX C)
{
	int		i, j;

	// if dimensions of C is wrong
	//if ( MatRow(A) != MatRow(C) || MatCol(A) != MatCol(C)  ) {
	//	printf("mat_sub error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(A); i++)
		for (j=0; j<MatCol(A); j++) {
			C[i][j] = A[i][j] - B[i][j];
		}
	//}
	return(C);
}
/*
*-----------------------------------------------------------------------------
*	funct:	mat_submat
*	desct:	return a submatrix S of A
*	given:	A = main matrix,
*		i,j = row and column of A to be deleted to obtained S
*	retrn:	S
*-----------------------------------------------------------------------------
*/
MATRIX mat_submat(MATRIX A,int i,int j, MATRIX S)
{
	int		m, m1, p, p1;

	// if dimensions of S is wrong
	if ( MatRow(A)-1 != MatRow(S) || MatCol(A)-1 != MatCol(S)  ) {
		printf("mat_submat error: incompatible output matrix size\n");
		_exit(-1);
	// if dimensions of S is correct
	} else {

		for (m=m1=0; m<MatRow(A); m++) {
			if (m==i) continue;
			for (p=p1=0; p<MatCol(A); p++) {
				if (p==j) continue;
				S[m1][p1] = A[m][p];
				p1++;
			}
			m1++;
		}
	}
	return(S);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_SymToeplz
*	desct:	create a n x n symmetric Toeplitz matrix from
*		a n x 1 correlation matrix
*	given:	R = correlation matrix (n x 1)
*	retrn:	the symmetric Toeplitz matrix
*-----------------------------------------------------------------------------
*/
MATRIX mat_SymToeplz(MATRIX R,MATRIX T)
{
	int		i, j, n;

	n = MatRow(R);
	// if dimensions of T is wrong
	if ( n != MatRow(T) || n != MatCol(T)  ) {
		printf("mat_SymToeplz error: incompatible output matrix size\n");
		_exit(-1);
	// if dimensions of T is correct
	} else {

		for (i=0; i<n; i++)
		for (j=0; j<n; j++) {
			T[i][j] = R[abs(i-j)][0];
		}
	}
	return(T);
}

/*
*-----------------------------------------------------------------------------
*	funct:	mat_tran
*	desct:	transpose of a matrix
*	given:	A = matrix A to be transposed
*	retrn:	allocated matrix for A^t
*	comen:
*-----------------------------------------------------------------------------
*/
MATRIX mat_tran(MATRIX A, MATRIX At)
{
	int		i, j;

	// if dimensions of At is wrong
	//if (  MatCol(A) != MatRow(At) ||  MatRow(A) != MatCol(At)  ) {
	//	printf("mat_tran error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of At is correct
	//} else {

		/*
		*	Transposing ...
		*/
		for (i=0; i<MatCol(A); i++)
		for (j=0; j<MatRow(A); j++) {
			At[i][j] = A[j][i];
		}
	//}
	return(At);
}


//  takes the norm of a single column
double mat_norm (MATRIX X, int column)
{
	int i;
	double tot, norm;

	tot=0;
	for (i=0; i<MatRow(X); i++)  {
		tot = tot + (X[i][column-1])*(X[i][column-1]);
	}
	norm=sqrt(tot);
	return (norm);
}

// make sure C is 3 x 3
MATRIX mat_T321 (double pitch, double roll, double yaw, MATRIX C)
{
	MATRIX TI2,T23,T3B,t1;
	double cphi,sphi,ctheta,stheta,cpsi,spsi;

	// if dimensions of C is wrong
	if (  MatCol(C) != 3 ||  MatRow(C) != 3  ) {
		printf("mat_T321 error: incompatible output matrix size\n");
		_exit(-1);
	// if dimensions of C is correct
	} else {

		if ((TI2 = mat_creat( 3, 3, UNDEFINED )) == NULL)
			return (NULL);
		if ((T23 = mat_creat( 3, 3, UNDEFINED )) == NULL)
			return (NULL);
		if ((T3B = mat_creat( 3, 3, UNDEFINED )) == NULL)
			return (NULL);

		cphi = cos(roll);
		sphi = sin(roll);

		ctheta = cos(pitch);
		stheta = sin(pitch);

		cpsi = cos(yaw);
		spsi = sin(yaw);

		TI2[0][0] = spsi;
		TI2[0][1] = cpsi;
		TI2[0][2] = 0;
		TI2[1][0] = cpsi;
		TI2[1][1] = -spsi;
		TI2[1][2] = 0;
		TI2[2][0] = 0;
		TI2[2][1] = 0;
		TI2[2][2] = -1;

		T23[0][0] = ctheta;
		T23[0][1] = 0;
		T23[0][2] = -stheta;
		T23[1][0] = 0;
		T23[1][1] = 1;
		T23[1][2] = 0;
		T23[2][0] = stheta;
		T23[2][1] = 0;
		T23[2][2] = ctheta;

		T3B[0][0] = 1;
		T3B[0][1] = 0;
		T3B[0][2] = 0;
		T3B[1][0] = 0;
		T3B[1][1] = cphi;
		T3B[1][2] = sphi;
		T3B[2][0] = 0;
		T3B[2][1] = -sphi;
		T3B[2][2] = cphi;

	//	C = T3B*T23*TI2;
		if ( ( t1 = mat_creat(3,3,UNDEFINED) ) == NULL )
			return (NULL);

		mat_mul(T23,TI2,t1);
		mat_mul(T3B,t1, C);
	}

	mat_free(T3B);
	mat_free(T23);
	mat_free(TI2);
	mat_free(t1);

	return(C);
}

MATRIX mat_round (MATRIX X, MATRIX C)
{
	int i, j;

	// if dimensions of C is wrong
	if (  MatRow(C) != MatRow(X) ||  MatCol(C) != MatCol(X)  ) {
		printf("mat_round error: incompatible output matrix size\n");
		_exit(-1);
	// if dimensions of C is correct
	} else {

		for (i=0; i<MatRow(X); i++)  {
			for (j=0; j<MatCol(X); j++)  {
				//temp = (int) (X[i][j]+.5*);
				C[i][j] =floor(X[i][j]+.5);
			}
		}
	}
	return(C);
}

//		Calculates the dot product of X and Y
double mat_dot (MATRIX X, MATRIX Y)
{
	int		 i,j;
	double 		dotProduct;

	dotProduct=0;
	for(i=0;i<MatRow(X);i++) {
		for(j=0;j<MatCol(X);j++) {
		dotProduct+=(X[i][j]*Y[i][j]);
		}
	}
	return(fabs(dotProduct));
}

MATRIX mat_scalMult (MATRIX X,double A, MATRIX C)
{
	int		i, j;

	// if dimensions of C is wrong
	//if (  MatRow(C) != MatRow(X) ||  MatCol(C) != MatCol(X)  ) {
	//	printf("mat_scalMult error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(X); i++)  {
			for (j=0; j<MatCol(X); j++)  {
				C[i][j] = X[i][j]*A;
			}
		}
	//}
	return(C);
}

MATRIX mat_scalMul(MATRIX X,double A, MATRIX C)
{
	int		i, j;

	// if dimensions of C is wrong
	//if (  MatRow(C) != MatRow(X) ||  MatCol(C) != MatCol(X)  ) {
	//	printf("mat_scalMult error: incompatible output matrix size\n");
	//	_exit(-1);
	// if dimensions of C is correct
	//} else {

		for (i=0; i<MatRow(X); i++)  {
			for (j=0; j<MatCol(X); j++)  {
				C[i][j] = X[i][j]*A;
			}
		}
	//}
	return(C);
}

MATRIX mat_subcopy(MATRIX A, int row, int col, MATRIX B)
{
	int i,j;

	for(i=0;i<row;i++) {
	   for(j=0;j<col;j++) {
	      B[i][j] = A[i][j];
	   }
	}
	return(B);
}

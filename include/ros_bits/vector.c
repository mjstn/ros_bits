/*********************************************************************
 * FileName:        vector.c
 ********************************************************************/
// take out mjstn:  #include "vector.h"

//***************************************************
//				 Math Utility 
//***************************************************
//Declare a global constant for pi and a few multiples
const float kPi = 3.1416f;
const float k2Pi = 6.2832f;						// kPi * 2.0f;
const float kPiOver2 = 1.5708;					// kPi / 2.0f;
const float k1OverPi = 0.3183;					// 1.0f / kPi;		
const float k1Over2Pi = 0.1591;					// 1.0f / k2Pi;
const float kRadToDeg = 57.2956;				// 180.0f / kPi;
const float kDegToRad = 0.0174;					// kPi / 180.0f;	  

//********************************************
// "Wrap" an angle in rage -pi...pi by adding the correct multiple
// of 2 pi
float wrapPi(float theta) {
	theta += kPi;
	theta -= floor(theta * k1Over2Pi) * k2Pi;
	theta -= kPi;
	return theta;
}				  

//********************************************
// safeAcos													 C 
float safeAcos(float x) 
{
	// Check limit conditions
	if (x <= -1.0f) {
		return kPi;
	}
	if (x >= 1.0f) {
		return 0.0f;
	}
	//Value is in the domain - use standard C function
	return acos(x);
}

// Compute the sin and cosine of an angle.  On some platforms, if we know
// that we need both values, it can be computed faster than computing
// the two values seperaly.
void sinCos(float *returnSin, float *returnCos, float theta)
{
	// For simplicity, we'll just use the normal trig function.
	// Note that on some platforms we may be able to do better
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}


//***************************************************
//					Vector
//***************************************************
struct vector3 MakeVector(float x, float y, float z)
{
	struct vector3 temp;
	temp.x = x;
	temp.y = y;
	temp.z = z;
	return temp;
}

struct vector3 AddVector(struct vector3 v1, struct vector3 v2)
{
	v1.x += v2.x;
	v1.y += v2.y;
	v1.z += v2.z;
	return v1;
}

float MagVector(struct vector3 v) 
{
	return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

struct vector3 Normalize(float x, float y, float z) 
{
	struct vector3 temp;
	float magSq = (x*x + y*y + z*z);
	if (magSq > 0.0f) { // check for divide-by-zero
		float oneOverMag = 1.0f / sqrt(magSq);
		temp.x = x * oneOverMag;
		temp.y = y * oneOverMag;
		temp.z = z * oneOverMag;
		return temp;
	}
	else{
		temp.x = 0;
		temp.y = 0;
		temp.z = 0;
	}
	return temp; 
}

// Compute the cross product of two vectors
float DotVector (struct vector3 v1, struct vector3 v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}


// Compute the cross product of two vectors
struct vector3 CrossProduct(struct vector3 v1, struct vector3 v2)
{
	struct vector3 temp;
	temp.x = v1.y*v2.z - v1.z*v2.y;
	temp.y = v1.z*v2.x - v1.x*v2.z;
	temp.z = v1.x*v2.y - v1.y*v2.x;
	return temp;
}

// Scalar on the left multiplication, for symmetry
struct vector3 Multiply(float k, struct vector3 v) 
{
	struct vector3 temp;
	temp.x = k*v.x;
	temp.y = k*v.y;
	temp.z = k*v.z;
	return temp;
}

//***************************************************
//					 Matrix 
//***************************************************

struct vector3	RotationMatrixInertialToObject(struct matrix m, struct vector3 v)
{
	// Perform the matrix multiplication in the "standard" way.
	struct vector3 temp;
	temp.x = m.m11*v.x + m.m21*v.y + m.m31*v.z;
	temp.y = m.m12*v.x + m.m22*v.y + m.m32*v.z;
	temp.z = m.m13*v.x + m.m23*v.y + m.m33*v.z;
	return temp;
}

struct vector3	RotationMatrixObjectToInertial(struct matrix m, struct vector3 v)
{

	// Multiply by the transpose
	struct vector3 temp;
	temp.x = m.m11*v.x + m.m12*v.y + m.m13*v.z;
	temp.y = m.m21*v.x + m.m22*v.y + m.m23*v.z;
	temp.z = m.m31*v.x + m.m32*v.y + m.m33*v.z;
	return temp;
}

struct matrix SetupRotationMatrix(int axis, float theta) {

	// Get sin and cosine of rotation angle

	struct matrix m;
	float	s, c;
	sinCos(&s, &c, theta);

	// Check which axis they are rotating about
	switch (axis) {
		case 1: // Rotate about the x-axis
			m.m11 = 1.0f; m.m12 = 0.0f; m.m13 = 0.0f;
			m.m21 = 0.0f; m.m22 = c;    m.m23 = s;
			m.m31 = 0.0f; m.m32 = -s;   m.m33 = c;
			break;

		case 2: // Rotate about the y-axis
			m.m11 = c;    m.m12 = 0.0f; m.m13 = -s;
			m.m21 = 0.0f; m.m22 = 1.0f; m.m23 = 0.0f;
			m.m31 = s;    m.m32 = 0.0f; m.m33 = c;
			break;

		case 3: // Rotate about the z-axis
			m.m11 = c;    m.m12 = s;    m.m13 = 0.0f;
			m.m21 = -s;   m.m22 = c;    m.m23 = 0.0f;
			m.m31 = 0.0f; m.m32 = 0.0f; m.m33 = 1.0f;
			break;

		default:
			// bogus axis index
			break;
	}
	return m;
}



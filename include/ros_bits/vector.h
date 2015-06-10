/*********************************************************************
 * FileName:        vector.h
 ********************************************************************/
#include <math.h>

struct vector3 
{
	float x;
	float y;
	float z;
}; 

struct matrix
{
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
};

struct vector3 MakeVector(float x, float y, float z);
struct vector3 AddVector(struct vector3 v1, struct vector3 v2);
struct vector3 Normalize(float x, float y, float z);
float DotVector (struct vector3 v1, struct vector3 v2);
struct vector3 CrossProduct(struct vector3 v1, struct vector3 v2);
struct vector3 Multiply(float k, struct vector3 v); 
struct matrix SetupRotationMatrix(int axis, float theta);
struct vector3	RotationMatrixObjectToInertial(struct matrix m, struct vector3 v);


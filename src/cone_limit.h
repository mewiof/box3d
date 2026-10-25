#pragma once

#include "core.h"

#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>

typedef struct b3Swing
{
	float angle;
	float ux;
	float uy;
} b3Swing;

B3_INLINE b3Swing b3DecomposeSwing( b3Quat relQ )
{
	float x = relQ.v.x;
	float y = relQ.v.y;
	float z = relQ.v.z;
	float s = relQ.s;

	float sx = x * s - y * z;
	float sy = y * s + x * z;

	b3Swing swing;
	swing.angle = b3GetSwingAngle( relQ );

	float length = sqrtf( sx * sx + sy * sy );
	float lateral = sqrtf( x * x + y * y );
	if ( length > 1000.0f * FLT_MIN )
	{
		float invLength = 1.0f / length;
		swing.ux = invLength * sx;
		swing.uy = invLength * sy;
	}
	else if ( lateral > 1000.0f * FLT_MIN )
	{
		float invLateral = 1.0f / lateral;
		swing.ux = invLateral * x;
		swing.uy = invLateral * y;
	}
	else
	{
		swing.ux = 1.0f;
		swing.uy = 0.0f;
	}

	return swing;
}

B3_INLINE float b3ConeLimitAngle( float ax, float ay, float ux, float uy )
{
	float px = ay * ux;
	float py = ax * uy;
	float denominator = sqrtf( px * px + py * py );
	if ( denominator > 1000.0f * FLT_MIN )
	{
		return ax * ay / denominator;
	}

	return 0.0f;
}

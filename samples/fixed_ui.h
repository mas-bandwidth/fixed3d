// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/math_functions.h"

#include "imgui.h"

// ImGui widgets for fixed-point fields. The UI edits a float proxy and the
// result converts once on change; b3Fixed fields never alias float pointers.

inline bool SliderFixed( const char* label, b3Fixed* v, float lo, float hi, const char* fmt = "%.3f",
						 ImGuiSliderFlags flags = 0 )
{
	float f = b3FixToFloat( *v );
	bool changed = ImGui::SliderFloat( label, &f, lo, hi, fmt, flags );
	if ( changed )
	{
		*v = b3FixFromFloat( f );
	}
	return changed;
}

inline bool SliderFixed3( const char* label, b3Vec3* v, float lo, float hi, const char* fmt = "%.3f" )
{
	float f[3] = { b3FixToFloat( v->x ), b3FixToFloat( v->y ), b3FixToFloat( v->z ) };
	bool changed = ImGui::SliderFloat3( label, f, lo, hi, fmt );
	if ( changed )
	{
		v->x = b3FixFromFloat( f[0] );
		v->y = b3FixFromFloat( f[1] );
		v->z = b3FixFromFloat( f[2] );
	}
	return changed;
}

inline bool InputFixed( const char* label, b3Fixed* v, float step = 0.0f, float stepFast = 0.0f,
						const char* fmt = "%.3f", ImGuiInputTextFlags flags = 0 )
{
	float f = b3FixToFloat( *v );
	bool changed = ImGui::InputFloat( label, &f, step, stepFast, fmt, flags );
	if ( changed )
	{
		*v = b3FixFromFloat( f );
	}
	return changed;
}

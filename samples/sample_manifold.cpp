// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "sample.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <imgui.h>

class Manifold : public Sample
{
public:
	explicit Manifold( SampleContext* sampleContext )
		: Sample( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 35.0f, 30.0f, 50.0f, { B3_FIX( 0.0f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) } );
		}

		memset( m_points, 0, sizeof( m_points ) );
		m_manifold = {};
		m_manifold.points = m_points;

		{
			m_transformA.p = { B3_FIX( 3.5f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) };
			m_transformA.q = b3MakeQuatFromAxisAngle( { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, B3_PI / 2 );
		}

		{
			m_transformB.p = { B3_FIX( 0.0f ), B3_FIX( 1.5f ), B3_FIX( 3.5f ) };
			m_transformB.q = b3Quat_identity;
		}

		m_simplexCache = {};
		m_satCache = {};
		m_manualFeature = 0;
		m_baseTranslation = b3Pos_zero;
		m_baseQuaternion = b3Quat_identity;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Pos_zero;
		m_useCache = false;
		m_tracking = false;
		m_rotating = false;
	}

	void Render() override
	{
		DrawTextLine( "origin: %g %g %g", b3FixToDouble( m_origin.x ), b3FixToDouble( m_origin.y ), b3FixToDouble( m_origin.z ) );
		DrawTextLine( "count = %d", m_manifold.pointCount );

		DrawAxes( b3WorldTransform_identity, 1.0f );

		if ( m_manifold.pointCount == 0 )
		{
			return;
		}

		b3Fixed length = b3GetLengthUnitsPerMeter() / 2;
		b3Vec3 normal = b3RotateVector( m_transformA.q, m_manifold.normal );

		for ( int pointIndex = 0; pointIndex < m_manifold.pointCount; ++pointIndex )
		{
			const b3LocalManifoldPoint& manifoldPoint = m_manifold.points[pointIndex];

			b3Pos point = b3TransformWorldPoint( m_transformA, manifoldPoint.point );

			DrawLine( point, point + length * normal, MakeColor( b3_colorWhite ) );

			if ( manifoldPoint.separation > B3_FIX( 0.0f ) )
			{
				DrawPoint( point, 10.0f, MakeColor( b3_colorWhite ) );
			}
			else
			{
				DrawPoint( point, 10.0f, MakeColor( b3_colorYellow ) );
			}

			DrawString3D( point, MakeColor( b3_colorWhite ), "   %.3f", b3FixToDouble( manifoldPoint.separation ) );

			b3Vec3 perp = b3Perp( normal );
			b3FeaturePair pair = manifoldPoint.pair;
			DrawString3D( point + B3_FIX( 0.025f ) * normal + B3_FIX( 0.05f ) * perp, MakeColor( b3_colorPapayaWhip ), "  %X:%X %X:%X", pair.owner1,
						  pair.index1, pair.owner2, pair.index2 );
		}

		Sample::Render();
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		ImGui::Checkbox( "Use cache", &m_useCache );
		if ( m_useCache )
		{
			ImGui::RadioButton( "auto", &m_manualFeature, 0 );
			ImGui::RadioButton( "faceA", &m_manualFeature, 1 );
			ImGui::RadioButton( "faceB", &m_manualFeature, 2 );
			ImGui::RadioButton( "edgePair", &m_manualFeature, 3 );
		}

		return true;
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & MOD_ALT ) == 0 )
		{
			if ( modifiers & MOD_SHIFT )
			{
				m_baseX = p.x;
				m_baseY = p.y;
				m_baseQuaternion = m_transformB.q;
				m_rotating = true;
			}
			else
			{
				PickRay pickRay = m_camera->BuildPickRay( b3FixToFloat( p.x ), b3FixToFloat( p.y ) );
				m_origin = pickRay.origin + B3_FIX( 10.0f ) * b3Normalize( pickRay.translation );
				m_baseTranslation = m_transformB.p;
				m_tracking = true;
			}
		}
	}

	void MouseUp( b3Vec2 p, int button ) override
	{
		m_tracking = false;
		m_rotating = false;
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			PickRay pickRay = m_camera->BuildPickRay( b3FixToFloat( p.x ), b3FixToFloat( p.y ) );
			b3Pos origin = pickRay.origin + B3_FIX( 10.0f ) * b3Normalize( pickRay.translation );
			m_transformB.p = m_baseTranslation + b3SubPos( origin, m_origin );
		}

		if ( m_rotating )
		{
			b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisY, b3FixMul( B3_FIX( 0.01f ), p.x - m_baseX ) );
			b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, b3FixMul( B3_FIX( 0.01f ), p.y - m_baseY ) );
			m_transformB.q = b3NormalizeQuat( b3MulQuat( m_baseQuaternion, b3MulQuat( qx, qz ) ) );
		}
	}

	static constexpr int m_pointCapacity = 64;
	b3LocalManifold m_manifold;
	b3LocalManifoldPoint m_points[m_pointCapacity];
	b3WorldTransform m_transformA;
	b3WorldTransform m_transformB;
	b3Pos m_baseTranslation;
	b3Quat m_baseQuaternion;
	b3Pos m_origin;
	b3SimplexCache m_simplexCache;
	b3SATCache m_satCache;
	int m_manualFeature;
	b3Fixed m_baseX;
	b3Fixed m_baseY;
	bool m_useCache;
	bool m_tracking;
	bool m_rotating;
};

class TriangleManifold : public Sample
{
public:
	explicit TriangleManifold( SampleContext* sampleContext )
		: Sample( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 35.0f, 30.0f, 50.0f, { B3_FIX( 0.0f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) } );
		}

		m_manifold = {};
		memset( m_points, 0, sizeof( m_points ) );
		m_manifold.points = m_points;

		{
			m_transformA.p = { B3_FIX( 3.5f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) };
			m_transformA.q = b3MakeQuatFromAxisAngle( { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, B3_PI / 2 );
		}

		{
			m_transformB.p = { B3_FIX( 0.0f ), B3_FIX( 1.5f ), B3_FIX( 3.5f ) };
			m_transformB.q = b3Quat_identity;
		}

		m_simplexCache = {};
		m_satCache = {};
		m_baseTranslation = b3Pos_zero;
		m_baseQuaternion = b3Quat_identity;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Pos_zero;
		m_useCache = false;
		m_tracking = false;
		m_rotating = false;

		m_manualFeature = 0;
	}

	void Render() override
	{
		DrawTextLine( "origin: %g %g %g", b3FixToDouble( m_origin.x ), b3FixToDouble( m_origin.y ), b3FixToDouble( m_origin.z ) );
		DrawTextLine( "count = %d", m_manifold.pointCount );
		DrawTextLine( "feature = %d", m_manifold.feature );
		DrawTextLine( "cache hit = %d", m_satCache.hit );

		DrawAxes( b3WorldTransform_identity, 1.0f );

		if ( m_manifold.pointCount > 0 )
		{
			b3Fixed length = b3GetLengthUnitsPerMeter() / 2;
			b3Vec3 normal = b3RotateVector( m_transformB.q, m_manifold.normal );

			for ( int pointIndex = 0; pointIndex < m_manifold.pointCount; ++pointIndex )
			{
				const b3LocalManifoldPoint& manifoldPoint = m_manifold.points[pointIndex];

				b3Pos point = b3TransformWorldPoint( m_transformB, manifoldPoint.point );
				DrawLine( point, point + length * normal, MakeColor( b3_colorWhite ) );

				if ( manifoldPoint.separation > B3_FIX( 0.0f ) )
				{
					DrawPoint( point, 10.0f, MakeColor( b3_colorWhite ) );
				}
				else
				{
					DrawPoint( point, 10.0f, MakeColor( b3_colorYellow ) );
				}

				DrawString3D( point, MakeColor( b3_colorWhite ), "  %.2f", 100.0f * b3FixToFloat( manifoldPoint.separation ) );

				b3FeaturePair pair = manifoldPoint.pair;
				DrawString3D( point + B3_FIX( 0.025f ) * normal, MakeColor( b3_colorPapayaWhip ), "  %X:%X %X:%X", pair.owner1, pair.index1,
							  pair.owner2, pair.index2 );
			}
		}

		DrawTriangle( m_transformA, m_triangle[0], m_triangle[1], m_triangle[2], MakeColor( b3_colorCyan ) );

		b3Pos p1 = b3TransformWorldPoint( m_transformA, m_triangle[0] );
		b3Pos p2 = b3TransformWorldPoint( m_transformA, m_triangle[1] );
		b3Pos p3 = b3TransformWorldPoint( m_transformA, m_triangle[2] );
		DrawString3D( p1, MakeColor( b3_colorWhite ), "0" );
		DrawString3D( p2, MakeColor( b3_colorWhite ), "1" );
		DrawString3D( p3, MakeColor( b3_colorWhite ), "2" );

		b3Vec3 normal = b3Normalize( b3Cross( p2 - p1, p3 - p1 ) );
		b3Pos center = p1 + B3_FIX( 1.0f / 3.0f ) * ( ( p2 - p1 ) + ( p3 - p1 ) );
		DrawArrow( center, center + B3_FIX( 0.5f ) * normal, MakeColor( b3_colorMediumPurple ) );

		Sample::Render();
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		ImGui::Checkbox( "Use cache", &m_useCache );
		if ( m_useCache )
		{
			ImGui::RadioButton( "auto", &m_manualFeature, 0 );
			ImGui::RadioButton( "faceA", &m_manualFeature, 1 );
			ImGui::RadioButton( "faceB", &m_manualFeature, 2 );
			ImGui::RadioButton( "edgePair", &m_manualFeature, 3 );
		}

		return true;
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & MOD_ALT ) == 0 )
		{
			if ( modifiers & MOD_SHIFT )
			{
				m_baseX = p.x;
				m_baseY = p.y;
				m_baseQuaternion = m_transformB.q;
				m_rotating = true;
			}
			else
			{
				PickRay pickRay = m_camera->BuildPickRay( b3FixToFloat( p.x ), b3FixToFloat( p.y ) );
				m_origin = pickRay.origin + B3_FIX( 10.0f ) * b3Normalize( pickRay.translation );
				m_baseTranslation = m_transformB.p;
				m_tracking = true;
			}
		}
	}

	void MouseUp( b3Vec2 p, int button ) override
	{
		m_tracking = false;
		m_rotating = false;
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			PickRay pickRay = m_camera->BuildPickRay( b3FixToFloat( p.x ), b3FixToFloat( p.y ) );
			b3Pos origin = pickRay.origin + B3_FIX( 10.0f ) * b3Normalize( pickRay.translation );
			m_transformB.p = m_baseTranslation + b3SubPos( origin, m_origin );
		}

		if ( m_rotating )
		{
			b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisY, b3FixMul( B3_FIX( 0.01f ), p.x - m_baseX ) );
			b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, b3FixMul( B3_FIX( 0.01f ), p.y - m_baseY ) );
			m_transformB.q = b3NormalizeQuat( b3MulQuat( m_baseQuaternion, b3MulQuat( qx, qz ) ) );
		}
	}

	static constexpr int m_pointCapacity = 8;
	b3LocalManifold m_manifold;
	b3LocalManifoldPoint m_points[m_pointCapacity];

	// Triangle transform
	b3WorldTransform m_transformA;

	// Convex shape transform
	b3WorldTransform m_transformB;

	b3Vec3 m_triangle[3] = {};
	b3Pos m_baseTranslation;
	b3Quat m_baseQuaternion;
	b3Pos m_origin;
	b3SimplexCache m_simplexCache;
	b3SATCache m_satCache;
	int m_manualFeature;
	b3Fixed m_baseX;
	b3Fixed m_baseY;
	bool m_useCache;
	bool m_tracking;
	bool m_rotating;
};

class SphereAndSphere : public Manifold
{
public:
	explicit SphereAndSphere( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		m_sphere = { { B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( -0.25f ) }, B3_FIX( 2.0f ) };
	}

	void Render() override
	{
		DrawSolidSphere( m_transformA, m_sphere, MakeColor( b3_colorGreen ) );
		DrawSolidSphere( m_transformB, m_sphere, MakeColor( b3_colorCyan ) );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideSpheres( &m_manifold, m_pointCapacity, &m_sphere, &m_sphere, transformBtoA );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new SphereAndSphere( sampleContext );
	}

	b3Sphere m_sphere;
};

static int sampleCollideSpheres = RegisterSample( "Manifold", "Sphere vs Sphere", SphereAndSphere::Create );

class CapsuleAndSphere : public Manifold
{
public:
	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndSphere( sampleContext );
	}

	explicit CapsuleAndSphere( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		m_capsule = { { B3_FIX( -2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		m_sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 2.0f ) };

		m_transformA = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		m_transformB = { { B3_FIX( -4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSolidCapsule( m_transformA, m_capsule, MakeColor( b3_colorCyan ) );
		DrawSolidSphere( m_transformB, m_sphere, MakeColor( b3_colorGreen ) );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideCapsuleAndSphere( &m_manifold, m_pointCapacity, &m_capsule, &m_sphere, transformBtoA );
	}

	b3Sphere m_sphere;
	b3Capsule m_capsule;
};

static int sampleSphereAndCapsule = RegisterSample( "Manifold", "Capsule vs Sphere", CapsuleAndSphere::Create );

class HullAndSphere : public Manifold
{
public:
	explicit HullAndSphere( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		m_sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		m_hull = b3MakeBoxHull( B3_FIX( 2.0f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

		m_transformA = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		m_transformB = { { B3_FIX( 1.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
	}

	void Render() override
	{
		DrawHull( m_transformA, &m_hull.base, MakeColor( b3_colorCyan ) );
		DrawSolidSphere( m_transformB, m_sphere, MakeColor( b3_colorGreen ) );

		Manifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideHullAndSphere( &m_manifold, m_pointCapacity, &m_hull.base, &m_sphere, transformBtoA, &m_simplexCache );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullAndSphere( sampleContext );
	}

	b3Sphere m_sphere;
	b3BoxHull m_hull;
};

static int sampleSphereAndHull = RegisterSample( "Manifold", "Hull vs Sphere", HullAndSphere::Create );

class TriangleAndSphere : public TriangleManifold
{
public:
	static Sample* Create( SampleContext* sampleContext )
	{
		return new TriangleAndSphere( sampleContext );
	}

	explicit TriangleAndSphere( SampleContext* sampleContext )
		: TriangleManifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 10.0f, b3Pos_zero );
		}

		m_sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.25f ) };
		m_triangle[0] = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		m_triangle[1] = { B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 4.0f ) };
		m_triangle[2] = { B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };

		// b3Quat qA = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, 2.0f );
		// m_transformA = { { 1.0f, 1.0f, 0.0f }, qA };

		m_transformA = b3WorldTransform_identity;
		m_transformB = { { B3_FIX( 2.0f ), B3_FIX( 0.5f ), B3_FIX( 1.0f ) }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSolidSphere( m_transformB, m_sphere, MakeColorAlpha( b3_colorGreen, 0.5f ) );

		TriangleManifold::Render();
	}

	void Step() override
	{
		// Convert triangle to frame B
		b3Transform xf = b3InvMulWorldTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideSphereAndTriangle( &m_manifold, m_pointCapacity, &m_sphere, localTriangle );
	}

	b3Sphere m_sphere;
};

static int sampleSphereAndTriangle = RegisterSample( "Manifold", "Triangle vs Sphere", TriangleAndSphere::Create );

class CapsuleAndCapsule : public Manifold
{
public:
	explicit CapsuleAndCapsule( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		m_capsule = { { B3_FIX( -2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };

		m_transformA = { { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		m_transformB = { { B3_FIX( -4.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSolidCapsule( m_transformA, m_capsule, MakeColor( b3_colorGreen ) );
		DrawSolidCapsule( m_transformB, m_capsule, MakeColor( b3_colorCyan ) );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideCapsules( &m_manifold, m_pointCapacity, &m_capsule, &m_capsule, transformBtoA );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndCapsule( sampleContext );
	}

	b3Capsule m_capsule;
};

static int sampleCapsuleAndCapsule = RegisterSample( "Manifold", "Capsule vs Capsule", CapsuleAndCapsule::Create );

class CapsuleAndHull : public Manifold
{
public:
	explicit CapsuleAndHull( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 5.0f, b3Pos_zero );
		}

		m_capsule = { { B3_FIX( -1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.15f ) };
		m_hull = b3MakeBoxHull( B3_FIX( 1.0f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

		m_transformA = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		m_transformB = { { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };

		/*
			p	[ 1.58523774, 0.729615569, 0.451690674 ]	b3Vec3
			q	[ [ -0.00256555085, -0.0201825816, 0.126076236 ], 0.991811991 ]	b3Quat
		 */
		// m_capsule = {
		//	{ 0.0799999982, -0.0151330000, -0.0918010026 }, { -0.0799999982, -0.0151330000, -0.0918010026 }, 0.100000001 };
		// m_hull = b3CreateBox( { 0.5f, 1.0f, 1.5f } );

		m_transformB = { { B3_FIX( 1.58523774 ), B3_FIX( 0.729615569 ), B3_FIX( 0.451690674 ) },
						 { { B3_FIX( -0.00256555085 ), B3_FIX( -0.0201825816 ), B3_FIX( 0.126076236 ) }, B3_FIX( 0.991811991 ) } };
	}

	void Render() override
	{
		DrawHull( m_transformA, &m_hull.base, MakeColor( b3_colorCyan ) );
		DrawSolidCapsule( m_transformB, m_capsule, MakeColor( b3_colorGreen ) );

		Manifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideHullAndCapsule( &m_manifold, m_pointCapacity, &m_hull.base, &m_capsule, transformBtoA, &m_simplexCache );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndHull( sampleContext );
	}

	b3BoxHull m_hull;
	b3Capsule m_capsule;
};

static int sampleCapsuleAndHull = RegisterSample( "Manifold", "Capsule vs Hull", CapsuleAndHull::Create );

class TriangleAndCapsule : public TriangleManifold
{
public:
	static Sample* Create( SampleContext* sampleContext )
	{
		return new TriangleAndCapsule( sampleContext );
	}

	explicit TriangleAndCapsule( SampleContext* sampleContext )
		: TriangleManifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 10.0f, b3Pos_zero );
		}

		m_capsule = { { B3_FIX( -0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.05f ) };
		m_triangle[0] = { B3_FIX( -4.0f ), B3_FIX( 0.0f ), B3_FIX( -4.0f ) };
		m_triangle[1] = { B3_FIX( -4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		m_triangle[2] = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };

		m_transformA = b3WorldTransform_identity;
		m_transformB = { { B3_FIX( -1.0f ), B3_FIX( 0.0f ), B3_FIX( -1.0f ) }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSolidCapsule( m_transformB, m_capsule, MakeColorAlpha( b3_colorGreen, 0.5f ) );
		DrawAxes( m_transformB, 0.1f );

		TriangleManifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		// Put triangle into capsule coordinates
		b3Transform xf = b3InvMulWorldTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideCapsuleAndTriangle( &m_manifold, m_pointCapacity, &m_capsule, localTriangle, &m_simplexCache );
	}

	b3Capsule m_capsule;
};

static int sampleCapsuleAndTriangle = RegisterSample( "Manifold", "Triangle vs Capsule", TriangleAndCapsule::Create );

class HullAndHull : public Manifold
{
public:
	explicit HullAndHull( SampleContext* context )
		: Manifold( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 4.0f, b3Pos_zero );
		}

		// m_boxA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, { { 0.0f, -0.5f, 0.0f }, b3Quat_identity } );

		b3Transform transform = { { B3_FIX( 1.0f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		m_boxA = b3MakeTransformedBoxHull( B3_FIX( 0.5f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ), transform );

		// m_hull = CreateConvex( 0.6f, 0.0f, 0.95f, 1.0f, context->arena );
		m_hullA = &m_boxA.base;

		m_boxB = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
		// m_hull = CreateConvex( 0.6f, 0.0f, 0.95f, 1.0f, context->arena );
		m_hullB = &m_boxB.base;

		/*
-		xfA	{p=[ -10.0000000, 0.00000000, -10.0000000 ] q=[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ] }	b3Transform
+		p	[ -10.0000000, 0.00000000, -10.0000000 ]	b3Vec3
+		q	[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ]	b3Quat
-		xfB	{p=[ -10.0000000, 1.00000000, -10.0000000 ] q=[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ] }	b3Transform
+		p	[ -10.0000000, 1.00000000, -10.0000000 ]	b3Vec3
+		q	[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ]	b3Quat

		*/
		m_transformA = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		m_transformB = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };

		/*
			p	[ 0.691772044, 0.951781273, 0.0741987228 ]	b3Vec3
			q	[ [ -0.00162522844, -0.0456664972, 0.0355294086 ], 0.998323441 ]	b3Quat
		 */
		// m_transformB = { { 0.691772044, 0.951781273, 0.0741987228 },
		//				 { { -0.00162522844, -0.0456664972, 0.0355294086 }, 0.998323441 } };

		m_satCache = {};
	}

	~HullAndHull() override
	{
		if ( m_hullA != &m_boxA.base )
		{
			b3DestroyHull( m_hullA );
		}

		if ( m_hullB != &m_boxB.base )
		{
			b3DestroyHull( m_hullB );
		}
	}

	b3HullData* CreateConvex( b3Fixed radius1, b3Fixed height1, b3Fixed radius2, b3Fixed height2 ) const
	{
		constexpr int sideCount = 32;
		const b3Fixed deltaAlpha = 2 * B3_PI / sideCount;

		int vertexCount = 2 * sideCount;
		b3Vec3 vertexBase[2 * sideCount];

		b3Fixed alpha = B3_FIX( 0.0f );
		for ( int sideIndex = 0; sideIndex < sideCount; ++sideIndex )
		{
			b3CosSin cs = b3ComputeCosSin( alpha );

			b3Fixed x1 = b3FixMul( radius1, cs.cosine );
			b3Fixed z1 = b3FixMul( radius1, cs.sine );
			b3Fixed x2 = b3FixMul( radius2, cs.cosine );
			b3Fixed z2 = b3FixMul( radius2, cs.sine );

			vertexBase[2 * sideIndex + 0] = { x1, height1, z1 };
			vertexBase[2 * sideIndex + 1] = { x2, height2, z2 };
			alpha += deltaAlpha;
		}

		return b3CreateHull( vertexBase, vertexCount, vertexCount );
	}

	void Render() override
	{
		DrawHull( m_transformA, m_hullA, MakeColor( b3_colorGreen ) );
		DrawHull( m_transformB, m_hullB, MakeColor( b3_colorCyan ) );

		Manifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == true )
		{
			if ( m_manualFeature == 1 )
			{
				m_satCache.type = b3_manualFaceAxisA;
			}
			else if ( m_manualFeature == 2 )
			{
				m_satCache.type = b3_manualFaceAxisB;
			}
			else if ( m_manualFeature == 3 )
			{
				m_satCache.type = b3_manualEdgePairAxis;
			}
		}
		else
		{
			m_satCache = {};
		}

		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideHulls( &m_manifold, m_pointCapacity, m_hullA, m_hullB, transformBtoA, &m_satCache );

		DrawTextLine( "SAT type: %d", m_satCache.type );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullAndHull( sampleContext );
	}

	b3BoxHull m_boxA;
	b3BoxHull m_boxB;
	b3HullData* m_hullA;
	b3HullData* m_hullB;
};

static int sampleCollideHulls = RegisterSample( "Manifold", "Hull vs Hull", HullAndHull::Create );

class TriangleAndHull : public TriangleManifold
{
public:
	static Sample* Create( SampleContext* sampleContext )
	{
		return new TriangleAndHull( sampleContext );
	}

	explicit TriangleAndHull( SampleContext* sampleContext )
		: TriangleManifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 3.0f, b3Pos_zero );
		}

		//m_triangle[0] = { 1.00000000, 0, 1.00000000 };
		//m_triangle[1] = { 1.00000000, 0, 0.00000000 };
		//m_triangle[2] = { 0.00000000, 0, 0.00000000 };

		m_triangle[0x00000000] = { B3_FIX( -1.82879996 ), B3_FIX( -0.0253999997 ), B3_FIX( -0.609600008 ) };
		m_triangle[0x00000001] = { B3_FIX( -1.82879996 ), B3_FIX( -0.0253999997 ), B3_FIX( -0.406399995 ) };
		m_triangle[0x00000002] = { B3_FIX( -1.79069996 ), B3_FIX( 0.00000000 ), B3_FIX( -0.406399995 ) };

		b3Fixed SRC = B3_FIX( 0.0254f );
		b3Fixed bodyHalfWidth = 16 * SRC;
		b3Fixed bodyHalfHeight = 36 * SRC;

		m_boxHull = b3MakeBoxHull( bodyHalfWidth, bodyHalfHeight, bodyHalfWidth );

		m_transformA = b3WorldTransform_identity;
		m_transformB = b3WorldTransform_identity;
		m_transformB.p = { B3_FIX( -2.16650009f ), B3_FIX( 0.912535489f ), B3_FIX( 0.00000000f ) };

		// b3MeshEdgeFlags
		m_flags = 0;

		/*
		 *		separation	-0.0415397286	float
		type	2 '\x2'	unsigned char
		indexA	0 '\0'	unsigned char
		indexB	1 '\x1'	unsigned char
		hit	1 '\x1'	unsigned char

		 */

		m_satCache = {
			.separation = B3_FIX( -0.0107582733f ),
			.type = 1,
			.indexA = 0,
			.indexB = 4,
		};

		m_satCache = {};
		m_useCache = false;
		m_cylinder = b3CreateCylinder( B3_FIX( 0.4f ), B3_FIX( 0.05f ), B3_FIX( 0.0f ), 6 );
		m_hull = &m_boxHull.base;
	}

	~TriangleAndHull() override
	{
		b3DestroyHull( m_cylinder );
	}

	void Render() override
	{
		DrawHull( m_transformB, m_hull, MakeColor( b3_colorGreen ) );

		b3WorldTransform xf;
		xf.p = b3TransformWorldPoint( m_transformB, m_hull->center );
		xf.q = m_transformB.q;
		DrawAxes( xf, 0.1f );

		TriangleManifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == true )
		{
			if ( m_manualFeature == 1 )
			{
				m_satCache.type = b3_manualFaceAxisA;
			}
			else if ( m_manualFeature == 2 )
			{
				m_satCache.type = b3_manualFaceAxisB;
			}
			else if ( m_manualFeature == 3 )
			{
				m_satCache.type = b3_manualEdgePairAxis;
			}
		}
		else
		{
			m_satCache = {};
		}

		b3Transform xf = b3InvMulWorldTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideHullAndTriangle( &m_manifold, m_pointCapacity, m_hull, localTriangle[0], localTriangle[1], localTriangle[2],
								  m_flags, &m_satCache, true );
	}

	int m_flags;
	b3BoxHull m_boxHull;
	b3HullData* m_cylinder;
	b3HullData* m_hull;
};

static int sampleHullAndTriangle = RegisterSample( "Manifold", "Triangle vs Hull", TriangleAndHull::Create );

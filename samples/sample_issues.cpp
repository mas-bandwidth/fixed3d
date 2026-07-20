// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "imgui.h"
#include "mesh_loader.h"
#include "sample.h"
#include "gfx/draw.h"

#include "box3d/box3d.h"

#include <vector>

class Crash : public Sample
{
public:
	explicit Crash( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, SamplePos( { B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } ) );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SamplePos( { B3_FIX( 0.0f ), B3_FIX( -1.0f ), B3_FIX( 0.0f ) } );
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_gridMesh = b3CreateGridMesh( 20, 20, B3_FIX( 2.0f ), 0, true );
			b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = SamplePos( { B3_FIX( 2.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) } );
		m_bodyId1 = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
		b3CreateHullShape( m_bodyId1, &shapeDef, &box.base );

		bodyDef.position = SamplePos( { B3_FIX( -2.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) } );
		m_bodyId2 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateHullShape( m_bodyId2, &shapeDef, &box.base );
	}

	~Crash() override
	{
		b3DestroyMesh( m_gridMesh );
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Add Joint" ) )
		{
			b3WeldJointDef jointDef = b3DefaultWeldJointDef();
			jointDef.base.bodyIdA = m_bodyId1;
			jointDef.base.bodyIdB = m_bodyId2;
			b3CreateWeldJoint( m_worldId, &jointDef );
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Crash( context );
	}

	b3BodyId m_bodyId1;
	b3BodyId m_bodyId2;
	b3MeshData* m_gridMesh;
};

static int sampleCrash = RegisterSample( "Issues", "Crash", Crash::Create );

class MultiplePrismatic : public Sample
{
public:
	explicit MultiplePrismatic( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 25.0f, SamplePos( { B3_FIX( 0.0f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) } ) );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SampleOrigin();
			groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
		b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.localFrameA.p = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		jointDef.base.localFrameB.p = { B3_FIX( 0.0f ), B3_FIX( -0.6f ), B3_FIX( 0.0f ) };
		jointDef.base.drawScale = B3_FIX( 2.0f );
		jointDef.base.constraintHertz = B3_FIX( 240.0f );
		jointDef.lowerTranslation = B3_FIX( -6.0f );
		jointDef.upperTranslation = B3_FIX( 6.0f );
		jointDef.enableLimit = true;

		for ( int i = 0; i < 6; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SamplePos( 0.0f, 0.6f + 1.2f * i, 0.0f );
			bodyDef.type = b3_dynamicBody;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			jointDef.base.bodyIdB = bodyId;
			b3CreatePrismaticJoint( m_worldId, &jointDef );

			jointDef.base.bodyIdA = bodyId;
			jointDef.base.localFrameA.p = { B3_FIX( 0.0f ), B3_FIX( 0.6f ), B3_FIX( 0.0f ) };
		}

		// huge mouse force
		m_mouseForceScale = 1000000.0f;
	}

	static Sample* Create( SampleContext* context )
	{
		return new MultiplePrismatic( context );
	}
};

static int sampleMultiplePrismatic = RegisterSample( "Issues", "Multiple Prismatic", MultiplePrismatic::Create );

class HullCrash : public Sample
{
public:
	explicit HullCrash( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 5.0f, SampleOrigin() );
		}

		m_hull = nullptr;

#if 0
		// bad hull SM_Waterfall_MED_Wide_01
		b3Vec3 points[] = {
			{ 0.100183107, -498.925385, -1275.39966 }, { 0.100183107, -498.925415, 0.125000000 },
			{ 0.100183107, 486.343750, 0.125000000 },  { 0.100183107, 486.343719, -1275.39966 },
			{ -395.117462, 486.343781, -1462.43750 },  { -395.117462, 486.343750, -96.7426758 },
			{ -395.117462, -498.925415, -96.7424469 }, { -395.117462, -498.925446, -1462.52612 },
			{ -186.979691, 486.294891, -1462.47949 },  { -298.250000, 486.294891, 0.125000000 },
			{ -395.121216, 486.294891, -1462.52612 },  { -186.984360, -498.913361, -1462.48413 },
			{ -298.250000, -498.913361, 0.125000000 },
		};

#elif 1
		b3Vec3 points[] = {
			{ B3_FIX( 100.000000 ), B3_FIX( -142.292389 ), B3_FIX( 130.826111 ) },  { B3_FIX( 99.5354385 ), B3_FIX( -71.3011093 ), B3_FIX( 130.826111 ) },
			{ B3_FIX( 99.5930862 ), B3_FIX( -80.1112213 ), B3_FIX( -100.000000 ) }, { B3_FIX( 100.000000 ), B3_FIX( -142.292389 ), B3_FIX( -100.000000 ) },
			{ B3_FIX( 99.5930862 ), B3_FIX( -80.1112213 ), B3_FIX( 130.826111 ) },
		};
#else
		b3Vec3 points[] = {
			{ -11.3861933, -24.2451687, -12.0037909 }, { -11.3889809, -24.2466526, -11.9013014 },
			{ -11.3804407, -24.3151531, -12.0046492 }, { -11.3832273, -24.3166409, -11.9021587 },
			{ -14.4396200, -24.3636723, -12.1324549 }, { -14.4432650, -24.3655701, -12.0299988 },
			{ -14.4356947, -24.4337788, -12.1336164 }, { -14.4393377, -24.4356804, -12.0311594 },
		};
#endif

		static_assert( sizeof( points ) / sizeof( points[0] ) < m_capacity, "bad" );

		m_count = sizeof( points ) / sizeof( points[0] );
		for ( int i = 0; i < m_count; ++i )
		{
			m_points[i] = B3_FIX( 0.01f ) * points[i];
		}

		// This shift shouldn't be necessary but I'm doing it so the hull
		// appears on the screen.
		// for ( int i = 0; i < m_count; ++i )
		//{
		//	m_points[i] -= m_points[0];
		//	m_points[i] *= 0.01f;
		//}

		m_hull = b3CreateHull( m_points, m_count, m_count );

		(void)m_hull;
	}

	~HullCrash() override
	{
		if ( m_hull != nullptr )
		{
			b3DestroyHull( m_hull );
		}
	}

	void Render() override
	{
		b3WorldTransform originTransform = { SampleOrigin(), b3Quat_identity };
		if ( m_hull != nullptr )
		{
			DrawHull( originTransform, m_hull, MakeColor( b3_colorYellow ) );
		}
		else
		{
			for ( int i = 0; i < m_count; ++i )
			{
				DrawPoint( SamplePos( m_points[i] ), 5.0f, MakeColor( b3_colorWhite ) );
			}
		}

		DrawAxes( originTransform, 1.0f );

		Sample::Render();
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullCrash( sampleContext );
	}

	static constexpr int m_capacity = 64;
	b3HullData* m_hull;
	b3Vec3 m_points[m_capacity];
	int m_count;
};

static int sampleHullCrash = RegisterSample( "Issues", "Hull Crash", HullCrash::Create );

class ConvexJitter : public Sample
{
public:
	explicit ConvexJitter( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 10.0f, SamplePos( { B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } ) );
			
		}

		AddGroundBox( 10.0f );

		b3Fixed s = B3_FIX( 0.01f );

		{
			b3Vec3 b = { B3_FIX( -459.292877f ), B3_FIX( 217.398331f ), B3_FIX( 1.00115335f ) };
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SamplePos( { b3FixMul( s, b.x ), b3FixMul( s, b.z ) + B3_FIX( 2.0f ), b3FixMul( s, b.y ) } );
			bodyDef.rotation = { { B3_FIX( 0.0f ), B3_FIX( -0.707106769f ), B3_FIX( 0.0f ) }, B3_FIX( 0.707106769f ) };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();

			constexpr int count = 16;
			b3Vec3 points[count];
			points[0] = { B3_FIX( -44.8770714 ), B3_FIX( -91.6598053 ), B3_FIX( -1.92012548 ) };
			points[1] = { B3_FIX( -92.5001831 ), B3_FIX( 51.0151291 ), B3_FIX( 15.8006573 ) };
			points[2] = { B3_FIX( -91.0282211 ), B3_FIX( -9.44371605 ), B3_FIX( 15.6148796 ) };
			points[3] = { B3_FIX( 90.2375641 ), B3_FIX( 77.3870087 ), B3_FIX( 15.9356089 ) };
			points[4] = { B3_FIX( -85.5353241 ), B3_FIX( 91.3750992 ), B3_FIX( -1.36629653 ) };
			points[5] = { B3_FIX( 88.9092178 ), B3_FIX( -87.2975464 ), B3_FIX( -1.86754704 ) };
			points[6] = { B3_FIX( 83.7932816 ), B3_FIX( -89.8572235 ), B3_FIX( 15.4168339 ) };
			points[7] = { B3_FIX( 87.0243988 ), B3_FIX( 88.9776535 ), B3_FIX( -1.32423306 ) };
			points[8] = { B3_FIX( -91.6564941 ), B3_FIX( -85.4949493 ), B3_FIX( 15.3782759 ) };
			points[9] = { B3_FIX( -90.2922516 ), B3_FIX( -87.2074127 ), B3_FIX( -1.92012548 ) };
			points[10] = { B3_FIX( -87.2944870 ), B3_FIX( 89.9510498 ), B3_FIX( 15.9215889 ) };
			points[11] = { B3_FIX( 79.2338104 ), B3_FIX( 89.9690781 ), B3_FIX( 15.9724140 ) };
			points[12] = { B3_FIX( -91.6744461 ), B3_FIX( 81.0823212 ), B3_FIX( -1.39959598 ) };
			points[13] = { B3_FIX( 90.3452759 ), B3_FIX( -76.4459610 ), B3_FIX( 15.4588966 ) };
			points[14] = { B3_FIX( -87.4021912 ), B3_FIX( -89.2263107 ), B3_FIX( 15.3677588 ) };
			points[15] = { B3_FIX( 76.3258057 ), B3_FIX( 92.0059967 ), B3_FIX( 1.82873762 ) };

			for ( int i = 0; i < count; ++i )
			{
				b3Vec3 p = points[i];
				points[i] = { b3FixMul( s, p.x ), b3FixMul( s, p.z ), b3FixMul( s, p.y ) };
			}

			b3HullData* hull = b3CreateHull( points, count, count );

			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}

		{
			b3Vec3 b = { B3_FIX( -402.321838f ), B3_FIX( 157.310364f ), B3_FIX( 16.8169250f ) };
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SamplePos( { b3FixMul( s, b.x ), b3FixMul( s, b.z ) + B3_FIX( 2.0f ), b3FixMul( s, b.y ) } );
			bodyDef.rotation = { { B3_FIX( 0.0f ), B3_FIX( -0.00152086187f ), B3_FIX( 0.0f ) }, B3_FIX( 0.999998868f ) };
			bodyDef.type = b3_dynamicBody;

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );

			constexpr int count = 18;
			b3Vec3 points[count];
			points[0] = { B3_FIX( 29.5000000 ), B3_FIX( 17.1488495 ), B3_FIX( 0.175081104 ) };
			points[1] = { B3_FIX( 29.5000000 ), B3_FIX( -17.2990532 ), B3_FIX( 0.125000000 ) };
			points[2] = { B3_FIX( 29.4840164 ), B3_FIX( -17.3057766 ), B3_FIX( 24.0200863 ) };
			points[3] = { B3_FIX( 29.4840164 ), B3_FIX( 17.1648350 ), B3_FIX( 24.1781254 ) };
			points[4] = { B3_FIX( -29.1345520 ), B3_FIX( 17.5529804 ), B3_FIX( 0.125000000 ) };
			points[5] = { B3_FIX( -29.1345520 ), B3_FIX( 17.5529804 ), B3_FIX( 23.7899799 ) };
			points[6] = { B3_FIX( -29.1441040 ), B3_FIX( 16.9679585 ), B3_FIX( 24.3750000 ) };
			points[7] = { B3_FIX( -29.1345520 ), B3_FIX( -17.2990532 ), B3_FIX( 24.3750000 ) };
			points[8] = { B3_FIX( -29.1345520 ), B3_FIX( -17.2990532 ), B3_FIX( 0.175081253 ) };
			points[9] = { B3_FIX( 29.0720215 ), B3_FIX( 17.5529785 ), B3_FIX( 0.125000000 ) };
			points[10] = { B3_FIX( 29.0859070 ), B3_FIX( 17.5629406 ), B3_FIX( 23.8120594 ) };
			points[11] = { B3_FIX( 29.1401348 ), B3_FIX( -17.2990532 ), B3_FIX( 24.3750000 ) };
			points[12] = { B3_FIX( 29.1123581 ), B3_FIX( 16.9722290 ), B3_FIX( 24.4027710 ) };
			points[13] = { B3_FIX( 29.3944912 ), B3_FIX( 17.2543602 ), B3_FIX( 24.1206398 ) };
			points[14] = { B3_FIX( -29.1345520 ), B3_FIX( -17.2990532 ), B3_FIX( 24.0759430 ) };
			points[15] = { B3_FIX( -29.1345520 ), B3_FIX( -16.9722252 ), B3_FIX( 24.4027710 ) };
			points[16] = { B3_FIX( 29.1123619 ), B3_FIX( -16.9722271 ), B3_FIX( 24.4027729 ) };
			points[17] = { B3_FIX( 29.5000000 ), B3_FIX( 17.3429642 ), B3_FIX( 24.0000000 ) };

			for ( int i = 0; i < count; ++i )
			{
				b3Vec3 p = points[i];
				points[i] = { b3FixMul( s, p.x ), b3FixMul( s, p.z ), b3FixMul( s, p.y ) };
			}

			b3HullData* hull = b3CreateHull( points, count, count );

			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new ConvexJitter( context );
	}
};

static int sampleConvexJitter = RegisterSample( "Issues", "Convex Jitter", ConvexJitter::Create );

class SBoxMover : public Sample
{
public:
	explicit SBoxMover( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 12.0f, SampleOrigin() );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SamplePos( { B3_FIX( -10.0f ), B3_FIX( 0.0f ), B3_FIX( -10.0f ) } );
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_heightField = b3CreateGrid( 40, 40, { B3_FIX( 0.5f ), B3_FIX( 1.0f ), B3_FIX( 0.5f ) }, false );
			//m_heightField = b3CreateWave( 40, 40, {1.0f, 2.0f, 1.0f}, 0.02f, 0.04f, false );
			b3CreateHeightFieldShape( groundId, &shapeDef, m_heightField );

			m_gridMesh = b3CreateGridMesh( 40, 40, B3_FIX( 0.5f ), 1, true );
			//b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SampleOrigin();
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			// m_boxMesh = b3CreateBoxMesh( { 0.0f, 1.0f, 0.0f }, { 1.0f, 1.0f, 1.0f }, true );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_boxMesh = b3CreatePlatformMesh( { B3_FIX( 0.0f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ), B3_FIX( 2.0f ), B3_FIX( 5.0f ) );
			b3Vec3 scale = b3Vec3_one;
			b3CreateMeshShape( groundId, &shapeDef, m_boxMesh, scale );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = SamplePos( { B3_FIX( 0.0f ), B3_FIX( 3.5f ), B3_FIX( 0.0f ) } );
			bodyDef.motionLocks.angularX = true;
			bodyDef.motionLocks.angularY = true;
			bodyDef.motionLocks.angularZ = true;
			bodyDef.enableContactRecycling = false;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.25f ), B3_FIX( 1.0f ), B3_FIX( 0.25f ) );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}

	~SBoxMover() override
	{
		b3DestroyMesh( m_boxMesh );
		b3DestroyHeightField( m_heightField );
		b3DestroyMesh( m_gridMesh );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { SamplePos( { B3_FIX( 0.0f ), B3_FIX( 1.1f ), B3_FIX( 0.0f ) } ), b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 3.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SBoxMover( context );
	}

	b3MeshData* m_boxMesh;
	b3HeightFieldData* m_heightField;
	b3MeshData* m_gridMesh;
};

static int sampleBoxMesh = RegisterSample( "Issues", "s&box mover", SBoxMover::Create );

class CapsuleMeshBug : public Sample
{
public:
	explicit CapsuleMeshBug( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 20.0f, 10.0f, 30.0f, SamplePos( { B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } ) );
		}

		// --- Ground plane ---
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SampleOrigin();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull ground = b3MakeBoxHull( B3_FIX( 50.0f ), B3_FIX( 0.1f ), B3_FIX( 50.0f ) );
			b3CreateHullShape( body, &shapeDef, &ground.base );
		}

		// --- Building mesh on top of ground ---
		m_building = CreateMeshData( "data/meshes/building.obj", 1.0f, false, false, true, true );
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = SamplePos( { B3_FIX( 0.0f ), B3_FIX( 0.1f ), B3_FIX( 0.0f ) } );
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( body, &shapeDef, m_building, b3Vec3_one );
		}

		// --- Locked capsule (same setup as player controller body) ---
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = SamplePos( { B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 10.0f ) } );
			bodyDef.motionLocks.angularX = true;
			bodyDef.motionLocks.angularY = true;
			bodyDef.motionLocks.angularZ = true;
			bodyDef.enableSleep = false;
			bodyDef.enableContactRecycling = false;
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.friction = B3_FIX( 0.3f );
			shapeDef.baseMaterial.customColor = b3_colorMagenta;

			b3Capsule capsule = { { B3_FIX( 0.0f ), B3_FIX( -0.5f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}
	}

	~CapsuleMeshBug() override
	{
		if ( m_building )
		{
			b3DestroyMesh( m_building );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new CapsuleMeshBug( context );
	}

	b3MeshData* m_building = nullptr;
};

static int sampleIndex = RegisterSample( "Issues", "Capsule Mesh", CapsuleMeshBug::Create );


// Reproduces s&box rigid body character ghost collisions on a FLAT floor.
//
// The s&box player is a fixed rotation dynamic body (zero radius box hull) moved by setting
// velocity. The floor here is modeled on an s&box map area that ghosts badly: concrete slabs
// and a run of parallel chamfered beams over deep pits. Every walkable vertex is at exactly
// y = 0 - there is nothing to climb and nothing to trip on. The only mesh features are AT or
// BELOW the walkable plane: chamfer facets sloping down from beam tops, pit walls and floors,
// T-junction seams between tiles tessellated at different resolutions, and a chunk seam where
// two mesh shapes meet (separate contact pairs, like s&box world chunks).
//
// Walking back and forth still launches the body upward (60-190 inch/s at run speed). No shape
// casts, no step up, no ground snapping - any upward velocity spike is a ghost collision from
// speculative hull vs triangle contacts against below-plane geometry.
class SBoxGhostCollisions : public Sample
{
public:
	// s&box works in inches: 1 unit = 0.0254 m (40 units per meter)
	static constexpr float SRC = 0.0254f;

	// Floor layout (s&box inches)
	static constexpr int m_halfLengthU = 256; // strip half length along x
	static constexpr int m_halfWidthU = 64;	  // strip half width along z
	static constexpr int m_tileSizeU = 32;	  // slab tile stride

	// Beam section: beams run along z, the character walks along x across them.
	// Tops are 12 wide with 1.5 chamfers, pits between are 10 wide and 24 deep.
	// The 16 wide hull always spans the 13 gap between flat tops.
	static constexpr float m_beamPitchU = 22.0f;
	static constexpr float m_beamWidthU = 12.0f;
	static constexpr float m_chamferWidthU = 1.5f;
	static constexpr float m_chamferDropU = 1.0f;
	static constexpr float m_pitDepthU = 24.0f;
	static constexpr int m_beamCount = 9;
	static constexpr float m_beamRegion0 = -94.0f; // first beam start
	static constexpr float m_beamRegion1 = 94.0f;  // last beam end

	// Character (s&box player: 16 wide zero radius box hull, 72 tall, mass 500)
	static constexpr float m_bodyHalfWidth = 16.0f * SRC;
	static constexpr float m_bodyHalfHeight = 36.0f * SRC;
	static constexpr float m_characterMass = 500.0f;

	explicit SBoxGhostCollisions( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 90.0f, 25.0f, 10.0f, SamplePos( { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) } ) );
		}

		// Two chunks meeting at x = 0, each its own body and mesh shape, so seam contacts live
		// in separate contact pairs like s&box world mesh chunks. A beam top straddles the seam.
		CreateFloorChunk( 0, -(float)m_halfLengthU, 0.0f );
		CreateFloorChunk( 1, 0.0f, (float)m_halfLengthU );

		// Character
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = SamplePos( -m_walkRangeX, m_bodyHalfHeight + 0.1f, 0.0f );
			bodyDef.motionLocks.angularX = true;
			bodyDef.motionLocks.angularY = true;
			bodyDef.motionLocks.angularZ = true;
			bodyDef.enableSleep = false;
			bodyDef.enableContactRecycling = false;
			bodyDef.gravityScale = B3_FIX( 2.03f ); // s&box gravity: 800 inch/s^2
			bodyDef.name = "character";
			m_characterId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.friction = B3_FIX( 0.0f );
			shapeDef.baseMaterial.restitution = B3_FIX( 0.0f );
			//shapeDef.baseMaterial.customColor = b3_colorLimeGreen;

			float volume = 8.0f * m_bodyHalfWidth * m_bodyHalfHeight * m_bodyHalfWidth;
			shapeDef.density = b3FixFromFloat( m_characterMass / volume );
			shapeDef.enableSpeculativeContact = false;

			b3BoxHull box = b3MakeBoxHull( b3FixFromFloat( m_bodyHalfWidth ), b3FixFromFloat( m_bodyHalfHeight ),
										   b3FixFromFloat( m_bodyHalfWidth ) );
			b3CreateHullShape( m_characterId, &shapeDef, &box.base );
		}

		m_walkDirectionX = 1.0f;
		m_walkDirectionZ = 1.0f;
		m_walkSpeedX = 350.0f * SRC; // s&box run speed
		m_walkSpeedZ = 20.0f * SRC;
		m_launchCount = 0;
		m_maxLaunchSpeed = 0.0f;
		m_launchMarkerCount = 0;
		m_wasLaunched = false;

		m_stepWhilePaused = true;
	}

	~SBoxGhostCollisions() override
	{
		b3DestroyMesh( m_chunkMesh[0] );
		b3DestroyMesh( m_chunkMesh[1] );
	}

	// Deterministic integer hash for tile tessellation selection
	static uint32_t Hash( uint32_t x )
	{
		x ^= x >> 16;
		x *= 0x7feb352du;
		x ^= x >> 15;
		x *= 0x846ca68bu;
		x ^= x >> 16;
		return x;
	}

	static void EmitTriangle( std::vector<b3Vec3>& vertices, std::vector<int32_t>& indices, b3Vec3 a, b3Vec3 b, b3Vec3 c )
	{
		int32_t base = (int32_t)vertices.size();
		vertices.push_back( a );
		vertices.push_back( b );
		vertices.push_back( c );
		indices.push_back( base );
		indices.push_back( base + 1 );
		indices.push_back( base + 2 );
	}

	// Horizontal patch at height y spanning [x0,x1]x[z0,z1] (inches), normal +y
	static void EmitPatch( std::vector<b3Vec3>& vertices, std::vector<int32_t>& indices, float x0, float x1, float z0, float z1,
						   float y, float cell )
	{
		if ( x1 - x0 < 0.01f )
		{
			return;
		}

		int countX = (int)( ( x1 - x0 ) / cell + 0.99f );
		int countZ = (int)( ( z1 - z0 ) / cell + 0.99f );

		for ( int ix = 0; ix < countX; ++ix )
		{
			for ( int iz = 0; iz < countZ; ++iz )
			{
				float cx0 = x0 + ( x1 - x0 ) * (float)ix / (float)countX;
				float cx1 = x0 + ( x1 - x0 ) * (float)( ix + 1 ) / (float)countX;
				float cz0 = z0 + ( z1 - z0 ) * (float)iz / (float)countZ;
				float cz1 = z0 + ( z1 - z0 ) * (float)( iz + 1 ) / (float)countZ;

				b3Vec3 a = { b3FixFromFloat( SRC * cx0 ), b3FixFromFloat( SRC * y ), b3FixFromFloat( SRC * cz0 ) };
				b3Vec3 b = { b3FixFromFloat( SRC * cx1 ), b3FixFromFloat( SRC * y ), b3FixFromFloat( SRC * cz0 ) };
				b3Vec3 c = { b3FixFromFloat( SRC * cx1 ), b3FixFromFloat( SRC * y ), b3FixFromFloat( SRC * cz1 ) };
				b3Vec3 d = { b3FixFromFloat( SRC * cx0 ), b3FixFromFloat( SRC * y ), b3FixFromFloat( SRC * cz1 ) };

				// Alternate the split diagonal like typical cooked map data
				if ( ( ix + iz ) & 1 )
				{
					EmitTriangle( vertices, indices, a, d, c );
					EmitTriangle( vertices, indices, a, c, b );
				}
				else
				{
					EmitTriangle( vertices, indices, a, d, b );
					EmitTriangle( vertices, indices, b, d, c );
				}
			}
		}
	}

	// Sloped strip from edge (xLow, yLow) to edge (xHigh, yHigh) spanning the full z width
	static void EmitSlope( std::vector<b3Vec3>& vertices, std::vector<int32_t>& indices, float xLow, float yLow, float xHigh,
						   float yHigh, float zCell )
	{
		int countZ = (int)( 2.0f * (float)m_halfWidthU / zCell + 0.99f );
		for ( int iz = 0; iz < countZ; ++iz )
		{
			float z0 = -(float)m_halfWidthU + 2.0f * (float)m_halfWidthU * (float)iz / (float)countZ;
			float z1 = -(float)m_halfWidthU + 2.0f * (float)m_halfWidthU * (float)( iz + 1 ) / (float)countZ;

			b3Vec3 l0 = { b3FixFromFloat( SRC * xLow ), b3FixFromFloat( SRC * yLow ), b3FixFromFloat( SRC * z0 ) };
			b3Vec3 l1 = { b3FixFromFloat( SRC * xLow ), b3FixFromFloat( SRC * yLow ), b3FixFromFloat( SRC * z1 ) };
			b3Vec3 h0 = { b3FixFromFloat( SRC * xHigh ), b3FixFromFloat( SRC * yHigh ), b3FixFromFloat( SRC * z0 ) };
			b3Vec3 h1 = { b3FixFromFloat( SRC * xHigh ), b3FixFromFloat( SRC * yHigh ), b3FixFromFloat( SRC * z1 ) };

			EmitTriangle( vertices, indices, l0, l1, h1 );
			EmitTriangle( vertices, indices, l0, h1, h0 );
		}
	}

	// Vertical wall at x from y0 (bottom) to y1 (top). facing = +1 faces +x, -1 faces -x
	static void EmitWall( std::vector<b3Vec3>& vertices, std::vector<int32_t>& indices, float x, float y0, float y1, int facing,
						  float zCell )
	{
		int countZ = (int)( 2.0f * (float)m_halfWidthU / zCell + 0.99f );
		for ( int iz = 0; iz < countZ; ++iz )
		{
			float z0 = -(float)m_halfWidthU + 2.0f * (float)m_halfWidthU * (float)iz / (float)countZ;
			float z1 = -(float)m_halfWidthU + 2.0f * (float)m_halfWidthU * (float)( iz + 1 ) / (float)countZ;

			b3Vec3 b0 = { b3FixFromFloat( SRC * x ), b3FixFromFloat( SRC * y0 ), b3FixFromFloat( SRC * z0 ) };
			b3Vec3 b1 = { b3FixFromFloat( SRC * x ), b3FixFromFloat( SRC * y0 ), b3FixFromFloat( SRC * z1 ) };
			b3Vec3 t0 = { b3FixFromFloat( SRC * x ), b3FixFromFloat( SRC * y1 ), b3FixFromFloat( SRC * z0 ) };
			b3Vec3 t1 = { b3FixFromFloat( SRC * x ), b3FixFromFloat( SRC * y1 ), b3FixFromFloat( SRC * z1 ) };

			if ( facing > 0 )
			{
				EmitTriangle( vertices, indices, b0, b1, t1 );
				EmitTriangle( vertices, indices, b0, t1, t0 );
			}
			else
			{
				EmitTriangle( vertices, indices, b0, t0, t1 );
				EmitTriangle( vertices, indices, b0, t1, b1 );
			}
		}
	}

	// Clip [a0,a1] to [c0,c1]
	static bool ClipSpan( float a0, float a1, float c0, float c1, float* o0, float* o1 )
	{
		*o0 = a0 > c0 ? a0 : c0;
		*o1 = a1 < c1 ? a1 : c1;
		return *o1 - *o0 > 0.01f;
	}

	void CreateFloorChunk( int chunk, float x0U, float x1U )
	{
		std::vector<b3Vec3> vertices;
		std::vector<int32_t> indices;

		float s0, s1;

		// --- Concrete slabs at y = 0 outside the beam region ---
		// Tiles tessellate at a hash-picked resolution so neighbors meet with T-junctions,
		// like cooked s&box map collision.
		float slabSpans[2][2] = { { -(float)m_halfLengthU, m_beamRegion0 }, { m_beamRegion1, (float)m_halfLengthU } };
		for ( int i = 0; i < 2; ++i )
		{
			if ( ClipSpan( slabSpans[i][0], slabSpans[i][1], x0U, x1U, &s0, &s1 ) == false )
			{
				continue;
			}

			for ( float tx = s0; tx < s1; tx += (float)m_tileSizeU )
			{
				float tx1 = b3MinFloat( tx + (float)m_tileSizeU, s1 );
				for ( int tz = -m_halfWidthU; tz < m_halfWidthU; tz += m_tileSizeU )
				{
					uint32_t h = Hash( (uint32_t)( (int)tx * 73856093 ) ^ (uint32_t)( tz * 19349663 ) ^
									   (uint32_t)( chunk * 2654435761u ) );
					float cells[3] = { 4.0f, 8.0f, 16.0f };
					EmitPatch( vertices, indices, tx, tx1, (float)tz, (float)( tz + m_tileSizeU ), 0.0f, cells[h % 3] );
				}
			}
		}

		// --- Beam section: flat tops at y = 0, chamfers dropping to pits ---
		float pitTop = -m_chamferDropU;
		float pitBottom = -m_pitDepthU;

		for ( int k = 0; k < m_beamCount; ++k )
		{
			float bx = m_beamRegion0 + m_beamPitchU * (float)k;
			bool pitLeft = k > 0;
			bool pitRight = k < m_beamCount - 1;

			// Flat top (flush with the slab on outer sides)
			float top0 = pitLeft ? bx + m_chamferWidthU : bx;
			float top1 = pitRight ? bx + m_beamWidthU - m_chamferWidthU : bx + m_beamWidthU;
			if ( ClipSpan( top0, top1, x0U, x1U, &s0, &s1 ) )
			{
				EmitPatch( vertices, indices, s0, s1, -(float)m_halfWidthU, (float)m_halfWidthU, 0.0f, 8.0f );
			}

			// Chamfers sloping below the walkable plane
			if ( pitLeft && bx >= x0U && bx < x1U )
			{
				EmitSlope( vertices, indices, bx, pitTop, bx + m_chamferWidthU, 0.0f, 8.0f );
			}
			if ( pitRight && bx + m_beamWidthU > x0U && bx + m_beamWidthU <= x1U )
			{
				EmitSlope( vertices, indices, bx + m_beamWidthU, pitTop, bx + m_beamWidthU - m_chamferWidthU, 0.0f, 8.0f );
			}

			// Pit to the right of this beam
			if ( pitRight )
			{
				float pitL = bx + m_beamWidthU;
				float pitR = bx + m_beamPitchU;
				if ( pitL >= x0U && pitL < x1U )
				{
					EmitWall( vertices, indices, pitL, pitBottom, pitTop, +1, 16.0f );
				}
				if ( pitR > x0U && pitR <= x1U )
				{
					EmitWall( vertices, indices, pitR, pitBottom, pitTop, -1, 16.0f );
				}
				if ( ClipSpan( pitL, pitR, x0U, x1U, &s0, &s1 ) )
				{
					EmitPatch( vertices, indices, s0, s1, -(float)m_halfWidthU, (float)m_halfWidthU, pitBottom, 16.0f );
				}
			}
		}

		b3MeshDef meshDef = {};
		meshDef.vertices = vertices.data();
		meshDef.indices = indices.data();
		meshDef.vertexCount = (int)vertices.size();
		meshDef.triangleCount = (int)( indices.size() / 3 );
		meshDef.weldVertices = true;
		meshDef.weldTolerance = B3_FIX( 0.005f ); // == B3_LINEAR_SLOP, same as s&box
		meshDef.identifyEdges = true;

		m_chunkMesh[chunk] = b3CreateMesh( &meshDef, nullptr, 0 );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = SampleOrigin();
		b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( body, &shapeDef, m_chunkMesh[chunk], b3Vec3_one );
	}

	void Step() override
	{
		// Drive the character with pure velocity control: keep the solver's vertical velocity,
		// set the horizontal velocity. This is how the s&box player controller moves.
		// Walk-range and grounded checks compare against authored scene-local constants,
		// so convert the world readback to the sample-local frame.
		b3Vec3 position = SampleLocal( b3Body_GetPosition( m_characterId ) );
		if ( position.x > b3FixFromFloat( m_walkRangeX ) )
		{
			m_walkDirectionX = -1.0f;
		}
		else if ( position.x < b3FixFromFloat( -m_walkRangeX ) )
		{
			m_walkDirectionX = 1.0f;
		}

		if ( position.z > b3FixFromFloat( m_walkRangeZ ) )
		{
			m_walkDirectionZ = -1.0f;
		}
		else if ( position.z < b3FixFromFloat( -m_walkRangeZ ) )
		{
			m_walkDirectionZ= 1.0f;
		}

		b3Vec3 velocity = b3Body_GetLinearVelocity( m_characterId );
		velocity.x = b3FixFromFloat( m_walkDirectionX * m_walkSpeedX );
		velocity.z = b3FixFromFloat( m_walkDirectionZ * m_walkSpeedZ );
		b3Body_SetLinearVelocity( m_characterId, velocity );

		Sample::Step();

		if ( m_didStep )
		{
			// The walkable plane is exactly y = 0, so the grounded body center never rises above
			// rest height. Any upward velocity spike while grounded is a ghost collision: there
			// is nothing to climb and nothing to bounce off.
			position = SampleLocal( b3Body_GetPosition( m_characterId ) );
			velocity = b3Body_GetLinearVelocity( m_characterId );

			bool grounded = position.y < b3FixFromFloat( m_bodyHalfHeight + 0.01f + 4.0f * SRC );
			bool launched = velocity.y > b3FixFromFloat( m_launchThreshold );

			if ( grounded && launched && m_wasLaunched == false )
			{
				m_launchCount += 1;
				m_maxLaunchSpeed = b3MaxFloat( m_maxLaunchSpeed, b3FixToFloat( velocity.y ) );

				if ( m_launchMarkerCount < m_markerCapacity )
				{
					// Markers are drawn in world space; position is sample-local here.
					m_launchMarkers[m_launchMarkerCount] = SamplePos( position );
					m_launchMarkerCount += 1;
				}
			}

			m_wasLaunched = launched;
		}

		for ( int i = 0; i < m_launchMarkerCount; ++i )
		{
			DrawPoint( m_launchMarkers[i], 8.0f, MakeColor( b3_colorRed ) );
		}

		b3Vec3 currentVelocity = b3Body_GetLinearVelocity( m_characterId );
		DrawTextLine( "ghost launches: %d, worst: %.2f m/s (%.0f inch/s)", m_launchCount, m_maxLaunchSpeed,
					  m_maxLaunchSpeed / SRC );
		DrawTextLine( "vertical velocity: %.2f m/s", b3FixToDouble( currentVelocity.y ) );
	}

	bool DrawControls() override
	{
		float speedUX = m_walkSpeedX / SRC;
		if ( ImGui::SliderFloat( "Walk Speed X (inch/s)", &speedUX, 100.0f, 400.0f, "%.0f" ) )
		{
			m_walkSpeedX = speedUX * SRC;
		}

		float speedUZ = m_walkSpeedZ / SRC;
		if ( ImGui::SliderFloat( "Walk Speed Z (inch/s)", &speedUZ, 10.0f, 100.0f, "%.0f" ) )
		{
			m_walkSpeedZ = speedUZ * SRC;
		}

		if ( ImGui::Button( "Reset Counters" ) )
		{
			m_launchCount = 0;
			m_maxLaunchSpeed = 0.0f;
			m_launchMarkerCount = 0;
		}

		ImGui::Text( "Launches: %d", m_launchCount );
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new SBoxGhostCollisions( context );
	}

	static constexpr float m_walkRangeX = 3.5f;		 // turn around beyond +/- this x (meters)
	static constexpr float m_walkRangeZ = 0.5f;		 // turn around beyond +/- this x (meters)
	static constexpr float m_launchThreshold = 0.5f; // upward m/s counted as a ghost launch (~20 inch/s)
	static constexpr int m_markerCapacity = 64;

	b3MeshData* m_chunkMesh[2] = {};
	b3BodyId m_characterId;
	float m_walkDirectionX;
	float m_walkDirectionZ;
	float m_walkSpeedX;
	float m_walkSpeedZ;
	int m_launchCount;
	float m_maxLaunchSpeed;
	bool m_wasLaunched;
	b3Pos m_launchMarkers[m_markerCapacity];
	int m_launchMarkerCount;
};

static int sampleSBoxGhostCollisions = RegisterSample( "Issues", "s&box Ghost Collisions", SBoxGhostCollisions::Create );

static const b3Vec3 s_metalWheel1Verts[317] = {
	{ B3_FIX( 0.010279f ), B3_FIX( 0.086341f ), B3_FIX( 0.051768f ) },	{ B3_FIX( -0.010314f ), B3_FIX( -0.084708f ), B3_FIX( 0.051804f ) },	{ B3_FIX( 0.010279f ), B3_FIX( -0.084708f ), B3_FIX( 0.051768f ) },
	{ B3_FIX( -0.010314f ), B3_FIX( 0.086341f ), B3_FIX( 0.051804f ) },	{ B3_FIX( 0.029138f ), B3_FIX( -0.084708f ), B3_FIX( -0.043911f ) },	{ B3_FIX( 0.043724f ), B3_FIX( -0.084708f ), B3_FIX( -0.029376f ) },
	{ B3_FIX( 0.010098f ), B3_FIX( -0.084708f ), B3_FIX( -0.051759f ) },	{ B3_FIX( -0.010494f ), B3_FIX( -0.084708f ), B3_FIX( -0.051723f ) }, { B3_FIX( 0.051638f ), B3_FIX( -0.084708f ), B3_FIX( -0.010364f ) },
	{ B3_FIX( 0.051674f ), B3_FIX( -0.084708f ), B3_FIX( 0.010229f ) },	{ B3_FIX( 0.043827f ), B3_FIX( -0.084708f ), B3_FIX( 0.029268f ) },	{ B3_FIX( 0.029291f ), B3_FIX( -0.084708f ), B3_FIX( 0.043855f ) },
	{ B3_FIX( -0.029353f ), B3_FIX( -0.084708f ), B3_FIX( 0.043957f ) },	{ B3_FIX( -0.051889f ), B3_FIX( -0.084708f ), B3_FIX( -0.010183f ) }, { B3_FIX( -0.051853f ), B3_FIX( -0.084708f ), B3_FIX( 0.010409f ) },
	{ B3_FIX( -0.043939f ), B3_FIX( -0.084708f ), B3_FIX( 0.029421f ) },	{ B3_FIX( -0.029506f ), B3_FIX( -0.084708f ), B3_FIX( -0.043809f ) }, { B3_FIX( -0.044042f ), B3_FIX( -0.084708f ), B3_FIX( -0.029222f ) },
	{ B3_FIX( -0.029353f ), B3_FIX( 0.086341f ), B3_FIX( 0.043957f ) },	{ B3_FIX( 0.043724f ), B3_FIX( 0.086341f ), B3_FIX( -0.029376f ) },	{ B3_FIX( 0.029138f ), B3_FIX( 0.086341f ), B3_FIX( -0.043911f ) },
	{ B3_FIX( 0.010098f ), B3_FIX( 0.086341f ), B3_FIX( -0.051759f ) },	{ B3_FIX( 0.051638f ), B3_FIX( 0.086341f ), B3_FIX( -0.010364f ) },	{ B3_FIX( 0.051674f ), B3_FIX( 0.086341f ), B3_FIX( 0.010229f ) },
	{ B3_FIX( -0.044042f ), B3_FIX( 0.086341f ), B3_FIX( -0.029222f ) },	{ B3_FIX( 0.043827f ), B3_FIX( 0.086341f ), B3_FIX( 0.029268f ) },	{ B3_FIX( -0.051889f ), B3_FIX( 0.086341f ), B3_FIX( -0.010183f ) },
	{ B3_FIX( -0.043939f ), B3_FIX( 0.086341f ), B3_FIX( 0.029421f ) },	{ B3_FIX( -0.029506f ), B3_FIX( 0.086341f ), B3_FIX( -0.043809f ) },	{ B3_FIX( 0.035354f ), B3_FIX( 0.070897f ), B3_FIX( 0.035658f ) },
	{ B3_FIX( 0.035354f ), B3_FIX( -0.069397f ), B3_FIX( 0.035658f ) },	{ B3_FIX( -0.035498f ), B3_FIX( 0.070897f ), B3_FIX( 0.035658f ) },	{ B3_FIX( -0.035498f ), B3_FIX( -0.069397f ), B3_FIX( 0.035658f ) },
	{ B3_FIX( 0.035354f ), B3_FIX( -0.069397f ), B3_FIX( 0.365503f ) },	{ B3_FIX( 0.035354f ), B3_FIX( 0.070897f ), B3_FIX( 0.365503f ) },	{ B3_FIX( -0.035498f ), B3_FIX( 0.070897f ), B3_FIX( 0.365503f ) },
	{ B3_FIX( -0.035498f ), B3_FIX( -0.069397f ), B3_FIX( 0.365503f ) },	{ B3_FIX( 0.035354f ), B3_FIX( 0.070897f ), B3_FIX( -0.365033f ) },	{ B3_FIX( 0.035354f ), B3_FIX( -0.069397f ), B3_FIX( -0.365033f ) },
	{ B3_FIX( -0.035498f ), B3_FIX( 0.070897f ), B3_FIX( -0.365033f ) },	{ B3_FIX( -0.035498f ), B3_FIX( -0.069397f ), B3_FIX( -0.365033f ) }, { B3_FIX( 0.035354f ), B3_FIX( -0.069397f ), B3_FIX( -0.035187f ) },
	{ B3_FIX( 0.035354f ), B3_FIX( 0.070897f ), B3_FIX( -0.035187f ) },	{ B3_FIX( -0.035498f ), B3_FIX( 0.070897f ), B3_FIX( -0.035187f ) },	{ B3_FIX( -0.035498f ), B3_FIX( -0.069397f ), B3_FIX( -0.035187f ) },
	{ B3_FIX( -0.035494f ), B3_FIX( -0.069397f ), B3_FIX( -0.035191f ) }, { B3_FIX( -0.365340f ), B3_FIX( -0.069397f ), B3_FIX( -0.035191f ) }, { B3_FIX( -0.035494f ), B3_FIX( 0.070897f ), B3_FIX( -0.035191f ) },
	{ B3_FIX( -0.365340f ), B3_FIX( 0.070897f ), B3_FIX( -0.035191f ) },	{ B3_FIX( -0.365340f ), B3_FIX( -0.069397f ), B3_FIX( 0.035661f ) },	{ B3_FIX( -0.035494f ), B3_FIX( -0.069397f ), B3_FIX( 0.035661f ) },
	{ B3_FIX( -0.035494f ), B3_FIX( 0.070897f ), B3_FIX( 0.035661f ) },	{ B3_FIX( -0.365340f ), B3_FIX( 0.070897f ), B3_FIX( 0.035661f ) },	{ B3_FIX( 0.035350f ), B3_FIX( 0.070897f ), B3_FIX( -0.035191f ) },
	{ B3_FIX( 0.365196f ), B3_FIX( 0.070897f ), B3_FIX( -0.035191f ) },	{ B3_FIX( 0.035350f ), B3_FIX( -0.069397f ), B3_FIX( -0.035191f ) },	{ B3_FIX( 0.365196f ), B3_FIX( -0.069397f ), B3_FIX( -0.035191f ) },
	{ B3_FIX( 0.365196f ), B3_FIX( 0.070897f ), B3_FIX( 0.035661f ) },	{ B3_FIX( 0.035350f ), B3_FIX( 0.070897f ), B3_FIX( 0.035661f ) },	{ B3_FIX( 0.035350f ), B3_FIX( -0.069397f ), B3_FIX( 0.035661f ) },
	{ B3_FIX( 0.365196f ), B3_FIX( -0.069397f ), B3_FIX( 0.035661f ) },	{ B3_FIX( -0.070802f ), B3_FIX( 0.086337f ), B3_FIX( 0.355420f ) },	{ B3_FIX( -0.070802f ), B3_FIX( -0.084705f ), B3_FIX( 0.355420f ) },
	{ B3_FIX( -0.097081f ), B3_FIX( 0.086335f ), B3_FIX( 0.487537f ) },	{ B3_FIX( -0.097081f ), B3_FIX( -0.084704f ), B3_FIX( 0.487537f ) },	{ B3_FIX( -0.000108f ), B3_FIX( 0.086337f ), B3_FIX( 0.362383f ) },
	{ B3_FIX( -0.000108f ), B3_FIX( 0.086335f ), B3_FIX( 0.497088f ) },	{ B3_FIX( -0.000108f ), B3_FIX( -0.084704f ), B3_FIX( 0.497088f ) },	{ B3_FIX( -0.000108f ), B3_FIX( -0.084705f ), B3_FIX( 0.362383f ) },
	{ B3_FIX( -0.138787f ), B3_FIX( -0.084705f ), B3_FIX( 0.334799f ) },	{ B3_FIX( -0.138787f ), B3_FIX( 0.086337f ), B3_FIX( 0.334799f ) },	{ B3_FIX( -0.070810f ), B3_FIX( -0.084705f ), B3_FIX( 0.355419f ) },
	{ B3_FIX( -0.070810f ), B3_FIX( 0.086337f ), B3_FIX( 0.355419f ) },	{ B3_FIX( -0.097090f ), B3_FIX( 0.086335f ), B3_FIX( 0.487536f ) },	{ B3_FIX( -0.190336f ), B3_FIX( 0.086335f ), B3_FIX( 0.459250f ) },
	{ B3_FIX( -0.190336f ), B3_FIX( -0.084704f ), B3_FIX( 0.459250f ) },	{ B3_FIX( -0.097090f ), B3_FIX( -0.084704f ), B3_FIX( 0.487536f ) },	{ B3_FIX( -0.138795f ), B3_FIX( 0.086337f ), B3_FIX( 0.334796f ) },
	{ B3_FIX( -0.138795f ), B3_FIX( -0.084705f ), B3_FIX( 0.334796f ) },	{ B3_FIX( -0.201442f ), B3_FIX( 0.086337f ), B3_FIX( 0.301310f ) },	{ B3_FIX( -0.201442f ), B3_FIX( -0.084705f ), B3_FIX( 0.301310f ) },
	{ B3_FIX( -0.276280f ), B3_FIX( -0.084704f ), B3_FIX( 0.413313f ) },	{ B3_FIX( -0.276280f ), B3_FIX( 0.086335f ), B3_FIX( 0.413313f ) },	{ B3_FIX( -0.190344f ), B3_FIX( 0.086335f ), B3_FIX( 0.459247f ) },
	{ B3_FIX( -0.190344f ), B3_FIX( -0.084704f ), B3_FIX( 0.459247f ) },	{ B3_FIX( -0.201450f ), B3_FIX( -0.084705f ), B3_FIX( 0.301306f ) },	{ B3_FIX( -0.256361f ), B3_FIX( -0.084705f ), B3_FIX( 0.256241f ) },
	{ B3_FIX( -0.201450f ), B3_FIX( 0.086337f ), B3_FIX( 0.301306f ) },	{ B3_FIX( -0.256361f ), B3_FIX( 0.086337f ), B3_FIX( 0.256241f ) },	{ B3_FIX( -0.351612f ), B3_FIX( 0.086335f ), B3_FIX( 0.351492f ) },
	{ B3_FIX( -0.351612f ), B3_FIX( -0.084704f ), B3_FIX( 0.351492f ) },	{ B3_FIX( -0.276288f ), B3_FIX( 0.086335f ), B3_FIX( 0.413309f ) },	{ B3_FIX( -0.276288f ), B3_FIX( -0.084704f ), B3_FIX( 0.413309f ) },
	{ B3_FIX( -0.301431f ), B3_FIX( -0.084705f ), B3_FIX( 0.201325f ) },	{ B3_FIX( -0.413435f ), B3_FIX( -0.084704f ), B3_FIX( 0.276163f ) },	{ B3_FIX( -0.413435f ), B3_FIX( 0.086335f ), B3_FIX( 0.276163f ) },
	{ B3_FIX( -0.301431f ), B3_FIX( 0.086337f ), B3_FIX( 0.201325f ) },	{ B3_FIX( -0.256367f ), B3_FIX( -0.084705f ), B3_FIX( 0.256236f ) },	{ B3_FIX( -0.256367f ), B3_FIX( 0.086337f ), B3_FIX( 0.256236f ) },
	{ B3_FIX( -0.351618f ), B3_FIX( 0.086335f ), B3_FIX( 0.351487f ) },	{ B3_FIX( -0.351618f ), B3_FIX( -0.084704f ), B3_FIX( 0.351487f ) },	{ B3_FIX( -0.334922f ), B3_FIX( -0.084705f ), B3_FIX( 0.138670f ) },
	{ B3_FIX( -0.459374f ), B3_FIX( -0.084704f ), B3_FIX( 0.190220f ) },	{ B3_FIX( -0.459374f ), B3_FIX( 0.086335f ), B3_FIX( 0.190220f ) },	{ B3_FIX( -0.334922f ), B3_FIX( 0.086337f ), B3_FIX( 0.138670f ) },
	{ B3_FIX( -0.301436f ), B3_FIX( 0.086337f ), B3_FIX( 0.201318f ) },	{ B3_FIX( -0.301436f ), B3_FIX( -0.084705f ), B3_FIX( 0.201318f ) },	{ B3_FIX( -0.413440f ), B3_FIX( 0.086335f ), B3_FIX( 0.276156f ) },
	{ B3_FIX( -0.413440f ), B3_FIX( -0.084704f ), B3_FIX( 0.276156f ) },	{ B3_FIX( -0.487663f ), B3_FIX( -0.084704f ), B3_FIX( 0.096966f ) },	{ B3_FIX( -0.355546f ), B3_FIX( -0.084705f ), B3_FIX( 0.070686f ) },
	{ B3_FIX( -0.459377f ), B3_FIX( -0.084704f ), B3_FIX( 0.190212f ) },	{ B3_FIX( -0.334926f ), B3_FIX( -0.084705f ), B3_FIX( 0.138663f ) },	{ B3_FIX( -0.355546f ), B3_FIX( 0.086337f ), B3_FIX( 0.070686f ) },
	{ B3_FIX( -0.334926f ), B3_FIX( 0.086337f ), B3_FIX( 0.138663f ) },	{ B3_FIX( -0.487663f ), B3_FIX( 0.086335f ), B3_FIX( 0.096966f ) },	{ B3_FIX( -0.459377f ), B3_FIX( 0.086335f ), B3_FIX( 0.190212f ) },
	{ B3_FIX( -0.362511f ), B3_FIX( -0.084705f ), B3_FIX( -0.000016f ) }, { B3_FIX( -0.355549f ), B3_FIX( -0.084705f ), B3_FIX( 0.070678f ) },	{ B3_FIX( -0.497217f ), B3_FIX( -0.084704f ), B3_FIX( -0.000016f ) },
	{ B3_FIX( -0.487665f ), B3_FIX( -0.084704f ), B3_FIX( 0.096957f ) },	{ B3_FIX( -0.497217f ), B3_FIX( 0.086335f ), B3_FIX( -0.000016f ) },	{ B3_FIX( -0.362511f ), B3_FIX( 0.086337f ), B3_FIX( -0.000016f ) },
	{ B3_FIX( -0.355549f ), B3_FIX( 0.086337f ), B3_FIX( 0.070678f ) },	{ B3_FIX( -0.487665f ), B3_FIX( 0.086335f ), B3_FIX( 0.096957f ) },	{ B3_FIX( -0.362512f ), B3_FIX( 0.086337f ), B3_FIX( -0.000024f ) },
	{ B3_FIX( -0.362512f ), B3_FIX( -0.084705f ), B3_FIX( -0.000024f ) }, { B3_FIX( -0.355549f ), B3_FIX( 0.086337f ), B3_FIX( -0.070717f ) },	{ B3_FIX( -0.355549f ), B3_FIX( -0.084705f ), B3_FIX( -0.070717f ) },
	{ B3_FIX( -0.497217f ), B3_FIX( -0.084704f ), B3_FIX( -0.000024f ) }, { B3_FIX( -0.487666f ), B3_FIX( -0.084704f ), B3_FIX( -0.096997f ) }, { B3_FIX( -0.497217f ), B3_FIX( 0.086335f ), B3_FIX( -0.000024f ) },
	{ B3_FIX( -0.487666f ), B3_FIX( 0.086335f ), B3_FIX( -0.096997f ) },	{ B3_FIX( -0.334927f ), B3_FIX( -0.084705f ), B3_FIX( -0.138702f ) }, { B3_FIX( -0.355548f ), B3_FIX( 0.086337f ), B3_FIX( -0.070726f ) },
	{ B3_FIX( -0.355548f ), B3_FIX( -0.084705f ), B3_FIX( -0.070726f ) }, { B3_FIX( -0.334927f ), B3_FIX( 0.086337f ), B3_FIX( -0.138702f ) },	{ B3_FIX( -0.487665f ), B3_FIX( -0.084704f ), B3_FIX( -0.097005f ) },
	{ B3_FIX( -0.459379f ), B3_FIX( -0.084704f ), B3_FIX( -0.190252f ) }, { B3_FIX( -0.487665f ), B3_FIX( 0.086335f ), B3_FIX( -0.097005f ) },	{ B3_FIX( -0.459379f ), B3_FIX( 0.086335f ), B3_FIX( -0.190252f ) },
	{ B3_FIX( -0.413442f ), B3_FIX( -0.084704f ), B3_FIX( -0.276196f ) }, { B3_FIX( -0.301439f ), B3_FIX( -0.084705f ), B3_FIX( -0.201358f ) }, { B3_FIX( -0.334925f ), B3_FIX( -0.084705f ), B3_FIX( -0.138710f ) },
	{ B3_FIX( -0.459376f ), B3_FIX( -0.084704f ), B3_FIX( -0.190260f ) }, { B3_FIX( -0.334925f ), B3_FIX( 0.086337f ), B3_FIX( -0.138710f ) },	{ B3_FIX( -0.301439f ), B3_FIX( 0.086337f ), B3_FIX( -0.201358f ) },
	{ B3_FIX( -0.459376f ), B3_FIX( 0.086335f ), B3_FIX( -0.190260f ) },	{ B3_FIX( -0.413442f ), B3_FIX( 0.086335f ), B3_FIX( -0.276196f ) },	{ B3_FIX( -0.256370f ), B3_FIX( -0.084705f ), B3_FIX( -0.256276f ) },
	{ B3_FIX( -0.301434f ), B3_FIX( 0.086337f ), B3_FIX( -0.201365f ) },	{ B3_FIX( -0.301434f ), B3_FIX( -0.084705f ), B3_FIX( -0.201365f ) }, { B3_FIX( -0.256370f ), B3_FIX( 0.086337f ), B3_FIX( -0.256276f ) },
	{ B3_FIX( -0.413438f ), B3_FIX( 0.086335f ), B3_FIX( -0.276203f ) },	{ B3_FIX( -0.351621f ), B3_FIX( 0.086335f ), B3_FIX( -0.351527f ) },	{ B3_FIX( -0.413438f ), B3_FIX( -0.084704f ), B3_FIX( -0.276203f ) },
	{ B3_FIX( -0.351621f ), B3_FIX( -0.084704f ), B3_FIX( -0.351527f ) }, { B3_FIX( -0.256364f ), B3_FIX( -0.084705f ), B3_FIX( -0.256283f ) }, { B3_FIX( -0.201453f ), B3_FIX( 0.086337f ), B3_FIX( -0.301347f ) },
	{ B3_FIX( -0.256364f ), B3_FIX( 0.086337f ), B3_FIX( -0.256283f ) },	{ B3_FIX( -0.201453f ), B3_FIX( -0.084705f ), B3_FIX( -0.301347f ) }, { B3_FIX( -0.276292f ), B3_FIX( -0.084704f ), B3_FIX( -0.413350f ) },
	{ B3_FIX( -0.351615f ), B3_FIX( -0.084704f ), B3_FIX( -0.351534f ) }, { B3_FIX( -0.351615f ), B3_FIX( 0.086335f ), B3_FIX( -0.351534f ) },	{ B3_FIX( -0.276292f ), B3_FIX( 0.086335f ), B3_FIX( -0.413350f ) },
	{ B3_FIX( -0.201447f ), B3_FIX( -0.084705f ), B3_FIX( -0.301352f ) }, { B3_FIX( -0.276285f ), B3_FIX( -0.084704f ), B3_FIX( -0.413355f ) }, { B3_FIX( -0.138799f ), B3_FIX( -0.084705f ), B3_FIX( -0.334838f ) },
	{ B3_FIX( -0.190349f ), B3_FIX( -0.084704f ), B3_FIX( -0.459289f ) }, { B3_FIX( -0.276285f ), B3_FIX( 0.086335f ), B3_FIX( -0.413355f ) },	{ B3_FIX( -0.201447f ), B3_FIX( 0.086337f ), B3_FIX( -0.301352f ) },
	{ B3_FIX( -0.190349f ), B3_FIX( 0.086335f ), B3_FIX( -0.459289f ) },	{ B3_FIX( -0.138799f ), B3_FIX( 0.086337f ), B3_FIX( -0.334838f ) },	{ B3_FIX( -0.190341f ), B3_FIX( 0.086335f ), B3_FIX( -0.459293f ) },
	{ B3_FIX( -0.190341f ), B3_FIX( -0.084704f ), B3_FIX( -0.459293f ) }, { B3_FIX( -0.138791f ), B3_FIX( -0.084705f ), B3_FIX( -0.334842f ) }, { B3_FIX( -0.138791f ), B3_FIX( 0.086337f ), B3_FIX( -0.334842f ) },
	{ B3_FIX( -0.070815f ), B3_FIX( 0.086337f ), B3_FIX( -0.355462f ) },	{ B3_FIX( -0.070815f ), B3_FIX( -0.084705f ), B3_FIX( -0.355462f ) }, { B3_FIX( -0.097095f ), B3_FIX( 0.086335f ), B3_FIX( -0.487579f ) },
	{ B3_FIX( -0.097095f ), B3_FIX( -0.084704f ), B3_FIX( -0.487579f ) }, { B3_FIX( -0.070807f ), B3_FIX( 0.086337f ), B3_FIX( -0.355464f ) },	{ B3_FIX( -0.097086f ), B3_FIX( 0.086335f ), B3_FIX( -0.487581f ) },
	{ B3_FIX( -0.097086f ), B3_FIX( -0.084704f ), B3_FIX( -0.487581f ) }, { B3_FIX( -0.070807f ), B3_FIX( -0.084705f ), B3_FIX( -0.355464f ) }, { B3_FIX( -0.000113f ), B3_FIX( -0.084705f ), B3_FIX( -0.362427f ) },
	{ B3_FIX( -0.000113f ), B3_FIX( 0.086337f ), B3_FIX( -0.362427f ) },	{ B3_FIX( -0.000113f ), B3_FIX( 0.086335f ), B3_FIX( -0.497132f ) },	{ B3_FIX( -0.000113f ), B3_FIX( -0.084704f ), B3_FIX( -0.497132f ) },
	{ B3_FIX( 0.070588f ), B3_FIX( 0.086337f ), B3_FIX( -0.355465f ) },	{ B3_FIX( 0.096868f ), B3_FIX( -0.084704f ), B3_FIX( -0.487582f ) },	{ B3_FIX( 0.096868f ), B3_FIX( 0.086335f ), B3_FIX( -0.487582f ) },
	{ B3_FIX( 0.070588f ), B3_FIX( -0.084705f ), B3_FIX( -0.355465f ) },	{ B3_FIX( -0.000105f ), B3_FIX( 0.086337f ), B3_FIX( -0.362427f ) },	{ B3_FIX( -0.000105f ), B3_FIX( 0.086335f ), B3_FIX( -0.497133f ) },
	{ B3_FIX( -0.000105f ), B3_FIX( -0.084704f ), B3_FIX( -0.497133f ) }, { B3_FIX( -0.000105f ), B3_FIX( -0.084705f ), B3_FIX( -0.362427f ) }, { B3_FIX( 0.190123f ), B3_FIX( -0.084704f ), B3_FIX( -0.459294f ) },
	{ B3_FIX( 0.190123f ), B3_FIX( 0.086335f ), B3_FIX( -0.459294f ) },	{ B3_FIX( 0.138573f ), B3_FIX( -0.084705f ), B3_FIX( -0.334843f ) },	{ B3_FIX( 0.138573f ), B3_FIX( 0.086337f ), B3_FIX( -0.334843f ) },
	{ B3_FIX( 0.070597f ), B3_FIX( 0.086337f ), B3_FIX( -0.355463f ) },	{ B3_FIX( 0.070597f ), B3_FIX( -0.084705f ), B3_FIX( -0.355463f ) },	{ B3_FIX( 0.096876f ), B3_FIX( -0.084704f ), B3_FIX( -0.487580f ) },
	{ B3_FIX( 0.096876f ), B3_FIX( 0.086335f ), B3_FIX( -0.487580f ) },	{ B3_FIX( 0.201229f ), B3_FIX( -0.084705f ), B3_FIX( -0.301354f ) },	{ B3_FIX( 0.276067f ), B3_FIX( -0.084704f ), B3_FIX( -0.413358f ) },
	{ B3_FIX( 0.201229f ), B3_FIX( 0.086337f ), B3_FIX( -0.301354f ) },	{ B3_FIX( 0.138581f ), B3_FIX( -0.084705f ), B3_FIX( -0.334840f ) },	{ B3_FIX( 0.138581f ), B3_FIX( 0.086337f ), B3_FIX( -0.334840f ) },
	{ B3_FIX( 0.190131f ), B3_FIX( 0.086335f ), B3_FIX( -0.459292f ) },	{ B3_FIX( 0.276067f ), B3_FIX( 0.086335f ), B3_FIX( -0.413358f ) },	{ B3_FIX( 0.190131f ), B3_FIX( -0.084704f ), B3_FIX( -0.459292f ) },
	{ B3_FIX( 0.201236f ), B3_FIX( 0.086337f ), B3_FIX( -0.301350f ) },	{ B3_FIX( 0.256147f ), B3_FIX( -0.084705f ), B3_FIX( -0.256286f ) },	{ B3_FIX( 0.256147f ), B3_FIX( 0.086337f ), B3_FIX( -0.256286f ) },
	{ B3_FIX( 0.201236f ), B3_FIX( -0.084705f ), B3_FIX( -0.301350f ) },	{ B3_FIX( 0.351398f ), B3_FIX( -0.084704f ), B3_FIX( -0.351537f ) },	{ B3_FIX( 0.351398f ), B3_FIX( 0.086335f ), B3_FIX( -0.351537f ) },
	{ B3_FIX( 0.276074f ), B3_FIX( -0.084704f ), B3_FIX( -0.413353f ) },	{ B3_FIX( 0.276074f ), B3_FIX( 0.086335f ), B3_FIX( -0.413353f ) },	{ B3_FIX( 0.301218f ), B3_FIX( -0.084705f ), B3_FIX( -0.201369f ) },
	{ B3_FIX( 0.301218f ), B3_FIX( 0.086337f ), B3_FIX( -0.201369f ) },	{ B3_FIX( 0.256154f ), B3_FIX( -0.084705f ), B3_FIX( -0.256280f ) },	{ B3_FIX( 0.256154f ), B3_FIX( 0.086337f ), B3_FIX( -0.256280f ) },
	{ B3_FIX( 0.413221f ), B3_FIX( 0.086335f ), B3_FIX( -0.276207f ) },	{ B3_FIX( 0.413221f ), B3_FIX( -0.084704f ), B3_FIX( -0.276207f ) },	{ B3_FIX( 0.351405f ), B3_FIX( 0.086335f ), B3_FIX( -0.351531f ) },
	{ B3_FIX( 0.351405f ), B3_FIX( -0.084704f ), B3_FIX( -0.351531f ) },	{ B3_FIX( 0.334709f ), B3_FIX( 0.086337f ), B3_FIX( -0.138715f ) },	{ B3_FIX( 0.301223f ), B3_FIX( 0.086337f ), B3_FIX( -0.201362f ) },
	{ B3_FIX( 0.334709f ), B3_FIX( -0.084705f ), B3_FIX( -0.138715f ) },	{ B3_FIX( 0.301223f ), B3_FIX( -0.084705f ), B3_FIX( -0.201362f ) },	{ B3_FIX( 0.459160f ), B3_FIX( -0.084704f ), B3_FIX( -0.190264f ) },
	{ B3_FIX( 0.459160f ), B3_FIX( 0.086335f ), B3_FIX( -0.190264f ) },	{ B3_FIX( 0.413226f ), B3_FIX( 0.086335f ), B3_FIX( -0.276200f ) },	{ B3_FIX( 0.413226f ), B3_FIX( -0.084704f ), B3_FIX( -0.276200f ) },
	{ B3_FIX( 0.334713f ), B3_FIX( -0.084705f ), B3_FIX( -0.138707f ) },	{ B3_FIX( 0.355333f ), B3_FIX( -0.084705f ), B3_FIX( -0.070731f ) },	{ B3_FIX( 0.334713f ), B3_FIX( 0.086337f ), B3_FIX( -0.138707f ) },
	{ B3_FIX( 0.355333f ), B3_FIX( 0.086337f ), B3_FIX( -0.070731f ) },	{ B3_FIX( 0.459164f ), B3_FIX( 0.086335f ), B3_FIX( -0.190257f ) },	{ B3_FIX( 0.487450f ), B3_FIX( 0.086335f ), B3_FIX( -0.097010f ) },
	{ B3_FIX( 0.487450f ), B3_FIX( -0.084704f ), B3_FIX( -0.097010f ) },	{ B3_FIX( 0.459164f ), B3_FIX( -0.084704f ), B3_FIX( -0.190257f ) },	{ B3_FIX( 0.355335f ), B3_FIX( -0.084705f ), B3_FIX( -0.070722f ) },
	{ B3_FIX( 0.362298f ), B3_FIX( -0.084705f ), B3_FIX( -0.000029f ) },	{ B3_FIX( 0.355335f ), B3_FIX( 0.086337f ), B3_FIX( -0.070722f ) },	{ B3_FIX( 0.362298f ), B3_FIX( 0.086337f ), B3_FIX( -0.000029f ) },
	{ B3_FIX( 0.497003f ), B3_FIX( -0.084704f ), B3_FIX( -0.000029f ) },	{ B3_FIX( 0.497003f ), B3_FIX( 0.086335f ), B3_FIX( -0.000029f ) },	{ B3_FIX( 0.487452f ), B3_FIX( 0.086335f ), B3_FIX( -0.097002f ) },
	{ B3_FIX( 0.487452f ), B3_FIX( -0.084704f ), B3_FIX( -0.097002f ) },	{ B3_FIX( 0.362298f ), B3_FIX( -0.084705f ), B3_FIX( -0.000021f ) },	{ B3_FIX( 0.497003f ), B3_FIX( -0.084704f ), B3_FIX( -0.000021f ) },
	{ B3_FIX( 0.355336f ), B3_FIX( -0.084705f ), B3_FIX( 0.070673f ) },	{ B3_FIX( 0.487453f ), B3_FIX( -0.084704f ), B3_FIX( 0.096952f ) },	{ B3_FIX( 0.497003f ), B3_FIX( 0.086335f ), B3_FIX( -0.000021f ) },
	{ B3_FIX( 0.362298f ), B3_FIX( 0.086337f ), B3_FIX( -0.000021f ) },	{ B3_FIX( 0.355336f ), B3_FIX( 0.086337f ), B3_FIX( 0.070673f ) },	{ B3_FIX( 0.487453f ), B3_FIX( 0.086335f ), B3_FIX( 0.096952f ) },
	{ B3_FIX( 0.355334f ), B3_FIX( -0.084705f ), B3_FIX( 0.070681f ) },	{ B3_FIX( 0.355334f ), B3_FIX( 0.086337f ), B3_FIX( 0.070681f ) },	{ B3_FIX( 0.487451f ), B3_FIX( 0.086335f ), B3_FIX( 0.096961f ) },
	{ B3_FIX( 0.487451f ), B3_FIX( -0.084704f ), B3_FIX( 0.096961f ) },	{ B3_FIX( 0.334714f ), B3_FIX( -0.084705f ), B3_FIX( 0.138658f ) },	{ B3_FIX( 0.459165f ), B3_FIX( -0.084704f ), B3_FIX( 0.190207f ) },
	{ B3_FIX( 0.334714f ), B3_FIX( 0.086337f ), B3_FIX( 0.138658f ) },	{ B3_FIX( 0.459165f ), B3_FIX( 0.086335f ), B3_FIX( 0.190207f ) },	{ B3_FIX( 0.334711f ), B3_FIX( 0.086337f ), B3_FIX( 0.138666f ) },
	{ B3_FIX( 0.301225f ), B3_FIX( 0.086337f ), B3_FIX( 0.201313f ) },	{ B3_FIX( 0.459163f ), B3_FIX( 0.086335f ), B3_FIX( 0.190215f ) },	{ B3_FIX( 0.413229f ), B3_FIX( 0.086335f ), B3_FIX( 0.276151f ) },
	{ B3_FIX( 0.334711f ), B3_FIX( -0.084705f ), B3_FIX( 0.138666f ) },	{ B3_FIX( 0.301225f ), B3_FIX( -0.084705f ), B3_FIX( 0.201313f ) },	{ B3_FIX( 0.413229f ), B3_FIX( -0.084704f ), B3_FIX( 0.276151f ) },
	{ B3_FIX( 0.459163f ), B3_FIX( -0.084704f ), B3_FIX( 0.190215f ) },	{ B3_FIX( 0.301221f ), B3_FIX( -0.084705f ), B3_FIX( 0.201321f ) },	{ B3_FIX( 0.256157f ), B3_FIX( -0.084705f ), B3_FIX( 0.256232f ) },
	{ B3_FIX( 0.301221f ), B3_FIX( 0.086337f ), B3_FIX( 0.201321f ) },	{ B3_FIX( 0.256157f ), B3_FIX( 0.086337f ), B3_FIX( 0.256232f ) },	{ B3_FIX( 0.413224f ), B3_FIX( -0.084704f ), B3_FIX( 0.276159f ) },
	{ B3_FIX( 0.413224f ), B3_FIX( 0.086335f ), B3_FIX( 0.276159f ) },	{ B3_FIX( 0.351408f ), B3_FIX( 0.086335f ), B3_FIX( 0.351483f ) },	{ B3_FIX( 0.351408f ), B3_FIX( -0.084704f ), B3_FIX( 0.351483f ) },
	{ B3_FIX( 0.201240f ), B3_FIX( -0.084705f ), B3_FIX( 0.301302f ) },	{ B3_FIX( 0.201240f ), B3_FIX( 0.086337f ), B3_FIX( 0.301302f ) },	{ B3_FIX( 0.256151f ), B3_FIX( -0.084705f ), B3_FIX( 0.256238f ) },
	{ B3_FIX( 0.256151f ), B3_FIX( 0.086337f ), B3_FIX( 0.256238f ) },	{ B3_FIX( 0.351402f ), B3_FIX( 0.086335f ), B3_FIX( 0.351489f ) },	{ B3_FIX( 0.351402f ), B3_FIX( -0.084704f ), B3_FIX( 0.351489f ) },
	{ B3_FIX( 0.276078f ), B3_FIX( 0.086335f ), B3_FIX( 0.413305f ) },	{ B3_FIX( 0.276078f ), B3_FIX( -0.084704f ), B3_FIX( 0.413305f ) },	{ B3_FIX( 0.138586f ), B3_FIX( -0.084705f ), B3_FIX( 0.334793f ) },
	{ B3_FIX( 0.201233f ), B3_FIX( -0.084705f ), B3_FIX( 0.301307f ) },	{ B3_FIX( 0.190135f ), B3_FIX( -0.084704f ), B3_FIX( 0.459244f ) },	{ B3_FIX( 0.276071f ), B3_FIX( -0.084704f ), B3_FIX( 0.413311f ) },
	{ B3_FIX( 0.201233f ), B3_FIX( 0.086337f ), B3_FIX( 0.301307f ) },	{ B3_FIX( 0.138586f ), B3_FIX( 0.086337f ), B3_FIX( 0.334793f ) },	{ B3_FIX( 0.276071f ), B3_FIX( 0.086335f ), B3_FIX( 0.413311f ) },
	{ B3_FIX( 0.190135f ), B3_FIX( 0.086335f ), B3_FIX( 0.459244f ) },	{ B3_FIX( 0.138578f ), B3_FIX( -0.084705f ), B3_FIX( 0.334797f ) },	{ B3_FIX( 0.070602f ), B3_FIX( -0.084705f ), B3_FIX( 0.355417f ) },
	{ B3_FIX( 0.138578f ), B3_FIX( 0.086337f ), B3_FIX( 0.334797f ) },	{ B3_FIX( 0.070602f ), B3_FIX( 0.086337f ), B3_FIX( 0.355417f ) },	{ B3_FIX( 0.190127f ), B3_FIX( 0.086335f ), B3_FIX( 0.459248f ) },
	{ B3_FIX( 0.190127f ), B3_FIX( -0.084704f ), B3_FIX( 0.459248f ) },	{ B3_FIX( 0.096881f ), B3_FIX( 0.086335f ), B3_FIX( 0.487534f ) },	{ B3_FIX( 0.096881f ), B3_FIX( -0.084704f ), B3_FIX( 0.487534f ) },
	{ B3_FIX( 0.070593f ), B3_FIX( -0.084705f ), B3_FIX( 0.355419f ) },	{ B3_FIX( 0.096873f ), B3_FIX( 0.086335f ), B3_FIX( 0.487536f ) },	{ B3_FIX( 0.096873f ), B3_FIX( -0.084704f ), B3_FIX( 0.487536f ) },
	{ B3_FIX( 0.070593f ), B3_FIX( 0.086337f ), B3_FIX( 0.355419f ) },	{ B3_FIX( -0.000100f ), B3_FIX( 0.086337f ), B3_FIX( 0.362382f ) },	{ B3_FIX( -0.000100f ), B3_FIX( 0.086335f ), B3_FIX( 0.497087f ) },
	{ B3_FIX( -0.000100f ), B3_FIX( -0.084704f ), B3_FIX( 0.497087f ) },	{ B3_FIX( -0.000100f ), B3_FIX( -0.084705f ), B3_FIX( 0.362382f ) },
};

typedef struct WheelHullSpan
{
	int offset;
	int count;
} WheelHullSpan;
static const WheelHullSpan s_metalWheel1Hulls[37] = {
	{ 0, 29 },	{ 29, 8 },	{ 37, 8 },	{ 45, 8 },	{ 53, 8 },	{ 61, 8 },	{ 69, 8 },	{ 77, 8 },	{ 85, 8 },	{ 93, 8 },
	{ 101, 8 }, { 109, 8 }, { 117, 8 }, { 125, 8 }, { 133, 8 }, { 141, 8 }, { 149, 8 }, { 157, 8 }, { 165, 8 }, { 173, 8 },
	{ 181, 8 }, { 189, 8 }, { 197, 8 }, { 205, 8 }, { 213, 8 }, { 221, 8 }, { 229, 8 }, { 237, 8 }, { 245, 8 }, { 253, 8 },
	{ 261, 8 }, { 269, 8 }, { 277, 8 }, { 285, 8 }, { 293, 8 }, { 301, 8 }, { 309, 8 },
};
#define s_metalWheel1HullCount 37

// 30 metal_wheel1 props (37-piece convex decompositions) stacked. Body-pair contact merging
// lets them settle and sleep instead of wobbling.
class WheelStack : public Sample
{
public:
	explicit WheelStack( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 12.0f, 5.0f, SamplePos( { B3_FIX( 0.0f ), B3_FIX( 0.85f ), B3_FIX( 0.0f ) } ) );
		}

		AddGroundBox( 10.0f );

		constexpr int N = 512;
		b3Vec3 buffer[N];
		int bufferCount = 0;
		for ( int h = 0; h < s_metalWheel1HullCount; ++h )
		{
			const WheelHullSpan& span = s_metalWheel1Hulls[h];
			m_hulls[h] = b3CreateHull( &s_metalWheel1Verts[span.offset], span.count, span.count );

			// Fixed-point quickhull returns NULL instead of building a degenerate hull
			if ( m_hulls[h] == nullptr )
			{
				continue;
			}

			const b3Vec3* points = b3GetHullPoints( m_hulls[h] );
			for ( int j = 0; j < m_hulls[h]->vertexCount && bufferCount < N; ++j )
			{
				buffer[bufferCount] = points[j];
				++bufferCount;
			}
		}

		// Create a single hull that wraps the input hulls.
		b3HullData* wheelHull = b3CreateHull( buffer, bufferCount, bufferCount );
		if ( wheelHull == nullptr )
		{
			return;
		}

		const b3Fixed height = B3_FIX( 0.171f );
		const b3Fixed spacing = height + B3_FIX( 0.006f );
		const b3Fixed startY = height / 2 + B3_FIX( 0.004f );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = B3_FIX( 0.6f );

		for ( int i = 0; i < m_wheelCount; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.name = "wheel";
			bodyDef.position = SamplePos( { B3_FIX( 0.0f ), startY + i * spacing, B3_FIX( 0.0f ) } );
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			//for ( int h = 0; h < s_metalWheel1HullCount; ++h )
			//{
			//	b3CreateHullShape( bodyId, &shapeDef, m_hulls[h] );
			//}

			// Using a single hull improves the simulation.
			b3CreateHullShape( bodyId, &shapeDef, wheelHull );
		}

		b3DestroyHull( wheelHull );

		b3World_SetContactTuning( m_worldId, B3_FIX( 240.0f ), B3_FIX( 10.0f ), B3_FIX( 3.0f ) );
	}

	~WheelStack() override
	{
		for ( int h = 0; h < s_metalWheel1HullCount; ++h )
		{
			if ( m_hulls[h] != nullptr )
			{
				b3DestroyHull( m_hulls[h] );
			}
		}
	}

	void Step() override
	{
		Sample::Step();

		b3Profile p = b3World_GetProfile( m_worldId );
		float substepRate = m_context->hertz * m_context->subStepCount;
		float effectiveHz = 0.125f * substepRate < 240.0f ? 0.125f * substepRate : 240.0f;
		double stepMs = b3FixToDouble( p.step );
		double collideMs = b3FixToDouble( p.collide );
		double solveMs = b3FixToDouble( p.solve );

		DrawTextLine( "wheels %d, hull pieces/wheel %d (%d total shapes)", m_wheelCount, s_metalWheel1HullCount,
					  m_wheelCount * s_metalWheel1HullCount );
		DrawTextLine( "step %.0f hz, sub-steps %d -> substep rate %.0f hz", m_context->hertz, m_context->subStepCount,
					  substepRate );
		DrawTextLine( "eff contact hz = min(240, %.0f) = %.0f", 0.125f * substepRate, effectiveHz );
		DrawTextLine( "step %.3f ms | collide %.3f ms (%.0f%%) | solve %.3f ms (%.0f%%)", stepMs, collideMs,
					  stepMs > 0.0 ? 100.0 * collideMs / stepMs : 0.0, solveMs, stepMs > 0.0 ? 100.0 * solveMs / stepMs : 0.0 );
	}

	static Sample* Create( SampleContext* context )
	{
		return new WheelStack( context );
	}

	b3HullData* m_hulls[s_metalWheel1HullCount] = {};
	static constexpr int m_wheelCount = 30;
};

static int sampleWheelStack = RegisterSample( "Issues", "GMod Wheel Stack", WheelStack::Create );

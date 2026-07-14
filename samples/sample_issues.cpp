// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "imgui.h"
#include "mesh_loader.h"
#include "sample.h"
#include "gfx/draw.h"

#include "box3d/box3d.h"

#include <vector>

class DumpLoader : public Sample
{
public:
	explicit DumpLoader( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } );
			// m_camera->SetView( 45.0f, 30.0f, 300.0f, { 3910.62109f, 9862.50293f, 875.395081f } );
		}

		b3SetLengthUnitsPerMeter( B3_FIX( 1.0f ) );

		const char* dumpPrefix = "data/dumps/single_box/";

#include "dumps/single_box/box3d_dump.inl"
	}

	~DumpLoader() override
	{
		for ( b3MeshData* md : m_meshes )
		{
			b3DestroyMesh( md );
		}

		b3SetLengthUnitsPerMeter( B3_FIX( 1.0f ) );
	}

	static Sample* Create( SampleContext* context )
	{
		return new DumpLoader( context );
	}

	std::vector<b3MeshData*> m_meshes;
};

static int sampleDumpLoader = RegisterSample( "Issues", "Dump Loader", DumpLoader::Create );

class Crash : public Sample
{
public:
	explicit Crash( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { B3_FIX( 0.0f ), B3_FIX( -1.0f ), B3_FIX( 0.0f ) };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_gridMesh = b3CreateGridMesh( 20, 20, B3_FIX( 2.0f ), 0, true );
			b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { B3_FIX( 2.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) };
		m_bodyId1 = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
		b3CreateHullShape( m_bodyId1, &shapeDef, &box.base );

		bodyDef.position = { B3_FIX( -2.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) };
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
			m_camera->SetView( 0.0f, 0.0f, 25.0f, { B3_FIX( 0.0f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) } );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
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
			bodyDef.position = { B3_FIX( 0.0f ), b3FixFromFloat( 0.6f + 1.2f * (float)i ), B3_FIX( 0.0f ) };
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
			m_camera->SetView( 0.0f, 15.0f, 5.0f, b3Pos_zero );
		}

		m_hull = nullptr;

#if 0
		// bad hull SM_Waterfall_MED_Wide_01
		b3Vec3 points[] = {
			{ B3_FIX( 0.100183107 ), B3_FIX( -498.925385 ), B3_FIX( -1275.39966 ) }, { B3_FIX( 0.100183107 ), B3_FIX( -498.925415 ), B3_FIX( 0.125000000 ) },
			{ B3_FIX( 0.100183107 ), B3_FIX( 486.343750 ), B3_FIX( 0.125000000 ) },  { B3_FIX( 0.100183107 ), B3_FIX( 486.343719 ), B3_FIX( -1275.39966 ) },
			{ B3_FIX( -395.117462 ), B3_FIX( 486.343781 ), B3_FIX( -1462.43750 ) },  { B3_FIX( -395.117462 ), B3_FIX( 486.343750 ), B3_FIX( -96.7426758 ) },
			{ B3_FIX( -395.117462 ), B3_FIX( -498.925415 ), B3_FIX( -96.7424469 ) }, { B3_FIX( -395.117462 ), B3_FIX( -498.925446 ), B3_FIX( -1462.52612 ) },
			{ B3_FIX( -186.979691 ), B3_FIX( 486.294891 ), B3_FIX( -1462.47949 ) },  { B3_FIX( -298.250000 ), B3_FIX( 486.294891 ), B3_FIX( 0.125000000 ) },
			{ B3_FIX( -395.121216 ), B3_FIX( 486.294891 ), B3_FIX( -1462.52612 ) },  { B3_FIX( -186.984360 ), B3_FIX( -498.913361 ), B3_FIX( -1462.48413 ) },
			{ B3_FIX( -298.250000 ), B3_FIX( -498.913361 ), B3_FIX( 0.125000000 ) },
		};

#elif 1
		b3Vec3 points[] = {
			{ B3_FIX( 100.000000 ), B3_FIX( -142.292389 ), B3_FIX( 130.826111 ) },  { B3_FIX( 99.5354385 ), B3_FIX( -71.3011093 ), B3_FIX( 130.826111 ) },
			{ B3_FIX( 99.5930862 ), B3_FIX( -80.1112213 ), B3_FIX( -100.000000 ) }, { B3_FIX( 100.000000 ), B3_FIX( -142.292389 ), B3_FIX( -100.000000 ) },
			{ B3_FIX( 99.5930862 ), B3_FIX( -80.1112213 ), B3_FIX( 130.826111 ) },
		};
#else
		b3Vec3 points[] = {
			{ B3_FIX( -11.3861933 ), B3_FIX( -24.2451687 ), B3_FIX( -12.0037909 ) }, { B3_FIX( -11.3889809 ), B3_FIX( -24.2466526 ), B3_FIX( -11.9013014 ) },
			{ B3_FIX( -11.3804407 ), B3_FIX( -24.3151531 ), B3_FIX( -12.0046492 ) }, { B3_FIX( -11.3832273 ), B3_FIX( -24.3166409 ), B3_FIX( -11.9021587 ) },
			{ B3_FIX( -14.4396200 ), B3_FIX( -24.3636723 ), B3_FIX( -12.1324549 ) }, { B3_FIX( -14.4432650 ), B3_FIX( -24.3655701 ), B3_FIX( -12.0299988 ) },
			{ B3_FIX( -14.4356947 ), B3_FIX( -24.4337788 ), B3_FIX( -12.1336164 ) }, { B3_FIX( -14.4393377 ), B3_FIX( -24.4356804 ), B3_FIX( -12.0311594 ) },
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
		if ( m_hull != nullptr )
		{
			DrawHull( b3WorldTransform_identity, m_hull, MakeColor( b3_colorYellow ) );
		}
		else
		{
			for ( int i = 0; i < m_count; ++i )
			{
				DrawPoint( b3ToPos( m_points[i] ), 5.0f, MakeColor( b3_colorWhite ) );
			}
		}

		DrawAxes( b3WorldTransform_identity, 1.0f );

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
			m_camera->SetView( 0.0f, 15.0f, 10.0f, { B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } );
			
		}

		AddGroundBox( 10.0f );

		// Source data is scaled by 1/100 (exact integer scaling in fixed point).

		{
			b3Vec3 b = { B3_FIX( -459.292877f ), B3_FIX( 217.398331f ), B3_FIX( 1.00115335f ) };
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { b.x / 100, b.z / 100 + B3_FIX( 2.0f ), b.y / 100 };
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
				points[i] = { p.x / 100, p.z / 100, p.y / 100 };
			}

			b3HullData* hull = b3CreateHull( points, count, count );

			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}

		{
			b3Vec3 b = { B3_FIX( -402.321838f ), B3_FIX( 157.310364f ), B3_FIX( 16.8169250f ) };
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { b.x / 100, b.z / 100 + B3_FIX( 2.0f ), b.y / 100 };
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
				points[i] = { p.x / 100, p.z / 100, p.y / 100 };
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
			m_camera->SetView( 45.0f, 30.0f, 12.0f, b3Pos_zero );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { B3_FIX( -10.0f ), B3_FIX( 0.0f ), B3_FIX( -10.0f ) };
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
			bodyDef.position = { B3_FIX( 0.0f ), B3_FIX( 3.5f ), B3_FIX( 0.0f ) };
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
		b3Transform transform = { { B3_FIX( 0.0f ), B3_FIX( 1.1f ), B3_FIX( 0.0f ) }, b3Quat_identity };
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
			m_camera->SetView( 20.0f, 10.0f, 30.0f, { B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } );
		}

		// --- Ground plane ---
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull ground = b3MakeBoxHull( B3_FIX( 50.0f ), B3_FIX( 0.1f ), B3_FIX( 50.0f ) );
			b3CreateHullShape( body, &shapeDef, &ground.base );
		}

		// --- Building mesh on top of ground ---
		m_building = CreateMeshData( "data/meshes/building.obj", 1.0f, false, false, true, true );
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { B3_FIX( 0.0f ), B3_FIX( 0.1f ), B3_FIX( 0.0f ) };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( body, &shapeDef, m_building, b3Vec3_one );
		}

		// --- Locked capsule (same setup as player controller body) ---
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 10.0f ) };
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
			m_camera->SetView( 90.0f, 25.0f, 10.0f, { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		}

		// Two chunks meeting at x = 0, each its own body and mesh shape, so seam contacts live
		// in separate contact pairs like s&box world mesh chunks. A beam top straddles the seam.
		CreateFloorChunk( 0, -(float)m_halfLengthU, 0.0f );
		CreateFloorChunk( 1, 0.0f, (float)m_halfLengthU );

		// Character
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -m_walkRangeX, b3FixFromFloat( m_bodyHalfHeight + 0.1f ), B3_FIX( 0.0f ) };
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

		m_walkDirectionX = 1;
		m_walkDirectionZ = 1;
		m_walkSpeedX = b3FixFromFloat( 350.0f * SRC ); // s&box run speed
		m_walkSpeedZ = b3FixFromFloat( 20.0f * SRC );
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
		b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( body, &shapeDef, m_chunkMesh[chunk], b3Vec3_one );
	}

	void Step() override
	{
		// Drive the character with pure velocity control: keep the solver's vertical velocity,
		// set the horizontal velocity. This is how the s&box player controller moves.
		b3Pos position = b3Body_GetPosition( m_characterId );
		if ( position.x > m_walkRangeX )
		{
			m_walkDirectionX = -1;
		}
		else if ( position.x < -m_walkRangeX )
		{
			m_walkDirectionX = 1;
		}

		if ( position.z > m_walkRangeZ )
		{
			m_walkDirectionZ = -1;
		}
		else if ( position.z < -m_walkRangeZ )
		{
			m_walkDirectionZ= 1;
		}

		b3Vec3 velocity = b3Body_GetLinearVelocity( m_characterId );
		velocity.x = m_walkDirectionX * m_walkSpeedX;
		velocity.z = m_walkDirectionZ * m_walkSpeedZ;
		b3Body_SetLinearVelocity( m_characterId, velocity );

		Sample::Step();

		if ( m_didStep )
		{
			// The walkable plane is exactly y = 0, so the grounded body center never rises above
			// rest height. Any upward velocity spike while grounded is a ghost collision: there
			// is nothing to climb and nothing to bounce off.
			position = b3Body_GetPosition( m_characterId );
			velocity = b3Body_GetLinearVelocity( m_characterId );

			bool grounded = position.y < b3FixFromFloat( m_bodyHalfHeight + 0.01f + 4.0f * SRC );
			bool launched = velocity.y > m_launchThreshold;

			if ( grounded && launched && m_wasLaunched == false )
			{
				m_launchCount += 1;
				m_maxLaunchSpeed = b3MaxFloat( m_maxLaunchSpeed, b3FixToFloat( velocity.y ) );

				if ( m_launchMarkerCount < m_markerCapacity )
				{
					m_launchMarkers[m_launchMarkerCount] = position;
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
		float speedUX = b3FixToFloat( m_walkSpeedX ) / SRC;
		if ( ImGui::SliderFloat( "Walk Speed X (inch/s)", &speedUX, 100.0f, 400.0f, "%.0f" ) )
		{
			m_walkSpeedX = b3FixFromFloat( speedUX * SRC );
		}

		float speedUZ = b3FixToFloat( m_walkSpeedZ ) / SRC;
		if ( ImGui::SliderFloat( "Walk Speed Z (inch/s)", &speedUZ, 10.0f, 100.0f, "%.0f" ) )
		{
			m_walkSpeedZ = b3FixFromFloat( speedUZ * SRC );
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

	static constexpr b3Fixed m_walkRangeX = B3_FIX( 3.5f );		 // turn around beyond +/- this x (meters)
	static constexpr b3Fixed m_walkRangeZ = B3_FIX( 0.5f );		 // turn around beyond +/- this x (meters)
	static constexpr b3Fixed m_launchThreshold = B3_FIX( 0.5f ); // upward m/s counted as a ghost launch (~20 inch/s)
	static constexpr int m_markerCapacity = 64;

	b3MeshData* m_chunkMesh[2] = {};
	b3BodyId m_characterId;
	int m_walkDirectionX;
	int m_walkDirectionZ;
	b3Fixed m_walkSpeedX;
	b3Fixed m_walkSpeedZ;
	int m_launchCount;
	float m_maxLaunchSpeed;
	bool m_wasLaunched;
	b3Pos m_launchMarkers[m_markerCapacity];
	int m_launchMarkerCount;
};

static int sampleSBoxGhostCollisions = RegisterSample( "Issues", "s&box Ghost Collisions", SBoxGhostCollisions::Create );

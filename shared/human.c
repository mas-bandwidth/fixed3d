// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "human.h"

#include "utils.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <stddef.h>

void CreateHuman( Human* human, b3WorldId worldId, b3Pos position, b3Fixed frictionTorque, b3Fixed hertz, b3Fixed dampingRatio,
				  int groupIndex, void* userData, bool colorize )
{
	assert( human->isSpawned == false );

	for ( int i = 0; i < bone_count; ++i )
	{
		human->bones[i].bodyId = b3_nullBodyId;
		human->bones[i].anchorId = b3_nullBodyId;
		human->bones[i].jointId = b3_nullJointId;
		human->bones[i].jointFriction = B3_FIX( 1.0f );
		human->bones[i].parentIndex = -1;
	}

	for ( int i = 0; i < FILTER_JOINT_COUNT; ++i )
	{
		human->filterJoints[i] = b3_nullJointId;
	}

	human->frictionTorque = frictionTorque;

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.userData = userData;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Fixed defaultDensity = shapeDef.density;

	b3HexColor shirtColor = b3_colorMediumTurquoise;
	b3HexColor pantColor = b3_colorDodgerBlue;
	b3HexColor skinColors[4] = { b3_colorNavajoWhite, b3_colorLightYellow, b3_colorPeru, b3_colorTan };
	b3HexColor skinColor = skinColors[groupIndex % 4];

	{
		Bone* bone = human->bones + bone_pelvis;
		bone->parentIndex = -1;

		bodyDef.name = "pelvis";
		bone->referenceFrame = (b3Transform){ { B3_FIX( 0.000000f ), B3_FIX( 0.996219f ), -B3_FIX( 0.023868f ) }, { { B3_FIX( 1.000000f ), B3_FIX( 0.000000f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.000000f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.6f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { B3_FIX( 0.040000f ), B3_FIX( 0.000001f ), B3_FIX( 0.000000f ) }, { -B3_FIX( 0.040000f ), -B3_FIX( 0.000001f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.150000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );
	}

	{
		Bone* bone = human->bones + bone_spine_01;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "spine_01";
		bone->referenceFrame = (b3Transform){ { B3_FIX( 0.000000f ), B3_FIX( 1.017288f ), -B3_FIX( 0.024882f ) }, { { B3_FIX( 1.000000f ), B3_FIX( 0.000000f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.000000f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.5f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { B3_FIX( 0.029876f ), -B3_FIX( 0.146581f ), B3_FIX( 0.006260f ) }, { -B3_FIX( 0.029876f ), -B3_FIX( 0.146574f ), B3_FIX( 0.006260f ) }, B3_FIX( 0.145663f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { B3_FIX( 0.000000f ), -B3_FIX( 0.021069f ), B3_FIX( 0.001014f ) }, { { -B3_FIX( 0.642737f ), B3_FIX( 0.000000f ), B3_FIX( 0.000000f ) }, -B3_FIX( 0.766087f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.707107f ), B3_FIX( 0.000000f ), B3_FIX( 0.000000f ) }, -B3_FIX( 0.707107f ) } };
		bone->swingLimit = b3FixMul( B3_FIX( 35.0f ), B3_DEG_TO_RAD );
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 17.5f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 17.5f ), B3_DEG_TO_RAD ) };
	}

	{
		Bone* bone = human->bones + bone_spine_03;
		bone->parentIndex = bone_spine_01;

		bodyDef.name = "spine_03";
		bone->referenceFrame = (b3Transform){ { B3_FIX( 0.000000f ), B3_FIX( 1.267766f ), -B3_FIX( 0.022320f ) }, { { B3_FIX( 1.000000f ), B3_FIX( 0.000000f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.000000f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.5f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { B3_FIX( 0.063996f ), -B3_FIX( 0.117434f ), -B3_FIX( 0.040199f ) }, { -B3_FIX( 0.063996f ), -B3_FIX( 0.117432f ), -B3_FIX( 0.040199f ) }, B3_FIX( 0.165004f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { B3_FIX( 0.000000f ), -B3_FIX( 0.250478f ), -B3_FIX( 0.002562f ) }, { { -B3_FIX( 0.642766f ), B3_FIX( 0.000000f ), B3_FIX( 0.000000f ) }, -B3_FIX( 0.766063f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.707107f ), B3_FIX( 0.000000f ), B3_FIX( 0.000000f ) }, -B3_FIX( 0.707107f ) } };
		bone->swingLimit = b3FixMul( B3_FIX( 35.0f ), B3_DEG_TO_RAD );
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 17.5f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 17.5f ), B3_DEG_TO_RAD ) };
	}

	{
		Bone* bone = human->bones + bone_neck;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "neck_01";
		bone->referenceFrame = (b3Transform){ { B3_FIX( 0.000000f ), B3_FIX( 1.498783f ), B3_FIX( 0.024462f ) }, { { B3_FIX( 0.987879f ), B3_FIX( 0.000000f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.155228f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.2f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.05f );
		b3Capsule head = { { B3_FIX( 0.000000f ), -B3_FIX( 0.212340f ), B3_FIX( 0.000000f ) }, { B3_FIX( 0.000000f ), -B3_FIX( 0.087340f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.110000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &head );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { B3_FIX( 0.000000f ), -B3_FIX( 0.231017f ), -B3_FIX( 0.046781f ) }, { { -B3_FIX( 0.516157f ), -B3_FIX( 0.000002f ), B3_FIX( 0.000003f ) }, -B3_FIX( 0.856494f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.707108f ), -B3_FIX( 0.000002f ), B3_FIX( 0.000002f ) }, -B3_FIX( 0.707106f ) } };
		bone->swingLimit = b3FixMul( B3_FIX( 30.0f ), B3_DEG_TO_RAD );
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 17.5f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 17.5f ), B3_DEG_TO_RAD ) };
	}

	{
		Bone* bone = human->bones + bone_thigh_l;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "thigh_l";
		bone->referenceFrame = (b3Transform){ { B3_FIX( 0.092175f ), B3_FIX( 0.971562f ), -B3_FIX( 0.011177f ) }, { { B3_FIX( 0.471412f ), B3_FIX( 0.529510f ), -B3_FIX( 0.499888f ) }, -B3_FIX( 0.497495f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.6f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { -B3_FIX( 0.047269f ), B3_FIX( 0.000001f ), B3_FIX( 0.000000f ) }, { -B3_FIX( 0.379769f ), B3_FIX( 0.000005f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.091539f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { B3_FIX( 0.092175f ), B3_FIX( 0.024656f ), -B3_FIX( 0.012691f ) }, { { -B3_FIX( 0.460186f ), -B3_FIX( 0.365368f ), -B3_FIX( 0.637662f ) }, B3_FIX( 0.498118f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.499997f ), -B3_FIX( 0.500003f ), B3_FIX( 0.500002f ) }, B3_FIX( 0.499998f ) } };
		bone->swingLimit = b3FixMul( B3_FIX( 30.0f ), B3_DEG_TO_RAD );
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 10.0f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 10.0f ), B3_DEG_TO_RAD ) };
	}

	{
		Bone* bone = human->bones + bone_calf_l;
		bone->parentIndex = bone_thigh_l;

		bodyDef.name = "calf_l";
		bone->referenceFrame = (b3Transform){ { B3_FIX( 0.118720f ), B3_FIX( 0.534541f ), -B3_FIX( 0.035362f ) }, { { B3_FIX( 0.521188f ), B3_FIX( 0.480597f ), -B3_FIX( 0.546380f ) }, -B3_FIX( 0.445935f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.1f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.0f );
		b3Capsule shin = { { -B3_FIX( 0.445000f ), B3_FIX( 0.000001f ), B3_FIX( 0.000000f ) }, { B3_FIX( 0.005000f ), -B3_FIX( 0.000001f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.080000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &shin );

		shapeDef.density = defaultDensity / 2;
		b3Capsule foot = { { -B3_FIX( 0.456371f ), B3_FIX( 0.129321f ), B3_FIX( 0.000013f ) }, { -B3_FIX( 0.359977f ), B3_FIX( 0.014394f ), B3_FIX( 0.000013f ) }, B3_FIX( 0.070000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &foot );
		shapeDef.density = defaultDensity;

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { -B3_FIX( 0.438494f ), -B3_FIX( 0.000175f ), B3_FIX( 0.000000f ) }, { { B3_FIX( 0.344866f ), -B3_FIX( 0.938652f ), B3_FIX( 0.000002f ) }, -B3_FIX( 0.000008f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.008182f ), -B3_FIX( 0.999967f ), B3_FIX( 0.000001f ) }, -B3_FIX( 0.000007f ) } };
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 30.0f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 30.0f ), B3_DEG_TO_RAD ) };
	}

	{
		Bone* bone = human->bones + bone_thigh_r;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "thigh_r";
		bone->referenceFrame = (b3Transform){ { -B3_FIX( 0.092175f ), B3_FIX( 0.971562f ), -B3_FIX( 0.011177f ) }, { { -B3_FIX( 0.497495f ), B3_FIX( 0.499888f ), B3_FIX( 0.529510f ) }, -B3_FIX( 0.471412f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.6f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { B3_FIX( 0.047269f ), -B3_FIX( 0.000001f ), B3_FIX( 0.000000f ) }, { B3_FIX( 0.379769f ), -B3_FIX( 0.000005f ), B3_FIX( 0.000000f ) }, B3_FIX( 0.092587f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { -B3_FIX( 0.092175f ), B3_FIX( 0.024656f ), -B3_FIX( 0.012691f ) }, { { B3_FIX( 0.380666f ), B3_FIX( 0.491846f ), B3_FIX( 0.488644f ) }, -B3_FIX( 0.611889f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.500002f ), B3_FIX( 0.499998f ), -B3_FIX( 0.499998f ) }, B3_FIX( 0.500002f ) } };
		bone->swingLimit = b3FixMul( B3_FIX( 30.0f ), B3_DEG_TO_RAD );
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 10.0f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 10.0f ), B3_DEG_TO_RAD ) };
	}

	{
		Bone* bone = human->bones + bone_calf_r;
		bone->parentIndex = bone_thigh_r;

		bodyDef.name = "calf_r";
		bone->referenceFrame = (b3Transform){ { -B3_FIX( 0.118720f ), B3_FIX( 0.534541f ), -B3_FIX( 0.035362f ) }, { { -B3_FIX( 0.445935f ), B3_FIX( 0.546380f ), B3_FIX( 0.480597f ) }, -B3_FIX( 0.521188f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.1f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.0f );
		b3Capsule shin = { { B3_FIX( 0.445000f ), -B3_FIX( 0.000002f ), B3_FIX( 0.000003f ) }, { -B3_FIX( 0.005000f ), B3_FIX( 0.000000f ), B3_FIX( 0.000003f ) }, B3_FIX( 0.080000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &shin );

		shapeDef.density = defaultDensity / 2;
		b3Capsule foot = { { B3_FIX( 0.369113f ), -B3_FIX( 0.006830f ), B3_FIX( 0.000017f ) }, { B3_FIX( 0.465507f ), -B3_FIX( 0.121757f ), B3_FIX( 0.000017f ) }, B3_FIX( 0.070000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &foot );
		shapeDef.density = defaultDensity;

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { B3_FIX( 0.438494f ), B3_FIX( 0.000175f ), B3_FIX( 0.000000f ) }, { { B3_FIX( 0.000007f ), B3_FIX( 0.000005f ), -B3_FIX( 0.344860f ) }, -B3_FIX( 0.938654f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { B3_FIX( 0.000006f ), B3_FIX( 0.000005f ), B3_FIX( 0.008188f ) }, -B3_FIX( 0.999966f ) } };
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 30.0f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 30.0f ), B3_DEG_TO_RAD ) };
	}

	{
		Bone* bone = human->bones + bone_upper_arm_l;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "upperarm_l";
		bone->referenceFrame = (b3Transform){ { B3_FIX( 0.185817f ), B3_FIX( 1.443630f ), B3_FIX( 0.031306f ) }, { { B3_FIX( 0.560220f ), -B3_FIX( 0.307179f ), B3_FIX( 0.366233f ) }, -B3_FIX( 0.676512f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.6f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { B3_FIX( 0.296038f ), B3_FIX( 0.004386f ), -B3_FIX( 0.000794f ) }, { -B3_FIX( 0.016086f ), B3_FIX( 0.004384f ), -B3_FIX( 0.016127f ) }, B3_FIX( 0.065000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { B3_FIX( 0.185817f ), -B3_FIX( 0.175865f ), -B3_FIX( 0.053625f ) }, { { -B3_FIX( 0.559466f ), -B3_FIX( 0.213784f ), -B3_FIX( 0.670199f ) }, B3_FIX( 0.438323f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.500953f ), -B3_FIX( 0.502617f ), -B3_FIX( 0.509318f ) }, -B3_FIX( 0.486844f ) } };
		bone->swingLimit = b3FixMul( B3_FIX( 45.0f ), B3_DEG_TO_RAD );
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 20.0f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 20.0f ), B3_DEG_TO_RAD ) };
		bone->jointFriction = B3_FIX( 0.8f );
	}

	{
		Bone* bone = human->bones + bone_lower_arm_l;
		bone->parentIndex = bone_upper_arm_l;

		bodyDef.name = "lowerarm_l";
		bone->referenceFrame = (b3Transform){ { B3_FIX( 0.347683f ), B3_FIX( 1.193455f ), B3_FIX( 0.029378f ) }, { { B3_FIX( 0.406736f ), -B3_FIX( 0.493012f ), B3_FIX( 0.089135f ) }, -B3_FIX( 0.763911f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.3f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { B3_FIX( 0.312329f ), -B3_FIX( 0.008256f ), -B3_FIX( 0.016454f ) }, { B3_FIX( 0.033153f ), B3_FIX( 0.008259f ), -B3_FIX( 0.002741f ) }, B3_FIX( 0.060000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { B3_FIX( 0.297979f ), B3_FIX( 0.000361f ), B3_FIX( 0.000000f ) }, { { B3_FIX( 0.916726f ), B3_FIX( 0.399352f ), B3_FIX( 0.000839f ) }, -B3_FIX( 0.011456f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { B3_FIX( 0.999922f ), -B3_FIX( 0.003857f ), B3_FIX( 0.004659f ) }, -B3_FIX( 0.010908f ) } };
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 35.0f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 35.0f ), B3_DEG_TO_RAD ) };
		bone->jointFriction = B3_FIX( 0.06f );
	}

	{
		Bone* bone = human->bones + bone_upper_arm_r;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "upperarm_r";
		bone->referenceFrame = (b3Transform){ { -B3_FIX( 0.185817f ), B3_FIX( 1.443630f ), B3_FIX( 0.031306f ) }, { { B3_FIX( 0.676512f ), B3_FIX( 0.366233f ), B3_FIX( 0.307179f ) }, B3_FIX( 0.560220f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.6f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { -B3_FIX( 0.300237f ), -B3_FIX( 0.000001f ), -B3_FIX( 0.000319f ) }, { B3_FIX( 0.011887f ), B3_FIX( 0.000001f ), B3_FIX( 0.015013f ) }, B3_FIX( 0.065000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { -B3_FIX( 0.185817f ), -B3_FIX( 0.175865f ), -B3_FIX( 0.053625f ) }, { { B3_FIX( 0.213793f ), B3_FIX( 0.559449f ), B3_FIX( 0.438334f ) }, -B3_FIX( 0.670204f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.509319f ), B3_FIX( 0.486843f ), B3_FIX( 0.500954f ) }, -B3_FIX( 0.502617f ) } };
		bone->swingLimit = b3FixMul( B3_FIX( 45.0f ), B3_DEG_TO_RAD );
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 20.0f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 20.0f ), B3_DEG_TO_RAD ) };
		bone->jointFriction = B3_FIX( 0.8f );
	}

	{
		Bone* bone = human->bones + bone_lower_arm_r;
		bone->parentIndex = bone_upper_arm_r;

		bodyDef.name = "lowerarm_r";
		bone->referenceFrame = (b3Transform){ { -B3_FIX( 0.347683f ), B3_FIX( 1.193455f ), B3_FIX( 0.029378f ) }, { { B3_FIX( 0.763911f ), B3_FIX( 0.089135f ), B3_FIX( 0.493012f ) }, B3_FIX( 0.406736f ) } };
		bone->referenceFrame.q = b3NormalizeQuat( bone->referenceFrame.q );
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;

		shapeDef.baseMaterial.friction = B3_FIX( 0.3f );
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );
		b3Capsule capsule = { { -B3_FIX( 0.312649f ), -B3_FIX( 0.000003f ), B3_FIX( 0.016471f ) }, { -B3_FIX( 0.032986f ), B3_FIX( 0.000000f ), B3_FIX( 0.002732f ) }, B3_FIX( 0.060000f ) };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { -B3_FIX( 0.297979f ), -B3_FIX( 0.000361f ), B3_FIX( 0.000000f ) }, { { -B3_FIX( 0.405444f ), B3_FIX( 0.914059f ), B3_FIX( 0.010541f ) }, B3_FIX( 0.000852f ) } };
		bone->localFrameB = (b3Transform){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { -B3_FIX( 0.002794f ), B3_FIX( 0.999936f ), B3_FIX( 0.010042f ) }, B3_FIX( 0.004364f ) } };
		bone->twistLimit = (b3Vec2){ b3FixMul( -B3_FIX( 35.0f ), B3_DEG_TO_RAD ), b3FixMul( B3_FIX( 35.0f ), B3_DEG_TO_RAD ) };
		bone->jointFriction = B3_FIX( 0.06f );
	}

	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		Bone* parent = human->bones + bone->parentIndex;

		b3BodyId bodyIdA = parent->bodyId;
		b3BodyId bodyIdB = bone->bodyId;

		bone->localFrameA.q = b3NormalizeQuat( bone->localFrameA.q );
		bone->localFrameB.q = b3NormalizeQuat( bone->localFrameB.q );

		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = bodyIdA;
			jointDef.base.bodyIdB = bodyIdB;
			jointDef.base.localFrameA = bone->localFrameA;
			jointDef.base.localFrameB = bone->localFrameB;
			jointDef.enableLimit = true;
			jointDef.lowerAngle = bone->twistLimit.x;
			jointDef.upperAngle = bone->twistLimit.y;
			jointDef.enableSpring = hertz > B3_FIX( 0.0f );
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = b3FixMul( bone->jointFriction, frictionTorque );
			bone->jointId = b3CreateRevoluteJoint( worldId, &jointDef );
		}
		else if ( bone->jointType == b3_sphericalJoint )
		{
			b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
			jointDef.base.bodyIdA = bodyIdA;
			jointDef.base.bodyIdB = bodyIdB;
			jointDef.base.localFrameA = bone->localFrameA;
			jointDef.base.localFrameB = bone->localFrameB;
			jointDef.enableConeLimit = true;
			jointDef.coneAngle = bone->swingLimit;
			jointDef.enableTwistLimit = true;
			jointDef.lowerTwistAngle = bone->twistLimit.x;
			jointDef.upperTwistAngle = bone->twistLimit.y;
			jointDef.enableSpring = hertz > B3_FIX( 0.0f );
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = b3FixMul( bone->jointFriction, frictionTorque );
			bone->jointId = b3CreateSphericalJoint( worldId, &jointDef );
		}
	}

	// Disable some collisions
	human->filterJointCount = 0;
	b3FilterJointDef filterDef = b3DefaultFilterJointDef();
	filterDef.base.bodyIdA = human->bones[bone_thigh_l].bodyId;
	filterDef.base.bodyIdB = human->bones[bone_thigh_r].bodyId;
	human->filterJoints[human->filterJointCount++] = b3CreateFilterJoint( worldId, &filterDef );
	filterDef.base.bodyIdA = human->bones[bone_neck].bodyId;
	filterDef.base.bodyIdB = human->bones[bone_upper_arm_l].bodyId;
	human->filterJoints[human->filterJointCount++] = b3CreateFilterJoint( worldId, &filterDef );
	filterDef.base.bodyIdA = human->bones[bone_neck].bodyId;
	filterDef.base.bodyIdB = human->bones[bone_upper_arm_r].bodyId;
	human->filterJoints[human->filterJointCount++] = b3CreateFilterJoint( worldId, &filterDef );

	human->isSpawned = true;
}
void DestroyHuman( Human* human )
{
	assert( human->isSpawned == true );

	for ( int i = 0; i < human->filterJointCount; ++i )
	{
		b3DestroyJoint( human->filterJoints[i], false );
		human->filterJoints[i] = b3_nullJointId;
	}

	for ( int i = 0; i < bone_count; ++i )
	{
		if ( B3_IS_NULL( human->bones[i].jointId ) )
		{
			continue;
		}

		b3DestroyJoint( human->bones[i].jointId, false );
		human->bones[i].jointId = b3_nullJointId;
	}

	for ( int i = 0; i < bone_count; ++i )
	{
		if ( B3_IS_NULL( human->bones[i].bodyId ) )
		{
			continue;
		}

		b3DestroyBody( human->bones[i].bodyId );
		human->bones[i].bodyId = b3_nullBodyId;
	}

	human->isSpawned = false;
}

void Human_SetVelocity( Human* human, b3Vec3 velocity )
{
	for ( int i = 0; i < bone_count; ++i )
	{
		b3BodyId bodyId = human->bones[i].bodyId;

		if ( B3_IS_NULL( bodyId ) )
		{
			continue;
		}

		b3Body_SetLinearVelocity( bodyId, velocity );
	}
}

void Human_ApplyRandomAngularImpulse( Human* human, b3Fixed magnitude )
{
	assert( human->isSpawned == true );
	b3Vec3 range = { magnitude, magnitude, magnitude };
	b3Vec3 impulse = RandomVec3( b3Neg( range ), range );
	b3Body_ApplyAngularImpulse( human->bones[bone_spine_01].bodyId, impulse, true );
}

void Human_SetJointFrictionTorque( Human* human, b3Fixed torque )
{
	assert( human->isSpawned == true );
	human->frictionTorque = torque;

	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJoint_SetMaxMotorTorque( bone->jointId, b3FixMul( bone->jointFriction , torque ) );
		}
		else
		{
			b3SphericalJoint_SetMaxMotorTorque( bone->jointId, b3FixMul( bone->jointFriction , torque ) );
		}
	}
}

void Human_SetJointSpringHertz( Human* human, b3Fixed hertz )
{
	assert( human->isSpawned == true );
	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJoint_SetSpringHertz( bone->jointId, hertz );
		}
		else
		{
			b3SphericalJoint_SetSpringHertz( bone->jointId, hertz );
		}
	}
}

void Human_SetJointDampingRatio( Human* human, b3Fixed dampingRatio )
{
	assert( human->isSpawned == true );
	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJoint_SetSpringDampingRatio( bone->jointId, dampingRatio );
		}
		else
		{
			b3SphericalJoint_SetSpringDampingRatio( bone->jointId, dampingRatio );
		}
	}
}

void Human_AlignSpring( Human* human, b3WorldId worldId, b3BodyId groundId, b3Fixed hertz, b3Fixed dampingRatio )
{
	assert( human->isSpawned == true );

	Bone* bone = human->bones + bone_pelvis;
	assert( B3_IS_NULL( bone->jointId ) == true );
	b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
	b3Quat qb = b3Body_GetRotation( bone->bodyId );

	b3ParallelJointDef jointDef = b3DefaultParallelJointDef();
	jointDef.base.bodyIdA = groundId;
	jointDef.base.bodyIdB = bone->bodyId;
	jointDef.base.localFrameA.q = q;
	jointDef.base.localFrameB.q = b3InvMulQuat( qb, q );
	jointDef.base.drawScale = B3_FIX( 2.0f );
	jointDef.base.collideConnected = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;

	bone->jointId = b3CreateParallelJoint( worldId, &jointDef );
}

void Human_CreateMotorAnchors( Human* human, b3WorldId worldId )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_kinematicBody;

	b3MotorJointDef motorDef = b3DefaultMotorJointDef();
	motorDef.angularHertz = B3_FIX( 5.0f );
	motorDef.angularDampingRatio = B3_FIX( 1.0f );
	motorDef.linearHertz = B3_FIX( 5.0f );
	motorDef.linearDampingRatio = B3_FIX( 1.0f );
	motorDef.maxSpringForce = B3_FIXED_MAX;
	motorDef.maxSpringTorque = B3_FIXED_MAX;

	for ( int i = 0; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		b3WorldTransform bodyTransform = b3Body_GetTransform( bone->bodyId );
		anchorDef.position = bodyTransform.p;
		anchorDef.rotation = bodyTransform.q;
		bone->anchorId = b3CreateBody( worldId, &anchorDef );

		motorDef.base.bodyIdA = bone->anchorId;
		motorDef.base.bodyIdB = bone->bodyId;

		bone->anchorJointId = b3CreateMotorJoint( worldId, &motorDef );
	}
}

void Human_CreateParallelAnchors( Human* human, b3WorldId worldId )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_kinematicBody;

	b3Quat qFrameWorld = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
	b3ParallelJointDef jointDef = b3DefaultParallelJointDef();
	jointDef.hertz = B3_FIX( 8.0f );
	jointDef.dampingRatio = B3_FIX( 1.0f );
	jointDef.maxTorque = B3_FIX( 800.0f );

	for ( int i = 0; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		b3WorldTransform bodyTransform = b3Body_GetTransform( bone->bodyId );
		anchorDef.position = bodyTransform.p;
		anchorDef.rotation = bodyTransform.q;
		bone->anchorId = b3CreateBody( worldId, &anchorDef );

		jointDef.base.bodyIdA = bone->anchorId;
		jointDef.base.bodyIdB = bone->bodyId;

		b3Quat frameQuat = b3InvMulQuat( bodyTransform.q, qFrameWorld );
		jointDef.base.localFrameA.q = frameQuat;
		jointDef.base.localFrameB.q = frameQuat;

		bone->anchorJointId = b3CreateParallelJoint( worldId, &jointDef );
	}
}

void Human_SetBullet( Human* human, bool flag )
{
	for ( int i = 0; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		b3Body_SetBullet( bone->bodyId, flag );
	}
}

#if 0
void Human::EnablePoseControl( b3World* world, b3Fixed springHertz, bool poseControl )
{
	if ( poseControl == m_poseControl )
	{
		return;
	}

	if ( m_poseControl )
	{
		for ( int i = 0; i < bone_count; ++i )
		{
			Bone* bone = human->bones + i;

			world->RemoveJoint( bone->poseJoint, true );
			bone->poseJoint = nullptr;
		}

		world->DestroyBody( m_rootBody, true );
		m_rootBody = nullptr;
	}
	else
	{
		b3BodyDef rootDef = b3DefaultBodyDef();
		rootDef.type = b3_kinematicBody;
		rootDef.position = m_baseTransform.p;
		rootDef.rotation = m_baseTransform.q;
		m_rootBody = world->CreateBody( &rootDef );

		b3Fixed dampingRatio = 0.9f;

		for ( int i = 0; i < bone_count; ++i )
		{
			Bone* bone = human->bones + i;

			b3Body* bodyIdA = m_rootBody;
			b3Body* bodyIdB = bone->bodyId;

			bool enableCollision = false;
			bool useBlockSolver = false;

			// bone->poseJoint = world->AddRigidJoint( bodyIdA, bodyIdB, bodyIdB->GetPosition(), enableCollision, useBlockSolver );
			bone->poseJoint = world->AddRigidJoint( bodyIdA, bone->referenceFrame, bodyIdB, b3Transform_identity,
													b3Transform_identity, enableCollision, useBlockSolver );
			bone->poseJoint->SetAngularFrequency( springHertz );
			bone->poseJoint->SetAngularDampingRatio( dampingRatio );
			bone->poseJoint->SetLinearFrequency( springHertz );
			bone->poseJoint->SetLinearDampingRatio( dampingRatio );
		}
	}

	m_poseControl = poseControl;
}

void Human::AdjustPoseControl( b3Fixed springHertz )
{
	if ( m_poseControl == false )
	{
		return;
	}

	for ( int i = 0; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		bone->poseJoint->SetAngularFrequency( springHertz );
		bone->poseJoint->SetLinearFrequency( springHertz );
	}
}

void Human::DriveBase( const b3Transform& transform, b3Fixed timeStep )
{
	m_rootBody->SetVelocityFromKeyframe( transform, timeStep );
	m_baseTransform = transform;
}

void Human::EnableMotors( bool enableMotors )
{
	if ( enableMotors == m_motorized )
	{
		return;
	}

	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		if ( bone->joint == nullptr )
		{
			continue;
		}

		if ( bone->joint->mType == b3_revoluteJoint )
		{
			b3RevoluteJoint* joint = static_cast<b3RevoluteJoint*>( bone->joint );
			if ( enableMotors )
			{
				joint->SetMotorMode( B3_POSITION_MODE );
			}
			else
			{
				joint->SetMotorFriction( bone->jointFriction );
			}
		}
		else if ( bone->joint->mType == b3_sphericalJoint )
		{
			b3SphericalJoint* joint = static_cast<b3SphericalJoint*>( bone->joint );
			if ( enableMotors )
			{
				joint->SetMotorMode( B3_POSITION_MODE );
			}
			else
			{
				joint->SetMotorFriction( bone->jointFriction );
			}
		}
	}

	m_motorized = enableMotors;
}

void Human::AdjustMotors( b3Fixed springHertz )
{
	if ( m_motorized == false )
	{
		return;
	}

	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		if ( bone->joint == nullptr )
		{
			continue;
		}

		if ( bone->joint->mType == b3_revoluteJoint )
		{
			b3RevoluteJoint* joint = static_cast<b3RevoluteJoint*>( bone->joint );
			joint->mAngularSpring.Frequency = springHertz;
		}
		else if ( bone->joint->mType == b3_sphericalJoint )
		{
			b3SphericalJoint* joint = static_cast<b3SphericalJoint*>( bone->joint );
			joint->mAngularSpring.Frequency = springHertz;
		}
	}
}
#endif

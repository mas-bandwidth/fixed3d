b3Vec3 gravity = {0, -10, 0};
b3World_SetGravity(m_worldId, gravity);
std::vector<b3BodyId> bodies;
std::vector<b3JointId> joints;

{
  b3BodyDef bd = b3DefaultBodyDef();
  bd.name = "cube";
  bd.type = b3BodyType(2);
  bd.position = {B3_FIX( -7.98573353e-07 ), B3_FIX( 0.499929696 ), B3_FIX( -9.86034479e-07 )};
  bd.rotation = {{B3_FIX( -4.03613463e-08 ), B3_FIX( 0.825855613 ), B3_FIX( 4.68083634e-08 )}, B3_FIX( 0.563881755 )};
  bd.linearVelocity = {B3_FIX( 2.47239775e-08 ), B3_FIX( -1.7695136e-08 ), B3_FIX( 8.65693384e-09 )};
  bd.angularVelocity = {B3_FIX( 1.73163173e-08 ), B3_FIX( -4.2026933e-15 ), B3_FIX( -4.94549042e-08 )};
  bd.linearDamping = 0;
  bd.angularDamping = 0;
  bd.enableSleep = bool(1);
  bd.isAwake = bool(1);
  bd.gravityScale = 1;
  b3BodyId bodyId = b3CreateBody(m_worldId, &bd);
  bodies.push_back(bodyId);

  {
    b3ShapeDef sd = b3DefaultShapeDef();
    sd.density = 1000;
    sd.isSensor = bool(0);
    sd.filter.categoryBits = 0x1;
    sd.filter.maskBits = 0xffffffffffffffff;
    sd.filter.groupIndex = 0;
    sd.baseMaterial.friction = B3_FIX( 0.600000024 );
    sd.baseMaterial.restitution = 0;
    sd.baseMaterial.rollingResistance = 0;
    b3Vec3 vs[8];
    vs[0] = {B3_FIX( 0.5 ), B3_FIX( 0.5 ), B3_FIX( 0.5 )};
    vs[1] = {B3_FIX( -0.5 ), B3_FIX( 0.5 ), B3_FIX( 0.5 )};
    vs[2] = {B3_FIX( -0.5 ), B3_FIX( -0.5 ), B3_FIX( 0.5 )};
    vs[3] = {B3_FIX( 0.5 ), B3_FIX( -0.5 ), B3_FIX( 0.5 )};
    vs[4] = {B3_FIX( 0.5 ), B3_FIX( 0.5 ), B3_FIX( -0.5 )};
    vs[5] = {B3_FIX( -0.5 ), B3_FIX( 0.5 ), B3_FIX( -0.5 )};
    vs[6] = {B3_FIX( -0.5 ), B3_FIX( -0.5 ), B3_FIX( -0.5 )};
    vs[7] = {B3_FIX( 0.5 ), B3_FIX( -0.5 ), B3_FIX( -0.5 )};
    b3HullData* hullData = b3CreateHull(vs, 8, 8);
    b3CreateHullShape(bodyId, &sd, hullData);
    b3DestroyHull(hullData);
  }
}
{
  b3BodyDef bd = b3DefaultBodyDef();
  bd.name = "ground";
  bd.type = b3BodyType(0);
  bd.position = {0, -1, 0};
  bd.rotation = {{0, 0, 0}, 1};
  bd.linearVelocity = {0, 0, 0};
  bd.angularVelocity = {0, 0, 0};
  bd.linearDamping = 0;
  bd.angularDamping = 0;
  bd.enableSleep = bool(1);
  bd.isAwake = bool(0);
  bd.gravityScale = 1;
  b3BodyId bodyId = b3CreateBody(m_worldId, &bd);
  bodies.push_back(bodyId);

  {
    b3ShapeDef sd = b3DefaultShapeDef();
    sd.density = 1000;
    sd.isSensor = bool(0);
    sd.filter.categoryBits = 0x1;
    sd.filter.maskBits = 0xffffffffffffffff;
    sd.filter.groupIndex = 0;
    sd.baseMaterial.friction = B3_FIX( 0.600000024 );
    sd.baseMaterial.restitution = 0;
    sd.baseMaterial.rollingResistance = 0;
    b3Vec3 vs[8];
    vs[0] = {15, 1, 15};
    vs[1] = {-15, 1, 15};
    vs[2] = {-15, -1, 15};
    vs[3] = {15, -1, 15};
    vs[4] = {15, 1, -15};
    vs[5] = {-15, 1, -15};
    vs[6] = {-15, -1, -15};
    vs[7] = {15, -1, -15};
    b3HullData* hullData = b3CreateHull(vs, 8, 8);
    b3CreateHullShape(bodyId, &sd, hullData);
    b3DestroyHull(hullData);
  }
}

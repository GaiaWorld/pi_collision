use parry3d::{
    bounding_volume::AABB,
    math::{Isometry as Isometry3D, Point, Vector},
    query::Ray as Ray3D,
    shape::{
        Ball as Ball3D, ConvexPolyhedron as ConvexPolyhedron3D, Cuboid as Cuboid3D,
        HalfSpace as HalfSpace3D, Triangle as Triangle3D,
    },
};

pub type Point3 = Point<f32>;
pub type Vector3 = Vector<f32>;
pub type Isometry = Isometry3D<f32>;
pub type ConvexPolyhedron = ConvexPolyhedron3D;
pub type HalfSpace = HalfSpace3D;
pub type Ball = Ball3D;
pub type Cuboid = Cuboid3D;
pub type MTriangle = Triangle3D;
pub type Ray = Ray3D;

pub struct Triangle(pub MTriangle, pub Isometry);
pub struct Sphere(pub Ball, pub Isometry);
pub struct Plane(pub HalfSpace, pub Isometry);
pub struct Frustum(pub ConvexPolyhedron, pub Isometry);

pub struct Aabb(pub Cuboid, pub Isometry, pub AABB);

impl Aabb {
    pub fn new(mins: Point3, maxs: Point3) -> Self {
        let r = (maxs - mins) * 0.5;
        let cuboid = Cuboid::new(r);
        let pos = Isometry::translation(mins.x + r.x, mins.y + r.y, mins.z + r.z);

        let aabb = AABB::new(mins, maxs);

        Self(cuboid, pos, aabb)
    }
}

pub struct Obb(pub Cuboid, pub Isometry);

impl Obb {
    pub fn new(pos: Point3, dir: Vector3, half_extents: Vector3) -> Self {
        let cuboid = Cuboid::new(half_extents);

        Self(
            cuboid,
            Isometry::new(Vector3::new(pos.x, pos.y, pos.z), dir),
        )
    }
}

pub mod shape;

use parry3d::bounding_volume::BoundingVolume;
use parry3d::{
    math::Isometry,
    query::{PointQuery, RayCast},
};
use shape::{Aabb, Ball, Frustum, Obb, Plane, Point3, Ray, Sphere, Triangle};

use crate::shape::{ConvexPolyhedron, HalfSpace, MTriangle, Vector3};

pub fn compute_point_and_sphere(point: &Point3, sphere: &Sphere) -> bool {
    sphere.0.contains_point(&sphere.1, point)
}

#[test]
fn test_point_and_sphere() {
    let point = Point3::new(0.0, 0.0, 0.0);

    let sphere = Sphere(Ball::new(2.0), Isometry::translation(0.0, 0.0, 0.0));
    assert_eq!(compute_point_and_sphere(&point, &sphere), true);

    let sphere = Sphere(Ball::new(2.0), Isometry::translation(0.0, 2.0, 0.0));
    assert_eq!(compute_point_and_sphere(&point, &sphere), true);

    let sphere = Sphere(Ball::new(2.0), Isometry::translation(0.0, 2.0, 2.0));
    assert_eq!(compute_point_and_sphere(&point, &sphere), false);
}

pub fn compute_point_and_aabb(point: &Point3, aabb: &Aabb) -> bool {
    aabb.2.contains_point(&Isometry::identity(), point)
}

#[test]
fn test_point_and_aabb() {
    let point = Point3::new(0.0, 0.0, 0.0);

    let aabb = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
    assert_eq!(compute_point_and_aabb(&point, &aabb), true);

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    assert_eq!(compute_point_and_aabb(&point, &aabb), true);

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 1.0), Point3::new(2.0, 2.0, 2.0));
    assert_eq!(compute_point_and_aabb(&point, &aabb), false);
}

pub fn compute_point_and_obb(point: &Point3, obb: &Obb) -> bool {
    obb.0.contains_point(&obb.1, point)
}

#[test]
fn test_point_and_obb() {
    let point = Point3::new(0.0, 0.0, 0.0);

    let obb = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_point_and_obb(&point, &obb), true);

    let obb = Obb::new(
        Point3::new(1.0, 1.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_point_and_obb(&point, &obb), true);

    let obb = Obb::new(
        Point3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_point_and_obb(&point, &obb), false);
}

pub fn compute_line_and_sphere(line: &Ray, sphere: &Sphere) -> bool {
    let ray = line;
    if sphere.0.intersects_ray(&sphere.1, ray, f32::MAX) {
        return true;
    }

    let reverse_ray = Ray::new(line.origin, -line.dir);
    sphere.0.intersects_ray(&sphere.1, &reverse_ray, f32::MAX)
}

#[test]
fn test_line_and_sphere() {
    let line = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    let sphere = Sphere(Ball::new(1.0), Isometry::identity());
    assert_eq!(compute_line_and_sphere(&line, &sphere), true);

    let sphere = Sphere(
        Ball::new(1.0),
        Isometry::rotation(Vector3::new(1.0, 2.0, 2.0)),
    );
    assert_eq!(compute_line_and_sphere(&line, &sphere), true);

    let sphere = Sphere(Ball::new(1.0), Isometry::translation(1.0, 1.0, -1.0));
    assert_eq!(compute_line_and_sphere(&line, &sphere), false);
}

pub fn compute_line_and_aabb(line: &Ray, aabb: &Aabb) -> bool {
    let ray = line;
    if aabb.2.intersects_ray(&Isometry::identity(), ray, f32::MAX) {
        return true;
    }

    let reverse_ray = Ray::new(line.origin, -line.dir);
    aabb.2
        .intersects_ray(&Isometry::identity(), &reverse_ray, f32::MAX)
}

#[test]
fn test_line_and_aabb() {
    let line = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    assert_eq!(compute_line_and_aabb(&line, &aabb), true);

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, -1.0));
    assert_eq!(compute_line_and_aabb(&line, &aabb), true);

    let aabb = Aabb::new(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 2.0, -1.0));
    assert_eq!(compute_line_and_aabb(&line, &aabb), false);
}

pub fn compute_line_and_obb(line: &Ray, obb: &Obb) -> bool {
    let ray = line;
    if obb.0.intersects_ray(&obb.1, ray, f32::MAX) {
        return true;
    }

    let reverse_ray = Ray::new(line.origin, -line.dir);
    obb.0.intersects_ray(&obb.1, &reverse_ray, f32::MAX)
}

#[test]
fn test_line_and_obb() {
    let line = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    let obb = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_line_and_obb(&line, &obb), true);

    let obb = Obb::new(
        Point3::new(1.0, 1.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_line_and_obb(&line, &obb), true);

    let obb = Obb::new(
        Point3::new(2.0, 2.0, -2.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_line_and_obb(&line, &obb), false);
}

pub fn compute_line_and_triangle(line: &Ray, tri: &Triangle) -> bool {
    let ray = line;
    if tri.0.intersects_ray(&tri.1, ray, f32::MAX) {
        return true;
    }

    let reverse_ray = Ray::new(line.origin, -line.dir);
    tri.0.intersects_ray(&tri.1, &reverse_ray, f32::MAX)
}

#[test]
fn test_line_and_triangle() {
    let line = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::identity(),
    );
    assert_eq!(compute_line_and_triangle(&line, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::rotation(Vector3::new(1.0, 0.0, 0.0)),
    );
    assert_eq!(compute_line_and_triangle(&line, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::translation(1.0, 0.0, -1.0),
    );
    assert_eq!(compute_line_and_triangle(&line, &tri), false);
}

pub fn compute_ray_and_sphere(ray: &Ray, sphere: &Sphere) -> bool {
    sphere.0.intersects_ray(&sphere.1, ray, f32::MAX)
}

#[test]
fn test_ray_and_sphere() {
    let ray = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    let sphere = Sphere(Ball::new(1.0), Isometry::identity());
    assert_eq!(compute_ray_and_sphere(&ray, &sphere), true);

    let sphere = Sphere(Ball::new(1.0), Isometry::translation(1.0, 0.0, 0.0));
    assert_eq!(compute_ray_and_sphere(&ray, &sphere), true);

    let sphere = Sphere(Ball::new(1.0), Isometry::translation(2.0, 0.0, 0.0));
    assert_eq!(compute_ray_and_sphere(&ray, &sphere), false);
}

pub fn compute_ray_and_aabb(ray: &Ray, aabb: &Aabb) -> bool {
    aabb.2.intersects_ray(&Isometry::identity(), ray, f32::MAX)
}

#[test]
fn test_ray_and_aabb() {
    let ray = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    assert_eq!(compute_ray_and_aabb(&ray, &aabb), true);

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, -1.0));
    assert_eq!(compute_ray_and_aabb(&ray, &aabb), true);

    let aabb = Aabb::new(Point3::new(1.0, 1.0, -1.0), Point3::new(2.0, 2.0, -2.0));
    assert_eq!(compute_ray_and_aabb(&ray, &aabb), false);
}

pub fn compute_ray_and_obb(ray: &Ray, obb: &Obb) -> bool {
    obb.0.intersects_ray(&obb.1, ray, f32::MAX)
}

#[test]
fn test_ray_and_obb() {
    let ray = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    let obb = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_ray_and_obb(&ray, &obb), true);

    let obb = Obb::new(
        Point3::new(1.0, 1.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_ray_and_obb(&ray, &obb), true);

    let obb = Obb::new(
        Point3::new(2.0, 2.0, -2.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_ray_and_obb(&ray, &obb), false);
}

pub fn compute_ray_and_triangle(ray: &Ray, tri: &Triangle) -> bool {
    tri.0.intersects_ray(&tri.1, ray, f32::MAX)
}

#[test]
fn test_ray_and_triangle() {
    let ray = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::identity(),
    );
    assert_eq!(compute_ray_and_triangle(&ray, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::rotation(Vector3::new(1.0, 0.0, 0.0)),
    );
    assert_eq!(compute_ray_and_triangle(&ray, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::translation(1.0, 0.0, -1.0),
    );
    assert_eq!(compute_ray_and_triangle(&ray, &tri), false);
}

pub fn compute_sphere_and_sphere(sphere0: &Sphere, sphere1: &Sphere) -> bool {
    parry3d::query::intersection_test(&sphere0.1, &sphere0.0, &sphere1.1, &sphere1.0).unwrap()
}

#[test]
fn test_sphere_and_sphere() {
    let sphere0 = Sphere(Ball::new(1.0), Isometry::identity());

    let sphere = Sphere(Ball::new(1.0), Isometry::translation(-1.0, 0.0, 0.0));
    assert_eq!(compute_sphere_and_sphere(&sphere0, &sphere), true);

    let sphere = Sphere(Ball::new(1.0), Isometry::translation(2.0, 0.0, 0.0));
    assert_eq!(compute_sphere_and_sphere(&sphere0, &sphere), true);

    let sphere = Sphere(Ball::new(1.0), Isometry::translation(2.0, 2.0, 2.0));
    assert_eq!(compute_sphere_and_sphere(&sphere0, &sphere), false);
}

pub fn compute_sphere_and_aabb(sphere: &Sphere, aabb: &Aabb) -> bool {
    parry3d::query::intersection_test(&sphere.1, &sphere.0, &aabb.1, &aabb.0).unwrap()
}

#[test]
fn test_sphere_and_aabb() {
    let sphere = Sphere(Ball::new(1.0), Isometry::identity());

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    assert_eq!(compute_sphere_and_aabb(&sphere, &aabb), true);

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, -1.0));
    assert_eq!(compute_sphere_and_aabb(&sphere, &aabb), true);

    let aabb = Aabb::new(Point3::new(1.0, 1.0, -1.0), Point3::new(2.0, 2.0, -2.0));
    assert_eq!(compute_sphere_and_aabb(&sphere, &aabb), false);
}

pub fn compute_sphere_and_obb(sphere: &Sphere, obb: &Obb) -> bool {
    parry3d::query::intersection_test(&sphere.1, &sphere.0, &obb.1, &obb.0).unwrap()
}

#[test]
fn test_sphere_and_obb() {
    let sphere = Sphere(Ball::new(1.0), Isometry::identity());

    let obb = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_sphere_and_obb(&sphere, &obb), true);

    let obb = Obb::new(
        Point3::new(1.0, 1.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_sphere_and_obb(&sphere, &obb), true);

    let obb = Obb::new(
        Point3::new(2.0, 2.0, -2.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_sphere_and_obb(&sphere, &obb), false);
}

pub fn compute_sphere_and_plane(sphere: &Sphere, plane: &Plane) -> bool {
    parry3d::query::intersection_test(&sphere.1, &sphere.0, &plane.1, &plane.0).unwrap()
}

#[test]
fn test_sphere_and_plane() {
    let sphere = Sphere(Ball::new(1.0), Isometry::identity());

    let plane = Plane(HalfSpace::new(Vector3::y_axis()), Isometry::identity());
    assert_eq!(compute_sphere_and_plane(&sphere, &plane), true);

    let plane = Plane(
        HalfSpace::new(Vector3::y_axis()),
        Isometry::translation(0.0, -1.0, 0.0),
    );
    assert_eq!(compute_sphere_and_plane(&sphere, &plane), true);

    let plane = Plane(
        HalfSpace::new(Vector3::y_axis()),
        Isometry::translation(0.0, -2.0, 0.0),
    );
    assert_eq!(compute_sphere_and_plane(&sphere, &plane), false);
}

pub fn compute_sphere_and_triangle(sphere: &Sphere, tri: &Triangle) -> bool {
    parry3d::query::intersection_test(&sphere.1, &sphere.0, &tri.1, &tri.0).unwrap()
}

#[test]
fn test_sphere_and_triangle() {
    let sphere = Sphere(Ball::new(1.0), Isometry::identity());

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::identity(),
    );
    assert_eq!(compute_sphere_and_triangle(&sphere, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::rotation(Vector3::new(1.0, 0.0, 0.0)),
    );
    assert_eq!(compute_sphere_and_triangle(&sphere, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::translation(1.0, 0.0, -1.0),
    );
    assert_eq!(compute_sphere_and_triangle(&sphere, &tri), false);
}

pub fn compute_sphere_and_frustum(sphere: &Sphere, frustum: &Frustum) -> bool {
    parry3d::query::intersection_test(&sphere.1, &sphere.0, &frustum.1, &frustum.0).unwrap()
}

#[test]
fn test_sphere_and_frustum() {
    let sphere = Sphere(Ball::new(1.0), Isometry::identity());

    let points = vec![
        // near
        Point3::new(2.0f32, 1.0, 0.0),
        Point3::new(-2.0, 1.0, 0.0),
        Point3::new(-2.0, -1.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        // far
        Point3::new(3.0, 2.0, -5.0),
        Point3::new(-3.0, 2.0, -5.0),
        Point3::new(-3.0, -2.0, -5.0),
        Point3::new(3.0, -2.0, -5.0),
    ];

    let indices = vec![
        // near
        [0, 1, 2],
        [2, 3, 0],
        // far
        [4, 5, 6],
        [6, 7, 4],
        // top
        [0, 1, 5],
        [5, 4, 0],
        // bottom
        [3, 2, 6],
        [6, 7, 3],
        // left
        [1, 5, 6],
        [6, 2, 1],
        // right
        [0, 4, 7],
        [7, 3, 0],
    ];
    let convex =
        ConvexPolyhedron::from_convex_mesh(points, &indices).expect("Invalid convex shape.");

    let frustum = Frustum(convex.clone(), Isometry::identity());
    assert_eq!(compute_sphere_and_frustum(&sphere, &frustum), true);

    let frustum = Frustum(convex.clone(), Isometry::translation(0.0, 0.0, -1.0));
    assert_eq!(compute_sphere_and_frustum(&sphere, &frustum), true);

    let frustum = Frustum(convex, Isometry::translation(0.0, 0.0, -2.0));
    assert_eq!(compute_sphere_and_frustum(&sphere, &frustum), false);
}

pub fn compute_aabb_and_aabb(aabb0: &Aabb, aabb1: &Aabb) -> bool {
    aabb0.2.intersects(&aabb1.2)
}

#[test]
fn test_aabb_and_aabb() {
    let aabb0 = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));

    let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
    assert_eq!(compute_aabb_and_aabb(&aabb0, &aabb), true);

    let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    assert_eq!(compute_aabb_and_aabb(&aabb0, &aabb), true);

    let aabb = Aabb::new(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 1.0, 1.0));
    assert_eq!(compute_aabb_and_aabb(&aabb0, &aabb), false);
}

pub fn compute_aabb_and_obb(aabb: &Aabb, obb: &Obb) -> bool {
    parry3d::query::intersection_test(&aabb.1, &aabb.0, &obb.1, &obb.0).unwrap()
}

#[test]
fn test_aabb_and_obb() {
    let aabb0 = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));

    let obb = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_aabb_and_obb(&aabb0, &obb), true);

    let obb = Obb::new(
        Point3::new(1.0, 1.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_aabb_and_obb(&aabb0, &obb), true);

    let obb = Obb::new(
        Point3::new(3.0, 3.0, -3.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_aabb_and_obb(&aabb0, &obb), false);
}

pub fn compute_aabb_and_plane(aabb: &Aabb, plane: &Plane) -> bool {
    parry3d::query::intersection_test(&aabb.1, &aabb.0, &plane.1, &plane.0).unwrap()
}

#[test]
fn test_aabb_and_plane() {
    let aabb0 = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));

    let plane = Plane(HalfSpace::new(Vector3::y_axis()), Isometry::identity());
    assert_eq!(compute_aabb_and_plane(&aabb0, &plane), true);

    let plane = Plane(
        HalfSpace::new(Vector3::y_axis()),
        Isometry::translation(0.0, -1.0, 0.0),
    );
    assert_eq!(compute_aabb_and_plane(&aabb0, &plane), true);

    let plane = Plane(
        HalfSpace::new(Vector3::y_axis()),
        Isometry::translation(0.0, -2.0, 0.0),
    );
    assert_eq!(compute_aabb_and_plane(&aabb0, &plane), false);
}

pub fn compute_aabb_and_triangle(aabb: &Aabb, tri: &Triangle) -> bool {
    parry3d::query::intersection_test(&aabb.1, &aabb.0, &tri.1, &tri.0).unwrap()
}

#[test]
fn test_aabb_and_triangle() {
    let aabb0 = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::identity(),
    );
    assert_eq!(compute_aabb_and_triangle(&aabb0, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::rotation(Vector3::new(1.0, 0.0, 0.0)),
    );
    assert_eq!(compute_aabb_and_triangle(&aabb0, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::translation(2.0, 0.0, -1.0),
    );
    assert_eq!(compute_aabb_and_triangle(&aabb0, &tri), false);
}

pub fn compute_aabb_and_frustum(aabb: &Aabb, frustum: &Frustum) -> bool {
    parry3d::query::intersection_test(&aabb.1, &aabb.0, &frustum.1, &frustum.0).unwrap()
}

#[test]
fn test_aabb_and_frustum() {
    let aabb0 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));

    let points = vec![
        // near
        Point3::new(2.0f32, 1.0, 0.0),
        Point3::new(-2.0, 1.0, 0.0),
        Point3::new(-2.0, -1.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        // far
        Point3::new(3.0, 2.0, -5.0),
        Point3::new(-3.0, 2.0, -5.0),
        Point3::new(-3.0, -2.0, -5.0),
        Point3::new(3.0, -2.0, -5.0),
    ];

    let indices = vec![
        // near
        [0, 1, 2],
        [2, 3, 0],
        // far
        [4, 5, 6],
        [6, 7, 4],
        // top
        [0, 1, 5],
        [5, 4, 0],
        // bottom
        [3, 2, 6],
        [6, 7, 3],
        // left
        [1, 5, 6],
        [6, 2, 1],
        // right
        [0, 4, 7],
        [7, 3, 0],
    ];
    let convex =
        ConvexPolyhedron::from_convex_mesh(points, &indices).expect("Invalid convex shape.");

    let frustum = Frustum(convex.clone(), Isometry::identity());
    assert_eq!(compute_aabb_and_frustum(&aabb0, &frustum), true);

    let frustum = Frustum(convex.clone(), Isometry::translation(0.0, 0.0, 6.0));
    assert_eq!(compute_aabb_and_frustum(&aabb0, &frustum), true);

    let frustum = Frustum(convex, Isometry::translation(0.0, 0.0, -2.0));
    assert_eq!(compute_aabb_and_frustum(&aabb0, &frustum), false);
}

pub fn compute_obb_and_obb(obb0: &Obb, obb1: &Obb) -> bool {
    parry3d::query::intersection_test(&obb0.1, &obb0.0, &obb1.1, &obb1.0).unwrap()
}

#[test]
fn test_obb_and_obb() {
    let obb0 = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );

    let obb = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_obb_and_obb(&obb0, &obb), true);

    let obb = Obb::new(
        Point3::new(1.0, 1.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_obb_and_obb(&obb0, &obb), true);

    let obb = Obb::new(
        Point3::new(3.0, 3.0, -3.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    assert_eq!(compute_obb_and_obb(&obb0, &obb), false);
}

pub fn compute_obb_and_plane(obb: &Obb, plane: &Plane) -> bool {
    parry3d::query::intersection_test(&obb.1, &obb.0, &plane.1, &plane.0).unwrap()
}

#[test]
fn test_obb_and_plane() {
    let obb0 = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );

    let plane = Plane(HalfSpace::new(Vector3::y_axis()), Isometry::identity());
    assert_eq!(compute_obb_and_plane(&obb0, &plane), true);

    let plane = Plane(
        HalfSpace::new(Vector3::y_axis()),
        Isometry::translation(0.0, -1.0, 0.0),
    );
    assert_eq!(compute_obb_and_plane(&obb0, &plane), true);

    let plane = Plane(
        HalfSpace::new(Vector3::y_axis()),
        Isometry::translation(0.0, -2.0, 0.0),
    );
    assert_eq!(compute_obb_and_plane(&obb0, &plane), false);
}

pub fn compute_obb_and_triangle(obb: &Obb, tri: &Triangle) -> bool {
    parry3d::query::intersection_test(&obb.1, &obb.0, &tri.1, &tri.0).unwrap()
}

#[test]
fn test_obb_and_triangle() {
    let obb0 = Obb::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    );

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::identity(),
    );
    assert_eq!(compute_obb_and_triangle(&obb0, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::rotation(Vector3::new(1.0, 0.0, 0.0)),
    );
    assert_eq!(compute_obb_and_triangle(&obb0, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::translation(2.0, 0.0, -1.0),
    );
    assert_eq!(compute_obb_and_triangle(&obb0, &tri), false);
}

pub fn compute_plane_and_triangle(plane: &Plane, tri: &Triangle) -> bool {
    parry3d::query::intersection_test(&plane.1, &plane.0, &tri.1, &tri.0).unwrap()
}

#[test]
fn test_plane_and_triangle() {
    let plane = Plane(HalfSpace::new(Vector3::y_axis()), Isometry::identity());

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::identity(),
    );
    assert_eq!(compute_plane_and_triangle(&plane, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::rotation(Vector3::new(1.0, 0.0, 0.0)),
    );
    assert_eq!(compute_plane_and_triangle(&plane, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::translation(0.0, 1.0, 0.0),
    );
    assert_eq!(compute_plane_and_triangle(&plane, &tri), false);
}

pub fn compute_triangle_and_triangle(tri0: &Triangle, tri1: &Triangle) -> bool {
    parry3d::query::intersection_test(&tri0.1, &tri0.0, &tri1.1, &tri1.0).unwrap()
}

#[test]
fn test_triangle_and_triangle() {
    let tri0 = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::identity(),
    );

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::translation(1.0, 0.0, 0.0),
    );
    assert_eq!(compute_triangle_and_triangle(&tri0, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::rotation(Vector3::new(1.0, 0.0, 0.0)),
    );
    assert_eq!(compute_triangle_and_triangle(&tri0, &tri), true);

    let tri = Triangle(
        MTriangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ),
        Isometry::translation(2.0, 0.0, -1.0),
    );
    assert_eq!(compute_triangle_and_triangle(&tri0, &tri), false);
}

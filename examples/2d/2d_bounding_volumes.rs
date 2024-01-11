//! Shows how to render simple primitive shapes with a single color.

use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_internal::math::bounding::{Bounded2d, BoundingVolume};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                (
                    draw_aabbs::<primitives::Circle>,
                    draw_aabbs::<primitives::Ellipse>,
                    draw_aabbs::<primitives::Rectangle>,
                    draw_aabbs::<primitives::RegularPolygon>,
                    draw_aabbs::<primitives::Triangle2d>,
                ),
                rotate,
            )
                .chain(),
        )
        .run();
}

#[derive(Component)]
struct Shape<T: Bounded2d>(T);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2dBundle::default());

    let circle = primitives::Circle { radius: 50. };
    let ellipse = primitives::Ellipse::new(50., 100.);
    let rectangle = primitives::Rectangle::new(50., 100.);
    let regular_polygon = primitives::RegularPolygon::new(50., 6);
    let triangle = primitives::Triangle2d::new(
        Vec2::new(-50.0, 50.0),
        Vec2::new(-50.0, -50.0),
        Vec2::new(50.0, -100.0),
    );

    // Circle
    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(circle.into()).into(),
            material: materials.add(ColorMaterial::from(Color::PURPLE)),
            transform: Transform::from_translation(Vec3::new(-200., 100., 0.)),
            ..default()
        },
        Shape(circle),
    ));

    // Rectangle
    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(ellipse.into()).into(),
            material: materials.add(ColorMaterial::from(Color::ORANGE)),
            transform: Transform::from_translation(Vec3::new(-75., 100., 0.)),
            ..default()
        },
        Shape(ellipse),
    ));

    // Quad
    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(rectangle.into()).into(),
            material: materials.add(ColorMaterial::from(Color::LIME_GREEN)),
            transform: Transform::from_translation(Vec3::new(75., 100., 0.)),
            ..default()
        },
        Shape(rectangle),
    ));

    // Hexagon
    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(regular_polygon.into()).into(),
            material: materials.add(ColorMaterial::from(Color::TURQUOISE)),
            transform: Transform::from_translation(Vec3::new(200., 100., 0.)),
            ..default()
        },
        Shape(regular_polygon),
    ));

    // Triangle
    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(triangle.into()).into(),
            material: materials.add(ColorMaterial::from(Color::YELLOW_GREEN)),
            transform: Transform::from_translation(Vec3::new(0., -100., 0.)),
            ..default()
        },
        Shape(triangle),
    ));
}

fn draw_aabbs<T: Bounded2d + Sync + Send + 'static>(
    query: Query<(&Transform, &Shape<T>)>,
    mut gizmos: Gizmos,
) {
    for (transform, shape) in &query {
        let aabb = shape
            .0
            .aabb_2d(default(), transform.rotation.to_euler(EulerRot::XYZ).2);
        let bounding_circle = shape.0.bounding_circle(default());
        gizmos.rect_2d(
            transform.translation.truncate() + aabb.center(),
            default(),
            (aabb.max - aabb.min).abs(),
            Color::WHITE,
        );
        gizmos.circle_2d(
            transform.translation.truncate() + bounding_circle.center(),
            bounding_circle.radius(),
            Color::CYAN,
        );
    }
}

fn rotate(mut query: Query<&mut Transform, Without<Camera>>) {
    for mut transform in &mut query {
        transform.rotate_z(0.02);
    }
}

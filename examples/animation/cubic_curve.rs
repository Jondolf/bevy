//! Demonstrates how to work with Cubic curves.

use bevy::{
    color::palettes::css::{ORANGE, SILVER, WHITE},
    math::vec3,
    prelude::*,
};

#[derive(Component)]
struct Curve(CubicCurve<Vec3>);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, animate_cube)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Define your control points
    // These points will define the curve
    // You can learn more about bezier curves here
    // https://en.wikipedia.org/wiki/B%C3%A9zier_curve
    let points = [[
        vec3(-6., 2., 0.),
        vec3(12., 8., 0.),
        vec3(-12., 8., 0.),
        vec3(6., 2., 0.),
    ]];

    // Make a CubicCurve
    let bezier = CubicBezier::new(points).to_curve().unwrap();

    // Spawning a cube to experiment on
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::from(ORANGE)),
            transform: Transform::from_translation(points[0][0]),
            ..default()
        },
        Curve(bezier),
    ));

    // Some light to see something
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            shadows_enabled: true,
            intensity: 10_000_000.,
            range: 100.0,
            ..default()
        },
        transform: Transform::from_xyz(8., 16., 8.),
        ..default()
    });

    // ground plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(50., 50.)),
        material: materials.add(Color::from(SILVER)),
        ..default()
    });

    // The camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0., 6., 12.).looking_at(Vec3::new(0., 3., 0.), Vec3::Y),
    ));
}

fn animate_cube(time: Res<Time>, mut query: Query<(&mut Transform, &Curve)>, mut gizmos: Gizmos) {
    let t = (ops::sin(time.elapsed_seconds()) + 1.) / 2.;

    for (mut transform, cubic_curve) in &mut query {
        // Draw the curve
        gizmos.linestrip(cubic_curve.0.iter_positions(50), WHITE);
        // position takes a point from the curve where 0 is the initial point
        // and 1 is the last point
        transform.translation = cubic_curve.0.position(t);
    }
}

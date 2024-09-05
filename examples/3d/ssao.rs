//! A scene showcasing screen space ambient occlusion.

use bevy::{
    core_pipeline::experimental::taa::{TemporalAntiAliasBundle, TemporalAntiAliasPlugin},
    pbr::{
        ScreenSpaceAmbientOcclusionBundle, ScreenSpaceAmbientOcclusionQualityLevel,
        ScreenSpaceAmbientOcclusionSettings,
    },
    prelude::*,
    render::camera::TemporalJitter,
};
use std::f32::consts::PI;

fn main() {
    App::new()
        .insert_resource(AmbientLight {
            brightness: 1000.,
            ..default()
        })
        .add_plugins((DefaultPlugins, TemporalAntiAliasPlugin))
        .add_systems(Startup, setup)
        .add_systems(Update, update)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn(Camera3dBundle {
            camera: Camera {
                hdr: true,
                ..default()
            },
            transform: Transform::from_xyz(-2.0, 2.0, -2.0).looking_at(Vec3::ZERO, Vec3::Y),
            msaa: Msaa::Off,
            ..default()
        })
        .insert(ScreenSpaceAmbientOcclusionBundle::default())
        .insert(TemporalAntiAliasBundle::default());

    let material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.5, 0.5, 0.5),
        perceptual_roughness: 1.0,
        reflectance: 0.0,
        ..default()
    });
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()).into(),
            material: material.clone().into(),
        },
        Transform::from_xyz(0.0, 0.0, 1.0),
    ));
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()).into(),
            material: material.clone().into(),
        },
        Transform::from_xyz(0.0, -1.0, 0.0),
    ));
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()).into(),
            material: material.into(),
        },
        Transform::from_xyz(1.0, 0.0, 0.0),
    ));
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Sphere::new(0.4).mesh().uv(72, 36)).into(),
            material: materials
                .add(StandardMaterial {
                    base_color: Color::srgb(0.4, 0.4, 0.4),
                    perceptual_roughness: 1.0,
                    reflectance: 0.0,
                    ..default()
                })
                .into(),
        },
        SphereMarker,
    ));

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(
            EulerRot::ZYX,
            0.0,
            PI * -0.15,
            PI * -0.15,
        )),
        ..default()
    });

    commands.spawn(
        TextBundle::from_section("", TextStyle::default()).with_style(Style {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        }),
    );
}

fn update(
    camera: Query<
        (
            Entity,
            Option<&ScreenSpaceAmbientOcclusionSettings>,
            Option<&TemporalJitter>,
        ),
        With<Camera>,
    >,
    mut text: Query<&mut Text>,
    mut sphere: Query<&mut Transform, With<SphereMarker>>,
    mut commands: Commands,
    keycode: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    let mut sphere = sphere.single_mut();
    sphere.translation.y = (time.elapsed_seconds() / 1.7).sin() * 0.7;

    let (camera_entity, ssao_settings, temporal_jitter) = camera.single();

    let mut commands = commands
        .entity(camera_entity)
        .insert_if(
            ScreenSpaceAmbientOcclusionSettings {
                quality_level: ScreenSpaceAmbientOcclusionQualityLevel::Low,
            },
            || keycode.just_pressed(KeyCode::Digit2),
        )
        .insert_if(
            ScreenSpaceAmbientOcclusionSettings {
                quality_level: ScreenSpaceAmbientOcclusionQualityLevel::Medium,
            },
            || keycode.just_pressed(KeyCode::Digit3),
        )
        .insert_if(
            ScreenSpaceAmbientOcclusionSettings {
                quality_level: ScreenSpaceAmbientOcclusionQualityLevel::High,
            },
            || keycode.just_pressed(KeyCode::Digit4),
        )
        .insert_if(
            ScreenSpaceAmbientOcclusionSettings {
                quality_level: ScreenSpaceAmbientOcclusionQualityLevel::Ultra,
            },
            || keycode.just_pressed(KeyCode::Digit5),
        );
    if keycode.just_pressed(KeyCode::Digit1) {
        commands = commands.remove::<ScreenSpaceAmbientOcclusionSettings>();
    }
    if keycode.just_pressed(KeyCode::Space) {
        if temporal_jitter.is_some() {
            commands.remove::<TemporalJitter>();
        } else {
            commands.insert(TemporalJitter::default());
        }
    }

    let mut text = text.single_mut();
    let text = &mut text.sections[0].value;
    text.clear();

    let (o, l, m, h, u) = match ssao_settings.map(|s| s.quality_level) {
        None => ("*", "", "", "", ""),
        Some(ScreenSpaceAmbientOcclusionQualityLevel::Low) => ("", "*", "", "", ""),
        Some(ScreenSpaceAmbientOcclusionQualityLevel::Medium) => ("", "", "*", "", ""),
        Some(ScreenSpaceAmbientOcclusionQualityLevel::High) => ("", "", "", "*", ""),
        Some(ScreenSpaceAmbientOcclusionQualityLevel::Ultra) => ("", "", "", "", "*"),
        _ => unreachable!(),
    };

    text.push_str("SSAO Quality:\n");
    text.push_str(&format!("(1) {o}Off{o}\n"));
    text.push_str(&format!("(2) {l}Low{l}\n"));
    text.push_str(&format!("(3) {m}Medium{m}\n"));
    text.push_str(&format!("(4) {h}High{h}\n"));
    text.push_str(&format!("(5) {u}Ultra{u}\n\n"));

    text.push_str("Temporal Antialiasing:\n");
    text.push_str(match temporal_jitter {
        Some(_) => "(Space) Enabled",
        None => "(Space) Disabled",
    });
}

#[derive(Component)]
struct SphereMarker;

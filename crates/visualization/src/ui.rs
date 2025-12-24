use bevy::prelude::*;
use bevy::ui::{node_bundles::*, Val, UiRect, PositionType};
use bevy::text::{Text, TextStyle, TextSection};

#[derive(Component)]
pub struct MetricsUI;

#[derive(Resource, Default)]
pub struct TrainingStats {
    pub episode: usize,
    pub total_reward: f32,
    pub avg_reward: f32,
    pub policy_loss: f32,
    pub value_loss: f32,
    pub success_rate: f32,
}

pub fn setup_ui(mut commands: Commands) {
    // Simple text display without custom fonts for Bevy 0.12 compatibility
    commands.spawn((
        NodeBundle {
            style: Style {
                width: Val::Px(300.0),
                height: Val::Px(150.0),
                position_type: PositionType::Absolute,
                left: Val::Px(10.0),
                top: Val::Px(10.0),
                ..default()
            },
            background_color: Color::rgba(0.0, 0.0, 0.0, 0.5).into(),
            ..default()
        },
    )).with_children(|parent| {
        parent.spawn((
            TextBundle {
                text: Text::from_section(
                    "Training Stats\nEpisode: 0\nReward: 0.00",
                    TextStyle {
                        font_size: 16.0,
                        color: Color::WHITE,
                        ..default()
                    },
                ),
                style: Style {
                    margin: UiRect::all(Val::Px(5.0)),
                    ..default()
                },
                ..default()
            },
            MetricsUI,
        ));
    });
}

pub fn update_ui(
    stats: Res<TrainingStats>,
    mut query: Query<&mut Text, With<MetricsUI>>,
) {
    for mut text in query.iter_mut() {
        text.sections[0].value = format!(
            "Training Stats\nEpisode: {}\nReward: {:.2}\nAvg Reward: {:.2}\nPolicy Loss: {:.4}\nValue Loss: {:.4}\nSuccess Rate: {:.1}%",
            stats.episode,
            stats.total_reward,
            stats.avg_reward,
            stats.policy_loss,
            stats.value_loss,
            stats.success_rate * 100.0
        );
    }
}

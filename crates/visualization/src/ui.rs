use bevy::prelude::*;

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
    commands.spawn((
        TextBundle::from_section(
            "Training Stats",
            TextStyle {
                font_size: 20.0,
                color: Color::WHITE,
                ..default()
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        }),
        MetricsUI,
    ));
}

pub fn update_ui(
    stats: Res<TrainingStats>,
    mut query: Query<&mut Text, With<MetricsUI>>,
) {
    for mut text in query.iter_mut() {
        text.sections[0].value = format!(
            "Episode: {}\nReward: {:.2}\nAvg Reward: {:.2}\nPolicy Loss: {:.4}\nValue Loss: {:.4}\nSuccess Rate: {:.1}%",
            stats.episode,
            stats.total_reward,
            stats.avg_reward,
            stats.policy_loss,
            stats.value_loss,
            stats.success_rate * 100.0
        );
    }
}

use std::f64::consts::PI;

use eframe::egui;
use egui_plot::{Legend, Line, MarkerShape, Plot, PlotPoints, Points};

use crate::dsp::biquad::BiquadCascade;

pub struct FilterResponseView;

impl FilterResponseView {
    pub fn ui(
        ui: &mut egui::Ui,
        cascade: &BiquadCascade,
        sample_rate_hz: f64,
        zero_phase: bool,
    ) {
        ui.horizontal(|ui| {
            ui.label(format!(
                "Showing response at fs = {:.1} Hz   (Nyquist {:.1} Hz)",
                sample_rate_hz,
                sample_rate_hz / 2.0
            ));
            if zero_phase {
                ui.separator();
                ui.colored_label(
                    egui::Color32::LIGHT_BLUE,
                    "Zero-phase mode: showing |H|², phase = 0",
                );
            }
        });
        ui.separator();

        ui.columns(2, |cols| {
            cols[0].vertical(|ui| {
                Self::magnitude_plot(ui, cascade, sample_rate_hz, zero_phase);
                ui.add_space(4.0);
                Self::phase_plot(ui, cascade, sample_rate_hz, zero_phase);
            });
            cols[1].vertical(|ui| {
                ui.label("Pole-zero map (z-plane)");
                Self::pole_zero_plot(ui, cascade);
                ui.small("× = pole,  ○ = zero,  unit circle = stability boundary");
                ui.small("Poles inside the circle ⇒ stable filter.");
            });
        });
    }

    fn magnitude_plot(ui: &mut egui::Ui, cascade: &BiquadCascade, fs: f64, zero_phase: bool) {
        let n = 1024;
        let pts: PlotPoints = (1..n)
            .map(|i| {
                let omega = PI * i as f64 / n as f64;
                let f = omega * fs / (2.0 * PI);
                let (mag, _) = cascade.frequency_response(omega);
                // Filtfilt applies the filter twice, so effective magnitude
                // is squared (i.e. dB doubles).
                let effective = if zero_phase { mag * mag } else { mag };
                let db = 20.0 * effective.max(1.0e-12).log10();
                [f, db]
            })
            .collect();

        Plot::new("filter_magnitude")
            .height(190.0)
            .x_axis_label("frequency (Hz)")
            .y_axis_label("|H(f)|  (dB)")
            .legend(Legend::default())
            .show(ui, |plot_ui| {
                let name = if zero_phase { "magnitude (squared)" } else { "magnitude" };
                plot_ui.line(Line::new(pts).name(name));
            });
    }

    fn phase_plot(ui: &mut egui::Ui, cascade: &BiquadCascade, fs: f64, zero_phase: bool) {
        let n = 1024;
        let pts: PlotPoints = (1..n)
            .map(|i| {
                let omega = PI * i as f64 / n as f64;
                let f = omega * fs / (2.0 * PI);
                let (_, phase) = cascade.frequency_response(omega);
                // Filtfilt: phase response is exactly zero by construction.
                let p_deg = if zero_phase { 0.0 } else { phase.to_degrees() };
                [f, p_deg]
            })
            .collect();

        Plot::new("filter_phase")
            .height(190.0)
            .x_axis_label("frequency (Hz)")
            .y_axis_label("phase (deg)")
            .legend(Legend::default())
            .show(ui, |plot_ui| {
                let name = if zero_phase { "phase (zero by construction)" } else { "phase" };
                plot_ui.line(Line::new(pts).name(name));
            });
    }

    // pole_zero_plot is unchanged

    fn pole_zero_plot(ui: &mut egui::Ui, cascade: &BiquadCascade) {
        // Unit circle, sampled as a polyline.
        let circle: PlotPoints = (0..=256)
            .map(|i| {
                let t = 2.0 * PI * i as f64 / 256.0;
                [t.cos(), t.sin()]
            })
            .collect();

        // Collect every section's poles and zeros. First-order sections
        // contribute "phantom" roots at the origin (b2/a2 = 0) — those just
        // cluster harmlessly at z = 0 in the visualization.
        let mut pole_pts: Vec<[f64; 2]> = Vec::new();
        let mut zero_pts: Vec<[f64; 2]> = Vec::new();
        for bq in &cascade.sections {
            let (p1, p2) = bq.poles();
            pole_pts.push([p1.re, p1.im]);
            pole_pts.push([p2.re, p2.im]);
            let (z1, z2) = bq.zeros();
            zero_pts.push([z1.re, z1.im]);
            zero_pts.push([z2.re, z2.im]);
        }

        Plot::new("pole_zero")
            .data_aspect(1.0)              // critical: keeps the unit circle round
            .height(420.0)
            .x_axis_label("Re(z)")
            .y_axis_label("Im(z)")
            .legend(Legend::default())
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new(circle).name("|z| = 1"));
                plot_ui.points(
                    Points::new(PlotPoints::from(pole_pts))
                        .shape(MarkerShape::Cross)
                        .radius(7.0)
                        .name("poles"),
                );
                plot_ui.points(
                    Points::new(PlotPoints::from(zero_pts))
                        .shape(MarkerShape::Circle)
                        .radius(7.0)
                        .name("zeros"),
                );
            });
    }
}
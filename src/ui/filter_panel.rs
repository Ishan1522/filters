use eframe::egui;

use crate::dsp::spec::{FilterKind, FilterSpec};

pub struct FilterPanel;

impl FilterPanel {
    /// Returns `true` if the user clicked "Export…" this frame.
    pub fn ui(
        ui: &mut egui::Ui,
        spec: &mut FilterSpec,
        sample_rate_hz: Option<f64>,
    ) -> bool {
        ui.heading("Filter");
        ui.checkbox(&mut spec.enabled, "Enabled");
        ui.separator();

        match sample_rate_hz {
            Some(fs) => ui.label(format!(
                "Detected fs ≈ {:.1} Hz   (Nyquist {:.1} Hz)",
                fs, fs / 2.0
            )),
            None => ui.label("No channel selected — fs unknown"),
        };
        ui.separator();

        egui::ComboBox::from_label("Type")
            .selected_text(spec.kind.label())
            .show_ui(ui, |ui| {
                for k in [
                    FilterKind::ButterworthLowpass,
                    FilterKind::Chebyshev1Lowpass,
                    FilterKind::CookbookLowpass,
                    FilterKind::CookbookHighpass,
                    FilterKind::CookbookBandpass,
                    FilterKind::CookbookNotch,
                ] {
                    ui.selectable_value(&mut spec.kind, k, k.label());
                }
            });

        ui.separator();

        let max_cutoff = sample_rate_hz.map(|fs| fs / 2.0 * 0.95).unwrap_or(500.0);
        ui.add(
            egui::Slider::new(&mut spec.cutoff_hz, 0.1..=max_cutoff)
                .logarithmic(true)
                .text("Cutoff (Hz)"),
        );

        if spec.kind.uses_q() {
            ui.add(
                egui::Slider::new(&mut spec.q, 0.1..=20.0)
                    .logarithmic(true)
                    .text("Q"),
            );
            ui.small("Q ≈ 0.707 = Butterworth-flat. Higher Q = peakier resonance.");
        }
        if spec.kind.uses_order() {
            ui.add(egui::Slider::new(&mut spec.order, 1..=10).text("Order"));
            ui.small("Higher order = steeper rolloff, more phase delay.");
        }
        if spec.kind.uses_ripple() {
            ui.add(
                egui::Slider::new(&mut spec.ripple_db, 0.1..=6.0)
                    .text("Passband ripple (dB)"),
            );
            ui.small("Smaller ripple → closer to Butterworth.");
        }

        ui.separator();
        ui.checkbox(&mut spec.zero_phase, "Zero-phase (filtfilt)");
        ui.small("Offline analysis only. Doubles effective order, cancels phase lag.");
        ui.small("Not exported — ROS 2 filter nodes are always causal single-pass.");

        ui.separator();
        ui.button("Export…").clicked()
    }
}
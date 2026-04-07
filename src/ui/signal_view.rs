use std::collections::HashSet;

use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints, VLine};

use crate::analysis::{self, fft};
use crate::dsp::spec::FilterSpec;
use crate::io::log_loader::{LogFile, SampleValue};
use crate::ui::filter_response::FilterResponseView;

#[derive(Default, PartialEq, Eq)]
enum ViewMode {
    #[default]
    Time,
    Spectrum,
    FilterResponse,
}

#[derive(Default)]
pub struct SignalView {
    selected:    HashSet<u32>,
    name_filter: String,
    mode:        ViewMode,
}

struct ChannelTrace<'a> {
    name:       &'a str,
    timestamps: Vec<u64>,
    values:     Vec<f64>,
}

impl SignalView {
    pub fn estimated_sample_rate(&self, log: &LogFile) -> Option<f64> {
        for &id in &self.selected {
            let trace = self.extract_trace(log, id)?;
            if let Some(fs) = analysis::estimate_sample_rate_hz(&trace.timestamps) {
                return Some(fs);
            }
        }
        None
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, log: &LogFile, filter: &mut FilterSpec) {
        egui::SidePanel::left("channel_list_panel")
            .default_width(260.0)
            .resizable(true)
            .show_inside(ui, |ui| self.channel_list(ui, log));

        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.mode, ViewMode::Time, "Time");
                ui.selectable_value(&mut self.mode, ViewMode::Spectrum, "Spectrum");
                ui.selectable_value(&mut self.mode, ViewMode::FilterResponse, "Filter response");
            });
            ui.separator();

            match self.mode {
                ViewMode::Time     => self.time_plot(ui, log, filter),
                ViewMode::Spectrum => self.spectrum_plot(ui, log, filter),
                ViewMode::FilterResponse => {
                    let fs = self.estimated_sample_rate(log).unwrap_or(1000.0);
                    if !filter.enabled {
                        ui.label("Enable the filter (right panel) to see its response.");
                        return;
                    }
                    match filter.compile(fs) {
                        Some(cascade) => FilterResponseView::ui(ui, &cascade, fs, filter.zero_phase),
                        None => { ui.label("Invalid filter spec for this sample rate."); }
                    }
                }
            }
        });
    }

    fn channel_list(&mut self, ui: &mut egui::Ui, log: &LogFile) {
        ui.heading("Channels");
        ui.horizontal(|ui| {
            ui.label("Filter:");
            ui.text_edit_singleline(&mut self.name_filter);
        });
        if ui.button("Clear selection").clicked() {
            self.selected.clear();
        }
        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            let needle = self.name_filter.to_lowercase();
            for ch in &log.channels {
                let plottable = is_plottable(&ch.data_type);
                if !needle.is_empty() && !ch.name.to_lowercase().contains(&needle) {
                    continue;
                }
                let label = format!("{}  ({})", ch.name, ch.data_type);
                let mut checked = self.selected.contains(&ch.entry_id);
                ui.add_enabled_ui(plottable, |ui| {
                    if ui.checkbox(&mut checked, label).changed() {
                        if checked { self.selected.insert(ch.entry_id); }
                        else       { self.selected.remove(&ch.entry_id); }
                    }
                });
            }
        });
    }

    fn time_plot(&self, ui: &mut egui::Ui, log: &LogFile, filter: &FilterSpec) {
        Plot::new("time_plot")
            .legend(Legend::default())
            .x_axis_label("time (s)")
            .y_axis_label("value")
            .show(ui, |plot_ui| {
                for &id in &self.selected {
                    let Some(trace) = self.extract_trace(log, id) else { continue };
                    if trace.timestamps.is_empty() { continue; }
                    let t0 = trace.timestamps[0];

                    let raw_points: PlotPoints = trace.timestamps.iter()
                        .zip(trace.values.iter())
                        .map(|(&t, &v)| [(t - t0) as f64 / 1.0e6, v])
                        .collect();
                    plot_ui.line(Line::new(raw_points).name(trace.name));

                    if filter.enabled {
                        if let Some(fs) = analysis::estimate_sample_rate_hz(&trace.timestamps) {
                            let (uniform_t, uniform_v) =
                                analysis::resample_uniform(&trace.timestamps, &trace.values, fs);
                            if let Some(mut cascade) = filter.compile(fs) {
                                let filtered = if filter.zero_phase {
                                    cascade.process_block_filtfilt(&uniform_v)
                                } else {
                                    let mut out = vec![0.0; uniform_v.len()];
                                    cascade.process_block(&uniform_v, &mut out);
                                    out
                                };

                                let label = if filter.zero_phase {
                                    format!("{} (filtfilt)", trace.name)
                                } else {
                                    format!("{} (filtered)", trace.name)
                                };
                                let pts: PlotPoints = uniform_t.iter()
                                    .zip(filtered.iter())
                                    .map(|(&t, &v)| [t - (t0 as f64 / 1.0e6), v])
                                    .collect();
                                plot_ui.line(Line::new(pts).name(label));
                            }
                        }
                    }
                }
            });
    }

    fn spectrum_plot(&self, ui: &mut egui::Ui, log: &LogFile, filter: &mut FilterSpec) {
        ui.small("💡 Click anywhere on the spectrum to set the filter cutoff.");

        let mut clicked_freq: Option<f64> = None;

        Plot::new("spectrum_plot")
            .legend(Legend::default())
            .x_axis_label("frequency (Hz)")
            .y_axis_label("magnitude (dB)")
            .show(ui, |plot_ui| {
                for &id in &self.selected {
                    let Some(trace) = self.extract_trace(log, id) else { continue };
                    let Some(fs) = analysis::estimate_sample_rate_hz(&trace.timestamps) else { continue };

                    let (_, uniform_v) =
                        analysis::resample_uniform(&trace.timestamps, &trace.values, fs);
                    if uniform_v.len() < 16 { continue; }

                    let spec = fft::compute(&uniform_v, fs);
                    let raw_pts: PlotPoints = spec.freqs_hz.iter()
                        .zip(spec.magnitudes_db.iter())
                        .map(|(&f, &m)| [f, m])
                        .collect();
                    plot_ui.line(Line::new(raw_pts).name(trace.name));

                    if filter.enabled {
                        if let Some(mut cascade) = filter.compile(fs) {
                            let filtered = if filter.zero_phase {
                                cascade.process_block_filtfilt(&uniform_v)
                            } else {
                                let mut out = vec![0.0; uniform_v.len()];
                                cascade.process_block(&uniform_v, &mut out);
                                out
                            };
                            let fspec = fft::compute(&filtered, fs);
                            let label = if filter.zero_phase {
                                format!("{} (filtfilt)", trace.name)
                            } else {
                                format!("{} (filtered)", trace.name)
                            };
                            let pts: PlotPoints = fspec.freqs_hz.iter()
                                .zip(fspec.magnitudes_db.iter())
                                .map(|(&f, &m)| [f, m])
                                .collect();
                            plot_ui.line(Line::new(pts).name(label));
                        }
                    }
                }

                plot_ui.vline(
                    VLine::new(filter.cutoff_hz)
                        .name(format!("cutoff = {:.1} Hz", filter.cutoff_hz)),
                );

                if plot_ui.response().clicked() {
                    if let Some(coord) = plot_ui.pointer_coordinate() {
                        if coord.x > 0.0 {
                            clicked_freq = Some(coord.x);
                        }
                    }
                }
            });

        if let Some(f) = clicked_freq {
            filter.cutoff_hz = f;
        }
    }

    fn extract_trace<'a>(&self, log: &'a LogFile, id: u32) -> Option<ChannelTrace<'a>> {
        let ch = log.channels.iter().find(|c| c.entry_id == id)?;
        let samples = log.data.get(&id)?;
        let mut timestamps = Vec::with_capacity(samples.len());
        let mut values     = Vec::with_capacity(samples.len());
        for s in samples {
            if let Some(v) = sample_to_f64(&s.value) {
                timestamps.push(s.timestamp_us);
                values.push(v);
            }
        }
        Some(ChannelTrace { name: &ch.name, timestamps, values })
    }
}

fn is_plottable(dtype: &str) -> bool {
    matches!(dtype, "double" | "float" | "int64" | "boolean")
}

fn sample_to_f64(v: &SampleValue) -> Option<f64> {
    match v {
        SampleValue::Double(x)    => Some(*x),
        SampleValue::Float(x)     => Some(*x as f64),
        SampleValue::Int64(x)     => Some(*x as f64),
        SampleValue::Boolean(b)   => Some(if *b { 1.0 } else { 0.0 }),
        SampleValue::StringVal(_) => None,
    }
}
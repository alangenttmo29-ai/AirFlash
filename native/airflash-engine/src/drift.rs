//! Capture-clock drift servo for the resampler ratio.
//! Queue depth swings by a WASAPI period and an RTP packet on every update; feeding that
//! sawtooth straight into the ratio phase-modulates the audio (noise rising with frequency).
//! Two one-second low-pass stages leave only the slow drift for a critically damped PI loop.
const SMOOTHING_S: f64 = 1.0;
const PROPORTIONAL: f64 = 0.05;
const INTEGRAL: f64 = 0.000625;
pub const LIMIT: f64 = 0.0005;

pub struct Servo {
    target: f64,
    rate: f64,
    fast: f64,
    slow: f64,
    integral: f64,
}
impl Servo {
    pub fn new(target_frames: usize, rate: u32) -> Self {
        Self {
            target: target_frames as f64,
            rate: f64::from(rate),
            fast: target_frames as f64,
            slow: target_frames as f64,
            integral: 0.0,
        }
    }
    /// Relative resample ratio for the next chunk, given the queue depth before it.
    pub fn update(&mut self, depth: usize, interval_s: f64) -> f64 {
        let k = (interval_s / SMOOTHING_S).min(1.0);
        self.fast += (depth as f64 - self.fast) * k;
        self.slow += (self.fast - self.slow) * k;
        let error_s = (self.target - self.slow) / self.rate;
        self.integral = (self.integral + error_s * INTEGRAL * interval_s).clamp(-LIMIT, LIMIT);
        1.0 + (error_s * PROPORTIONAL + self.integral).clamp(-LIMIT, LIMIT)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 10 ms capture chunks against 352-frame packets at 48 kHz, device clock off by `ppm`.
    fn run(ppm: f64, seconds: f64) -> (Vec<f64>, Vec<usize>) {
        let (rate, chunk, packet, target) = (48000.0, 480.0, 352.0, 960usize);
        let mut servo = Servo::new(target, 48000);
        let (mut depth, mut capture_at, mut send_at) = (target as f64, 0.0, 0.0);
        let (mut ratios, mut depths) = (Vec::new(), Vec::new());
        while send_at < seconds {
            if capture_at <= send_at {
                let ratio = servo.update(depth as usize, 0.01);
                ratios.push(ratio);
                depth += chunk * ratio;
                capture_at += chunk / rate * (1.0 + ppm * 1e-6);
            } else {
                depth = (depth - packet).max(0.0);
                depths.push(depth as usize);
                send_at += packet / rate;
            }
        }
        (ratios, depths)
    }

    #[test]
    fn packet_granularity_does_not_modulate_the_ratio() {
        let (ratios, _) = run(0.0, 30.0);
        let tail = &ratios[ratios.len() / 2..];
        let step = tail.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0, f64::max);
        assert!(step < 1e-7, "ratio step {step}");
    }

    #[test]
    fn tracks_device_drift_without_starving_or_overflowing() {
        for ppm in [-300.0, 150.0, 300.0] {
            let (ratios, depths) = run(ppm, 240.0);
            let ratio = ratios.last().unwrap();
            assert!((ratio - 1.0 - ppm * 1e-6).abs() < 2e-5, "{ppm} ppm -> {ratio}");
            let tail = &depths[depths.len() / 2..];
            assert!(tail.iter().all(|d| (300..1900).contains(d)), "{ppm} ppm depth {tail:?}");
        }
    }

    #[test]
    fn correction_is_bounded() {
        let mut servo = Servo::new(960, 48000);
        for _ in 0..100_000 {
            assert!((servo.update(0, 0.01) - 1.0).abs() <= LIMIT + 1e-12);
        }
    }
}

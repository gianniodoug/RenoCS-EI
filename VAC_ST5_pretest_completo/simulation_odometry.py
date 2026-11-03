"""NO HARDWARE: demo of progressive encoder odometry and turn monitoring."""
from encoder_odometry import EncoderCalibration, SegmentProgressEstimator, EncoderTurnMonitor

# SYNTHETIC CALIBRATION ONLY. These numbers do not describe the VAC robot.
cal=EncoderCalibration(ticks_per_wheel_rev=1000, wheel_diameter_m=0.10,
                       forward_signs=(1,1,1,1), effective_track_width_m=0.22)
progress=SegmentProgressEstimator(cal,segment_length_m=1.0)
progress.reset((0,0,0,0))
print('SEGMENT (fictional geometry):')
for t in (250,500,1000):
    p=progress.update((t,t,t,t))
    print(f'count={t}: estimated distance={p["distance_m"]:.3f}m, progress={p["progress"]:.1%}')
turn=EncoderTurnMonitor(cal)
turn.begin((0,0,0,0),'LEFT')
print('TURN LEFT (fictional geometry):')
for t in (200,450,550):
    r=turn.update((t,-t,t,-t))
    print(r)
print('NOTE: encoder angle cannot prove white-line reacquisition.')

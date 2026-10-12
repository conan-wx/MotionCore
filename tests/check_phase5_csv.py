import csv, math, sys
path = sys.argv[1]
required = {
    'time_s','motion_state','command_id','command_status','x_ref_mm','y_ref_mm','z_ref_mm',
    'x_actual_mm','y_actual_mm','z_actual_mm','vx_actual_mm_s','vy_actual_mm_s','vz_actual_mm_s',
    'drive_force_x_n','drive_force_y_n','drive_force_z_n','homed_x','homed_y','homed_z',
    'following_error_mm','fault_code','fault_active','soft_limit_margin_x_mm',
    'soft_limit_margin_y_mm','soft_limit_margin_z_mm'
}
with open(path, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    if not required.issubset(set(reader.fieldnames or [])):
        raise SystemExit('missing CSV column')
    previous = -math.inf; count = 0
    for row in reader:
        t = float(row['time_s'])
        if not math.isfinite(t) or t < previous:
            raise SystemExit('time is not finite/monotonic')
        previous = t
        for key, value in row.items():
            if key in {'motion_state','command_status','fault_code'}: continue
            if value == '': continue
            if not math.isfinite(float(value)):
                raise SystemExit(f'non-finite value in {key}')
        count += 1
if count < 10: raise SystemExit('too few rows')
print(f'validated {count} rows')

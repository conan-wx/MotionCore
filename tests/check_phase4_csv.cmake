if(NOT DEFINED MOTIONCORE_PHASE4_EXE OR NOT DEFINED OUTPUT_CSV)
    message(FATAL_ERROR "motioncore_phase4 CSV test requires executable and output path")
endif()

execute_process(
    COMMAND "${MOTIONCORE_PHASE4_EXE}"
        --geometry line --profile scurve --mode planner
        --start-mm 0,0,0 --target-mm 30,40,50
        --vmax-mm-s 50 --amax-mm-s2 100 --jmax-mm-s3 500 --dt-ms 1
        --output "${OUTPUT_CSV}"
    RESULT_VARIABLE run_result
    OUTPUT_VARIABLE run_output
    ERROR_VARIABLE run_error
)
if(NOT run_result EQUAL 0)
    message(FATAL_ERROR "motioncore_phase4 failed: ${run_output}${run_error}")
endif()

if(NOT EXISTS "${OUTPUT_CSV}")
    message(FATAL_ERROR "motioncore_phase4 did not create ${OUTPUT_CSV}")
endif()
file(READ "${OUTPUT_CSV}" csv_text)
set(required_columns
    time_s x_ref_mm y_ref_mm z_ref_mm
    x_actual_mm y_actual_mm z_actual_mm
    vx_ref_mm_s vy_ref_mm_s vz_ref_mm_s
    vx_actual_mm_s vy_actual_mm_s vz_actual_mm_s
    jx_ref_mm_s3 jy_ref_mm_s3 jz_ref_mm_s3 jerk_ref_magnitude_mm_s3
    tracking_error_mm path_deviation_mm sync_error
    radial_error_mm
    drive_force_x_N drive_force_y_N drive_force_z_N
    drive_saturated_x drive_saturated_y drive_saturated_z
)
foreach(column IN LISTS required_columns)
    string(FIND "${csv_text}" "${column}" column_index)
    if(column_index LESS 0)
        message(FATAL_ERROR "CSV header is missing ${column}")
    endif()
endforeach()
string(FIND "${csv_text}" "jerk_ref_mm_s3" legacy_jerk_index)
if(NOT legacy_jerk_index LESS 0)
    message(FATAL_ERROR "CSV still contains the obsolete combined jerk_ref_mm_s3 column")
endif()
string(REGEX MATCH "\n[^\n]+\n" data_rows "${csv_text}")
if(data_rows STREQUAL "")
    message(FATAL_ERROR "CSV contains no data rows")
endif()

# Header-only checks cannot catch a unit conversion regression.  Run the
# deterministic Python numeric check as part of CTest so each finite Jerk row
# verifies magnitude == sqrt(jx^2 + jy^2 + jz^2) in mm/s^3.
if(NOT DEFINED SOURCE_DIR)
    set(SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
endif()
execute_process(
    COMMAND python3 "${SOURCE_DIR}/tests/check_phase4_csv.py" "${OUTPUT_CSV}"
    RESULT_VARIABLE numeric_result
    OUTPUT_VARIABLE numeric_output
    ERROR_VARIABLE numeric_error
)
if(NOT numeric_result EQUAL 0)
    message(FATAL_ERROR "Phase 4 CSV numeric check failed: ${numeric_output}${numeric_error}")
endif()

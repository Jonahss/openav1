# Oracle pass, Argon full set, 2026-09-27 (sexica, dav1d bf5a879 / libaom 0dbfa9f)

| set | streams | dav1d==md5_ref | dav1d==aomdec raw | fails | wall s |
|---|---|---|---|---|---|
| profile0_core_special | 55 | 55 | 0 | 0 | 101 |
| profile0_core | 764 | 764 | 125 | 0 | 487 |
| profile0_not_annexb_special | 55 | 55 | 0 | 0 | 108 |
| profile0_not_annexb | 15 | 15 | 2 | 0 | 6 |
| profile1_core_special | 55 | 55 | 1 | 0 | 250 |
| profile1_core | 731 | 731 | 110 | 0 | 767 |
| profile1_not_annexb_special | 55 | 55 | 0 | 0 | 215 |
| profile1_not_annexb | 14 | 14 | 2 | 0 | 18 |
| profile2_core_special | 55 | 55 | 0 | 0 | 167 |
| profile2_core | 894 | 894 | 162 | 0 | 779 |
| profile2_not_annexb_special | 55 | 55 | 0 | 0 | 143 |
| profile2_not_annexb | 8 | 8 | 0 | 0 | 17 |
| profile_switching | 7 | 7 | 0 | 0 | 1 |

aomdec raw-output mismatches are format differences (monochrome plane writing, film grain, mid-stream frame-size changes); dav1d + md5_ref is the oracle. Per-stream lines in logs/oracle_<set>.txt (not in git).

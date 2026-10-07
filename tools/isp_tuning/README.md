# Offline IMX296 chart tools

Version-controlled snapshot of the v2 chart and calibration work developed at
`/home/jetson/drive_logs/tools/isp_tuning/`. The original local copies and older
v1 experiments remain untouched. This directory does not contain private camera
captures, experiment results, installed ISP profiles or the legacy tuning scripts.

- `chart/v2/output/imx296_chart_v2_a4.pdf`: printable three-sheet vector pack.
- `chart/v2/README.md`: chart generation, layout, printing and positioning.
- `chart/v2/CALIBRATION.md`: saved-image colour/noise/detail analysis and limits.
- `csi_decode.py`: existing rig-specific limited-range nvJPEG decoder dependency,
  copied unchanged from the working tuning directory.

The detailed guides retain the original local paths so existing bench commands
continue to work. To run the **repository copy**, replace
`/home/jetson/drive_logs/tools/isp_tuning` in script/geometry paths with
`/home/jetson/Documents/github_repos/dashcam/tools/isp_tuning`. Keep actual capture
paths under `drive_logs` unchanged. The Docker wrapper mounts this tool directory
and `drive_logs` read-only, plus one new result directory read-write. Results go
under the copy of `chart/v2/results` used to launch it, which Git ignores.

No camera acquisition, service control or ISP installation is performed. A
post-render colour-matching matrix is **not** an Argus ISP CCM. Live validation
of the printed v2 chart remains pending; synthetic test results are not a claim
of improved real camera colour or noise.

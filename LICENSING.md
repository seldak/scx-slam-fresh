# Licensing

Project-authored application code, client APIs, shared interface headers,
scripts, build files, tests, and documentation are licensed under the
[MIT License](LICENSE), except where a file carries a different license notice.

The scheduler implementation remains
[GPL-2.0-only](LICENSES/GPL-2.0-only.txt):

- `bpf/scx_slam_fresh.bpf.c`
- `bpf/execution_trace.bpf.h`
- `src/scx_slam_fresh_user.c`
- `tests/test_enqueue_slice.py`

The loader embeds the compiled BPF program through its generated skeleton.
The slice test compiles scheduler code. Those components stay with the GPL
scheduler; they are not part of the application client library.

The MIT client library (`src/slamqos.c` and `src/slamqos.h`) and shared headers
in `include/` describe and update the hint interface without incorporating
the scheduler implementation. The ROS packages use this client library and
do not link the loader or embed its BPF skeleton. Using MIT components in
the GPL scheduler does not change the license offered for those components
separately.

## Dependencies and generated files

External dependencies retain their own licenses. In particular, libbpf offers
LGPL-2.1 or BSD-2-Clause licensing; ROS dependencies retain their respective
licenses. Their notices and distribution requirements still apply. The MIT
license here does not relicense external SLAM applications or datasets.

Generated artifacts do not acquire the repository's default license merely
because they are produced by an MIT build script. A generated BPF skeleton
embeds the GPL scheduler, and generated kernel type declarations must be
considered separately from project-authored interface headers.

## Earlier versions

This licensing change applies to the files distributed with it. Earlier
revisions published under GPL remain available under those terms; their
history and notices are not rewritten.

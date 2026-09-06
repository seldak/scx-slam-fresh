# Licensing

Project-authored application code, scripts, build files, tests, and documentation are licensed under the
[MIT License](LICENSE), except where a file carries a different license notice.

The scheduler implementation, loader, and scheduler tests have moved to the
external `scx_fresh` repository. Its BPF implementation, loader, and slice test
remain GPL-2.0-only; its client library and shared interface headers are MIT.

The loader embeds the compiled BPF program through its generated skeleton.
The slice test compiles scheduler code. Those components stay with the GPL
scheduler; they are not part of the application client library.

The external MIT client library and shared headers describe and update the hint interface without incorporating
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

The build copies external scheduler binaries into the ignored build directory
for use by evaluation runners. Distributing those artifacts requires the
external scheduler's GPL license and corresponding source, even though the
evaluation sources are MIT-licensed.

## Earlier versions

This licensing change applies to the files distributed with it. Earlier
revisions published under GPL remain available under those terms; their
history and notices are not rewritten.

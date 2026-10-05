# Runtime TTL recovery performance

`runtime.ttl recovered in ...` is not a metadata-only operation. Recovery reads
and decodes the complete `frame_log.pb`, replays the frames into the runtime RDF
graph, then serializes that graph as Turtle.

Measured on 2026-09-03 for
`collab_icra_sim_near_right/.../run-20260903T202549778996Z`:

- 93,469 frames, 238,983,805-byte frame log
- frame decoding: 16.1 s
- RDF projection: 1.1 s
- Turtle serialization: 0.03 s
- total: 17.2 s

The decoder is the bottleneck. A 1 kHz control loop produces roughly one frame
per simulated millisecond, so recovery time grows with simulated duration and
frame payload size, not with `--rtf`. Real-time pacing changes execution wall
time, but not the number of frames required to recover a given simulation.

The current path materializes all decoded frames before projection. A future
optimization could project frames as a stream to reduce allocation and memory
pressure, but it would still need to read and decode every frame to recover the
ordered temporal runtime graph.

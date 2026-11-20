# bin/: launchers of the build machine

These start the long jobs of the project on the machine that runs them (atium: the simulations, Vivado, the Alveo
U200 card). They are written for that machine: its home directory layout (`~/openav1-synth`, `~/measure1080`, `~/logs`),
the job runner `ramq`, and the ticket board the hooks report to (`vk`, which never fails a build). `env.sh` puts this
directory on the PATH; the login profile of the machine adds the main checkout's copy.

| script | what it does |
|---|---|
| `nightly_run.sh <worktree> <tag> [VAR=value ...]` | one run of `tools/nightly.sh` with its ticket (every nightly goes through this) |
| `nightly_main.sh` | the cron nightly on `origin/main` |
| `gate.sh` | the gate rung: the whole decoder's netlist under Verilator against five recorded shards |
| `blk_gate.sh <block> <test env...>` | netlist replay of one block |
| `ooc_synth.sh <top> <tag> <worktree>` | out-of-context synthesis of one module for the U200 part |
| `image.sh <worktree> <tag> <TE> <MHz> <strategy>` | one U200 image, through `ramq`, with its ticket and its line in `~/logs/u200-builds.log` |
| `after_build.sh <name> <sha> <tag>...` | waits for images, then measures each on the card |
| `card_measure.sh <bitstream> <tag>` | programs an image, decodes the two 1080p clips checked and unchecked with the host software of the image's own commit; a bit-exact, timing-clean image stays on the card, otherwise the previous one is put back |
| `idle_measure.sh <bitfile> <tag>` | the same on an idle host (other jobs paused meanwhile); refuses while the card is in use; `--list` shows what it would pause |
| `dec.sh <stream>` | a stream through the card with every picture on the panel |
| `wt_setup.sh <dir> <ref>` | a worktree with everything the suites need that is not in git |
| `vk ...` | the ticket hook the scripts above call |

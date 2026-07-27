1. Finishing the IPS on comm and nav uncertaitny
2. Include more metrics on the log: total delta velocity, total extra flight time, total extra distance, total path deviation, total time resolving conflict
3. Make the user-facing interface very minimal. Things they only need is writing their own conf detect, conf reso, recovery (or combinations of them).
4. Switch the a first run and how it works. People want to run things first and see how if they are curious
5. V1.0 release:
	1. First level, someone can install the opencdarr package, import the library, run it on their jupyter notebook without error, and get the figures. This is only for the pairwise conflict
	2. Second level, someone can write their own conflict detection, resolution, and recovery code, run it, and get the results, this is all MC
	3. Third level, someone can write their own conflict detection, resolution, and recovery code, run it, and get the results, this is all IPS
6. V1.1 release:
	1. Someone can write their own dynamics, run it, and get the results
	2. People can do like v1.0, but on a multi-agent env, default has 3 env: the ring encounter, the waypoint in DH-ORCA, the circular airspace sector
7. V1.2 release:
	1. People can create their own environment using a GUI. Scaling will be an issue (think of aircraft vs drone, there should be a guard on flight time that drone can't fly too long)
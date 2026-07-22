I think, I think, this can be wrapped into one line called CNS. This is from loop.py. Wdyt?

```
# each aircraft's fresh (noisy) self-fix; both endpoints carry noise

if navigation is not None and rng is not None:

fix_own = navigation.measure(own, t, rng).state

fix_intr = navigation.measure(intr, t, rng).state

else:

fix_own, fix_intr = own, intr

  

# an aircraft knows its own intent exactly, never through communication

self_own = replace(fix_own, desired=own.desired)

self_intr = replace(fix_intr, desired=intr.desired)

# what leaves the transmitter: intent stripped here (before comm), not at perceive

# time, so a dropped/held message never carries intent it was never sent with

tx_own = replace(fix_own, desired=own.desired if share_intent else None)

tx_intr = replace(fix_intr, desired=intr.desired if share_intent else None)

  

if communication is not None:

broadcasts = (

Message(source=own.id, state=tx_own, t_meas=t),

Message(source=intr.id, state=tx_intr, t_meas=t),

)

comm_state = communication.step(

comm_state, broadcasts, (own.id, intr.id), t, comm_rng

)

perceived_intr = surveil.perceived(comm_state, own.id, intr.id, t)

perceived_own = surveil.perceived(comm_state, intr.id, own.id, t)

else:

perceived_intr, perceived_own = tx_intr, tx_own # instant, perfect delivery
```
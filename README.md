# Project 1 — PMSM Field-Oriented Control (FOC)

A complete **Field-Oriented Control** drive for a Permanent-Magnet Synchronous Motor,
built from the motor's differential equations up in **base MATLAB**. FOC is the control
scheme behind essentially every modern EV traction motor and industrial servo — it makes
an AC motor behave like an easily-controlled DC motor by regulating current in the rotor
(dq) reference frame.

## Control structure (cascaded loops)

```
 speed_ref ─►[ speed PI ]─► iq_ref ─┐
                          id_ref=0 ─┴─►[ current PI ×2 ]─► vd,vq
                                     ─► inverse Park ─► (inverter) ─► PMSM
   measured phase currents ─► Clarke ─► Park(θe) ─► id, iq  (feedback)
```

- **id → 0**  : no flux-weakening, so all stator current produces torque
- **iq → torque** : commanded by the outer speed loop
- **decoupling feedforward** cancels the ω·L cross-coupling between the d and q axes

## Motor & control parameters

| | |
|---|---|
| Rs, Ld=Lq, ψm | 0.5 Ω, 4 mH, 0.10 Wb |
| Pole pairs, J, B | 4, 1×10⁻³ kg·m², 5×10⁻⁴ N·m·s |
| Current-loop bandwidth | 300 Hz |
| Speed-loop bandwidth | 15 Hz |
| Test | step to 1500 rpm, then 2 N·m load at 180 ms |

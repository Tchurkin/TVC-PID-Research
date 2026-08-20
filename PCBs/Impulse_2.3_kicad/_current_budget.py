import math
CU_RHO=1.72e-8; CU_CV=3.45e6; T=35e-6      # 1 oz outer
def ipc_w(I,dT=10.0):
    # invert IPC-2221 external: I = 0.048 * dT^0.44 * A_mil2^0.725
    A=(I/(0.048*dT**0.44))**(1/0.725)       # mil^2
    return A/1550.0031/ (T*1000)            # -> mm width
def pulse_w(I,secs,dTmax):
    # adiabatic: dT/dt = rho*J^2/cv ; solve for the width giving dTmax over `secs`
    rate=dTmax/secs
    J=math.sqrt(rate*CU_CV/CU_RHO)
    A=I/J
    return A/T*1e3
print("%-22s %-9s %-8s %-11s %-11s %-9s"%("net","current","type","IPC 10C","IPC 20C","pulse"))
rows=[
 ("7.4V / 7.4V_RAW",18.0,"pulse 0.9 s"),
 ("Net-(PnO) firing leg",18.0,"pulse 0.9 s"),
 ("VBAT / VBAT_RAW",2.4,"continuous"),
 ("5V-DIRTY (servos)",2.8,"continuous"),
 ("Net-(U3-SW)",2.8,"continuous"),
 ("5V-CLEAN",0.5,"continuous"),
 ("VIN_T",0.35,"continuous"),
 ("3.3V",0.30,"continuous"),
 ("Net-(PnO) sense leg",0.0006,"signal"),
 ("PYROn gates",0.001,"signal"),
 ("LED branches",0.010,"signal"),
]
for n,I,kind in rows:
    p = pulse_w(I,0.9,100.0) if kind.startswith("pulse") else float('nan')
    print("%-22s %-9s %-8s %-11s %-11s %-9s"%(
        n,"%.4g A"%I,kind.split()[0],"%.2f mm"%ipc_w(I),"%.2f mm"%ipc_w(I,20),
        ("%.2f mm"%p) if p==p else "-"))
print()
print("pulse column = width for <=100 C ADIABATIC rise over 900 ms (the firmware timeout,")
print("i.e. the wire-never-clears worst case the FETs are already sized for).")
print()
print("shorter pulses, 18 A, <=100 C adiabatic:")
for s in (0.05,0.1,0.2,0.4,0.9):
    print("   %5.0f ms -> %.2f mm"%(s*1000,pulse_w(18.0,s,100.0)))

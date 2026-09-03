"""Post-process a KiCad board imported from the EasyEDA Pro project.

Applies the EasyEDA design rules and zone settings, marks the two mounting holes
as board-only, refills zones, and saves in place. Idempotent.
Run with KiCad's bundled python:
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 post_import.py BOARD.kicad_pcb
"""
import os, sys, pcbnew

MM = pcbnew.FromMM
board_path = sys.argv[1]
b = pcbnew.LoadBoard(board_path)
proj_dir = os.path.dirname(os.path.abspath(board_path))

# --- board-level limits (EasyEDA rule "1"/"2"/"5"/"9", mil -> mm) -----------
ds = b.GetDesignSettings()
ds.m_MinClearance = MM(0.127)
ds.m_TrackMinWidth = MM(0.127)
ds.m_ViasMinSize = MM(0.40)
ds.m_MinThroughDrill = MM(0.20)
ds.m_HoleClearance = MM(0.175)   # EasyEDA "Hole to Track" 0.175
ds.m_HoleToHoleMin = MM(0.30)
ds.m_CopperEdgeClearance = MM(0.30)  # JLCPCB recommendation; USB1 edge pads exempted in openrz67.kicad_dru
ds.m_SolderMaskExpansion = MM(0.051)
ds.m_SolderMaskMinWidth = MM(0.0)
ds.m_SolderPasteMargin = 0
ds.m_MinSilkTextHeight = MM(0.5)

# --- net classes (EasyEDA rule "3" widths, "5" vias, RULE_SELECTOR) --------
ns = ds.m_NetSettings
def netclass(name, clearance, track, via, drill, assign=()):
    nc = pcbnew.NETCLASS(name)
    nc.SetClearance(MM(clearance)); nc.SetTrackWidth(MM(track))
    nc.SetViaDiameter(MM(via)); nc.SetViaDrill(MM(drill))
    if name == "Default":
        ns.SetDefaultNetclass(nc)
    else:
        ns.SetNetclass(name, nc)
        for net in assign:
            ns.SetNetclassPatternAssignment(net, name)
    return nc
netclass("Default", 0.127, 0.16, 0.45, 0.20)  # EasyEDA pour-to-track gap is 0.127
netclass("gnd", 0.127, 0.13, 0.45, 0.20, ["GND"])
netclass("3v", 0.127, 0.20, 0.45, 0.20, ["VCC"])
netclass("5v", 0.127, 0.254, 0.50, 0.30, ["+5V"])

# --- the importer kept EasyEDA's layer names; restore KiCad defaults ----------
for lid, name in {pcbnew.F_Cu: "F.Cu", pcbnew.B_Cu: "B.Cu", pcbnew.F_SilkS: "F.SilkS", pcbnew.B_SilkS: "B.SilkS",
                  pcbnew.F_Mask: "F.Mask", pcbnew.B_Mask: "B.Mask", pcbnew.F_Paste: "F.Paste", pcbnew.B_Paste: "B.Paste",
                  pcbnew.Edge_Cuts: "Edge.Cuts", pcbnew.F_Fab: "F.Fab", pcbnew.B_Fab: "B.Fab",
                  pcbnew.F_CrtYd: "F.Courtyard", pcbnew.B_CrtYd: "B.Courtyard"}.items():
    b.SetLayerName(lid, name)

# --- mounting holes came in as reference-less footprints: mark board-only ----
for fp in b.GetFootprints():
    if fp.GetReference() == "":
        fp.SetReference("H" + str(1 + sum(1 for f in b.GetFootprints() if f.GetReference().startswith("H"))))
        fp.SetBoardOnly(True); fp.SetExcludedFromBOM(True); fp.SetExcludedFromPosFiles(True)

# --- refill zones with KiCad's own filler so pour gaps obey the rules exactly
for z in b.Zones():
    z.SetLocalClearance(MM(0.127))
    z.SetThermalReliefGap(MM(0.152))        # EasyEDA copperRegion rule: gap 6 mil
    z.SetThermalReliefSpokeWidth(MM(0.254))  # spoke 10 mil
    z.SetMinThickness(MM(0.127))
    if z.GetNetname() == "VBUS":              # small power pour at USB1: solid connection, spokes don't fit
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())

b.Save(board_path)
print(f"saved {board_path}")

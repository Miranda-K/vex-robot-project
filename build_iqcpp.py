"""Packs wake_up_arm.cpp into Wake_Up_Arm.iqcpp so VEXcode IQ can open it.

Run after editing the .cpp:  python build_iqcpp.py
Settings match the class projects (IQ 2nd gen brain, VEXcode IQ 4.68).
"""
import json
from pathlib import Path

here = Path(__file__).parent
source = (here / "wake_up_arm.cpp").read_text(encoding="utf-8").replace("\r\n", "\n")

project = {
    "mode": "Text",
    "hardwareTarget": "brain",
    "textContent": source,
    "textLanguage": "cpp",
    "robotConfig": [],
    "slot": 0,
    "platform": "IQ",
    "sdkVersion": "20230818.11.00.00",
    "appVersion": "4.68.0",
    "fileFormat": "2.0.0",
    "targetBrainGen": "Second",
    "v5SoundsEnabled": False,
    "aiVisionSettings": {
        "colors": [], "codes": [], "tags": True, "AIObjects": True,
        "AIObjectModel": [], "aiModelDropDownValue": None,
    },
}

out = here / "Wake_Up_Arm.iqcpp"
out.write_text(json.dumps(project, separators=(",", ":")), encoding="utf-8")
print(f"Wrote {out.name}")

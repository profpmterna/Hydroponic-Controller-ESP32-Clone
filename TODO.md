# Unique Device ID Implementation TODO

## Plan Summary
Update device ID to \"COSYFARM-X\" where X is decimal number from complete eFUSE MAC (48-bit uint64_t).

**Files:** src/main.cpp, src/WiFi_Manager.cpp

**Status:** In Progress

## Steps
- [ ] Step 1: Update src/main.cpp - Change to decimal full MAC, load/save prefs.
- [ ] Step 2: Update src/WiFi_Manager.cpp - Add WiFi.setHostname().
- ✅ Step 3: Build/test PlatformIO.
- [ ] Step 4: Commit/push to main.

**Followup:** Flash, check Serial \"Device ID: COSYFARM-XXXXXXXXXX\"
